#include "dronecontroller.h"
#include "multimodule.h"
#include "flightplan.h"
#include "quaternion.h"
#include "linalg.h"

using namespace cv;
using namespace tracking;

// Create an instance of Joystick
static Joystick joystick("/dev/input/js0");
static JoystickEvent event;

bool DroneController::joystick_ready() {
    return joystick.isFound();
}

void DroneController::init(std::ofstream *logger,string replay_dir,bool generator, MultiModule * rc, tracking::DroneTracker *dtrk, CameraView *camview, float exposure) {
    _rc = rc;
    _dtrk = dtrk;
    _logger = logger;
    log_replay_mode = replay_dir!="";
    generator_mode = generator;
    _camview = camview;
    control_history_max_size = pparams.fps;
    (*_logger) << "valid;flight_mode;" <<
               "target_pos_x;target_pos_y;target_pos_z;" <<
               "autoThrottle;autoRoll;autoPitch;autoYaw;" <<
               "joyThrottle;joyRoll;joyPitch;joyYaw; " <<
               "joyArmSwitch;joyModeSwitch;joyTakeoffSwitch;" <<
               "mmArmSwitch;mmModeSwitch;" <<
               "dt;propwash;" <<
               "thrust; integrator_x;integrator_y;integrator_z;model_error;" <<
               "batt_cell_v;rssi;arm;";
    ;
    std::cout << "Initialising control." << std::endl;

    load_calibration(replay_dir);
    load_control_parameters();

    if (pparams.control_tuning) {
        std::cout << "Creating control tuning window." << std::endl;
        // create GUI to set control parameters
        namedWindow("Control", WINDOW_NORMAL);
        createTrackbar("p_pos_roll", "Control", &kp_pos_roll, 3000);
        createTrackbar("p_pos_pitch", "Control", &kp_pos_pitch, 3000);
        createTrackbar("p_pos_throttle", "Control", &kp_pos_throttle, 3000);
        createTrackbar("i_pos_roll", "Control", &ki_pos_roll, 1000);
        createTrackbar("i_pos_pitch", "Control", &ki_pos_pitch, 1000);
        createTrackbar("i_thrust", "Control", &ki_thrust, 1000);
        createTrackbar("d_pos_roll", "Control", &kd_pos_roll, 1000);
        createTrackbar("d_pos_pitch", "Control", &kd_pos_pitch, 1000);
        createTrackbar("d_pos_throttle", "Control", &kd_pos_throttle, 1000);
        createTrackbar("p_v_roll", "Control", &kp_v_roll, 1000);
        createTrackbar("p_v_pitch", "Control", &kp_v_pitch, 1000);
        createTrackbar("p_v_throttle", "Control", &kp_v_throttle, 1000);
        createTrackbar("d_v_roll", "Control", &kd_v_roll, 100);
        createTrackbar("d_v_pitch", "Control", &kd_v_pitch, 100);
        createTrackbar("d_v_throttle", "Control", &kd_v_throttle, 200);
        createTrackbar("Hover p_pos_roll", "Control", &kp_pos_roll_hover, 3000);
        createTrackbar("Hover p_pos_pitch", "Control", &kp_pos_pitch_hover, 3000);
        createTrackbar("Hover p_pos_throttle", "Control", &kp_pos_throttle_hover, 3000);
        createTrackbar("Hover i_pos_roll", "Control", &ki_pos_roll_hover, 1000);
        createTrackbar("Hover i_pos_pitch", "Control", &ki_pos_pitch_hover, 1000);
        createTrackbar("Hover i_thrust", "Control", &ki_thrust_hover, 1000);
        createTrackbar("Hover d_pos_roll", "Control", &kd_pos_roll_hover, 1000);
        createTrackbar("Hover d_pos_pitch", "Control", &kd_pos_pitch_hover, 1000);
        createTrackbar("Hover d_pos_throttle", "Control", &kd_pos_throttle_hover, 1000);
    }

    d_vel_err_x.init (1.f/pparams.fps);
    d_vel_err_y.init (1.f/pparams.fps);
    d_vel_err_z.init (1.f/pparams.fps);
    pos_err_i = {0,0,0};
    pos_err_i.y = 0;
    pos_modelx.init(1.f/pparams.fps, 1, 0.2, 0.38);
    pos_modely.init(1.f/pparams.fps, 1, 0.2, 0.38);
    pos_modelz.init(1.f/pparams.fps, 1, 0.2, 0.38);
    for (uint i=0; i<N_PLANES; i++) {
        d_vel_err_kiv.at(i).init(1.f/pparams.fps);
        d_pos_err_kiv.at(i).init(1.f/pparams.fps);
    }

    set_led_strength(exposure);

    drone_calibration.thrust = std::clamp(drone_calibration.thrust,dparams.default_thrust/2.f,dparams.default_thrust*2.f);
    initial_hover_throttle_guess_non3d = GRAVITY/dparams.default_thrust*RC_BOUND_RANGE+dparams.min_throttle;
    initialized = true;
}

void DroneController::set_led_strength(float exposure) {
    float max_ae = 20000;
    if (pparams.fps == 90)
        max_ae = 10000;
    else if (pparams.fps == 60)
        max_ae = 15000;
    else
        std::cout << "Warning, led strength not properly implemented for this fps!" << std::endl;

    float thresh_dark = (0.1333f* max_ae);
    float thresh_bright = (0.0333f* max_ae);

    if (exposure < thresh_bright)
        dparams.drone_led_strength = 100;
    else if (exposure < thresh_dark)
        dparams.drone_led_strength = 100 - (exposure/thresh_dark*50.f);
    else
        dparams.drone_led_strength = 50 - ((std::clamp(exposure,thresh_bright,max_ae)-thresh_bright)/(max_ae - thresh_bright)*45.f);

    std::cout << "Led strength set to: " << dparams.drone_led_strength << std::endl;
}

void DroneController::control(TrackData data_drone, TrackData data_target_new, TrackData data_raw_insect, double time) {
    _time = time;

    if (!log_replay_mode && pparams.joystick != rc_none)
        read_joystick();
    process_joystick();

    if (_joy_state== js_waypoint)
        data_raw_insect = data_target_new; // the takeoff burn uses raw insect, but wp flight mode also uses takeoff burn
    _dtrk->update_target(data_raw_insect.pos());

    if (!data_raw_insect.pos_valid) // if tracking is lost, set the raw setpoint to just above takeoff location, because otherwise the (takeoff) may go towards 0,0,0 (the camera)
        data_raw_insect.state.pos = _dtrk->takeoff_location() + cv::Point3f(0,0.5,0);

    _dist_to_setpoint = normf(data_drone.state.pos - data_target_new.state.pos); // TODO: [k] update this with raw data in burn mode???

    int throttle = 0,roll = 0,pitch = 0,yaw = 0, mode = bf_angle;
    bool joy_control = false;
    switch(_flight_mode) {
    case fm_manual: {
        mode += bf_headless_disabled;
        throttle = joy_throttle;
        _rc->arm(_joy_arm_switch);
        roll = joy_roll;
        pitch = joy_pitch;
        yaw = joy_yaw;
        joy_control = true;
        break;
    } case fm_blink: {
        _rc->arm(bf_disarmed);
        blink(time);
        //#268:
        // mode+=bf_spin_motor;
        // blink_motors(time);
        break;
    } case fm_spinup: {
        _rc->arm(bf_armed);
#if ENABLE_SPINUP==true
        start_takeoff_burn_time = 0;
        if (spin_up_start_time < 0.01)
            spin_up_start_time = time;
        auto_throttle = spinup_throttle();
#else
        auto_throttle = JOY_BOUND_MIN;
        spin_up_start_time = 0;
#endif
        mode += bf_PID_loop_disabled;
        auto_roll = RC_MIDDLE;
        auto_pitch = RC_MIDDLE;
        auto_yaw = RC_MIDDLE;
        break;
    } case fm_start_takeoff: {
        take_off_start_time = time;
        _flight_mode = fm_take_off_aim;
        _burn_direction_for_thrust_approx = {0};
        auto_throttle = spinup_throttle();
        auto_roll = RC_MIDDLE;
        auto_pitch = RC_MIDDLE;
        auto_yaw = RC_MIDDLE;

        landing_acc_calibration_cnt = -1;
        if (dparams.mode3d) {
            _rc->arm(bf_armed);
            break;
        }
        [[fallthrough]];
    } case fm_take_off_aim: {
        StateData state_drone_takeoff = data_drone.state;
        state_drone_takeoff.pos = _dtrk->takeoff_location() + cv::Point3f(0,lift_off_dist_take_off_aim,0);
        state_drone_takeoff.vel = {0};

        float remaing_spinup_time = dparams.full_bat_and_throttle_spinup_duration - aim_duration - time_spent_spinning_up(take_off_start_time);
        if (remaing_spinup_time< 0)
            remaing_spinup_time = 0;

        bool burn_limit_hack = true;
        float remaining_aim_duration = remaing_spinup_time+aim_duration  - static_cast<float>(time - take_off_start_time);
        if (remaining_aim_duration <= aim_duration) {
            cv::Point3f burn_direction;
            std::tie (auto_roll, auto_pitch,auto_burn_duration,burn_direction) = calc_directional_burn(state_drone_takeoff,data_raw_insect.state,0);
            if (burn_limit_hack)
                auto_burn_duration = take_off_burn_duration; //TODO: make this number dynamic such that we have just enough time to do a second directional burn?
            aim_direction_history.push_back(burn_direction);
            auto_throttle = initial_hover_throttle_guess();

            _burn_direction_for_thrust_approx = burn_direction; // to be used later to approx effective thrust
            std::vector<StateData> trajectory = predict_trajectory(auto_burn_duration, 0, burn_direction, state_drone_takeoff);
            if (log_replay_mode) // todo: tmp solution, call from visualizer instead if this viz remains to be needed
                draw_viz(state_drone_takeoff,data_raw_insect.state,time,burn_direction,auto_burn_duration,remaining_aim_duration,trajectory);
        } else
            auto_throttle = spinup_throttle();
        if (remaining_aim_duration <= 0) {
            remaining_aim_duration = 0;

            //check if insect path was constant:
            cv::Point3f avg_dir = {0};
            for (auto dir : aim_direction_history)
                avg_dir+=dir;

            avg_dir /= static_cast<float>(aim_direction_history.size());
            cv::Point3f predicted_drone_pos;
            std::tie(predicted_drone_pos, std::ignore,std::ignore) = predict_drone_after_burn(state_drone_takeoff,avg_dir,0,auto_burn_duration);
            cv::Point3f predicted_target_pos = data_raw_insect.pos() + auto_burn_duration*data_raw_insect.vel();

            double predicted_error = cv::norm(predicted_drone_pos - predicted_target_pos);
            if (predicted_error > 0.1 && !burn_limit_hack) {
                _flight_mode = fm_spinup;
                spin_up_start_time = time - static_cast<double>(dparams.full_bat_and_throttle_spinup_duration); // normal spinup may not have happened always, so just set it hard so that we don't spinup wait next attempt
            } else {
                spin_up_start_time = 0;
                _flight_mode = fm_max_burn;
                start_takeoff_burn_time = time;
                std::cout << "Take off burn" << std::endl;
            }

            aim_direction_history.clear();

        }
        break;
    } case fm_max_burn: {
        auto_throttle = RC_BOUND_MAX;
        if (static_cast<float>(time - start_takeoff_burn_time) >  auto_burn_duration)
            _flight_mode = fm_max_burn_spin_down;
        break;
    } case fm_max_burn_spin_down: {
        auto_throttle = initial_hover_throttle_guess();
        if (static_cast<float>(time - start_takeoff_burn_time) > auto_burn_duration + effective_burn_spin_down_duration)
            _flight_mode = fm_1g;
        break;
    }  case fm_1g: {
        // Wait until velocity of drone after take-off is constant, then estimate this velocity using sufficient samples
        if (static_cast<float>(time - start_takeoff_burn_time) > auto_burn_duration + effective_burn_spin_down_duration + transmission_delay_duration - 1.f / pparams.fps) {
            if(_joy_state == js_waypoint)
                _flight_mode = fm_flying_pid_init;
            else {
                _flight_mode = fm_interception_aim_start;

            }
        }
        _flight_mode = fm_flying_pid_init;
        break;
    }  case fm_interception_aim_start: {
        std::cout << "Aiming" << std::endl;
        interception_start_time = time;
        approx_effective_thrust(data_drone,_burn_direction_for_thrust_approx,auto_burn_duration,static_cast<float>(time-start_takeoff_burn_time));
        _burn_direction_for_thrust_approx = {0};
        std::cout << "Estimated acc: " << drone_calibration.thrust << std::endl;
        _flight_mode =   fm_interception_aim;
        [[fallthrough]];
    }   case fm_interception_aim: {
        cv::Point3f burn_direction;
        StateData state_drone_better = data_drone.state;
        state_drone_better.vel = drone_vel_after_takeoff;

        float remaining_aim_duration = aim_duration - static_cast<float>(time - interception_start_time);
        data_raw_insect.state.vel = {0}; // aim to the current target position

        if (remaining_aim_duration<0)
            remaining_aim_duration = 0;
        std::vector<StateData> traj;
        std::tie (auto_roll, auto_pitch, auto_burn_duration,burn_direction,traj) = calc_burn(state_drone_better,data_raw_insect.state,remaining_aim_duration);

        if (log_replay_mode) // todo: tmp solution, call from visualizer instead if this viz remains to be needed
            draw_viz(state_drone_better,data_raw_insect.state,time,burn_direction,auto_burn_duration,remaining_aim_duration,traj);

        auto_throttle = initial_hover_throttle_guess();

        if (!trajectory_in_view(traj,CameraView::relaxed) || auto_burn_duration > 1.1f || auto_burn_duration == 0.0f) {
            _flight_mode = fm_flying_pid_init;
        } else {
            std::vector<StateData> traj_back;
            cv::Point3f burn_direction_back;
            float auto_burn_duration_back;
            StateData target_back = traj.front();
            target_back.vel = {0};
            target_back.acc = {0};
            std::tie (std::ignore, std::ignore,auto_burn_duration_back,burn_direction_back,traj_back) = calc_burn(traj.back(),target_back,aim_duration);
            if (!trajectory_in_view(traj_back,CameraView::relaxed)) {
                _flight_mode = fm_flying_pid_init;
            } else if (remaining_aim_duration < 0.01f)
                _flight_mode = fm_interception_burn_start;
        }
        break;
    } case fm_interception_burn_start: {
        _flight_mode = fm_interception_burn;
        auto_throttle = RC_BOUND_MAX;
        std::cout << "Burning: " << auto_burn_duration << std::endl;
        [[fallthrough]];
    } case fm_interception_burn: {
        if (static_cast<float>(time - interception_start_time) > aim_duration + auto_burn_duration) {
            _flight_mode = fm_interception_burn_spin_down;
            std::cout << "Spindown" << std::endl;
        }
        break;
    } case fm_interception_burn_spin_down: {
        auto_throttle = initial_hover_throttle_guess();

        float spindown_duration = effective_burn_spin_down_duration;
        if (effective_burn_spin_down_duration > auto_burn_duration*0.8f)
            spindown_duration = auto_burn_duration*0.8f;

        if (static_cast<float>(time - interception_start_time) > aim_duration + auto_burn_duration + spindown_duration)
            _flight_mode = fm_retry_aim_start;
        break;
    }  case fm_retry_aim_start: {
        interception_start_time = time;
        _flight_mode = fm_interception_aim;
        std::cout << "Re-aiming" << std::endl;
        _burn_direction_for_thrust_approx = {0};
        break;
    } case fm_flying_pid_init: {
        _flight_mode = fm_flying_pid;

        d_vel_err_x.preset(data_target_new.vel().x - data_drone.state.vel.x);
        d_vel_err_y.preset(data_target_new.vel().y - data_drone.state.vel.y);
        d_vel_err_z.preset(data_target_new.vel().z - data_drone.state.vel.z);

        model_error = 0;
        if(!data_drone.pos_valid && _time-take_off_start_time < 0.5) {
            pos_modelx.internal_states(_dtrk->takeoff_location().x, _dtrk->takeoff_location().x);
            pos_modely.internal_states(_dtrk->takeoff_location().y, _dtrk->takeoff_location().y);
            pos_modelz.internal_states(_dtrk->takeoff_location().z, _dtrk->takeoff_location().z);

        } else {
            pos_modelx.internal_states(data_drone.pos().x, data_drone.pos().x);
            pos_modely.internal_states(data_drone.pos().y, data_drone.pos().y);
            pos_modelz.internal_states(data_drone.pos().z, data_drone.pos().z);

            if(data_drone.vel_valid) {
                float correction_gain = 5.f;
                pos_modelx.internal_states(data_drone.pos().x-data_drone.vel().x/pparams.fps*correction_gain, data_drone.pos().x-2*data_drone.vel().x/pparams.fps*correction_gain);
                pos_modely.internal_states(data_drone.pos().y-data_drone.vel().y/pparams.fps*correction_gain, data_drone.pos().y-2*data_drone.vel().y/pparams.fps*correction_gain);
                pos_modelz.internal_states(data_drone.pos().z-data_drone.vel().z/pparams.fps*correction_gain, data_drone.pos().z-2*data_drone.vel().z/pparams.fps*correction_gain);

            }
        }
        [[fallthrough]];
    } case fm_flying_pid: {
        if(!data_drone.pos_valid) {
            pos_err_i = {0,0,0};
            if(_time-take_off_start_time < 0.5)
                data_drone.state.pos = _dtrk->takeoff_location ();
        }

        control_model_based(data_drone, data_target_new.pos(),data_target_new.vel());

        break;
    } case fm_flying_headed_pid: { // at the moment used for landing after the yaw reset
        auto_yaw = RC_MIDDLE;
        mode += bf_headless_disabled;
        if(!data_drone.pos_valid) {
            pos_err_i = {0,0,0};
            if(_time-take_off_start_time < 0.5)
                data_drone.state.pos = _dtrk->takeoff_location ();
        }
        control_model_based(data_drone, data_target_new.pos(),data_target_new.vel());
        break;
    } case fm_initial_reset_yaw: {
        control_model_based(data_drone, data_target_new.pos(), data_target_new.vel());
        mode += bf_headless_disabled;
        break;
    } case fm_reset_yaw: {
        mode += bf_headless_disabled;
        control_model_based(data_drone, data_target_new.pos(), data_target_new.vel());
        if (data_drone.yaw_deviation_valid)
            correct_yaw(data_drone.yaw_deviation);

        break;
    } case fm_ff_landing_start: {
        _flight_mode = fm_ff_landing;
        ff_land_start_time = time;
        ff_auto_throttle_start = auto_throttle;
        auto_yaw = RC_MIDDLE;
        [[fallthrough]];
    } case fm_ff_landing: {
        control_model_based(data_drone, data_target_new.pos(), data_target_new.vel());
        float dt = static_cast<float>(time - ff_land_start_time);
        const float target_time = 0.3f; // TODO: comment
        auto_throttle  = (ff_auto_throttle_start-RC_BOUND_MIN) - (1.f/target_time) * dt * (ff_auto_throttle_start-RC_BOUND_MIN) + RC_BOUND_MIN;
        if (dt > target_time) {
            auto_throttle = RC_BOUND_MIN;
            _flight_mode = fm_shake_it_baby;
        }
        auto_yaw = RC_MIDDLE;
        break;
    } case fm_shake_it_baby: {
        _rc->arm(bf_disarmed);
        mode+=bf_spin_motor;
        int itime = round(time*5);
        int staticspin = RC_BOUND_MIN + 125;
        auto_roll = staticspin;
        auto_pitch = staticspin;
        auto_throttle = staticspin;
        auto_yaw = staticspin;

        int spin_value = RC_BOUND_MIN + 325;
        if (itime % 5 == 3) {
            auto_pitch = spin_value;
        } else if (itime % 5 == 1) {
            auto_roll = spin_value;
        } else if (itime % 5 == 2) {
            auto_throttle = spin_value;
        } else if (itime % 5 == 0) {
            auto_yaw = spin_value;
        } else if (itime % 5 == 4) {
            auto_roll = spin_value + 50;
            auto_throttle = spin_value +50;
        }
        break;
    } case fm_disarmed: {
        auto_roll = RC_MIDDLE;
        auto_pitch = RC_MIDDLE;
        auto_yaw = RC_MIDDLE;
        calibrate_landing();
        if (dparams.mode3d)
            auto_throttle = RC_MIDDLE;
        else
            auto_throttle = RC_BOUND_MIN;
        _rc->arm(bf_disarmed);
        break;
    } case fm_inactive: {
        auto_roll = RC_MIDDLE;
        auto_pitch = RC_MIDDLE;
        auto_yaw = RC_MIDDLE;
        spin_up_start_time = 0;
        mode += bf_yaw_reset;

        calibrate_landing();

        if (dparams.mode3d)
            auto_throttle = RC_MIDDLE;
        else
            auto_throttle = RC_BOUND_MIN;
        if (pparams.joystick == rc_none && !dparams.mode3d)
            _rc->arm(bf_armed);
        else if (dparams.mode3d)
            _rc->arm(bf_disarmed);
        break;
    } case fm_abort: {
        if (dparams.mode3d)
            auto_throttle = RC_MIDDLE;
        else
            auto_throttle = RC_BOUND_MIN;
        auto_roll = RC_MIDDLE;
        auto_pitch = RC_MIDDLE;
        auto_yaw = RC_MIDDLE;
        break;
    } case fm_joystick_check: {
        if (pparams.joystick == rc_none) {
            _flight_mode = fm_disarmed;
            _joy_state = js_none;
            break;
        }
        _joy_state = js_checking;
        if (dparams.mode3d)
            auto_throttle = RC_MIDDLE;
        else
            auto_throttle = RC_BOUND_MIN;
        auto_roll = RC_MIDDLE;
        auto_pitch = RC_MIDDLE;
        auto_yaw = RC_MIDDLE;
        _rc->arm(bf_disarmed);
        break;
    }
    }

    if (!joy_control) {
        pitch =  auto_pitch;
        roll = auto_roll;
        throttle = auto_throttle;
        yaw = auto_yaw;
    }

    throttle = std::clamp(throttle,RC_BOUND_MIN,RC_BOUND_MAX);
    roll = std::clamp(roll,RC_BOUND_MIN,RC_BOUND_MAX);
    pitch = std::clamp(pitch,RC_BOUND_MIN,RC_BOUND_MAX);
    yaw = std::clamp(yaw,RC_BOUND_MIN,RC_BOUND_MAX);
    auto_throttle = std::clamp(auto_throttle,RC_BOUND_MIN,RC_BOUND_MAX);
    auto_pitch = std::clamp(auto_pitch,RC_BOUND_MIN,RC_BOUND_MAX);
    auto_roll = std::clamp(auto_roll,RC_BOUND_MIN,RC_BOUND_MAX);
    auto_yaw = std::clamp(auto_yaw,RC_BOUND_MIN,RC_BOUND_MAX);

    // std::cout << time <<  " rpyt: " << roll << ", " << pitch << ", " << yaw << ", " << throttle << std::endl;
    _rc->queue_commands(throttle,roll,pitch,yaw,mode,time);

    ControlData c(Roll(), Throttle(), Pitch(), time);
    control_history.push_back(c);
    while (control_history.size() > control_history_max_size)
        control_history.erase(control_history.begin());

    (*_logger) << static_cast<int>(data_drone.pos_valid)  << ";" <<
               static_cast<int16_t>(_flight_mode) << ";" <<
               data_target_new.pos().x << ";" <<
               data_target_new.pos().y  << ";" <<
               data_target_new.pos().z << ";" <<
               auto_throttle << ";" <<
               auto_roll << ";" <<
               auto_pitch << ";" <<
               auto_yaw <<  ";" <<
               joy_throttle <<  ";" <<
               joy_roll <<  ";" <<
               joy_pitch <<  ";" <<
               joy_yaw << ";" <<
               static_cast<int>(_joy_arm_switch) << ";" <<
               static_cast<int>(_joy_mode_switch) << ";" <<
               static_cast<int>(_joy_takeoff_switch) << ";" <<
               _rc->arm_switch << ";" <<
               _rc->mode << ";" <<
               data_drone.dt << ";" <<
               propwash << ";" <<
               drone_calibration.thrust << ";" <<
               pos_err_i.x << ";" << pos_err_i.y << ";" << pos_err_i.z << ";" <<
               model_error << ";" <<
               _rc->telemetry.batt_cell_v  << ";" <<
               static_cast<int>(_rc->telemetry.rssi)  << ";" <<
               _rc->telemetry.arming_state  << ";";
}

std::tuple<int, int, float, Point3f, std::vector<StateData> > DroneController::calc_burn(StateData state_drone,StateData state_target,float remaining_aim_duration) {
    remaining_aim_duration +=  transmission_delay_duration + 1.f / pparams.fps;
    auto [auto_roll_burn, auto_pitch_burn, burn_duration,burn_direction] = calc_directional_burn(state_drone,state_target,remaining_aim_duration);
    auto traj = predict_trajectory(burn_duration, remaining_aim_duration, burn_direction, state_drone);
    return std::make_tuple(auto_roll_burn, auto_pitch_burn, burn_duration,burn_direction,traj);
}

std::tuple<int,int,float,cv::Point3f> DroneController::calc_directional_burn(StateData state_drone, StateData state_target, float remaining_aim_duration) {

    cv::Point3f target_pos_after_aim = state_target.pos + remaining_aim_duration * state_target.vel;

    //do an initial guess of the necessary burn direction and duration. Crude, but better algorithms are used during the iterative process
    cv::Point3f drone_pos_after_aim = state_drone.pos + remaining_aim_duration * state_drone.vel;
    cv::Point3f burn_dist = target_pos_after_aim - drone_pos_after_aim;
    float burn_duration = 5.f / pparams.fps; // initial guess: minimum burn time
    cv::Point3f drone_pos_after_burn,target_pos_after_burn,burn_direction;
    burn_direction = burn_dist/norm(burn_dist);
    float conversion_speed = 1;
    //iterative approximate the acceleration needed to hit the insect
    for (int i = 0; i < 100; i++) {
        cv::Point3f integrated_pos, integrated_vel;
        std::tie(integrated_pos, integrated_vel, std::ignore) = predict_drone_after_burn(
                    state_drone,burn_direction,remaining_aim_duration,burn_duration);

        drone_pos_after_burn = integrated_pos;
        target_pos_after_burn = target_pos_after_aim + burn_duration * state_target.vel;

        cv::Point3f pos_err = target_pos_after_burn - integrated_pos;
        burn_dist +=pos_err;
        burn_direction = burn_dist/norm(burn_dist);

        double derr = norm(pos_err);
        if (derr < 0.01) { // if converged
            break;
        }

        //to determine whether the burn_duration needs to in/decrease we check if the pos after
        //the burn is moving away or towards the insect. This is done by checking if the angle between
        //the vel vector and the delta_pos vector is bigger then 90 degrees:
        if (integrated_vel.dot(target_pos_after_burn-integrated_pos ) > 0)
            burn_duration += conversion_speed * static_cast<float>(norm(pos_err)/norm(integrated_vel));
        else
            burn_duration -= conversion_speed * static_cast<float>(norm(pos_err)/norm(integrated_vel));

        //in/decrease the conversion speed and retry if the loop converged to a wrong maximum:
        if (burn_duration< 0) {
            burn_duration = 1.0;
            conversion_speed *=0.5f;
        } else if (burn_duration > 1) {
            burn_duration = 0.2f;
            conversion_speed *=0.5f;
        }

        if (i == 20 && conversion_speed > 0.9f && conversion_speed  < 1.1f )
            conversion_speed *=0.25f;

        if (i>=99) {
            std::cout << "Warning: calc_directional_burn not converged!" << std::endl;
            burn_duration = 0.0;
            burn_direction = {0,1,0};
        }
    }

    if(burn_direction.z>0) { // Quick hack to ensure that the drone is not taking of twards the camera
        burn_direction.z = 0;
        burn_direction = burn_direction/norm(burn_direction);
    }

    burn_direction = lowest_direction_to_horizontal(burn_direction, min_takeoff_angle);

    auto [roll_quat, pitch_quat] = acc_to_quaternion(burn_direction);
    int roll_cmd =  roundf((roll_quat * RC_BOUND_RANGE / 2.f) + RC_MIDDLE); // convert to RC commands range
    int pitch_cmd = roundf((-pitch_quat * RC_BOUND_RANGE / 2.f) + RC_MIDDLE);

    return std::make_tuple(roll_cmd,pitch_cmd,burn_duration,burn_direction);
}

//calculate the predicted drone location after a burn
std::tuple<cv::Point3f, cv::Point3f, cv::Point3f> DroneController::predict_drone_after_burn(
    StateData state_drone, cv::Point3f burn_direction,float remaining_aim_duration, float burn_duration) {

    cv::Point3f integrated_pos = state_drone.pos;
    cv::Point3f integrated_vel = state_drone.vel;

    float partial_effective_burn_spin_up_duration = effective_burn_spin_up_duration;
    if (burn_duration  < effective_burn_spin_up_duration)
        partial_effective_burn_spin_up_duration = burn_duration;

    //state after aiming:
    if (remaining_aim_duration>0) {
        cv::Point3f drone_acc_after_aim = burn_direction*GRAVITY; // thrust at then end of aim
        drone_acc_after_aim.y -= GRAVITY; // acc = thrust - gravity
        cv::Point3f acc_now = (aim_duration-remaining_aim_duration)/ aim_duration *  drone_acc_after_aim; // acc now (somewhere during the aim), assuming linear acc
        cv::Point3f drone_acc_during_aim = 0.5f*(drone_acc_after_aim + acc_now); // average acc during aim time, assuming linear acc
        integrated_pos += 0.5f*drone_acc_during_aim*powf(remaining_aim_duration,2) + integrated_vel * remaining_aim_duration;
        integrated_vel += drone_acc_during_aim * remaining_aim_duration;
    }

    cv::Point3f burn_accelleration = burn_direction * drone_calibration.thrust;
    if (burn_duration  > effective_burn_spin_up_duration) {
        burn_accelleration -=cv::Point3f(0,GRAVITY,0);
        // Phase: spin up
        integrated_pos += 0.25f * burn_accelleration *powf(effective_burn_spin_up_duration,2) + integrated_vel * effective_burn_spin_up_duration;
        integrated_vel += 0.5f * burn_accelleration  * effective_burn_spin_up_duration;
        // Phase: max burn
        float t_mt = burn_duration - effective_burn_spin_up_duration;
        integrated_pos += 0.5f * burn_accelleration *powf(t_mt,2) + integrated_vel * t_mt;
        integrated_vel += burn_accelleration * t_mt;
    } else {
        burn_accelleration = burn_accelleration * (partial_effective_burn_spin_up_duration / effective_burn_spin_up_duration); // only part of the max acc is reached because the burn was too short
        burn_accelleration -=cv::Point3f(0,GRAVITY,0);

        // Phase: spin up
        integrated_pos += 0.25f * burn_accelleration *powf(partial_effective_burn_spin_up_duration,2) + integrated_vel * partial_effective_burn_spin_up_duration;
        integrated_vel += 0.5f * burn_accelleration * partial_effective_burn_spin_up_duration;
    }
    return std::make_tuple(integrated_pos, integrated_vel, burn_accelleration);
}

void DroneController::draw_viz(
    StateData state_drone, StateData state_target,double time, cv::Point3f burn_direction,
    float burn_duration, float remaining_aim_duration, std::vector<StateData> traj) {

    double viz_time_after_aim = time + static_cast<double>(remaining_aim_duration + transmission_delay_duration + 1.f/pparams.fps);
    viz_target_pos_after_burn = state_target.pos + (remaining_aim_duration + burn_duration) * state_target.vel; // TODO: transmission_delay_duration + 1.f/pparams.fps ???
    if (_flight_mode == fm_take_off_aim || _flight_mode == fm_max_burn) {
        cv::Point3f integrated_pos, integrated_vel;
        cv::Point3f burn_accelleration;
        std::tie(integrated_pos, integrated_vel,burn_accelleration) = predict_drone_after_burn(
                    state_drone, burn_direction,0,burn_duration);

        viz_drone_pos_after_burn =  traj.back().pos;
        viz_time_after_burn = viz_time_after_aim + static_cast<double>(burn_duration );
        viz_drone_trajectory = predict_trajectory(burn_duration, 0, burn_direction, state_drone);
    } else {
        viz_time_after_burn = viz_time_after_aim + static_cast<double>(burn_duration);
        viz_drone_trajectory = traj;
        viz_drone_pos_after_burn =  traj.back().pos;
    }
}

void DroneController::calibrate_landing() {
    if (landing_acc_calibration_cnt>0) {
        if (_rc->telemetry.batt_cell_v > 4.0f && _rc->telemetry.batt_cell_v < 4.4f) { // TODO: need better way to determine if drone is being charged
            cv::Point3f max_acc(0.2f,0.2f,1.1f);
            cv::Point3f min_acc(-0.2f,-0.2f,0.9f);
            if (_rc->telemetry.acc.x < max_acc.x && _rc->telemetry.acc.x > min_acc.x &&
                    _rc->telemetry.acc.y < max_acc.y && _rc->telemetry.acc.y > min_acc.y &&
                    _rc->telemetry.acc.z < max_acc.z && _rc->telemetry.acc.z > min_acc.z ) {
                landing_acc_calibration += _rc->telemetry.acc;
                landing_acc_calibration_cnt--;
            }
        }
    } else if (landing_acc_calibration_cnt == 0) {
        landing_acc_calibration /= static_cast<float>(required_landing_acc_calibration_cnt);
        drone_calibration.landed_acc_x = landing_acc_calibration.x;
        drone_calibration.landed_acc_y = landing_acc_calibration.y;
        drone_calibration.landed_acc_z = landing_acc_calibration.z;
        std::cout << "Landing attitude calibrated: " << landing_acc_calibration << std::endl;
        landing_acc_calibration_cnt = -1; // flag that calibration is finished
    }
}

std::vector<StateData> DroneController::predict_trajectory(float burn_duration, float remaining_aim_duration, cv::Point3f burn_direction, StateData state_drone) {
    std::vector<StateData> traj;
    cv::Point3f integrated_pos, integrated_vel;
    for (float f=0; f <= burn_duration; f+=1.f/pparams.fps) {
        std::tie (integrated_pos, integrated_vel,std::ignore) = predict_drone_after_burn(
                    state_drone, burn_direction,remaining_aim_duration,f);
        StateData state;
        state.pos = integrated_pos;
        state.vel = integrated_vel;
        traj.push_back(state);
    }
    return traj;
}

bool DroneController::trajectory_in_view(std::vector<StateData> traj, CameraView::view_volume_check_mode c) {
    for (auto state : traj) {
        bool inview;
        std::tie(inview, ignore) = _camview->in_view(state.pos,c);
        if (!inview)
            return false;
    }
    return true;
}

std::tuple<float,float> DroneController::acc_to_deg(cv::Point3f acc) {
    float norm_burn_vector_XZ = sqrt(powf(acc.x,2)+powf(acc.z,2));
    float rotation_angle = atan2f(norm_burn_vector_XZ,(acc.y))*rad2deg;
    float roll = -acc.x/norm_burn_vector_XZ*rotation_angle;
    float pitch = -acc.z/norm_burn_vector_XZ*rotation_angle;

    roll = std::clamp(roll,-max_bank_angle,max_bank_angle);
    pitch = std::clamp(pitch,-max_bank_angle,max_bank_angle);
    return std::make_tuple(roll,pitch);
}

std::tuple<float,float> DroneController::acc_to_quaternion(cv::Point3f acc) {
    cv::Point3f acc_BF = {-acc.z,-acc.x,-acc.y};
    cv::Point3f acc_BF_hover = {0,0,-1};
    Quaternion q = rot_quat(acc_BF_hover, acc_BF);
    return std::make_tuple(q.v.x,q.v.y);
}

//considering a take off order from just after the aiming phase of: spinning up, burning, spin down, 1g, find the max drone acceleration that best desccribes the current position given dt
void DroneController::approx_effective_thrust(TrackData data_drone, cv::Point3f burn_direction, float burn_duration, float dt_burn) {

    cv::Point3f pos_after_aim =  _dtrk->takeoff_location() + cv::Point3f(0,lift_off_dist_take_off_aim,0);
    float partial_effective_burn_spin_up_duration = effective_burn_spin_down_duration; // if the burn duration is long enough this is equal otherwise it may be shortened
    cv::Point3f acc = burn_direction * drone_calibration.thrust ; // initial guess, variable to be optimized
    cv::Point3f meas_pos =  data_drone.pos(); // current measured drone position
    cv::Point3f integrated_vel2 = {0};
    for (int i = 0; i<100; i++) {

        cv::Point3f max_acc = acc;
        cv::Point3f integrated_pos2 = pos_after_aim;
        cv::Point3f max_jerk; // = acc/effective_burn_spin_up_duration;

        if (burn_duration  > effective_burn_spin_up_duration) {
            max_acc -=cv::Point3f(0,GRAVITY,0);
            max_jerk = max_acc/effective_burn_spin_up_duration;
            // Phase: spin up
            // using jerk  x=x0+v0t+1/2a0*t^2+1/6j*t^3 v=v0+a0t+1/2j*t^2
            integrated_pos2 += 0.167f * max_jerk * powf(effective_burn_spin_up_duration,3);
            integrated_vel2 = 0.5f * max_jerk  * powf(effective_burn_spin_up_duration,2);

            // Phase: max burn
            float t_mt = burn_duration - effective_burn_spin_up_duration;
            integrated_pos2 += 0.5f * max_acc *powf(t_mt,2) + integrated_vel2 * t_mt;
            integrated_vel2 += max_acc * t_mt;

        } else {
            partial_effective_burn_spin_up_duration = burn_duration;
            max_acc = acc * (burn_duration / effective_burn_spin_up_duration); // only part of the max acc is reached because the burn was too short
            max_acc -=cv::Point3f(0,GRAVITY,0);
            max_jerk = max_acc/effective_burn_spin_up_duration;

            // Phase: spin up
            // using jerk  x=x0+v0t+1/2a0*t^2+1/6j*t^3 v=v0+a0t+1/2j*t^2
            integrated_pos2 += 0.167f * max_jerk * powf(partial_effective_burn_spin_up_duration,3);
            integrated_vel2 = 0.5f * max_jerk  * powf(partial_effective_burn_spin_up_duration,2);
        }

        float partial_effective_burn_spin_down_duration = dt_burn - burn_duration;
        if (partial_effective_burn_spin_down_duration > 0) {
            // Phase: spin down
            integrated_pos2 += 0.167f * max_jerk * powf(effective_burn_spin_down_duration,3) + integrated_vel2*effective_burn_spin_down_duration;
            integrated_vel2 += 0.5f * max_jerk  * powf(effective_burn_spin_down_duration,2) ;
        } else
            std::cout << "Error: not implemented" << std::endl;  //todo: or never needed... ?

        // Phase: 1G: (the remaining time)
        integrated_pos2 += integrated_vel2 * (dt_burn-burn_duration-partial_effective_burn_spin_up_duration);

        cv::Point3f err = meas_pos - integrated_pos2;
        acc += err / powf(dt_burn,2);
        if (norm(err)< 0.01)
            break;

    }
    drone_calibration.thrust = static_cast<float>(norm(acc));
    drone_vel_after_takeoff = integrated_vel2 * 1.15f; // TODO determine magical battery healt factor :(
}

float DroneController::thrust_to_throttle(float thrust_ratio) {
    float p1 = 3433.2;
    float p2 = -4833.5;
    float p3 = 1838.6;
    float p4 = 1011.6;
    float p5 = 355.45;
    return p1*powf(thrust_ratio,4) + p2*powf(thrust_ratio,3) + p3*powf(thrust_ratio,2) + p4*thrust_ratio + p5;
}

cv::Point3f DroneController::keep_in_volume_correction_acceleration(TrackData data_drone) {
    float drone_rotating_time = 6.f/pparams.fps; // Est. time to rotate the drone around 180 deg. see betaflight
    float safety = 2.f;
    std::array<float, N_PLANES> speed_error_normal_to_plane;
    float effective_acceleration, remaining_breaking_distance_normal_to_plane, required_breaking_time, allowed_velocity_normal_to_plane;
    bool enough_braking_distance_left=true;
    std::array<bool, N_PLANES> violated_planes_brakedistance = {false};
    float current_drone_speed_normal_to_plane;
    for(uint i=0; i<N_PLANES; i++) {
        current_drone_speed_normal_to_plane = data_drone.state.vel.dot(-cv::Point3f(_camview->plane_normals.at(i)));
        remaining_breaking_distance_normal_to_plane = _camview->calc_shortest_distance_to_plane(data_drone.pos(), i, CameraView::relaxed)
                - current_drone_speed_normal_to_plane*(drone_rotating_time+transmission_delay_duration);
        if(remaining_breaking_distance_normal_to_plane<0)
            remaining_breaking_distance_normal_to_plane = 0;
        effective_acceleration = drone_calibration.thrust/safety + cv::Point3f(0,-GRAVITY,0).dot(cv::Point3f(_camview->plane_normals.at(i)));
        required_breaking_time = sqrt(2*remaining_breaking_distance_normal_to_plane/effective_acceleration);
        allowed_velocity_normal_to_plane = required_breaking_time * effective_acceleration;
        speed_error_normal_to_plane.at(i) = current_drone_speed_normal_to_plane - allowed_velocity_normal_to_plane;
        if(speed_error_normal_to_plane.at(i)>0) {
            enough_braking_distance_left = false;
            violated_planes_brakedistance.at(i) = true;
        }
    }

    bool drone_in_boundaries;
    std::array<bool, N_PLANES> violated_planes_inview;
    std::tie(drone_in_boundaries, violated_planes_inview) = _camview->in_view(data_drone.pos(), CameraView::relaxed);

    for (uint i=0; i<N_PLANES; i++) {
        pos_err_kiv.at(i) = -_camview->calc_shortest_distance_to_plane(data_drone.pos(), i, CameraView::relaxed);
        d_pos_err_kiv.at(i).new_sample(pos_err_kiv.at(i));
        if(data_drone.vel_valid) { // ask sjoerd
            vel_err_kiv.at(i) = speed_error_normal_to_plane.at(i);
            d_vel_err_kiv.at(i).new_sample(vel_err_kiv.at(i));
        }
    }

    bool flight_mode_with_kiv = _flight_mode==fm_flying_pid || _flight_mode==fm_initial_reset_yaw
                                || _flight_mode==fm_reset_yaw;
    if(!flight_mode_with_kiv)
        return cv::Point3f(0,0,0);

    if(!drone_in_boundaries || !enough_braking_distance_left) {
        cv::Point3f correction_acceleration = kiv_acceleration(violated_planes_inview, violated_planes_brakedistance);
        // std::cout <<"KIV: " << correction_acceleration << std::endl;
        return correction_acceleration;
    }
    return cv::Point3f(0,0,0);
}

cv::Point3f DroneController::kiv_acceleration(std::array<bool, N_PLANES> violated_planes_inview, std::array<bool, N_PLANES> violated_planes_brakedistance) {
#if CAMERA_VIEW_DEBUGGING
    _camview->cout_plane_violation(violated_planes_inview, violated_planes_brakedistance);
#endif
    cv::Point3f correction_acceleration(0,0,0);
    for(uint i=0; i<N_PLANES; i++) {
        if(!(_time-start_takeoff_burn_time<0.45 && i==CameraView::bottom_plane)) {
            if(violated_planes_inview.at(i)) {
                bool d_against_p_error = (sign(d_pos_err_kiv.at(i).current_output())!=sign(pos_err_kiv.at(i)));
                correction_acceleration += _camview->normal_vector(i)*(4.f*pos_err_kiv.at(i)
                                           + d_against_p_error * 0.1f*d_pos_err_kiv.at(i).current_output());
            }

            if(violated_planes_brakedistance.at(i))
                correction_acceleration += _camview->normal_vector(i)*(0.5f*vel_err_kiv.at(i) + 0.01f*d_vel_err_kiv.at(i).current_output());
        }
    }
    return correction_acceleration;
}

std::tuple<int,int,int> DroneController::calc_feedforward_control(cv::Point3f desired_acceleration) {

    cv::Point3f drone_vel = _dtrk->last_track_data().vel();
    propwash = prop_wash(drone_vel, desired_acceleration);
    if(propwash) {
        desired_acceleration /= normf(desired_acceleration);
        desired_acceleration *= 0.65f*drone_calibration.thrust; //at ~0.8 drone starts crashing
    }

    cv::Point3f des_acc_drone = desired_acceleration_drone(desired_acceleration, drone_calibration.thrust);
    cv::Point3f direction = des_acc_drone/normf(des_acc_drone);

    float throttlef = normf(des_acc_drone)/drone_calibration.thrust;
    int throttle_cmd =  static_cast<uint16_t>(roundf(thrust_to_throttle(throttlef)));

    if (throttle_cmd < dparams.min_throttle)
        throttle_cmd = dparams.min_throttle;

    auto [roll_quat, pitch_quat] = acc_to_quaternion(direction);
    int roll_cmd =  roundf((roll_quat * RC_BOUND_RANGE / 2.f) + RC_MIDDLE); // convert to RC commands range
    int pitch_cmd = roundf((-pitch_quat * RC_BOUND_RANGE / 2.f) + RC_MIDDLE);
    return std::make_tuple(roll_cmd,pitch_cmd,throttle_cmd);
}


cv::Point3f DroneController::desired_acceleration_drone(cv::Point3f des_acc, float thrust) {
    // Calculate safe acceleration (max desired acceleration which cannot the
    // thrust constraints if it gravity compensated) for case in the further calculation
    // something goes wrong
    cv::Point3f acc_backup = des_acc/normf(des_acc)*(thrust-GRAVITY) + cv::Point3f(0,GRAVITY,0);

    cv::Point3f req_acc = des_acc + cv::Point3f(0,GRAVITY,0);

    if(normf(req_acc)<thrust)
        return req_acc;

    if(normf(des_acc)>thrust)
        des_acc = des_acc*thrust/normf(des_acc);

    float denominator = des_acc.dot(des_acc);
    float p = 2*des_acc.y*GRAVITY/denominator;
    float q = (powf(GRAVITY, 2) - powf(thrust, 2))/denominator;

    float root_square = powf(p/2.f, 2)-q;
    if(root_square<0.f) {
        std::cout << "WARNING: Unexpected results in `desired_acceleration_drone`. Return safe acceleration."<<std::endl;
        return acc_backup;
    }

    float k1 = -p/2.f + sqrt(root_square);
    float k2 = -p/2.f - sqrt(root_square);

    bool k1_valid = 0<=k1 && k1<=1;
    bool k2_valid = 0<=k2 && k2<=1;

    if(k1_valid && !k2_valid) {
        return k1*des_acc + cv::Point3f(0,GRAVITY,0);
    } else if(!k1_valid && k2_valid) {
        return k2*des_acc + cv::Point3f(0,GRAVITY,0);
    } else if(k1_valid && k2_valid) {
        if(k1>=k2) {
            return k1*des_acc + cv::Point3f(0,GRAVITY,0);
        } else {
            return k2*des_acc + cv::Point3f(0,GRAVITY,0);
        }
    } else if(!k1_valid && !k2_valid) {
        std::cout << "WARNING: Unexpected results in `desired_acceleration_drone`. Return safe acceleration."<<std::endl;
        return acc_backup;
    }
    else {
        std::cout << "WARNING: Unexpected results in `desired_acceleration_drone`. Return safe acceleration."<<std::endl;
        return acc_backup;
    }

}

bool DroneController::prop_wash(cv::Point3f drone_velocity, cv::Point3f des_acc_drone) {
    float speed = normf(drone_velocity);

    if(speed==0 || normf(des_acc_drone)==0)
        return false; //Assume no propwash is happening

    float dot = des_acc_drone.dot(drone_velocity)/normf(des_acc_drone)/normf(drone_velocity);
    dot = 1 - (dot + 1)/2;
    float vel_ind = speed>3.f ? 1.f : speed/3.f;
    std::cout << "Propwash-indicator: " << dot << ", " << vel_ind << "->" << dot*vel_ind << std::endl;
    if(dot*vel_ind > 0.75f) {
        return true;
    } else
        return false;
}

void DroneController::control_model_based(TrackData data_drone, cv::Point3f setpoint_pos, cv::Point3f setpoint_vel) {

    if (_dtrk->image_predict_item().out_of_image) {
        auto_roll = RC_MIDDLE;
        auto_pitch = RC_MIDDLE;
        auto_yaw = RC_MIDDLE;
        auto_throttle = spinup_throttle();
        return;
    }

    pos_modelx.new_sample(setpoint_pos.x);
    pos_modely.new_sample(setpoint_pos.y);
    pos_modelz.new_sample(setpoint_pos.z);

    cv::Point3f kp_pos, ki_pos, kd_pos, kp_vel, kd_vel;
    std::tie(kp_pos, ki_pos, kd_pos, kp_vel, kd_vel) = adjust_control_gains(data_drone, setpoint_pos, setpoint_vel);

    cv::Point3f pos_err_p, pos_err_d, vel_err_p, vel_err_d;
    std::tie(pos_err_p, pos_err_d, vel_err_p, vel_err_d) = control_error(data_drone, setpoint_pos, setpoint_vel,ki_pos);

    cv::Point3f desired_acceleration = {0};
    desired_acceleration += multf(kp_pos, pos_err_p) + multf(ki_pos, pos_err_i) + multf(kd_pos, pos_err_d); // position controld
    if( !(norm(setpoint_vel)<0.1 && norm(setpoint_pos-data_drone.pos())<0.2 && data_drone.pos_valid) ) // Needed to improve hovering at waypoint
        desired_acceleration += multf(kp_vel, vel_err_p) + multf(kd_vel, vel_err_d); // velocity control
    if (data_drone.pos_valid && data_drone.vel_valid)
        desired_acceleration += keep_in_volume_correction_acceleration(data_drone);
    std::tie(auto_roll, auto_pitch, auto_throttle) = calc_feedforward_control(desired_acceleration);
}

std::tuple<cv::Point3f, cv::Point3f, cv::Point3f, cv::Point3f, cv::Point3f> DroneController::adjust_control_gains(TrackData data_drone, cv::Point3f setpoint_pos, cv::Point3f setpoint_vel) {
    float kp_pos_roll_scaled, kp_pos_throttle_scaled, kp_pos_pitch_scaled, ki_pos_roll_scaled, ki_thrust_scaled, ki_pos_pitch_scaled, kd_pos_roll_scaled, kd_pos_throttle_scaled, kd_pos_pitch_scaled, kp_v_roll_scaled, kp_v_throttle_scaled, kp_v_pitch_scaled, kd_v_roll_scaled, kd_v_throttle_scaled, kd_v_pitch_scaled;

    cv::Point3f scale_pos_p = {0.01f, 0.01f, 0.01f};
    cv::Point3f scale_pos_i = {0.001f, 0.001f, 0.001f};
    cv::Point3f scale_pos_d = {0.01f, 0.01f, 0.01f};
    cv::Point3f scale_vel_p = {0.01f, 0.01f, 0.01f};
    cv::Point3f scale_vel_d = {0.01f, 0.01f, 0.01f};

    float duration_waypoint_update = duration_since_waypoint_changed(data_drone.time);
    if (normf(setpoint_vel) >= 0.01f || duration_waypoint_update <= 1) {
        scale_pos_i.x = -1; // flag that i is not used currently
        scale_pos_i.z = -1;
    }

    if (_hover_mode) {
        kp_pos_roll_scaled = scale_pos_p.x*kp_pos_roll_hover;
        kp_pos_throttle_scaled = scale_pos_p.y*kp_pos_throttle_hover;
        kp_pos_pitch_scaled = scale_pos_p.z*kp_pos_pitch_hover;
        ki_pos_roll_scaled = scale_pos_i.x*ki_pos_roll_hover;
        ki_thrust_scaled = scale_pos_i.y*ki_thrust_hover;
        ki_pos_pitch_scaled = scale_pos_i.z*ki_pos_pitch_hover;
        kd_pos_roll_scaled = scale_pos_d.x*kd_pos_roll_hover;
        kd_pos_throttle_scaled = scale_pos_d.y*kd_pos_throttle_hover;
        kd_pos_pitch_scaled = scale_pos_d.z*kd_pos_pitch_hover;
        kp_v_roll_scaled = 0;
        kp_v_throttle_scaled = 0;
        kp_v_pitch_scaled = 0;
        kd_v_roll_scaled = 0;
        kd_v_throttle_scaled = 0;
        kd_v_pitch_scaled = 0;
    } else {
        if(!data_drone.pos_valid) {
            scale_pos_p *= 0.8;
        } else if ( normf(data_drone.state.vel)<0.1f && normf(setpoint_vel)<0.1f && normf(setpoint_pos-data_drone.pos())<0.2f) {
            scale_pos_d *= 1.1f;
            scale_pos_i *= 1.1f;
        }
        kp_pos_roll_scaled = scale_pos_p.x*kp_pos_roll;
        kp_pos_throttle_scaled = scale_pos_p.y*kp_pos_throttle;
        kp_pos_pitch_scaled = scale_pos_p.z*kp_pos_pitch;
        ki_pos_roll_scaled = scale_pos_i.x*ki_pos_roll;
        ki_thrust_scaled = scale_pos_i.y*ki_thrust;
        ki_pos_pitch_scaled = scale_pos_i.z*ki_pos_pitch;
        kd_pos_roll_scaled = scale_pos_d.x*kd_pos_roll;
        kd_pos_throttle_scaled = scale_pos_d.y*kd_pos_throttle;
        kd_pos_pitch_scaled = scale_pos_d.z*kd_pos_pitch;
        kp_v_roll_scaled = scale_vel_p.x * kp_v_roll;
        kp_v_throttle_scaled = scale_vel_p.x * kp_v_throttle;
        kp_v_pitch_scaled = scale_vel_p.x * kp_v_pitch;
        kd_v_roll_scaled = scale_vel_d.x * kd_v_roll;
        kd_v_throttle_scaled = scale_vel_d.x * kd_v_throttle;
        kd_v_pitch_scaled = scale_vel_d.x * kd_v_pitch;
    }

    // Arrange gains (needed because may be updated from trackbars)
    cv::Point3f kp_pos(kp_pos_roll_scaled,kp_pos_throttle_scaled,kp_pos_pitch_scaled);
    cv::Point3f kd_pos(kd_pos_roll_scaled,kd_pos_throttle_scaled,kd_pos_pitch_scaled);
    cv::Point3f ki_pos(ki_pos_roll_scaled,ki_thrust_scaled,ki_pos_pitch_scaled);
    cv::Point3f kp_vel(kp_v_roll_scaled,kp_v_throttle_scaled,kp_v_pitch_scaled);
    cv::Point3f kd_vel(kd_v_roll_scaled,kd_v_throttle_scaled,kd_v_pitch_scaled);

    return std::tuple(kp_pos, ki_pos, kd_pos, kp_vel, kd_vel);
}

std::tuple<cv::Point3f, cv::Point3f, cv::Point3f, cv::Point3f> DroneController::control_error(TrackData data_drone, cv::Point3f setpoint_pos, cv::Point3f setpoint_vel, cv::Point3f ki_pos) {
    float err_x_filtered = 0, err_y_filtered = 0, err_z_filtered = 0;
    if(data_drone.pos_valid) {
        err_x_filtered = setpoint_pos.x - data_drone.state.spos.x;
        err_y_filtered = setpoint_pos.y - data_drone.state.spos.y;
        err_z_filtered = setpoint_pos.z - data_drone.state.spos.z;
    }
    cv::Point3f pos_err_p = {err_x_filtered, err_y_filtered, err_z_filtered};

    // If the distance is large, then increase the velocity setpoint such that the velocity controller is not counteracting to the position controller:
    cv::Point3f pos_err2vel_set = deadzone(pos_err_p,-1,1) * 1.f;
    float err_velx_filtered = 0,err_vely_filtered = 0,err_velz_filtered = 0;
    float errDx = 0,errDy = 0,errDz = 0;
    float errvDx = 0,errvDy = 0,errvDz = 0;
    if(data_drone.vel_valid) {
        err_velx_filtered = setpoint_vel.x + pos_err2vel_set.x - data_drone.state.vel.x;
        err_vely_filtered = setpoint_vel.y + pos_err2vel_set.y - data_drone.state.vel.y;
        err_velz_filtered = setpoint_vel.z + pos_err2vel_set.z - data_drone.state.vel.z;

        errDx = -data_drone.state.vel.x;
        errDy = -data_drone.state.vel.y;
        errDz = -data_drone.state.vel.z;

        errvDx = d_vel_err_x.new_sample (err_velx_filtered);
        errvDy = d_vel_err_y.new_sample (err_vely_filtered);
        errvDz = d_vel_err_z.new_sample (err_velz_filtered);
    }

    cv::Point3f vel_err_p = {err_velx_filtered, err_vely_filtered, err_velz_filtered};
    cv::Point3f pos_err_d = {errDx, errDy, errDz};
    cv::Point3f vel_err_d = {errvDx, errvDy, errvDz};

    if (ki_pos.x>0) {
        pos_err_i.x += (err_x_filtered - setpoint_pos.x + pos_modelx.current_output());
        if (!enable_thrust_calibration)
            pos_err_i.y += (err_y_filtered - setpoint_pos.y + pos_modely.current_output());
        pos_err_i.z += (err_z_filtered - setpoint_pos.z + pos_modelz.current_output());
    } else {
        if(pos_err_i.y!=0)
            remember_last_integrated_y_err = pos_err_i.y;
        pos_err_i = {0};
    }

    if (enable_thrust_calibration) {
        if(remember_last_integrated_y_err!=0) {
            drone_calibration.thrust -= ki_thrust_hover*0.001f*remember_last_integrated_y_err;
            remember_last_integrated_y_err = 0;
        } else {
            drone_calibration.thrust -= 0.1f * ((err_y_filtered - setpoint_pos.y + pos_modely.current_output()));
            pos_err_i.y = 0;
        }

    }
    return std::tuple(pos_err_p, pos_err_d, vel_err_p, vel_err_d);
}

void DroneController::correct_yaw(float deviation_angle) {
    int target_theta_velocity = 0;

    if (deviation_angle < - _dtrk->min_yaw_deviation)
        target_theta_velocity = -3;
    else if (deviation_angle > _dtrk->min_yaw_deviation)
        target_theta_velocity = 3;

    auto_yaw = RC_MIDDLE + target_theta_velocity;
}

void DroneController::blink(double time) {
    static double last_blink_time = time;
    static bool blink_state;

    if (static_cast<float>(time-last_blink_time)>dparams.blink_period) {
        if (blink_state)
            blink_state = false;
        else
            blink_state = true;
        last_blink_time = time;
    }
    LED(blink_state);
}

void DroneController::blink_motors(double time) {
    int itime = round(time)*3;
    auto_roll = RC_BOUND_MIN;
    auto_pitch = RC_BOUND_MIN;
    auto_throttle = RC_BOUND_MIN;
    auto_yaw = RC_BOUND_MIN;

    int spin_value = RC_BOUND_MIN+ 100;
    if (itime % 4 == 0) {
        auto_roll = spin_value;
    } else if (itime % 4 == 1) {
        auto_pitch = spin_value;
    } else if (itime % 4 == 2) {
        auto_yaw = spin_value;
    } else if (itime % 4 == 3) {
        auto_throttle = spin_value;
    }
}

void DroneController::read_joystick(void) {
    while (joystick.sample(&event))
    {
        if (event.isAxis()) {
            if (pparams.joystick == rc_devo) {
                switch ( event.number ) {
                case 0: // roll
                    joy_roll = (event.value >> 5) + RC_MIDDLE;
                    break;
                case 1: // pitch
                    joy_pitch =  (event.value >> 5) + RC_MIDDLE;
                    break;
                case 2: //throttle
                    joy_throttle =  (event.value >> 5) + RC_MIDDLE - 100;
                    break;
                case 5:  //switch
                    if (event.value>0) {
                        _joy_arm_switch = bf_armed;
                    } else {
                        _joy_arm_switch = bf_disarmed;
                    }
                    break;
                case 3: //dial
                    joyDial = event.value; // goes between +/-32768
                    scaledjoydial = joyDial+32767;
                    scaledjoydial = (scaledjoydial / 65536)*100+35;
                    break;
                case 4: //yaw
                    joy_yaw =  (event.value >> 5) + RC_MIDDLE;
                    break;
                default:
                    std::cout << "Unkown joystick event: " << std::to_string(event.number) << ". Value: " << std::to_string(event.value) << std::endl;
                    break;
                }
            }  else if (pparams.joystick == rc_xlite) {
                switch ( event.number ) {
                case 0: // roll
                    joy_roll = static_cast<uint16_t>((event.value >> 5)*0.8 + RC_MIDDLE);
                    break;
                case 1: // pitch
                    joy_pitch = static_cast<uint16_t>((event.value >> 5)*0.8 + RC_MIDDLE);
                    break;
                case 2: //throttle
                    joy_throttle =  static_cast<uint16_t>((event.value >> 5)*0.8 + RC_MIDDLE);
                    break;
                case 3: //yaw
                    joy_yaw = static_cast<uint16_t>((event.value >> 5)*0.4 + RC_MIDDLE);
                    break;
                case 4: //arm switch (two way)
                    if (event.value>0) {
                        _joy_arm_switch = bf_armed;
                    } else {
                        _joy_arm_switch = bf_disarmed;
                    }
                    break;
                case 5: //mode switch (3 way)
                    if (event.value<-16384 ) {
                        _joy_mode_switch = jmsm_manual;
                    } else if (event.value>16384) {
                        _joy_mode_switch = jmsm_hunt;
                    } else {
                        _joy_mode_switch = jmsm_waypoint;
                    }
                    break;
                case 6: //switch (3 way)
                    _joy_takeoff_switch = event.value>0; //TODO: tmp fix for weird x-lite channel problem https://github.com/pats-drones/pats/issues/93
                    break;
                case 7: //switch (2 way)
                    _joy_takeoff_switch = event.value>0;
                    break;
                default:
                    //this joystick seems to have 16 extra buttons... weird whatever
                    //                std::cout << "Unkown joystick event: " << std::to_string(event.number) << ". Value: " << std::to_string(event.value) << std::endl;
                    break;
                }
            }
        }
    }
}

void DroneController::process_joystick() {

    //check the demo flightplan trigger, which overrides the joystick_takeoff_switch (even though there was no joystick connected)
    if (log_replay_mode && pparams.joystick == rc_none && _joy_takeoff_switch == true)
        _manual_override_take_off_now = true;
    else
        _manual_override_take_off_now = false;

    if (pparams.joystick == rc_none || _joy_state == js_none )
        return;

    // prevent accidental take offs at start up
    if (_joy_state == js_checking) {
        if (_joy_arm_switch == bf_disarmed &&
                ((joy_throttle <= RC_MIN_THRESH && !dparams.mode3d) || (abs(joy_throttle- RC_MIDDLE) < 50 && dparams.mode3d) )&&
                !_joy_takeoff_switch) {
            _flight_mode = fm_disarmed;
            _joy_state = js_disarmed;
        } else {
            _flight_mode = fm_joystick_check;
        }
    } else {

        if (_flight_mode == fm_inactive) {
            _manual_override_take_off_now = _joy_takeoff_switch;
        }
        if (_joy_arm_switch == bf_disarmed) {
            if (!dparams.mode3d)
                _rc->arm(bf_disarmed);
            _joy_takeoff_switch = false;
            _manual_override_take_off_now = false;
            _joy_state = js_disarmed;
            if (_flight_mode != fm_blink)
                _flight_mode = fm_disarmed;
        } else {
            if (!dparams.mode3d)
                _rc->arm(bf_armed);
            if (_joy_mode_switch == jmsm_manual) {
                _joy_state = js_manual;
                _flight_mode = fm_manual;
            } else if (_joy_mode_switch == jmsm_waypoint)
                _joy_state = js_waypoint;
            else if (_joy_mode_switch == jmsm_hunt)
                _joy_state = js_hunt;
        }
    }
}

void DroneController::load_calibration(std::string replay_dir) {
    if (replay_dir!="") {
        drone_calib_rfn = replay_dir + '/' + drone_calib_fn;
        drone_calib_wfn = "./logging/replay/" + drone_calib_fn;
    }

    if (file_exist(drone_calib_rfn)) {
        xmls::DroneCalibration tmp;
        try {
            tmp.deserialize(drone_calib_rfn);
        } catch(MyExit const &err) {
            std::cout << "Error, corrupted drone calibration xml:" << err.msg << std::endl;
        }
        if (tmp.drone_id != _rc->drone_id()) {
            std::cout << "Drone ID mismatch. Using default values, but calibration is needed!" << std::endl;
            drone_calibration.thrust = dparams.default_thrust;
            drone_calibration.drone_id = _rc->drone_id();
        } else {
            drone_calibration.deserialize(drone_calib_rfn);
        }
    } else {
        std::cout << "Drone calibration missing. Using default values, but calibration is needed!" << std::endl;
        drone_calibration.thrust = dparams.default_thrust;
        drone_calibration.drone_id = _rc->drone_id();
    }
    drone_calibration.serialize(drone_calib_wfn);
}

void DroneController::save_calibration() {
    if(!log_replay_mode) {
        drone_calibration.serialize("../../xml/" + drone_calib_fn);
    }
}

void DroneController::load_control_parameters() {
    control_parameters_rfn = "../../xml/" + dparams.control + ".xml";
    std::cout << "Reading settings from: " << control_parameters_rfn << std::endl;
    ControlParameters params;
    if (file_exist(control_parameters_rfn)) {
        std::ifstream infile(control_parameters_rfn);
        std::string xmlData((std::istreambuf_iterator<char>(infile)),
                            std::istreambuf_iterator<char>());

        if (!xmls::Serializable::fromXML(xmlData, &params))
            throw MyExit("Cannot read: " + control_parameters_rfn);

        ControlParameters tmp;
        auto v1 = params.getVersion();
        auto v2 = tmp.getVersion();
        if (v1 != v2) {
            throw MyExit("XML version difference detected from " + control_parameters_rfn);
        }
        infile.close();
    } else {
        throw MyExit("File not found: " + control_parameters_rfn);
    }

    kp_pos_roll= params.kp_pos_roll.value();
    kp_pos_pitch= params.kp_pos_pitch.value();
    kp_pos_throttle= params.kp_pos_throttle.value();
    ki_pos_roll= params.ki_pos_roll.value();
    ki_pos_pitch= params.ki_pos_pitch.value();
    ki_thrust= params.ki_thrust.value();
    kd_pos_roll= params.kd_pos_roll.value();
    kd_pos_pitch= params.kd_pos_pitch.value();
    kd_pos_throttle= params.kd_pos_throttle.value();
    kp_pos_roll_hover= params.kp_pos_roll_hover.value();
    kp_pos_pitch_hover = params.kp_pos_pitch_hover.value();
    kp_pos_throttle_hover= params.kp_pos_throttle_hover.value();
    ki_pos_roll_hover = params.ki_pos_roll_hover.value();
    ki_pos_pitch_hover = params.ki_pos_pitch_hover.value();
    ki_thrust_hover = params.ki_thrust_hover.value();
    kd_pos_roll_hover = params.kd_pos_roll_hover.value();
    kd_pos_pitch_hover = params.kd_pos_pitch_hover.value();
    kd_pos_throttle_hover = params.kd_pos_throttle_hover.value();

    kp_v_roll= params.kp_v_roll.value();
    kp_v_pitch= params.kp_v_pitch.value();
    kp_v_throttle= params.kp_v_throttle.value();
    kd_v_roll= params.kd_v_roll.value();
    kd_v_pitch= params.kd_v_pitch.value();
    kd_v_throttle= params.kd_v_throttle.value();
}

void DroneController::serialize_settings() {
    ControlParameters params;
    params.kp_pos_roll = kp_pos_roll;
    params.kp_pos_pitch = kp_pos_pitch;
    params.kp_pos_throttle = kp_pos_throttle;
    params.ki_pos_roll = ki_pos_roll;
    params.ki_pos_pitch = ki_pos_pitch;
    params.ki_thrust = ki_thrust;
    params.kd_pos_roll = kd_pos_roll;
    params.kd_pos_pitch = kd_pos_pitch;
    params.kd_pos_throttle = kd_pos_throttle;
    params.kp_pos_roll_hover = kp_pos_roll_hover;
    params.kp_pos_pitch_hover = kp_pos_pitch_hover;
    params.kp_pos_throttle_hover = kp_pos_throttle_hover;
    params.ki_pos_roll_hover = ki_pos_roll_hover;
    params.ki_pos_pitch_hover = ki_pos_pitch_hover;
    params.ki_thrust_hover = ki_thrust_hover;
    params.kd_pos_roll_hover = kd_pos_roll_hover;
    params.kd_pos_pitch_hover = kd_pos_pitch_hover;
    params.kd_pos_throttle_hover = kd_pos_throttle_hover;
    params.kp_v_roll = kp_v_roll;
    params.kp_v_pitch = kp_v_pitch;
    params.kp_v_throttle = kp_v_throttle;
    params.kd_v_roll = kd_v_roll;
    params.kd_v_pitch = kd_v_pitch;
    params.kd_v_throttle = kd_v_throttle;

    std::string xmlData = params.toXML();
    std::ofstream outfile = std::ofstream (control_parameters_rfn);
    outfile << xmlData ;
    outfile.close();
}

void DroneController::close() {
    if (initialized) {
        std::cout << "Closing controller." << std::endl;
        if (pparams.control_tuning)
            serialize_settings();
        initialized = false;
    }
}
