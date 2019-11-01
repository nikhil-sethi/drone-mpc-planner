#include "dronecontroller.h"
#include "defines.h"

using namespace cv;

// Create an instance of Joystick
Joystick joystick("/dev/input/js0");
JoystickEvent event;

bool DroneController::joystick_ready(){
    return joystick.isFound();
}

void DroneController::init(std::ofstream *logger,bool fromfile, MultiModule * rc, DroneTracker *dtrk, CameraVolume *camvol) {
    _rc = rc;
    _dtrk = dtrk;
    _logger = logger;
    _fromfile = fromfile;
    _camvol = camvol;
    control_history_max_size = pparams.fps;
    (*_logger) << "valid; posErrX; posErrY; posErrZ; velX; velY; velZ; accX; accY; accZ; hoverthrottle; autoThrottle; autoRoll; autoPitch; autoYaw; joyThrottle; joyRoll; joyPitch; joyYaw; joyArmSwitch; joyModeSwitch; joyTakeoffSwitch; dt; velx_sp; vely_sp; velz_sp;";
    std::cout << "Initialising control." << std::endl;

    pid_roll_smoother.init(6);
    pid_pitch_smoother.init(12);
    pid_throttle_smoother.init(2);

    settings_file = "../../xml/" + dparams.control + ".xml";

    // Load saved control paremeters
    deserialize_settings();

    if (pparams.control_tuning) {
        std::cout << "Creating control tuning window." << std::endl;
        // create GUI to set control parameters
        namedWindow("Control", WINDOW_NORMAL);
//        createTrackbar("Throttle Pos", "Control", &gain_throttle_pos, 3000);
//        createTrackbar("Throttle Vel", "Control", &gain_throttle_vel, 3000);
//        createTrackbar("Throttle Acc", "Control", &gain_throttle_acc, 3000);
//        createTrackbar("Throttle I", "Control", &gain_throttle_i, 100);
//        createTrackbar("Throttle D", "Control", &gain_throttle_d, 3000);
//        createTrackbar("Roll Pos", "Control", &gain_roll_pos, 3000);
//        createTrackbar("Roll Vel", "Control", &gain_roll_vel, 2000);
//        createTrackbar("Roll Acc", "Control", &gain_roll_acc, 100);
//        createTrackbar("Roll I", "Control", &gain_roll_i, 100);
//        createTrackbar("Roll D", "Control", &gain_roll_d, 3000);
//        createTrackbar("Pitch Pos", "Control", &gain_pitch_pos, 3000);
//        createTrackbar("Pitch Vel", "Control", &gain_pitch_vel, 2000);
//        createTrackbar("Pitch Acc", "Control", &gain_pitch_acc, 100);
//        createTrackbar("Pitch I", "Control", &gain_pitch_i, 100);
//        createTrackbar("Pitch D", "Control", &gain_pitch_d, 3000);
        createTrackbar("pid_max_angle_scaler", "Control", &pid_max_angle_scaler, 16);
        createTrackbar("Pposx", "Control", &p_pos_roll, 3000);
        createTrackbar("Pposz", "Control", &p_pos_pitch, 3000);
        createTrackbar("Pposy", "Control", &p_pos_throttle, 3000);
        createTrackbar("Iposx", "Control", &i_pos_roll, 1000);
        createTrackbar("Iposz", "Control", &i_pos_pitch, 1000);
        createTrackbar("Iposy", "Control", &i_pos_throttle, 1000);
        createTrackbar("Dposx", "Control", &d_pos_roll, 1000);
        createTrackbar("Dposz", "Control", &d_pos_pitch, 1000);
        createTrackbar("Dposy", "Control", &d_pos_throttle, 1000);
    }

    hoverthrottle = dparams.initial_hover_throttle;
    filerPosErrX.init(1.f/pparams.fps, 1, 1.f/pparams.fps);
    filerPosErrY.init(1.f/pparams.fps, 1, 1.f/pparams.fps);
    filerPosErrZ.init(1.f/pparams.fps, 1, 1.f/pparams.fps);
    dErrX.init (1.f/pparams.fps);
    dErrY.init (1.f/pparams.fps);
    dErrZ.init (1.f/pparams.fps);
    initialized = true;
}

int bound_throttle(int v) {
    return std::clamp(v,JOY_BOUND_MIN,JOY_BOUND_MAX);
}

int bound_joystick_value(int v) {
    return std::clamp(v,JOY_BOUND_MIN,JOY_BOUND_MAX);
}

void DroneController::control(track_data data_drone, track_data data_target, cv::Point3f setpoint_pos, cv::Point3f setpoint_vel, cv::Point3f setpoint_acc, double time) {

    if (!_fromfile && pparams.joystick != rc_none)
        readJoystick();
    if (!pparams.insect_logging_mode)
        process_joystick();

    if (recovery_mode && _joy_state!=js_waypoint)
        setpoint_pos = recovery_pos;

    if(_joy_state==js_waypoint || recovery_mode){
        data_target.state.pos = setpoint_pos;
        data_target.state.vel = {0};
    }

    // This is usefull as long blind reburn is not working
    if( (!data_drone.pos_valid && time > start_takeoff_burn_time+1 && start_takeoff_burn_time)
        && (_flight_mode!=fm_inactive && _flight_mode!=fm_disarmed)){
        _flight_mode = fm_abort_flight;
        flight_submode_name = "fm_abort_flight_tracking_lost";
    }

    int throttle,roll,pitch,yaw;
    bool joy_control = false;
    switch(_flight_mode) {
    case fm_manual: {
        throttle = joy_throttle;
        _rc->arm(_joy_arm_switch);
        roll = joy_roll;
        pitch = joy_pitch;
        yaw = joy_yaw;
        joy_control = true;
        break;
    } case fm_spinup: {
        _rc->arm(true);
        start_takeoff_burn_time = 0;
        if (spin_up_start_time == 0)
            spin_up_start_time = time;
        auto_roll = JOY_MIDDLE;
        auto_pitch = JOY_MIDDLE;
        auto_throttle = spinup_throttle();
        break;
    } case fm_start_takeoff: {
        take_off_start_time = time;
        _flight_mode = fm_take_off_aim;
        std::cout << "Take off aiming" << std::endl;
        _burn_direction_for_thrust_approx = {0};

        auto_throttle = spinup_throttle();
        auto_roll = JOY_MIDDLE;
        auto_pitch = JOY_MIDDLE;

        if (dparams.mode3d) {
            _rc->arm(true);
            break;
        }
        [[fallthrough]];
    } case fm_take_off_aim: {
        state_data state_drone_takeoff = data_drone.state;
        state_drone_takeoff.pos = _dtrk->drone_startup_location() + cv::Point3f(0,lift_off_dist_take_off_aim,0);
        state_drone_takeoff.vel = {0};

        float remaing_spinup_time = dparams.full_bat_and_throttle_spinup_duration - aim_duration - time_spent_spinning_up(take_off_start_time);
        if (remaing_spinup_time< 0)
            remaing_spinup_time = 0;

        bool burn_limit_hack = true;
        float remaining_aim_duration = remaing_spinup_time+aim_duration  - static_cast<float>(time - take_off_start_time);
        if (remaining_aim_duration <= aim_duration) {
            cv::Point3f burn_direction;
            std::tie (auto_roll, auto_pitch,auto_burn_duration,burn_direction) = calc_directional_burn(state_drone_takeoff,data_target.state,0);
            if (burn_limit_hack)
                auto_burn_duration = take_off_burn_duration; //TODO: make this number dynamic such that we have just enough time to do a second directional burn?
            aim_direction_history.push_back(burn_direction);
            auto_throttle = initial_hover_throttle_guess();

            _burn_direction_for_thrust_approx = burn_direction; // to be used later to approx effective thrust
            std::vector<state_data> trajectory = predict_trajectory(auto_burn_duration, 0, burn_direction, state_drone_takeoff);
            if (_fromfile) // todo: tmp solution, call from visualizer instead if this viz remains to be needed
                draw_viz(state_drone_takeoff,data_target.state,time,burn_direction,auto_burn_duration,remaining_aim_duration,trajectory);
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
            cv::Point3f predicted_target_pos = data_target.pos() + auto_burn_duration*data_target.vel();

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
        auto_throttle = JOY_BOUND_MAX;
        if (static_cast<float>(time - start_takeoff_burn_time) >  auto_burn_duration)
            _flight_mode = fm_max_burn_spin_down;
        break;
    } case fm_max_burn_spin_down: {
        auto_throttle = initial_hover_throttle_guess();
        if (static_cast<float>(time - start_takeoff_burn_time) > auto_burn_duration + effective_burn_spin_down_duration)
            _flight_mode = fm_1g;
        break;
    }  case fm_1g: {
        auto_roll = JOY_MIDDLE;
        auto_pitch = JOY_MIDDLE;

        // Wait until velocity of drone after take-off is constant, then estimate this velocity using sufficient samples
        if (static_cast<float>(time - start_takeoff_burn_time) > auto_burn_duration + effective_burn_spin_down_duration + transmission_delay_duration - 1.f / pparams.fps){
            if(_joy_state == js_waypoint)
                _flight_mode = fm_flying_pid_init;
            else{
                _flight_mode = fm_interception_aim_start;

            }
        }
        break;
    }  case fm_interception_aim_start: {
        std::cout << "Aiming" << std::endl;
        interception_start_time = time;
        approx_effective_thrust(data_drone,_burn_direction_for_thrust_approx,auto_burn_duration,static_cast<float>(time-start_takeoff_burn_time));
        _burn_direction_for_thrust_approx = {0};
        std::cout << "Estimated acc: " << thrust << std::endl;
        first_directional_burn = true;
        _flight_mode =   fm_interception_aim;
        [[fallthrough]];
    }   case fm_interception_aim: {
        cv::Point3f burn_direction;
        state_data state_drone_better = data_drone.state;
        state_drone_better.vel = drone_vel_after_takeoff;

        float remaining_aim_duration = aim_duration - static_cast<float>(time - interception_start_time);
        data_target.state.vel = {0}; // aim to the current target position

        if (remaining_aim_duration<0)
            remaining_aim_duration = 0;
        std::vector<state_data> traj;
        if (!recovery_mode)
            std::tie (auto_roll, auto_pitch, auto_burn_duration,burn_direction,traj) = calc_burn(state_drone_better,data_target.state,remaining_aim_duration);
        else
            std::tie (auto_roll, auto_pitch, auto_burn_duration,burn_direction,traj) = calc_burn(data_drone.state, data_target.state,remaining_aim_duration);

        if (_fromfile) // todo: tmp solution, call from visualizer instead if this viz remains to be needed
            draw_viz(state_drone_better,data_target.state,time,burn_direction,auto_burn_duration,remaining_aim_duration,traj);

        auto_throttle = initial_hover_throttle_guess();

        if (recovery_mode && remaining_aim_duration <= 0) {
            _flight_mode = fm_interception_burn_start;
        } else if (recovery_mode) { // do nothing
        } else if (!trajectory_in_view(traj,CameraVolume::relaxed) || auto_burn_duration > 1.1f || auto_burn_duration == 0.0f){
            _flight_mode = fm_flying_pid_init;
        } else {
            std::vector<state_data> traj_back;
            cv::Point3f burn_direction_back;
            float auto_burn_duration_back;
            state_data target_back = traj.front();
            target_back.vel = {0};
            target_back.acc = {0};
            std::tie (std::ignore, std::ignore,auto_burn_duration_back,burn_direction_back,traj_back) = calc_burn(traj.back(),target_back,aim_duration);
            //            draw_viz(traj_back.back(),target_back,time,burn_direction_back,auto_burn_duration_back,aim_duration,traj_back);
            //            for (auto t : traj_back)
            //                viz_drone_trajectory.push_back(t);
            if (!trajectory_in_view(traj_back,CameraVolume::relaxed)){
                _flight_mode = fm_flying_pid_init;
            } else if (remaining_aim_duration < 0.01f)
                _flight_mode = fm_interception_burn_start;
        }
        break;
    } case fm_interception_burn_start: {
        _flight_mode = fm_interception_burn;
        auto_throttle = JOY_BOUND_MAX;
        first_directional_burn = false;
        if (!recovery_mode)
            recovery_pos = data_drone.pos();
        std::cout << "Burning: " << auto_burn_duration << std::endl;
        [[fallthrough]];
    } case fm_interception_burn: {
        if (static_cast<float>(time - interception_start_time) > aim_duration + auto_burn_duration){
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
        //recovery_mode = !recovery_mode; // #131
        _flight_mode = fm_interception_aim;
        std::cout << "Re-aiming" << std::endl;
        _burn_direction_for_thrust_approx = {0};
        break;
    } case fm_flying_pid_init: {
        rollErrI = 0;
        pitchErrI = 0;
        throttleErrI = 0;
        _flight_mode = fm_flying_pid;

        dErrX.preset(setpoint_pos.x - data_drone.state.pos.x);
        dErrY.preset(setpoint_pos.y - data_drone.state.pos.y);
        dErrZ.preset(setpoint_pos.z - data_drone.state.pos.z);
        filerPosErrX.reset (setpoint_pos.x - data_drone.state.pos.x);
        filerPosErrY.reset (setpoint_pos.y - data_drone.state.pos.y);
        filerPosErrZ.reset (setpoint_pos.z - data_drone.state.pos.z);

        [[fallthrough]];
    } case fm_flying_pid: {
        //check whether the braking distance is large enough to stay in the frame
        // if not change the setpoint for the controller
//        float remaining_breaking_distance = _camvol->calc_distance_to_borders (data_drone);
//        remaining_breaking_distance -= static_cast<float>(norm(data_drone.vel()))*(1.f/60.f); // Also consider the time till the next check
//        float required_breaking_time = static_cast<float>(norm(data_drone.vel() )) / thrust;

//        float required_breaking_distance = static_cast<float>(.5L*thrust*pow(required_breaking_time, 2));
//        if(remaining_breaking_distance<=required_breaking_distance && _joy_state!=js_waypoint ){
//            setpoint_pos = data_drone.pos ();
//            setpoint_vel = data_drone.vel ();
//            setpoint_acc = {0};
//            flight_submode_name = "fm_pid_keep_in_volume";
//        } else {
//            flight_submode_name = "";
//        }

        calc_pid_error(data_drone,setpoint_pos,setpoint_vel,setpoint_acc,time);
        control_pid(data_drone);

        //TODO: streamline. Below replaces old control_pid stuff
        control_modelBased(data_drone, setpoint_pos);

        //check if we can go back to burning:
        if (data_drone.pos_valid && data_drone.vel_valid && _joy_state!=js_waypoint){
            std::vector<state_data> traj;
            cv::Point3f burn_direction;
            float burn_duration;
            std::tie (std::ignore, std::ignore,burn_duration,burn_direction,traj) = calc_burn(data_drone.state,data_target.state,aim_duration);

            cout << "burn_duration: " << burn_duration << endl;

            if (trajectory_in_view(traj,CameraVolume::strict) && burn_duration < 0.2f && burn_duration > 0.01f) {
                std::vector<state_data> traj_back;
                std::tie (std::ignore, std::ignore,std::ignore,std::ignore,traj_back) = calc_burn(traj.back(),traj.front(),aim_duration);
                if (trajectory_in_view(traj_back,CameraVolume::strict) && norm(data_target.state.vel)>0.1){ // norm vel is hack to check if not waypoint
                    _flight_mode = fm_retry_aim_start;
                    //                    recovery_mode = true; // will be set to false in retry_aim TMP disabled because #131
                    recovery_pos = data_drone.pos();
                }
            }
        }
        break;
    } case fm_landing_start: {
        landing_decent_yoffset = 0;
        landing_setpoint_height = setpoint_pos.y;
        _flight_mode = fm_landing;
        [[fallthrough]];
    } case fm_landing: {
        landing_decent_yoffset += landing_decent_rate;
        setpoint_pos.y += landing_decent_yoffset;

        landing_setpoint_height = setpoint_pos.y; //only for status feedback for dronenavigation

        calc_pid_error(data_drone,setpoint_pos,setpoint_vel,setpoint_acc,time);
        control_pid(data_drone);

        //TODO: streamline. Below replaces old control_pid stuff
        control_modelBased(data_drone, setpoint_pos);

        break;
    } case fm_disarmed: {
        auto_roll = JOY_MIDDLE;
        auto_pitch = JOY_MIDDLE;
        if (dparams.mode3d)
            auto_throttle = JOY_MIDDLE;
        else
            auto_throttle = JOY_BOUND_MIN;
        _rc->arm(false);
        if (time > 5 && pparams.joystick == rc_none)
            _flight_mode = fm_inactive;
        break;
    } case fm_inactive: {
        auto_roll = JOY_MIDDLE;
        auto_pitch = JOY_MIDDLE;
        if (dparams.mode3d)
            auto_throttle = JOY_MIDDLE;
        else
            auto_throttle = JOY_BOUND_MIN;
        if (pparams.joystick == rc_none && !dparams.mode3d)
            _rc->arm(true);
        else if (dparams.mode3d)
            _rc->arm(false);
        break;
    } case fm_abort_flight: {
        if (dparams.mode3d)
            auto_throttle = JOY_MIDDLE;
        else
            auto_throttle = JOY_BOUND_MIN;
        auto_roll = JOY_MIDDLE;
        auto_pitch = JOY_MIDDLE;
        break;
    } case fm_joystick_check: {
        if (pparams.joystick == rc_none) {
            _flight_mode = fm_disarmed;
            _joy_state = js_none;
            break;
        }
        _joy_state = js_checking;
        if (dparams.mode3d)
            auto_throttle = JOY_MIDDLE;
        else
            auto_throttle = JOY_BOUND_MIN;
        auto_roll = JOY_MIDDLE;
        auto_pitch = JOY_MIDDLE;
        _rc->arm(false);
        break;
    }
    }

    if (!joy_control){
        pitch =  auto_pitch;
        roll = auto_roll;
        throttle = auto_throttle;
    }

    yaw = joy_yaw; // tmp until auto yaw control is fixed #10

    throttle = bound_throttle(throttle);
    roll = bound_joystick_value(roll);
    pitch = bound_joystick_value(pitch);
    yaw = bound_joystick_value(yaw);

    //std::cout << time <<  " rpt: " << roll << ", " << pitch << ", " << throttle << std::endl;
    if (!_fromfile) {
        _rc->queue_commands(throttle,roll,pitch,yaw);
    }

    control_data c(Throttle(),Roll(),Pitch(),time);
    control_history.push_back(c);
    while (control_history.size() > control_history_max_size)
        control_history.erase(control_history.begin());

    (*_logger) << static_cast<int>(data_drone.pos_valid)  << "; " <<
        posErrX << "; " <<
        posErrY  << "; " <<
        posErrZ << "; " <<
        setpoint_pos.x << "; " <<
        setpoint_pos.y  << "; " <<
        setpoint_pos.z << "; " <<
        accx_sp << "; " <<
        accy_sp  << "; " <<
        accx_sp << "; " <<
        hoverthrottle << "; " <<
        auto_throttle << "; " <<
        auto_roll << "; " <<
        auto_pitch << "; " <<
        auto_yaw <<  "; " <<
        joy_throttle <<  "; " <<
        joy_roll <<  "; " <<
        joy_pitch <<  "; " <<
        joy_yaw << "; " <<
        static_cast<int>(_joy_arm_switch) << "; " <<
        static_cast<int>(_joy_mode_switch) << "; " <<
        static_cast<int>(_joy_takeoff_switch) << "; " <<
        data_drone.dt << "; " <<
        velx_sp << "; " <<
        vely_sp << "; " <<
        velz_sp << "; ";
}

std::tuple<int, int, float, Point3f, std::vector<state_data> > DroneController::calc_burn(state_data state_drone,state_data state_target,float remaining_aim_duration) {
    remaining_aim_duration +=  transmission_delay_duration + 1.f / pparams.fps;
    auto [auto_roll_burn, auto_pitch_burn, burn_duration,burn_direction] = calc_directional_burn(state_drone,state_target,remaining_aim_duration);

    //if (first_directional_burn && burn_duration > 0.0f)
    //    burn_duration = 0.075f;

    auto traj = predict_trajectory(burn_duration, remaining_aim_duration, burn_direction, state_drone);
    return std::make_tuple(auto_roll_burn, auto_pitch_burn, burn_duration,burn_direction,traj);
}

std::tuple<int,int,float,cv::Point3f> DroneController::calc_directional_burn(state_data state_drone, state_data state_target, float remaining_aim_duration) {

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
        if (burn_duration< 0){
            burn_duration = 1.0;
            conversion_speed *=0.5f;
        } else if (burn_duration > 1) {
            burn_duration = 0.2;
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

    auto [roll_deg, pitch_deg] = acc_to_deg(burn_direction);

    int auto_roll_burn =  ((roll_deg/max_bank_angle+1) / 2.f) * JOY_BOUND_RANGE + JOY_BOUND_MIN; // convert to RC commands range
    int auto_pitch_burn = ((pitch_deg/max_bank_angle+1) / 2.f) * JOY_BOUND_RANGE + JOY_BOUND_MIN;

    return std::make_tuple(auto_roll_burn,auto_pitch_burn,burn_duration,burn_direction);
}

//calculate the predicted drone location after a burn
std::tuple<cv::Point3f, cv::Point3f, cv::Point3f> DroneController::predict_drone_after_burn(
    state_data state_drone, cv::Point3f burn_direction,float remaining_aim_duration, float burn_duration) {

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

    cv::Point3f burn_accelleration = burn_direction * thrust;

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
    state_data state_drone, state_data state_target,double time, cv::Point3f burn_direction,
    float burn_duration, float remaining_aim_duration, std::vector<state_data> traj) {

    double viz_time_after_aim = time + static_cast<double>(remaining_aim_duration + transmission_delay_duration + 1.f/pparams.fps);
    viz_target_pos_after_burn = state_target.pos + (remaining_aim_duration + burn_duration) * state_target.vel; // TODO: transmission_delay_duration + 1.f/pparams.fps ???
    if (_flight_mode == fm_take_off_aim || _flight_mode == fm_max_burn) {
        cv::Point3f integrated_pos, integrated_vel;
        cv::Point3f burn_accelleration;
        std::tie(integrated_pos, integrated_vel,burn_accelleration) = predict_drone_after_burn(
            state_drone, burn_direction,0,burn_duration);

        //        std::tie(viz_drone_pos_after_burn, std::ignore) = predict_drone_state_after_spindown(integrated_pos, integrated_vel,burn_accelleration);
        //        viz_time_after_burn = viz_time_after_aim + static_cast<double>(burn_duration + effective_burn_spin_down_duration);

        viz_drone_pos_after_burn =  traj.back().pos;
        viz_time_after_burn = viz_time_after_aim + static_cast<double>(burn_duration );

        viz_drone_trajectory = predict_trajectory(burn_duration, 0, burn_direction, state_drone);

    } else {

        viz_time_after_burn = viz_time_after_aim + static_cast<double>(burn_duration);
        viz_drone_trajectory = traj;
        viz_drone_pos_after_burn =  traj.back().pos;
        //            float TMP_thrust = thrust;
        //            float TMP_effective_burn_spin_up_duration = effective_burn_spin_up_duration;
        //            for (float j = 0; j < 0.55f; j+=0.05f) {
        //                effective_burn_spin_up_duration = j;
        //                for (int i = 10; i < 100; i+=30) {
        //                    thrust = i;
        //                    burn_accelleration_max = burn_direction * thrust;
        //        for (float f=0;f < burn_duration;f+=1.f/pparams.fps) {
        //            std::tie (integrated_pos, integrated_vel,std::ignore) = predict_drone_state_after_burn(
        //                drone_state_after_aim, burn_direction, burn_accelleration_max, remaining_aim_duration,f);
        //                viz_drone_trajectory.push_back(integrated_pos);
        //        }
        //                }
        //            }
        //            thrust = TMP_thrust;
        //            effective_burn_spin_up_duration = TMP_effective_burn_spin_up_duration;
    }

}

std::vector<state_data> DroneController::predict_trajectory(float burn_duration, float remaining_aim_duration, cv::Point3f burn_direction, state_data state_drone) {
    std::vector<state_data> traj;
    cv::Point3f integrated_pos, integrated_vel;
    for (float f=0;f <= burn_duration;f+=1.f/pparams.fps) {
        std::tie (integrated_pos, integrated_vel,std::ignore) = predict_drone_after_burn(
            state_drone, burn_direction,remaining_aim_duration,f);
        state_data state;
        state.pos = integrated_pos;
        state.vel = integrated_vel;
        traj.push_back(state);
    }
    return traj;
}

bool DroneController::trajectory_in_view(std::vector<state_data> traj, CameraVolume::volume_check_mode c) {
    for (auto state : traj) {
        if (!_camvol->in_view(state.pos,c))
            return false;
    }
    return true;
}

std::tuple<float,float> DroneController::acc_to_deg(cv::Point3f acc) {
    //calculate roll/pitch commands
    float norm_burn_vector_XZ = sqrt(powf(acc.x,2)+powf(acc.z,2));
    float rotation_angle = atan2f(norm_burn_vector_XZ,(acc.y))*rad2deg;
    float roll = -acc.x/norm_burn_vector_XZ*rotation_angle;
    float pitch = -acc.z/norm_burn_vector_XZ*rotation_angle;

    // limit roll/pitch commands
    roll = std::clamp(roll,-max_bank_angle,max_bank_angle);
    pitch = std::clamp(pitch,-max_bank_angle,max_bank_angle);

    return std::make_tuple(roll,pitch);
}

std::tuple<cv::Point3f, cv::Point3f> DroneController::predict_drone_state_after_spindown(cv::Point3f integrated_pos, cv::Point3f integrated_vel, cv::Point3f burn_accelleration) {

    // Phase: spin down
    integrated_pos += 0.25f*burn_accelleration*powf(effective_burn_spin_down_duration,2) + integrated_vel * effective_burn_spin_down_duration;
    integrated_vel += burn_accelleration * effective_burn_spin_down_duration;


    //    // Phase: 1G
    //    if (remaining_1g_duration > 0)
    //        integrated_pos += integrated_vel * remaining_1g_duration;

    return std::make_tuple(integrated_pos, integrated_vel);
}

//considering a take off order from just after the aiming phase of: spinning up, burning, spin down, 1g, find the max drone acceleration that best desccribes the current position given dt
void DroneController::approx_effective_thrust(track_data data_drone, cv::Point3f burn_direction, float burn_duration, float dt_burn) {

    cv::Point3f pos_after_aim =  _dtrk->drone_startup_location() + cv::Point3f(0,lift_off_dist_take_off_aim,0);
    float partial_effective_burn_spin_up_duration = effective_burn_spin_down_duration; // if the burn duration is long enough this is equal otherwise it may be shortened
    cv::Point3f acc = burn_direction * thrust ; // initial guess, variable to be optimized
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
        if (partial_effective_burn_spin_down_duration > 0){
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
    thrust = static_cast<float>(norm(acc)) / ground_effect;
    drone_vel_after_takeoff = integrated_vel2 * 1.15f; // TODO determine magical battery healt factor :(
}

void DroneController::calc_pid_error(track_data data_drone, cv::Point3f setpoint_pos,cv::Point3f setpoint_vel,cv::Point3f setpoint_acc, double time) {

    float time_since_takeoff = static_cast<float>(time - start_takeoff_burn_time) - transmission_delay_duration;
    if (!data_drone.vel_valid && time_since_takeoff < 0.5f) {
        //assume an initial velocity of burn_direction * burn_duration  * thrust
        data_drone.state.vel = (_burn_direction_for_thrust_approx * thrust)* auto_burn_duration - time_since_takeoff * cv::Point3f(0,GRAVITY,0);
    }

    accErrX_prev = accErrX;
    accErrY_prev = accErrY;
    accErrZ_prev = accErrZ;

    // Roll Control - X
    posErrX = data_drone.sposX - setpoint_pos.x;              // position error
    if (posErrX>3.0f)
        posErrX = 3.0f;
    if (posErrX<-3.0f)
        posErrX = -3.0f;
    velx_sp = posErrX*gain_roll_pos/1000.f;           // desired velocity
    velErrX = data_drone.state.vel.x + velx_sp - setpoint_vel.x;    // velocity error
    accx_sp = velErrX*gain_roll_vel/100;              // desired acceleration
    accErrX = data_drone.state.acc.x + accx_sp - setpoint_acc.x;    // acceleration error

    // Altitude Control - Y
    posErrY = data_drone.sposY - setpoint_pos.y;              // position error
    vely_sp = posErrY*gain_throttle_pos/1000.f;       // (inversed) desired velocity
    velErrY = data_drone.state.vel.y + vely_sp - setpoint_vel.y;    // velocity error
    accy_sp = velErrY*gain_throttle_vel/100;          // (inversed) desired acceleration
    accErrY = data_drone.state.acc.y + accy_sp;// - setpoint_acc.y;    // acceleration error

    // Pitch Control - Z
    posErrZ = data_drone.sposZ - setpoint_pos.z;              // position error
    if (posErrZ>1.5f)
        posErrZ = 1.5f;
    if (posErrZ<-1.5f)
        posErrZ = -1.5f;
    velz_sp = posErrZ*gain_pitch_pos/1000.f;          // desired velocity
    velErrZ = data_drone.state.vel.z + velz_sp - setpoint_vel.z;    // velocity error
    accz_sp = velErrZ*gain_pitch_vel/100;             // desired acceleration
    accErrZ = data_drone.state.acc.z + accz_sp - setpoint_acc.z;    // acceleration error

    //when tuning, reset integrators when I gain set to 0:
    if (gain_throttle_i < 1)
        throttleErrI = 0;
    if (gain_roll_i < 1)
        rollErrI = 0;
    if (gain_pitch_i < 1)
        pitchErrI = 0;
}

void DroneController::control_pid(track_data data_drone) {
    //update integrators
    if (fabs(posErrY)<integratorThresholdDistance)
        throttleErrI += posErrY;
    else
        throttleErrI /= 1.05f;
    if (fabs(posErrX)<integratorThresholdDistance)
        rollErrI += posErrX;
    else
        rollErrI /= 1.05f;
    if (fabs(posErrZ)<integratorThresholdDistance)
        pitchErrI += posErrZ;
    else
        pitchErrI /= 1.05f;

    static float att2throttle_gain_scale = 1;
    float pid_roll,pid_pitch,pid_throttle;

    pid_throttle =   -(accErrY * gain_throttle_acc + throttleErrI * gain_throttle_i - (accErrY_prev - accErrY) * gain_throttle_d);
    if (dparams.mode3d)
        pid_throttle /=2; // with mode 3d the range is divided in two halfs. One up and one down.
    pid_throttle += initial_hover_throttle_guess();
    pid_roll =  accErrX * gain_roll_acc/att2throttle_gain_scale - (accErrX_prev - accErrX) * gain_roll_d/att2throttle_gain_scale;

    float depth_gain = 1;
    if (-data_drone.sposZ - 2 > 0) // if further then 2 meters
        depth_gain  = 1 + powf((-data_drone.sposZ-2),2) * depth_precision_gain;
    else // closer then 2 meters, do nothing:
        depth_gain = 1;

    pid_pitch = accErrZ * gain_pitch_acc/att2throttle_gain_scale/depth_gain - (accErrZ_prev - accErrZ) * gain_pitch_d/att2throttle_gain_scale;

    pid_roll    += (gain_roll_i*rollErrI)/att2throttle_gain_scale;
    pid_pitch   += (gain_pitch_i*pitchErrI)/att2throttle_gain_scale;

    //TMP fix for lacking control law handling of > 90 degree bank angle assuming BF angle limit 180
    float pid_angle_range = JOY_BOUND_RANGE/pid_max_angle_scaler;
    pid_roll = std::clamp(pid_roll,-pid_angle_range,pid_angle_range);
    pid_pitch = std::clamp(pid_pitch,-pid_angle_range,pid_angle_range);

    float tmptbf = dparams.throttle_bank_factor;

    // if we are higher then the target, use the fact more attitude makes us go down
    //    if (posErrY> 0 && abs(posErrX)<0.3f && abs(posErrZ)<0.3f) {
    //        tmptbf = 0;
    //    }
    if (dparams.mode3d)
        tmptbf *=0.5f;

    //calc throttle compensation needed for this attitude
    float att_angle;
    if (fabs(pid_pitch) > fabs(pid_roll))
        att_angle  = fabs(pid_pitch);
    else
        att_angle  = fabs(pid_roll);

    pid_throttle = std::clamp(pid_throttle,static_cast<float>(min_bound_throttle()),static_cast<float>(JOY_BOUND_MAX));

    att_angle = (att_angle/(0.5f*JOY_BOUND_RANGE))*M_PIf32;
    att2throttle_gain_scale = abs(1.f/cosf(att_angle))*tmptbf;
    float angle_comp = (pid_throttle-min_bound_throttle())*att2throttle_gain_scale-(pid_throttle-min_bound_throttle());
    pid_throttle += angle_comp;

    //make sure props don't stop:
    pid_throttle = std::clamp(pid_throttle,static_cast<float>(min_bound_throttle()),static_cast<float>(JOY_BOUND_MAX));

    pid_roll    += JOY_MIDDLE;
    pid_pitch   += JOY_MIDDLE;

    auto_roll = roundf(pid_roll_smoother.addSample(pid_roll));
    auto_pitch = roundf(pid_pitch_smoother.addSample(pid_pitch));
    auto_throttle = roundf(pid_throttle_smoother.addSample(pid_throttle));

}

std::tuple<int,int,int> DroneController::calc_feedforward_control(cv::Point3f desired_acceleration) {

    // Determine the required acceleration for the drone
    cv::Point3f req_acc = desired_acceleration + cv::Point3f(0,GRAVITY,0);
    cv::Point3f dir_req_acc = req_acc/norm(req_acc);
    cv::Point3f direction = dir_req_acc;

    // If the magnitude of the required acceleration is to high apply the maximal magnitude
    // and find the direction which still applies the correct direction.
    if(normf(req_acc)>thrust){
        req_acc *= thrust/normf(req_acc);
        direction = req_acc/normf(req_acc);

        cv::Point3f res_acc, dir_res_acc;
        float angle_err;

        // Iterative approximate the acceleration needed to find the target acceleration vector:
        for (int i = 0; i < 100; i++) {
            res_acc = direction*norm(req_acc) - cv::Point3f(0,GRAVITY,0);
            dir_res_acc = res_acc/norm(res_acc);

            angle_err = dir_res_acc.dot(dir_req_acc); //=cos(angle-err)

            if(angle_err>0.9998f){ //0.9998 error corresponds 1 deg
                break;
            } else {
                direction -= dir_res_acc - dir_req_acc;
                direction /= norm(direction);
            }
        }
    }

    // .. and then calc throttle control:
    int throttle = normf(req_acc)/thrust * (JOY_BOUND_RANGE) + JOY_BOUND_MIN;

    auto [roll_deg, pitch_deg] = acc_to_deg(direction);

    int roll =  ((roll_deg/max_bank_angle+1) / 2.f) * JOY_BOUND_RANGE + JOY_BOUND_MIN; // convert to RC commands range
    int pitch = ((pitch_deg/max_bank_angle+1) / 2.f) * JOY_BOUND_RANGE + JOY_BOUND_MIN;

    return std::make_tuple(roll,pitch,throttle);
}

void DroneController::control_modelBased(track_data data_drone, cv::Point3f setpoint_pos){

    // Set the control target:
    float target_speed = 0;

    // Speed gains;
    float KP_speed = 1;

    // Determine state errors:
    float errXfiltered = filerPosErrX.new_sample(setpoint_pos.x - data_drone.state.pos.x);
    float errYfiltered = filerPosErrY.new_sample(setpoint_pos.y - data_drone.state.pos.y);
    float errZfiltered = filerPosErrZ.new_sample(setpoint_pos.z - data_drone.state.pos.z);
    posErr_P = {errXfiltered, errYfiltered, errZfiltered};
    posErr_I += posErr_P;

    float errDx = dErrX.new_sample (errXfiltered);
    float errDy = dErrY.new_sample (errYfiltered);
    float errDz = dErrZ.new_sample (errZfiltered);
    posErr_D = {errDx, errDy, errDz};

    // Anti-Wind-Up-Handling:
    // Integrator error is counter acting to the current error:
    // ..

    //Determine the acceleration direction as PID-filtered postion error:
    cv::Point3f desired_acceleration;
    desired_acceleration.x = ((p_pos_roll/100.f)*posErr_P.x + static_cast<float>(i_pos_roll)/1000.f*posErr_I.x + (d_pos_roll/100.f)*posErr_D.x);
    desired_acceleration.y = ((p_pos_throttle/100.f)*posErr_P.y + static_cast<float>(i_pos_throttle)/1000.f*posErr_I.y + (d_pos_throttle/100.f)*posErr_D.y);
    desired_acceleration.z = ((p_pos_pitch/100.f)*posErr_P.z + static_cast<float>(i_pos_pitch)/1000.f*posErr_I.z + (d_pos_pitch/100.f)*posErr_D.z);

    desired_acceleration += KP_speed*target_speed*desired_acceleration/norm(desired_acceleration);

    // Determine the control outputs based on feed-forward calculations:
    std::tie(auto_roll, auto_pitch, auto_throttle) = calc_feedforward_control(desired_acceleration);

}


void DroneController::readJoystick(void) {
    while (joystick.sample(&event))
    {
        if (event.isAxis()) {
            if (pparams.joystick == rc_devo) {
                switch ( event.number ) {
                case 0: // roll
                    joy_roll = (event.value >> 5) + JOY_MIDDLE;
                    break;
                case 1: // pitch
                    joy_pitch =  (event.value >> 5) + JOY_MIDDLE;
                    break;
                case 2: //throttle
                    joy_throttle =  (event.value >> 5) + JOY_MIDDLE - 100;
                    break;
                case 5: //switch
                    _joy_arm_switch = event.value>0; // goes between +/-32768
                    break;
                case 3: //dial
                    joyDial = event.value; // goes between +/-32768
                    scaledjoydial = joyDial+32767;
                    scaledjoydial = (scaledjoydial / 65536)*100+35;
                    break;
                case 4: //yaw
                    joy_yaw =  (event.value >> 5) + JOY_MIDDLE;
                    break;
                default:
                    std::cout << "Unkown joystick event: " << std::to_string(event.number) << ". Value: " << std::to_string(event.value) << std::endl;
                    break;
                }
            }  else if (pparams.joystick == rc_xlite) {
                switch ( event.number ) {
                case 0: // roll
                    joy_roll = (event.value >> 5)*0.8 + JOY_MIDDLE;
                    break;
                case 1: // pitch
                    joy_pitch = (event.value >> 5)*0.8 + JOY_MIDDLE;
                    break;
                case 2: //throttle
                    joy_throttle =  (event.value >> 5)*0.8 + JOY_MIDDLE;
                    break;
                case 3: //yaw
                    joy_yaw = (event.value >> 5)*0.4 + JOY_MIDDLE;
                    break;
                case 4: //arm switch (two way)
                    _joy_arm_switch = event.value>0;
                    //                    _rc->arm(event.value>0);
                    break;
                case 5: //mode switch (3 way)
                    if (event.value<-16384 ){
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

    if (pparams.joystick == rc_none || _joy_state == js_none )
        return;

    // prevent accidental take offs at start up
    if (_joy_state == js_checking){
        if (!_joy_arm_switch &&
            ((joy_throttle <= JOY_MIN_THRESH && !dparams.mode3d) || (abs(joy_throttle- JOY_MIDDLE) < 50 && dparams.mode3d) )&&
            !_joy_takeoff_switch) {
            _flight_mode = fm_disarmed;
            _joy_state = js_disarmed;
        } else {
            _flight_mode = fm_joystick_check;
            //_rc->arm(false);
        }
    } else {

        if (_flight_mode == fm_inactive){
            _manual_override_take_off_now = _joy_takeoff_switch;
        }
        if (!_joy_arm_switch){
            if (!dparams.mode3d)
                _rc->arm(false);
            _joy_takeoff_switch = false;
            _manual_override_take_off_now = false;
            _joy_state = js_disarmed;
            _flight_mode = fm_disarmed;
        }else {
            if (!dparams.mode3d)
                _rc->arm(true);
            if (_joy_mode_switch == jmsm_manual){
                _joy_state = js_manual;
                _flight_mode = fm_manual;
            } else if (_joy_mode_switch == jmsm_waypoint)
                _joy_state = js_waypoint;
            else if (_joy_mode_switch == jmsm_hunt)
                _joy_state = js_hunt;
        }

#if CAMMODE == CAMMODE_GENERATOR
        joy_pitch = JOY_MIDDLE;
#endif
    }
}

void DroneController::deserialize_settings() {
    std::cout << "Reading settings from: " << settings_file << std::endl;
    ControlParameters params;
    if (file_exist(settings_file)) {
        std::ifstream infile(settings_file);
        std::string xmlData((std::istreambuf_iterator<char>(infile)),
                            std::istreambuf_iterator<char>());

        if (!xmls::Serializable::fromXML(xmlData, &params))
        { // Deserialization not successful
            throw my_exit("Cannot read: " + settings_file);
        }
        ControlParameters tmp;
        auto v1 = params.getVersion();
        auto v2 = tmp.getVersion();
        if (v1 != v2) {
            throw my_exit("XML version difference detected from " + settings_file);
        }
    } else {
        throw my_exit("File not found: " + settings_file);
    }

    gain_throttle_pos = params.gain_throttle_pos.value();
    gain_throttle_vel = params.gain_throttle_vel.value();
    gain_throttle_acc = params.gain_throttle_acc.value();
    gain_throttle_i = params.gain_throttle_i.value();
    gain_throttle_d = params.gain_throttle_d.value();
    gain_roll_pos = params.gain_roll_pos.value();
    gain_roll_vel = params.gain_roll_vel.value();
    gain_roll_acc = params.gain_roll_acc.value();
    gain_roll_i = params.gain_roll_i.value();
    gain_roll_d = params.gain_roll_d.value();
    gain_pitch_pos = params.gain_pitch_pos.value();
    gain_pitch_vel = params.gain_pitch_vel.value();
    gain_pitch_acc = params.gain_pitch_acc.value();
    gain_pitch_i = params.gain_pitch_i.value();
    gain_pitch_d = params.gain_pitch_d.value();


    p_pos_roll= params.p_pos_roll.value();
    p_pos_pitch= params.p_pos_pitch.value();
    p_pos_throttle= params.p_pos_throttle.value();
    i_pos_roll= params.i_pos_roll.value();
    i_pos_pitch= params.i_pos_pitch.value();
    i_pos_throttle= params.i_pos_throttle.value();
    d_pos_roll= params.d_pos_roll.value();
    d_pos_pitch= params.d_pos_pitch.value();
    d_pos_throttle= params.d_pos_throttle.value();

}

void DroneController::serialize_settings() {
    ControlParameters params;
    params.gain_throttle_pos = gain_throttle_pos;
    params.gain_throttle_vel = gain_throttle_vel;
    params.gain_throttle_acc = gain_throttle_acc;
    params.gain_throttle_i = gain_throttle_i;
    params.gain_throttle_d = gain_throttle_d;
    params.gain_roll_pos = gain_roll_pos;
    params.gain_roll_vel = gain_roll_vel;
    params.gain_roll_acc = gain_roll_acc;
    params.gain_roll_i = gain_roll_i;
    params.gain_roll_d = gain_roll_d;
    params.gain_pitch_pos = gain_pitch_pos;
    params.gain_pitch_vel = gain_pitch_vel;
    params.gain_pitch_acc = gain_pitch_acc;
    params.gain_pitch_i = gain_pitch_i;
    params.gain_pitch_d = gain_pitch_d;
    params.p_pos_roll = p_pos_roll;
    params.p_pos_pitch = p_pos_pitch;
    params.p_pos_throttle = p_pos_throttle;
    params.i_pos_roll = i_pos_roll;
    params.i_pos_pitch = i_pos_pitch;
    params.i_pos_throttle = i_pos_throttle;
    params.d_pos_roll = d_pos_roll;
    params.d_pos_pitch = d_pos_pitch;
    params.d_pos_throttle = d_pos_throttle;

    std::string xmlData = params.toXML();
    std::ofstream outfile = std::ofstream (settings_file);
    outfile << xmlData ;
    outfile.close();
}

void DroneController::close () {
    if (initialized) {
        std::cout << "closing controller" << std::endl;
        if (pparams.control_tuning)
            serialize_settings();
        initialized = false;
    }
}
