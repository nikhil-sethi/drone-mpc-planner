#include "dronecontroller.h"
#include "defines.h"

using namespace cv;

// Create an instance of Joystick
Joystick joystick("/dev/input/js0");
JoystickEvent event;

bool DroneController::joystick_ready(){
    return joystick.isFound();
}

void DroneController::init(std::ofstream *logger,bool fromfile, MultiModule * rc, DroneTracker *dtrk) {
    _rc = rc;
    _dtrk = dtrk;
    _logger = logger;
    _fromfile = fromfile;
    control_history_max_size = pparams.fps;
    (*_logger) << "valid; posErrX; posErrY; posErrZ; velX; velY; velZ; accX; accY; accZ; hoverthrottle; autoThrottle; autoRoll; autoPitch; autoYaw; joyThrottle; joyRoll; joyPitch; joyYaw; joyArmSwitch; joyModeSwitch; joyTakeoffSwitch; dt; dx; dy; dz; velx_sp; vely_sp; velz_sp;";
    std::cout << "Initialising control." << std::endl;

    settings_file = "../../xml/" + dparams.control + ".xml";

    // Load saved control paremeters
    deserialize_settings();

    if (pparams.control_tuning) {
        std::cout << "Creating control tuning window." << std::endl;
        // create GUI to set control parameters
        namedWindow("Control", WINDOW_NORMAL);
        createTrackbar("Throttle Pos", "Control", &gain_throttle_pos, 3000);
        createTrackbar("Throttle Vel", "Control", &gain_throttle_vel, 3000);
        createTrackbar("Throttle Acc", "Control", &gain_throttle_acc, 100);
        createTrackbar("Throttle I", "Control", &gain_throttle_i, 100);
        createTrackbar("Roll Pos", "Control", &gain_roll_pos, 3000);
        createTrackbar("Roll Vel", "Control", &gain_roll_vel, 2000);
        createTrackbar("Roll Acc", "Control", &gain_roll_acc, 100);
        createTrackbar("Roll I", "Control", &gain_roll_i, 100);
        createTrackbar("Pitch Pos", "Control", &gain_pitch_pos, 3000);
        createTrackbar("Pitch Vel", "Control", &gain_pitch_vel, 2000);
        createTrackbar("Pitch Acc", "Control", &gain_pitch_acc, 100);
        createTrackbar("Pitch I", "Control", &gain_pitch_i, 100);
    }

    hoverthrottle = dparams.initial_hover_throttle;
    initialized = true;
}

int bound_throttle(int v) {
    if ( v < JOY_BOUND_MIN )
        v = JOY_BOUND_MIN;
    if ( v > JOY_BOUND_MAX )
        v = JOY_BOUND_MAX;

    if (dparams.mode3d){
        float f = v-JOY_BOUND_MIN;
        float range = JOY_BOUND_MAX - JOY_BOUND_MIN;
        f/=range;
        v= 0.5f*f*range;
        v += JOY_MIDDLE-10;
    }
    return v;
}

int bound_joystick_value(int v) {
    if ( v < JOY_MIN )
        v = JOY_MIN;
    if ( v > JOY_MAX )
        v = JOY_MAX;
    return v;
}

void DroneController::control(track_data state_drone,track_data state_insect, cv::Point3f setpoint_pos, cv::Point3f setpoint_vel, cv::Point3f setpoint_acc, double time) {

    if (!_fromfile)
        readJoystick();
    if (!pparams.insect_logging_mode)
        process_joystick();

    // Roll Control - X
    posErrX = state_drone.sposX - setpoint_pos.x;              // position error
    if (posErrX>3.0f)
        posErrX = 3.0f;
    if (posErrX<-3.0f)
        posErrX = -3.0f;
    velx_sp = posErrX*gain_roll_pos/1000.f;           // desired velocity
    velErrX = state_drone.svelX + velx_sp - setpoint_vel.x;    // velocity error
    accx_sp = velErrX*gain_roll_vel/100;              // desired acceleration
    accErrX = state_drone.saccX + accx_sp - setpoint_acc.x;    // acceleration error

    // Altitude Control - Y
    posErrY = state_drone.sposY - setpoint_pos.y;              // position error
    vely_sp = posErrY*gain_throttle_pos/1000.f;       // (inversed) desired velocity
    velErrY = state_drone.svelY + vely_sp - setpoint_vel.y;    // velocity error
    accy_sp = velErrY*gain_throttle_vel/100;          // (inversed) desired acceleration
    accErrY = state_drone.saccY + accy_sp;// - setpoint_acc.y;    // acceleration error

    float tmptbf = dparams.throttle_bank_factor; // if we are higher then the target, use the fact more attitude makes us go down
    if (posErrY> 0 && abs(posErrX)<0.3f && abs(posErrZ)<0.3f) {
        tmptbf = 0;
    }

    // Pitch Control - Z
    posErrZ = state_drone.sposZ - setpoint_pos.z;              // position error
    if (posErrZ>1.5f)
        posErrZ = 1.5f;
    if (posErrZ<-1.5f)
        posErrZ = -1.5f;
    velz_sp = posErrZ*gain_pitch_pos/1000.f;          // desired velocity
    velErrZ = state_drone.svelZ + velz_sp - setpoint_vel.z;    // velocity error
    accz_sp = velErrZ*gain_pitch_vel/100;             // desired acceleration
    accErrZ = state_drone.saccZ + accz_sp - setpoint_acc.z;    // acceleration error

    //when tuning, reset integrators when I gain set to 0:
    if (gain_throttle_i < 1)
        throttleErrI = 0;
    if (gain_roll_i < 1)
        rollErrI = 0;
    if (gain_pitch_i < 1)
        pitchErrI = 0;

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
    } case fm_start_takeoff: {
        take_off_start_time = time;
        _flight_mode = fm_take_off_aim;
        std::cout << "Take off aiming" << std::endl;
        if (dparams.mode3d) {
            _rc->arm(true);
            auto_roll = JOY_MIDDLE;
            auto_pitch = JOY_MIDDLE;
            auto_throttle = JOY_BOUND_MIN;
            break;
        }
        [[fallthrough]];
    } case fm_take_off_aim: {
        auto_throttle = tmp_hover_throttle;
        //std::tie (auto_roll, auto_pitch, auto_burn_duration) = calc_takeoff_aim_burn(state_insect, _dtrk->drone_startup_location(),take_off_start_time);
        track_data state_drone_takeoff = state_drone;
        state_drone_takeoff.posX = _dtrk->drone_startup_location().x;
        state_drone_takeoff.posY = _dtrk->drone_startup_location().y;
        state_drone_takeoff.posZ = _dtrk->drone_startup_location().z;
        cv::Point3f drone_vel = {0};
        std::tie (auto_roll, auto_pitch,auto_burn_duration ) = calc_directional_burn(drone_vel,state_drone_takeoff,state_insect,time-10,time);
        if (static_cast<float>(time - take_off_start_time) > dparams.full_bat_and_throttle_spinup_duration) {
            _flight_mode = fm_max_burn;
            std::cout << "Take off burn" << std::endl;
        }
        break;
    } case fm_max_burn: {
        auto_throttle = JOY_BOUND_MAX;
        if (static_cast<float>(time - take_off_start_time) > dparams.full_bat_and_throttle_spinup_duration + auto_burn_duration)
            _flight_mode = fm_max_burn_spin_down;
        break;
    } case fm_max_burn_spin_down: {
        auto_throttle = tmp_hover_throttle;
        if (static_cast<float>(time - take_off_start_time) > dparams.full_bat_and_throttle_spinup_duration+ auto_burn_duration + effective_burn_spin_down_duration)
            _flight_mode = fm_1g;
        break;
    }  case fm_1g: {
        auto_roll = JOY_MIDDLE;
        auto_pitch = JOY_MIDDLE;

        // Wait until velocity of drone after take-off is constant, then estimate this velocity using sufficient samples
        if (!drone_1g_start_pos.pos_valid &&
            static_cast<float>(time - take_off_start_time) > dparams.full_bat_and_throttle_spinup_duration + auto_burn_duration + transmission_delay_duration + 6.f / pparams.fps) {
            drone_1g_start_pos = state_drone;
        } else  if (static_cast<float>(time - take_off_start_time) > dparams.full_bat_and_throttle_spinup_duration + auto_burn_duration + transmission_delay_duration + 12.f / pparams.fps){
            if(_joy_state == js_waypoint)
                _flight_mode = fm_flying_pid_init;
            else
                _flight_mode = fm_interception_aim_start;
        }
        break;
    }  case fm_interception_aim_start: {
        std::cout << "Aiming" << std::endl;
        interception_start_time = time;
        approx_effective_thrust(state_drone,_burn_direction_for_thrust_approx,auto_burn_duration,static_cast<float>(time-take_off_start_time)-dparams.full_bat_and_throttle_spinup_duration);
        std::cout << "Estimated acc: " << thrust << std::endl;
        _flight_mode =   fm_interception_aim;
        [[fallthrough]];
    }   case fm_interception_aim: {

        if (state_drone.vel_valid){
            //use normal state
            std::tie (auto_roll, auto_pitch,auto_burn_duration ) = calc_directional_burn(state_drone,state_insect,interception_start_time,time);
            auto_throttle = tmp_hover_throttle;
        } else {
            std::tie (auto_roll, auto_pitch,auto_burn_duration ) = calc_directional_burn(drone_1g_start_pos, state_drone,state_insect,interception_start_time,time);
            auto_throttle = tmp_hover_throttle;
        }

        float remaining_aiming_time = aim_duration - static_cast<float>(time - interception_start_time);
        if (remaining_aiming_time < 0)
            _flight_mode = fm_interception_burn_start;
        break;
    } case fm_interception_burn_start: {
        _flight_mode = fm_interception_burn;
        auto_throttle = JOY_BOUND_MAX;
        std::cout << "Burning" << std::endl;
        [[fallthrough]];
    } case fm_interception_burn: {
        if (static_cast<float>(time - interception_start_time) > aim_duration + auto_burn_duration){
            _flight_mode = fm_interception_burn_spin_down;
            std::cout << "Spindown" << std::endl;
        }
        break;
    } case fm_interception_burn_spin_down: {
        auto_throttle = tmp_hover_throttle;
        if (static_cast<float>(time - interception_start_time) > aim_duration + auto_burn_duration + effective_burn_spin_down_duration)
            _flight_mode = fm_interception_1g;
        break;
    } case fm_interception_1g: {
        //only control throttle:
        auto_pitch = JOY_MIDDLE;
        auto_roll = JOY_MIDDLE;
        auto_throttle = tmp_hover_throttle;
        throttle = auto_throttle;

        if (static_cast<float>(time - interception_start_time) > aim_duration + auto_burn_duration) //weird burn time is way too short?!
            _flight_mode = fm_retry_aim_start;
        if (time - interception_start_time > 2) // TODO: detect if we got it or stop at some sensible moment.
            _flight_mode = fm_flying_pid_init;
        break;
    }  case fm_retry_aim_start: {
        interception_start_time = time;

        if (!state_drone.pos_valid || !state_drone.vel_valid) {
            throttle = auto_throttle;
            pitch = JOY_MIDDLE;
            roll = JOY_MIDDLE;
            _flight_mode =   fm_disarmed; // todo: create a feedforward return to volume
            break;
        } else
            _flight_mode =   fm_interception_aim;
        std::cout << "Re-aiming" << std::endl;
        std::tie (auto_roll, auto_pitch, auto_burn_duration) = calc_directional_burn(state_drone,state_insect,aim_duration,time);
        auto_throttle = tmp_hover_throttle;
        break;
    } case fm_flying_pid_init: {
        rollErrI = 0;
        pitchErrI = 0;
        throttleErrI = 0;
        _flight_mode = fm_flying_pid;
        [[fallthrough]];
    } case fm_flying_pid: {
        //update integrators
        if (fabs(posErrZ)<integratorThresholdDistance)
            throttleErrI += velErrY; //posErrY;
        if (fabs(posErrX)<integratorThresholdDistance)
            rollErrI += posErrX;
        if (fabs(posErrZ)<integratorThresholdDistance)
            pitchErrI += posErrZ;

        auto_throttle =  hoverthrottle - (accErrY * gain_throttle_acc + throttleErrI * gain_throttle_i*0.1f);
        auto_roll =  accErrX * gain_roll_acc;

        float depth_gain = 1;
        if (-state_drone.sposZ - 2 > 0) // if further then 2 meters
            depth_gain  = 1 + powf((-state_drone.sposZ-2),2) * depth_precision_gain;
        else // closer then 2 meters, do nothing:
            depth_gain = 1;

        auto_pitch = accErrZ * static_cast<float>(gain_pitch_acc) / depth_gain;

        //TMP fix for lacking control law handling of > 90 degree bank angle assuming BF angle limit 160
        if (auto_roll < -JOY_MIDDLE/2)
            auto_roll = -JOY_MIDDLE/2;
        if (auto_roll > JOY_MIDDLE/2)
            auto_roll = JOY_MIDDLE/2;

        if (auto_pitch < -JOY_MIDDLE/2)
            auto_pitch = -JOY_MIDDLE/2;
        if (auto_pitch > JOY_MIDDLE/2)
            auto_pitch = JOY_MIDDLE/2;

        //TMP fix for lacking control law handling of > 60 degree bank angle and target far away assuming BF angle limit 160
        if (abs(posErrX)>0.3f || abs(posErrZ)>0.3f) {

            if (auto_roll < -JOY_MIDDLE/3)
                auto_roll = -JOY_MIDDLE/3;
            if (auto_roll > JOY_MIDDLE/3)
                auto_roll = JOY_MIDDLE/3;

            if (auto_pitch < -JOY_MIDDLE/3)
                auto_pitch = -JOY_MIDDLE/3;
            if (auto_pitch > JOY_MIDDLE/3)
                auto_pitch = JOY_MIDDLE/3;
        }

        if (fabs(auto_roll) > fabs(auto_pitch)){
            //auto_throttle += tmptbf*abs(auto_roll);
            float roll_angle = static_cast<float>(auto_roll)/JOY_MIDDLE*M_PIf32;
            float roll_comp = auto_throttle*abs(1.f/cosf(roll_angle))-auto_throttle;
            auto_throttle += roll_comp*tmptbf;
        }
        else {
            //auto_throttle += tmptbf*abs(auto_pitch);
            float pitch_angle = static_cast<float>(auto_pitch)/JOY_MIDDLE*M_PIf32;
            float pitch_comp = auto_throttle*abs(1.f/cosf(pitch_angle))-auto_throttle;
            auto_throttle += pitch_comp*tmptbf;
        }

        auto_roll    += JOY_MIDDLE + (gain_roll_i*rollErrI);
        auto_pitch   += JOY_MIDDLE + (gain_pitch_i*pitchErrI);

        //int minThrottle = 1300 + min(abs(autoRoll-1500)/10,50) + min(abs(autoPitch-1500)/10,50);
        if (auto_throttle<dparams.min_throttle)
            auto_throttle = dparams.min_throttle;
        if (auto_throttle>JOY_BOUND_MAX)
            auto_throttle = JOY_BOUND_MAX;
        break;
    } case fm_landing: {
        //slowly decrease throttle
        auto_throttle = hoverthrottle - (accErrY * gain_throttle_acc + throttleErrI * gain_throttle_i*0.1f)
                        - autoLandThrottleDecrease;
        //same as fm_flying:
        auto_roll =  JOY_MIDDLE + (accErrX * gain_roll_acc +  gain_roll_i*rollErrI);
        auto_pitch = JOY_MIDDLE + (accErrZ * gain_pitch_acc +  gain_pitch_i*pitchErrI);
        break;
    } case fm_disarmed: {
        auto_roll = JOY_MIDDLE;
        auto_pitch = JOY_MIDDLE;
        auto_throttle = JOY_BOUND_MIN;
        _rc->arm(false);
        break;
    } case fm_inactive: {
        auto_roll = JOY_MIDDLE;
        auto_pitch = JOY_MIDDLE;
        auto_throttle = JOY_BOUND_MIN;
        if (dparams.mode3d)
            _rc->arm(false);
        else
            _rc->arm(true);
        break;
    } case fm_abort_takeoff: {
        auto_throttle = JOY_BOUND_MIN;
        auto_roll = JOY_MIDDLE;
        auto_pitch = JOY_MIDDLE;
        break;
    } case fm_joystick_check: {
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

    std::cout << time <<  " rpt: " << roll << ", " << pitch << ", " << throttle << std::endl;
    if (!_fromfile) {
        _rc->queue_commands(throttle,roll,pitch,yaw);
    }

    control_data c(Throttle(),Roll(),Pitch(),time);
    control_history.push_back(c);
    while (control_history.size() > control_history_max_size)
        control_history.erase(control_history.begin());

    (*_logger) << static_cast<int>(state_drone.pos_valid)  << "; " <<
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
        state_drone.dt << "; " <<
        state_drone.dx << "; " <<
        state_drone.dy << "; " <<
        state_drone.dz << "; " <<
        velx_sp << "; " <<
        vely_sp << "; " <<
        velz_sp << "; ";
}

std::tuple<int, int, float> DroneController::calc_directional_burn(track_data state_drone,track_data state_insect, double aim_start_time, double time) {
    cv::Point3f drone_vel(state_drone.svelX,state_drone.svelY,state_drone.svelZ);
    return calc_directional_burn(drone_vel,state_drone,state_insect,aim_start_time,time);
}

std::tuple<int, int, float> DroneController::calc_directional_burn(track_data state_drone_start_1g, track_data state_drone,track_data state_insect, double aim_start_time, double time) {
    //calculate the exact speed of the drone assuming it started from the start position with zero speed, some dt ago.
    cv::Point3f drone_vel;
    float dt =  state_drone.time - state_drone_start_1g.time;
    drone_vel.x = (state_drone.posX - state_drone_start_1g.posX) / dt;
    drone_vel.y = (state_drone.posY - state_drone_start_1g.posY) / dt;
    drone_vel.z = (state_drone.posZ - state_drone_start_1g.posZ) / dt;
    return calc_directional_burn(drone_vel,state_drone,state_insect,aim_start_time,time);
}

std::tuple<int,int,float> DroneController::calc_directional_burn(cv::Point3f drone_vel, track_data state_drone, track_data state_insect, double aim_start_time, double time) {

    cv::Point3f current_drone_pos = state_drone.Pos();
    cv::Point3f current_insect_pos = state_insect.Pos();
    cv::Point3f current_insect_vel = state_insect.sVel();

    float remaining_aim_duration;
    if (_flight_mode == fm_take_off_aim) { //during take off aim, drones lifts from the ground a few cm
        current_drone_pos.y+=lift_off_dist_take_off_aim;
        remaining_aim_duration = 0;
    } else {
        remaining_aim_duration = (aim_duration -  static_cast<float>(time - aim_start_time)) + transmission_delay_duration + 1.f/ pparams.fps;
        if (remaining_aim_duration < 0)
            remaining_aim_duration = 0;
    }

    //predict drone location at the moment of the upcoming burn since the drone will have a delayed
    //reaction, so we need to calcuate this from that point on, which means we need to do a small
    //prediction step and acquire the new positions at that time
    cv::Point3f drone_pos_after_aim = current_drone_pos + remaining_aim_duration * drone_vel;
    //same for insect:
    cv::Point3f insect_pos_after_aim = current_insect_pos + remaining_aim_duration * current_insect_vel;

    cv::Point3f burn_dist = insect_pos_after_aim - drone_pos_after_aim;
    float burn_duration = sqrtf(2.f*static_cast<float>(norm(burn_dist))/thrust);
    cv::Point3f drone_pos_after_burn,insect_pos_after_burn,burn_direction,burn_accelleration_max;

    float conversion_speed = 1;
    //iterative approximate the acceleration needed to hit the insect
    for (int i = 0; i < 100; i++) {

        burn_direction = burn_dist/norm(burn_dist);
        burn_accelleration_max = burn_direction*thrust;

        cv::Point3f integrated_pos, integrated_vel;
        std::tie(integrated_pos, integrated_vel, std::ignore) = predict_drone_state_after_burn(
            current_drone_pos,drone_vel, burn_direction,burn_accelleration_max,remaining_aim_duration,burn_duration);
        drone_pos_after_burn = integrated_pos;
        insect_pos_after_burn = current_insect_pos + (remaining_aim_duration+burn_duration) * current_insect_vel;

        cv::Point3f pos_err = insect_pos_after_burn - integrated_pos;
        burn_dist +=pos_err;

        //to determine whether the burn_duration needs to in/decrease we check if the pos after
        //the burn is moving away or towards the insect. This is done by checking if the angle between
        //the vel vector and the delta_pos vector is bigger then 90 degrees:
        if (integrated_vel.dot(insect_pos_after_burn-integrated_pos ) > 0)
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

        double derr = norm(pos_err);
        if (derr < 0.01) // if converged
            break;

        if (i>=99)
            std::cout << "Warning: calc_directional_burn not converged!" << std::endl;
    }
    //update a final time (can prolly be streamlined)
    burn_direction = burn_dist/norm(burn_dist);
    burn_accelleration_max = burn_direction*thrust;

    auto [roll_deg, pitch_deg] = convert_acc_to_deg(burn_direction);

    int auto_roll_burn =  ((roll_deg/max_bank_angle+1) / 2.f) * JOY_MAX; // convert to RC commands range
    int auto_pitch_burn = ((pitch_deg/max_bank_angle+1) / 2.f) * JOY_MAX;

    if (_flight_mode == fm_interception_aim || _flight_mode == fm_interception_aim_start){
        viz_pos_after_aim =  drone_pos_after_aim;
        viz_time_after_aim = interception_start_time + static_cast<double>(transmission_delay_duration + aim_duration + 1.f/pparams.fps);
        viz_pos_after_burn = drone_pos_after_burn;
        viz_time_after_burn = viz_time_after_aim + static_cast<double>(burn_duration);

        if (_fromfile){ // visualize the interception path
            viz_drone_trajectory.clear();
            burn_accelleration_max = burn_direction * thrust;
            cv::Point3f integrated_pos;
            for (float f=0;f < burn_duration;f+=0.01f) {
                std::tie (integrated_pos, std::ignore,std::ignore) = predict_drone_state_after_burn(
                    current_drone_pos,drone_vel, burn_direction,burn_accelleration_max,remaining_aim_duration,f);
                viz_drone_trajectory.push_back(integrated_pos);
            }
        }
    } else if (_flight_mode == fm_take_off_aim) {
        remaining_aim_duration = dparams.full_bat_and_throttle_spinup_duration - static_cast<float>(time - take_off_start_time);
        if (remaining_aim_duration < 0)
            remaining_aim_duration = 0;

        viz_pos_after_aim =  drone_pos_after_aim;
        viz_time_after_aim = time + static_cast<double>(remaining_aim_duration + transmission_delay_duration + 1.f/pparams.fps);

        burn_duration = 0.1f; //TODO: make this number dynamic such that we have just enough time to do a second directional burn?
        auto [integrated_pos, integrated_vel,burn_accelleration] = predict_drone_state_after_burn(
            current_drone_pos,drone_vel, burn_direction,burn_accelleration_max,remaining_aim_duration,burn_duration);
        std::tie(viz_pos_after_burn, std::ignore) = predict_drone_state_after_spindown(integrated_pos, integrated_vel,burn_accelleration);
        viz_time_after_burn = viz_time_after_aim + static_cast<double>(burn_duration + effective_burn_spin_down_duration);

        if (_fromfile){ // visualize the take off path
            viz_drone_trajectory.clear();
            for (float f=0;f < burn_duration;f+=0.01f) {
                std::tie (integrated_pos, std::ignore,std::ignore) = predict_drone_state_after_burn(
                    current_drone_pos,drone_vel, burn_direction,burn_accelleration_max,remaining_aim_duration,f);
                viz_drone_trajectory.push_back(integrated_pos);
            }
        }

    }

    _burn_direction_for_thrust_approx = burn_direction; // to be used later to approx effective thrust
    return std::make_tuple(auto_roll_burn,auto_pitch_burn,burn_duration);
}

//calculate the predicted drone location after a burn
std::tuple<cv::Point3f, cv::Point3f, cv::Point3f> DroneController::predict_drone_state_after_burn
    (cv::Point3f current_drone_pos,
     cv::Point3f drone_vel, cv::Point3f burn_direction,
     cv::Point3f burn_accelleration_max,
     float remaining_aim_duration, float burn_duration) {

    cv::Point3f integrated_pos = current_drone_pos;
    cv::Point3f integrated_vel = drone_vel;

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

    cv::Point3f burn_accelleration = burn_accelleration_max;

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
        burn_accelleration = burn_accelleration_max * (partial_effective_burn_spin_up_duration / effective_burn_spin_up_duration); // only part of the max acc is reached because the burn was too short
        burn_accelleration -=cv::Point3f(0,GRAVITY,0);

        // Phase: spin up
        integrated_pos += 0.25f * burn_accelleration *powf(partial_effective_burn_spin_up_duration,2) + integrated_vel * partial_effective_burn_spin_up_duration;
        integrated_vel += 0.5f * burn_accelleration * partial_effective_burn_spin_up_duration;
    }

    return std::make_tuple(integrated_pos, integrated_vel, burn_accelleration);
}

std::tuple<float,float> DroneController::convert_acc_to_deg(cv::Point3f burn_dist) {
    //calculate roll/pitch commands for BF by applying eq. 37 & 38 from https://www.nxp.com/docs/en/application-note/AN3461.pdf
    //and then adapted (with a hamer) until it finally listened... TODO: math-fu opportunity:
    float insect_angle_roll,insect_angle_pitch;
    if (burn_dist.y>0) {
        insect_angle_roll  =  atan2f(-burn_dist.x,sqrtf(powf(burn_dist.y,2) + powf(burn_dist.z,2)));
        insect_angle_pitch =  atan2f(burn_dist.z,sqrtf(powf(burn_dist.y,2) + powf(burn_dist.x,2)));
    } else {
        insect_angle_roll  =  atan2f(burn_dist.x,sqrtf(powf(burn_dist.y,2) + powf(burn_dist.z,2))) + M_PIf32;
        insect_angle_pitch =  atan2f(burn_dist.z,sqrtf(powf(burn_dist.y,2) + powf(burn_dist.x,2)));

        if (insect_angle_roll > M_PIf32)
            insect_angle_roll = -(M_PIf32*2.f - insect_angle_roll);
        if (insect_angle_pitch > M_PIf32)
            insect_angle_pitch = -(M_PIf32*2.f - insect_angle_pitch);

        if (insect_angle_roll < -M_PIf32)
            insect_angle_roll = M_PIf32*2.f + insect_angle_roll;
        if (insect_angle_pitch < -M_PIf32)
            insect_angle_pitch = M_PIf32*2.f + insect_angle_pitch;
    }

    //this should reverse above again (for sanity check)
    //    float x = -sinf(insect_angle_roll) * cosf(insect_angle_pitch);
    //    float y = cosf(insect_angle_roll) * cosf(insect_angle_pitch);
    //    float z = sinf(insect_angle_pitch);

    insect_angle_roll *=rad2deg;
    insect_angle_pitch *=rad2deg;
    insect_angle_roll = std::clamp(insect_angle_roll,-max_bank_angle,max_bank_angle);
    insect_angle_pitch = std::clamp(insect_angle_pitch,-max_bank_angle,max_bank_angle);
    return std::make_tuple(insect_angle_roll,insect_angle_pitch);
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
void DroneController::approx_effective_thrust(track_data state_drone, cv::Point3f burn_direction, float burn_duration, float dt) {

    cv::Point3f pos_after_aim =  _dtrk->drone_startup_location() + cv::Point3f(0,lift_off_dist_take_off_aim,0);
    float partial_effective_burn_spin_down_duration = effective_burn_spin_down_duration; // if the burn duration is long enough this is equal otherwise it may be shortened
    cv::Point3f acc = burn_direction * thrust ; // initial guess, variable to be optimized
    cv::Point3f meas_pos =  state_drone.Pos(); // current measured drone position
    for (int i = 0; i<100; i++) {

        cv::Point3f max_acc = acc;
        cv::Point3f integrated_pos = pos_after_aim;
        cv::Point3f integrated_vel = {0};

        if (burn_duration  > effective_burn_spin_up_duration) {
            max_acc -=cv::Point3f(0,GRAVITY,0);
            // Phase: spin up
            integrated_pos += 0.25f * max_acc *powf(effective_burn_spin_up_duration,2);
            integrated_vel = 0.5f * max_acc  * effective_burn_spin_up_duration;
            // Phase: max burn
            float t_mt = burn_duration - effective_burn_spin_up_duration;
            integrated_pos += 0.5f * max_acc *powf(t_mt,2) + integrated_vel * t_mt;
            integrated_vel += max_acc * t_mt;
        } else {
            partial_effective_burn_spin_down_duration = burn_duration;
            max_acc = acc * (burn_duration / effective_burn_spin_up_duration); // only part of the max acc is reached because the burn was too short
            max_acc -=cv::Point3f(0,GRAVITY,0);

            // Phase: spin up
            integrated_pos += 0.25f * max_acc *powf(partial_effective_burn_spin_down_duration,2);
            integrated_vel = 0.5f * max_acc  * partial_effective_burn_spin_down_duration;
        }

        if (dt>burn_duration + partial_effective_burn_spin_down_duration){
            // Phase: spin down
            integrated_pos += 0.25f * max_acc *powf(partial_effective_burn_spin_down_duration,2) + integrated_vel * partial_effective_burn_spin_down_duration;
            integrated_vel += 0.5*max_acc * partial_effective_burn_spin_down_duration;
        } else
            std::cout << "Error: not implemented" << std::endl;  //todo: or never needed... ?

        // Phase: 1G: (the remaining time)
        integrated_pos += integrated_vel * (dt-burn_duration-partial_effective_burn_spin_down_duration);

        cv::Point3f err = meas_pos - integrated_pos;
        acc += err / powf(dt,2);
        if (norm(err)< 0.01)
            break;

    }
    thrust = static_cast<float>(norm(acc)) / ground_effect;
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
            } else if (pparams.joystick == rc_usb_hobbyking) {
                switch ( event.number ) {
                case 0: // roll
                    joy_roll = JOY_MIDDLE + (event.value*(JOY_MAX-JOY_MIN)/33000);
                    break;
                case 1: // pitch
                    joy_pitch = JOY_MIDDLE + (event.value*(JOY_MAX-JOY_MIN)/33000);
                    break;
                case 2: //throttle
                    joy_throttle = JOY_MIDDLE - (event.value*(JOY_MAX-JOY_MIN)/44000);
                    break;
                case 3: //switch
                    _joy_arm_switch = event.value>0; // goes between +/-32768
                    break;
                case 4: //dial
                    joyDial = event.value; // goes between +/-32768
                    scaledjoydial = joyDial+32767;
                    scaledjoydial = (scaledjoydial / 65536)*100+35;
                    break;
                case 5: //yaw
                    joy_yaw = JOY_MIDDLE + (event.value*(JOY_MAX-JOY_MIN)/33000);
                    break;
                default:
                    std::cout << "Unkown joystick event: " << std::to_string(event.number) << ". Value: " << std::to_string(event.value) << std::endl;
                    break;
                }
            } else if (pparams.joystick == rc_playstation) {
                switch ( event.number ) {
                case 2: // roll
                    joy_roll = JOY_MIDDLE + (event.value >> 5);
                    std::cout << "roll" << std::endl;
                    break;
                case 3: // pitch
                    joy_pitch = JOY_MIDDLE - (event.value >> 5);
                    std::cout << "pitch" << std::endl;
                    break;
                case 1: //throttle
                    joy_throttle = JOY_MIN - (event.value >> 5);
                    break;
                case 0: //yaw
                    joy_yaw = JOY_MIDDLE + (event.value >> 5);
                    std::cout << "yaw" << std::endl;
                    break;
                default:
                    std::cout << "Unkown joystick event: " << std::to_string(event.number) << ". Value: " << std::to_string(event.value) << std::endl;
                    break;
                }
            } else if (pparams.joystick == rc_xlite) {
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
                    joy_yaw = (event.value >> 5)*0.8 + JOY_MIDDLE;
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
            } else if (event.isButton() && pparams.joystick == rc_playstation) {
                switch ( event.number ) {
                case 2: //bind
                    if (event.value>0) {
                        _joy_arm_switch = 1;
                        joy_pitch = JOY_MAX_THRESH;
                    } else {
                        _joy_arm_switch = 0;
                        joy_pitch = JOY_MIDDLE;
                    }
                    break;
                case 9: //auto mode
                    _joy_arm_switch = 1;
                    break;
                case 4: //manual mode
                    _joy_arm_switch = 0;
                    break;
                case 6: //manual mode
                    _joy_arm_switch = 0;
                    break;
                default:
                    std::cout << "Unkown joystick button: " << std::to_string(event.number) << ". Value: " << std::to_string(event.value) << std::endl;
                    break;
                }
            }
        }
    }
}

void DroneController::process_joystick() {
    // prevent accidental take offs at start up
    if (pparams.joystick == rc_none)
        return;
    else if (_joy_state == js_none)
        _joy_state = js_checking;


    if (_joy_state == js_checking){
        if (!_joy_arm_switch &&
            joy_throttle <= JOY_MIN_THRESH &&
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

        if  (pparams.joystick == rc_usb_hobbyking) {
            //check switch functions
            static bool joySwitch_prev = _joy_arm_switch;
            if (_joy_arm_switch && !joySwitch_prev) {
                if (joy_pitch > JOY_MAX_THRESH) {
                    _rc->bind(true);
                    _joy_arm_switch = false;
                } else if(joy_throttle < JOY_MIN + 100) {
                    if (joy_pitch < JOY_MIN + 200) {
                        _flight_mode = fm_inactive;
                        if (joy_roll > JOY_MAX_THRESH)
                            _manual_override_take_off_now = true;
                    }
                }
            } else if (!_joy_arm_switch && joySwitch_prev && !_fromfile) {
                _joy_mode_switch = jmsm_manual;
            }
            joySwitch_prev = _joy_arm_switch;
        } else if  (pparams.joystick == rc_xlite) {
            if (!_joy_arm_switch){
                //_rc->arm(false);
                _joy_takeoff_switch = false;
                _manual_override_take_off_now = false;
                _joy_state = js_disarmed;
                _flight_mode = fm_disarmed;
            }else {
                //_rc->arm(true);
                if (_joy_mode_switch == jmsm_manual){
                    _joy_state = js_manual;
                    _flight_mode = fm_manual;
                } else if (_joy_mode_switch == jmsm_waypoint)
                    _joy_state = js_waypoint;
                else if (_joy_mode_switch == jmsm_slider)
                    _joy_state = js_slider;
                else if (_joy_mode_switch == jmsm_hunt)
                    _joy_state = js_hunt;
            }
        }
#if CAMMODE == CAMMODE_GENERATOR
        joyPitch = JOY_MIDDLE;
#endif
    }
}

void DroneController::deserialize_settings() {
    std::cout << "Reading settings from: " << settings_file << std::endl;
    ControlParameters params;
    if (checkFileExist(settings_file)) {
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
    gain_roll_pos = params.gain_roll_pos.value();
    gain_roll_vel = params.gain_roll_vel.value();
    gain_roll_acc = params.gain_roll_acc.value();
    gain_roll_i = params.gain_roll_i.value();
    gain_pitch_pos = params.gain_pitch_pos.value();
    gain_pitch_vel = params.gain_pitch_vel.value();
    gain_pitch_acc = params.gain_pitch_acc.value();
    gain_pitch_i = params.gain_pitch_i.value();
}

void DroneController::serialize_settings() {
    ControlParameters params;
    params.gain_throttle_pos = gain_throttle_pos;
    params.gain_throttle_vel = gain_throttle_vel;
    params.gain_throttle_acc = gain_throttle_acc;
    params.gain_throttle_i = gain_throttle_i;
    params.gain_roll_pos = gain_roll_pos;
    params.gain_roll_vel = gain_roll_vel;
    params.gain_roll_acc = gain_roll_acc;
    params.gain_roll_i = gain_roll_i;
    params.gain_pitch_pos = gain_pitch_pos;
    params.gain_pitch_vel = gain_pitch_vel;
    params.gain_pitch_acc = gain_pitch_acc;
    params.gain_pitch_i = gain_pitch_i;

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
