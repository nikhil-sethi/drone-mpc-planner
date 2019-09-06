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
    switch(_flight_mode) {
    case fm_manual : {
        //reset integrators (prevent ever increasing error)
        rollErrI = 0;
        pitchErrI = 0;
        throttleErrI = 0;
        throttle = joy_throttle;
        auto_throttle = JOY_BOUND_MIN;
        _rc->arm(_joy_arm_switch);
        roll = joy_roll;
        pitch = joy_pitch;
        yaw = joy_yaw;
        break;
    } case fm_start_takeoff : {
        take_off_start_time = time;
        //reset integrators
        rollErrI = 0;
        pitchErrI = 0;
        throttleErrI = 0;
        if (dparams.mode3d)
            _rc->arm(true);
        roll = JOY_MIDDLE;
        pitch = JOY_MIDDLE;
        throttle = JOY_BOUND_MIN;
        _flight_mode = fm_take_off_aim;
        break;
        [[fallthrough]];
    } case fm_take_off_aim : {
        auto_throttle = 650; // TODO: LUDWIG HELP initial hover throttle...

        calc_takeoff_aim_burn(setpoint_pos, _dtrk->drone_startup_location());
        std::cout << "fm_take_off_aim: " << auto_roll_burn << ", "  << auto_pitch_burn << ", "  << auto_takeoff_time_burn << std::endl;

        auto_pitch = auto_pitch_burn;
        auto_roll = auto_roll_burn;
        pitch =  auto_pitch;
        roll = auto_roll;
        throttle = auto_throttle;

        if (static_cast<float>(time - take_off_start_time) > dparams.full_bat_and_throttle_spinup_time)
            _flight_mode = fm_max_burn;
        break;

    } case fm_max_burn : {

        auto_throttle = JOY_BOUND_MAX;

        pitch =  auto_pitch;
        roll = auto_roll;
        throttle = auto_throttle;

        if (static_cast<float>(time - take_off_start_time) > dparams.full_bat_and_throttle_spinup_time+ auto_takeoff_time_burn) 
            _flight_mode = fm_1g;

        break;
    }  case fm_1g: {
        auto_throttle = 650; // TODO: LUDWIG HELP better hover throttle...
        auto_roll = JOY_MIDDLE;
        auto_pitch = JOY_MIDDLE;
        //only control throttle:
        throttle = auto_throttle;
        pitch =  auto_pitch;
        roll = auto_roll;

        // Wait until velocity of drone after take-off is constant, then estimate this velocity using sufficient samples
        if (!drone_1g_start_pos.pos_valid && static_cast<float>(time - take_off_burn_start_time) > dparams.full_bat_and_throttle_spinup_time *1.f + auto_takeoff_time_burn + tranmission_delay_time + 6.f / pparams.fps)
            drone_1g_start_pos = state_drone;
        else  if (static_cast<float>(time - take_off_burn_start_time) > dparams.full_bat_and_throttle_spinup_time *1.f + auto_takeoff_time_burn + tranmission_delay_time + 12.f / pparams.fps){
            if(_joy_state == js_waypoint){
                _flight_mode = fm_flying_pid;
            } else{
                _flight_mode = fm_interception_aim_start;
            }
        }

        break;
    }  case fm_interception_aim_start: {
        interception_burn_start_time = time;
        _flight_mode =   fm_interception_aim;
        calc_directional_burn(drone_1g_start_pos, state_drone,state_insect,interception_aim_time+tranmission_delay_time + 1./pparams.fps);
        std::cout << "fm_interception_aim_start: " << auto_roll_burn << ", "  << auto_pitch_burn << ", "  << auto_interception_burn_duration << std::endl;
        auto_pitch = auto_pitch_burn;
        auto_roll = auto_roll_burn;
        [[fallthrough]];
    }   case fm_interception_aim: {
        pitch = auto_pitch;
        roll = auto_roll;
        auto_throttle = 650; // TODO: LUDWIG HELP initial hover throttle...
        throttle = auto_throttle;

        if (time - interception_burn_start_time > interception_aim_time)
            _flight_mode = fm_interception_burn_start;
        break;
    } case fm_interception_burn_start : {
        _flight_mode = fm_interception_burn;
        pitch = auto_pitch;
        roll = auto_roll;
        auto_throttle = JOY_BOUND_MAX;
        throttle = auto_throttle;
        [[fallthrough]];
    } case fm_interception_burn : {
        pitch = auto_pitch;
        roll = auto_roll;
        throttle = auto_throttle;
        if (time - interception_burn_start_time > interception_aim_time + auto_interception_burn_duration)
            _flight_mode = fm_interception_1g;
        break;
    } case fm_interception_1g: {
        //only control throttle:
        pitch = JOY_MIDDLE;
        roll = JOY_MIDDLE;
        auto_throttle = 650; // TODO: LUDWIG HELP initial hover throttle...
        throttle = auto_throttle;

        if (time - interception_burn_start_time > interception_aim_time + auto_interception_tti)
            _flight_mode = fm_flying_pid;
        if (time - interception_burn_start_time > 2) // TODO: detect if we got it or stop at some sensible moment.
            _flight_mode = fm_flying_pid;
        break;
    }  case fm_retry_aim_start: {
        interception_burn_start_time = time;

        if (!state_drone.pos_valid || !state_drone.vel_valid) {
            throttle = auto_throttle;
            pitch = JOY_MIDDLE;
            roll = JOY_MIDDLE;
            _flight_mode =   fm_landing; // todo: create a feedforward return to volume
            break;
        } else
            _flight_mode =   fm_interception_aim;
        calc_directional_burn(state_drone,state_insect,interception_aim_time+tranmission_delay_time);
        std::cout << "fm_retry_aim_start: " << auto_roll_burn << ", "  << auto_pitch_burn << ", "  << auto_interception_burn_duration << std::endl;
        auto_pitch = auto_pitch_burn;
        auto_roll = auto_roll_burn;
        auto_throttle = 650; // TODO: LUDWIG HELP better hover throttle...
        //because can't fall through here:
        throttle = auto_throttle;
        pitch = auto_pitch;
        roll = auto_roll;
        break;
    } case fm_flying_pid : {
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

        throttle = auto_throttle ;
        roll = auto_roll;
        pitch = auto_pitch;
        break;
    } case fm_landing : {
        //slowly decrease throttle
        auto_throttle = hoverthrottle - (accErrY * gain_throttle_acc + throttleErrI * gain_throttle_i*0.1f);
        //same as fm_flying:
        auto_roll =  JOY_MIDDLE + (accErrX * gain_roll_acc +  gain_roll_i*rollErrI);
        auto_pitch = JOY_MIDDLE + (accErrZ * gain_pitch_acc +  gain_pitch_i*pitchErrI);
        throttle = auto_throttle ;
        roll = auto_roll;
        pitch = auto_pitch;
        break;
    } case fm_disarmed: {
        //reset integrators (prevent ever increasing error)
        rollErrI = 0;
        pitchErrI = 0;
        throttleErrI = 0;
        throttle = initial_throttle;
        roll = JOY_MIDDLE;
        pitch = JOY_MIDDLE;
        yaw = JOY_MIDDLE;
        _rc->arm(false);
        break;
    } case fm_inactive: {
        //reset integrators (prevent ever increasing error)
        rollErrI = 0;
        pitchErrI = 0;
        throttleErrI = 0;
        throttle = initial_throttle;
        roll = JOY_MIDDLE;
        pitch = JOY_MIDDLE;
        yaw = JOY_MIDDLE;
        if (dparams.mode3d)
            _rc->arm(false);
        else
            _rc->arm(true);
        break;
    } case fm_abort_takeoff : {
        auto_throttle = JOY_BOUND_MIN;
        throttle = auto_throttle;
        roll = JOY_MIDDLE;
        pitch = JOY_MIDDLE;
        break;
    } case fm_joystick_check: {
        //reset integrators (prevent ever increasing error)
        rollErrI = 0;
        pitchErrI = 0;
        throttleErrI = 0;
        throttle = initial_throttle;
        roll = JOY_MIDDLE;
        pitch = JOY_MIDDLE;
        yaw = JOY_MIDDLE;
        _rc->arm(false);
        break;
    }
    }

    yaw = joy_yaw; // tmp until auto yaw control is fixed #10

    throttle = bound_throttle(throttle);
    roll = bound_joystick_value(roll);
    pitch = bound_joystick_value(pitch);
    yaw = bound_joystick_value(yaw);

    //    std::cout << time <<  " rpt: " << roll << ", " << pitch << ", " << throttle << std::endl;
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

void DroneController::calc_directional_burn(track_data state_drone,track_data state_insect, float t_offset) {
    cv::Point3f drone_vel(state_drone.svelX,state_drone.svelY,state_drone.svelZ);
    calc_directional_burn(drone_vel,state_drone,state_insect,t_offset);
}

void DroneController::calc_directional_burn(track_data state_drone_start_1g, track_data state_drone,track_data state_insect, float t_offset) {
    //calculate the exact speed of the drone assuming it started from the start position with zero speed, some dt ago.
    cv::Point3f drone_vel;
    float dt =  state_drone.time - state_drone_start_1g.time;
    drone_vel.x = (state_drone.posX - state_drone_start_1g.posX) / dt;
    drone_vel.y = (state_drone.posY - state_drone_start_1g.posY) / dt;
    drone_vel.z = (state_drone.posZ - state_drone_start_1g.posZ) / dt;
    //approx_thrust_efficiency(drone_vel); //TODO: seems to give unreasonable numbers
    calc_directional_burn(drone_vel,state_drone,state_insect,t_offset);
}

void DroneController::calc_directional_burn(cv::Point3f drone_vel, track_data state_drone, track_data state_insect, float t_offset) {

    //predict drone location at the moment of the upcoming burn since the drone will have a delayed
    //reaction, so we need to calcuate this from that point on, which means we need to do a small
    //prediction step and acquire the new positions at that time
    cv::Point3f drone_pos_after_delay = cv::Point3f(state_drone.posX,state_drone.posY,state_drone.posZ) + t_offset * drone_vel;
    //same for insect:
    cv::Point3f insect_pos_after_delay = cv::Point3f(state_insect.posX,state_insect.posY,state_insect.posZ) +
                                         t_offset * cv::Point3f(state_insect.svelX,state_insect.svelY,state_insect.svelZ);

    //cv::Point3f drone_acc_during_aim = required_delta_v /norm(required_delta_v)*GRAVITY;
    //drone_acc_during_aim.y -= GRAVITY;
    //cv::Point3f drone_vel_during_aim = drone_acc_during_aim*t_offset*0.5f;
    //drone_pos_after_delay += 0.5f * drone_acc_during_aim *powf(t_offset*0.8f,2);

    cv::Point3f delta_pos = insect_pos_after_delay - drone_pos_after_delay;
    cv::Point3f burn_dist = delta_pos;

    float burn_time = sqrtf(2*norm(delta_pos)/drone_acc);
    cv::Point3f burn_accelleration = delta_pos/norm(delta_pos)*drone_acc;
    cv::Point3f drone_pos_after_burn = drone_pos_after_delay + drone_vel* burn_time + burn_dist;

    cv::Point3f delta_pos_after_burn = insect_pos_after_delay - drone_pos_after_burn;
    float delta_burn_time = sqrtf(2*norm(delta_pos_after_burn)/drone_acc);
    cv::Point3f delta_pos_drone_after_burn = drone_pos_after_burn-drone_pos_after_delay;

    if ( norm(delta_pos_drone_after_burn) > norm(delta_pos))
        burn_time -= delta_burn_time;
    else {
        burn_time += delta_burn_time;
    }
    min_tti = *tmp_min_tti;

    cv::Point3f drone_acc_during_aim = {0};

    for (int i = 0; i < 100; i++) {

        burn_dist += delta_pos_after_burn*0.1f;

        //burn_time = sqrtf(2*norm(burn_dist)/drone_acc);
        burn_accelleration = burn_dist/norm(burn_dist)*drone_acc;
        burn_dist = 0.5f*burn_accelleration*burn_time*burn_time;

        drone_acc_during_aim = burn_accelleration /norm(burn_accelleration)*GRAVITY;
        drone_acc_during_aim.y -= GRAVITY;

        drone_pos_after_burn = drone_pos_after_delay + (0.5f * drone_acc_during_aim *powf(t_offset*0.3f,2)) + drone_vel* burn_time + burn_dist;
        delta_pos_after_burn = insect_pos_after_delay - drone_pos_after_burn;

        delta_burn_time = sqrtf(2*norm(delta_pos_after_burn)/drone_acc);
        delta_pos_drone_after_burn = drone_pos_after_burn-drone_pos_after_delay;

        if ( norm(delta_pos_drone_after_burn) > norm(delta_pos))
            burn_time -= delta_burn_time*0.1f;
        else {
            burn_time += delta_burn_time*0.1f;
        }

        //cout << " burn_dist: " << burn_dist << endl;
        //cout << " burn_dist: " << norm(burn_dist) << endl;
        //cout << " burn_time: " << burn_time << endl;
        //cout << " delta_pos_after_burn: " << norm(delta_pos_after_burn) << endl;
        //cout << " " << endl;
    }

    //delta_pos = (0.5f * drone_acc_during_aim *powf(t_offset*0.5f,2));

    auto_interception_burn_duration = burn_time;
    //auto_interception_tti = min_tti;

    float insect_angle_roll =  atan2f(burn_accelleration.x,burn_accelleration.y);
    float insect_angle_pitch=  atan2f(-burn_accelleration.z,burn_accelleration.y);

    auto [commanded_roll,commanded_pitch] = approx_rp_command(insect_angle_roll,insect_angle_pitch);

    max_burn.x = commanded_roll / M_PIf32*180.f;
    max_burn.y = commanded_pitch / M_PIf32*180.f;

    if (max_burn.x>max_bank_angle)
        max_burn.x=max_bank_angle;
    else if (max_burn.x<-max_bank_angle)
        max_burn.x=-max_bank_angle;
    if (max_burn.y>max_bank_angle)
        max_burn.y=max_bank_angle;
    else if (max_burn.y<-max_bank_angle)
        max_burn.y=-max_bank_angle;

    auto_roll_burn =  ((max_burn.x/max_bank_angle+1) / 2.f) * JOY_MAX;
    auto_pitch_burn = ((max_burn.y/max_bank_angle+1) / 2.f) * JOY_MAX;

    viz_pos_after_aim =  drone_pos_after_delay;
    viz_time_after_aim = interception_burn_start_time + static_cast<double>(t_offset);
    viz_pos_after_burn = drone_pos_after_burn;
    viz_time_after_burn = viz_time_after_aim + static_cast<double>(burn_time);


}

void DroneController::calc_takeoff_aim_burn(track_data target,track_data drone, float tti,float t_offset) {

    tti = tti + t_offset;
    cv::Point3f est_target_pos,est_drone_pos;
    //estimated location of the insect after tti_tresh seconds, assuming constant v
    est_target_pos.x = target.posX + target.svelX*tti;
    est_target_pos.y = target.posY + target.svelY*tti;
    est_target_pos.z = target.posZ + target.svelZ*tti;

    //estimated location of the drone after tti_tresh seconds, with constant v
    est_drone_pos.x = drone.posX + drone.svelX*tti;
    est_drone_pos.y = drone.posY + drone.svelY*tti;
    est_drone_pos.z = drone.posZ + drone.svelZ*tti;

    calc_takeoff_aim_burn(est_target_pos,est_drone_pos);
}

void DroneController::calc_takeoff_aim_burn(cv::Point3f target, cv::Point3f drone) {
    float insect_angle_roll = atan2f((target.x - drone.x),(target.y - drone.y));
    float insect_angle_pitch = atan2f((-target.z - -drone.z),(target.y - drone.y));

    auto [commanded_roll,commanded_pitch] = approx_rp_command(insect_angle_roll,insect_angle_pitch);

    max_burn.x = commanded_roll / M_PIf32*180.f;
    max_burn.y = commanded_pitch / M_PIf32*180.f;

    if (max_burn.x>max_bank_angle)
        max_burn.x=max_bank_angle;
    else if (max_burn.x<-max_bank_angle)
        max_burn.x=-max_bank_angle;
    if (max_burn.y>max_bank_angle)
        max_burn.y=max_bank_angle;
    else if (max_burn.y<-max_bank_angle)
        max_burn.y=-max_bank_angle;

    float max_angle = commanded_roll;
    if (commanded_pitch > commanded_roll)
        max_angle = commanded_pitch;
    float dist = norm(target-drone);

    auto_interception_time_burn = sqrtf((2*dist)/drone_acc);
    auto_takeoff_time_burn = (0.05f+sqrtf(dist)/11 + (fabs(max_angle))/360) * 0.3f; //LUDWIG HELP! MODEL!
    auto_roll_burn =  ((max_burn.x/max_bank_angle+1) / 2.f) * JOY_MAX;
    auto_pitch_burn = ((max_burn.y/max_bank_angle+1) / 2.f) * JOY_MAX;

}

void DroneController::approx_thrust_efficiency(cv::Point3f drone_vel) {
    // UPDATE DRONE_ACC aka THRST-FORCE-EQUIVILENT with current velocity measurements:
    for(int i=0; i<2; i++){ // find drone_acc over multiple iterations
        float resulting_acc = sqrt(	pow(sin(max_burn.x*M_PIf32/180)*drone_acc,2)
                                   + pow(cos(max_burn.x*M_PIf32/180)*cos(max_burn.y*M_PIf32/180)*drone_acc-GRAVITY, 2)
                                   + pow(-cos(max_burn.x*M_PIf32/180)*sin(max_burn.y*M_PIf32/180)*drone_acc, 2) );
        float vel_est = resulting_acc * auto_takeoff_time_burn;
        drone_acc = (static_cast<float>(norm(drone_vel))/vel_est) * drone_acc;
    }
}

std::tuple<float,float> DroneController::approx_rp_command(float insect_angle_roll,float insect_angle_pitch) {
    float commanded_roll = insect_angle_roll, commanded_pitch = insect_angle_pitch;

    float insect_angle_roll_tmp, insect_angle_pitch_tmp;
    for(int i=0;i<3;i++){
        insect_angle_roll_tmp = atan2(sin(commanded_roll)*drone_acc,cos(commanded_roll)*drone_acc-GRAVITY);
        insect_angle_pitch_tmp = atan2(sin(commanded_pitch)*drone_acc,cos(commanded_pitch)*drone_acc-GRAVITY);
        commanded_roll *= insect_angle_roll/insect_angle_roll_tmp;
        commanded_pitch *= insect_angle_pitch/insect_angle_pitch_tmp;
    }
    commanded_roll *= (-1); // Adapt signs to our coordination system:
    return std::make_tuple(commanded_roll,commanded_pitch);
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
