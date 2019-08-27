#include "dronecontroller.h"
#include "defines.h"

using namespace cv;

// Create an instance of Joystick
Joystick joystick("/dev/input/js0");
JoystickEvent event;

bool DroneController::joystick_ready(){
    return joystick.isFound();
}
std::ofstream logger_tmp;
void DroneController::init(std::ofstream *logger,bool fromfile, MultiModule * rc, DroneTracker *dtrk) {
    _rc = rc;
    _dtrk = dtrk;
    _logger = logger;
    _fromfile = fromfile;
    control_history_max_size = pparams.fps;
    (*_logger) << "valid; posErrX; posErrY; posErrZ; velX; velY; velZ; accX; accY; accZ; hoverthrottle; autoThrottle; autoRoll; autoPitch; autoYaw; joyThrottle; joyRoll; joyPitch; joyYaw; joyArmSwitch; joyModeSwitch; joyTakeoffSwitch; dt; dx; dy; dz; velx_sp; vely_sp; velz_sp;";
    std::cout << "Initialising control." << std::endl;

    logger_tmp.open("../drone.csv",std::ofstream::app);

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

void DroneController::control(track_data state_drone,track_data state_insect, cv::Point3f setpoint_pos, cv::Point3f setpoint_vel, cv::Point3f setpoint_acc) {

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
        take_off_burn_start_time = state_drone.time;
        //reset integrators
        rollErrI = 0;
        pitchErrI = 0;
        throttleErrI = 0;
        if (dparams.mode3d)
            _rc->arm(true);
        throttle = JOY_BOUND_MIN;
        _flight_mode = fm_take_off_aim;
        tti_prev = 999999;
        break;
    } FALLTHROUGH_INTENDED; case fm_take_off_aim : {
        auto_throttle = 600; // TODO: hover throttle...

        calc_burn_direction(setpoint_pos, _dtrk->drone_startup_location());
        auto_pitch = auto_pitch_burn;
        auto_roll = auto_roll_burn;
        pitch =  auto_pitch;
        roll = auto_roll;
        throttle = auto_throttle;

        if (static_cast<float>(state_drone.time - take_off_burn_start_time) > dparams.full_bat_and_throttle_spinup_time)
            _flight_mode = fm_max_burn;
        break;

    } case fm_max_burn : {

        auto_throttle = JOY_BOUND_MAX;

        pitch =  auto_pitch;
        roll = auto_roll;
        throttle = auto_throttle;

        if (static_cast<float>(state_drone.time - take_off_burn_start_time) > dparams.full_bat_and_throttle_spinup_time+ auto_takeoff_time_burn)
            _flight_mode = fm_1g;
        break;
    }  case fm_1g: {
        auto_throttle = 600; // dparams.min_throttle;
        //only control throttle:
        throttle = auto_throttle;
        pitch =  auto_pitch;
        roll = auto_roll;

        if (!drone_1g_start_pos.pos_valid)
            drone_1g_start_pos = state_drone;
        else {

            //the drone is traveling at a vector vd m/s, the insect with vi
            //Assuming that the drone travels at a speed ||vd|| towards where the insect will be in tti seconds,
            //calculate optimal tti:
            //1. calculate virtual distance: (do something with a cirkel) (pro tip from friday night) (so must be right)

            //2.

            //

            float tti = calc_1g_tti(drone_1g_start_pos, state_drone,state_insect);
            std::cout << tti << std::endl;
//            if (tti_prev < tti){
            if (static_cast<float>(state_drone.time - take_off_burn_start_time) > dparams.full_bat_and_throttle_spinup_time+ auto_takeoff_time_burn + 6.f / pparams.fps) {
                _flight_mode = fm_interception_aim_start;

                //TMP drone.csv print:
                float angle_roll = atanf((state_drone.posX - _dtrk->drone_startup_location().x)/(_dtrk->drone_startup_location().y -state_drone.posY));
                float angle_pitch= atanf((state_drone.posZ - _dtrk->drone_startup_location().z)/(_dtrk->drone_startup_location().y -state_drone.posY));
                angle_roll *= 180.f / M_PIf32;
                angle_pitch *= 180.f / M_PIf32;
                logger_tmp << state_drone.time << ";" << max_burn.x << ";" << max_burn.y << ";" <<  _dtrk->drone_startup_location().x << ";"  << _dtrk->drone_startup_location().y << ";"  <<  _dtrk->drone_startup_location().z << ";"  << state_drone.posX << ";" << state_drone.posY << ";" << state_drone.posZ << ";" << angle_roll << ";" << angle_pitch << std::endl;
            }
//            tti_prev = tti;
        }

        if (static_cast<float>(state_drone.time - take_off_burn_start_time) > 20) //TODO improve this by also using tti and time_to_exit_safe_FOV
            _flight_mode = fm_flying;

        break;
    }  case fm_interception_aim_start: {
        interception_burn_start_time = state_drone.time;
        tti_prev = 999999;
        _flight_mode =   fm_interception_aim;
        kevin_burn(drone_1g_start_pos, state_drone,state_insect,tti_early_bird);
        std::cout << "kevin: " << auto_roll_burn << ", "  << auto_pitch_burn << ", "  << auto_interception_time_burn << std::endl;

        ludwig_burn( drone_1g_start_pos, state_insect,state_drone,tti_early_bird);
        std::cout << "Ludwig: " << auto_roll_burn << ", "  << auto_pitch_burn << ", "  << auto_interception_time_burn << std::endl;

        auto_pitch = auto_pitch_burn;
        auto_roll = auto_roll_burn;
    }  FALLTHROUGH_INTENDED; case fm_interception_aim: {
        //float tti = calc_tti(state_drone,state_insect);
        //calc_burn_direction(state_insect,state_drone,tti,interception_aim_time+tranmission_delay_time);

        pitch = auto_pitch;
        roll = auto_roll;
        auto_throttle = 600; // TODO: hover throttle...
        throttle = auto_throttle;

        if (state_drone.time - interception_burn_start_time > interception_aim_time) // TODO make 0.05 dynamic based on d_att using betaflight angle strength
            _flight_mode = fm_interception_burn_start;
        break;
    } case fm_interception_burn_start : {
        interception_burn_duration = auto_interception_time_burn;
        _flight_mode = fm_interception_burn; 
        pitch = auto_pitch;
        roll = auto_roll;
        auto_throttle = JOY_BOUND_MAX;
        throttle = auto_throttle;
    } FALLTHROUGH_INTENDED; case fm_interception_burn : {
        pitch = auto_pitch;
        roll = auto_roll;
        throttle = auto_throttle;
        if (state_drone.time - interception_burn_start_time > interception_burn_duration)
            _flight_mode = fm_interception_1g;
        break;
    } case fm_interception_1g: {
        //only control throttle:
        pitch = auto_pitch;
        roll = auto_roll;
        auto_throttle = 600; // dparams.min_throttle;
        throttle = auto_throttle;

        float tti = calc_tti(state_drone,state_insect);
        calc_burn_direction(state_insect,state_drone,tti,0);

        if (tti_prev < tti){
//            _flight_mode = fm_interception_aim_start;
        }
        tti_prev = tti;

         if (state_drone.time - interception_burn_start_time > 1+ interception_burn_duration) // TODO: detect if we got it. //TMP 1+
            _flight_mode = fm_flying;

        break;
    } case fm_flying : {
        //update integrators
        if (fabs(posErrZ)<integratorThresholdDistance)
            throttleErrI += velErrY; //posErrY;
        if (fabs(posErrX)<integratorThresholdDistance)
            rollErrI += posErrX;
        if (fabs(posErrZ)<integratorThresholdDistance)
            pitchErrI += posErrZ;

        auto_throttle =  hoverthrottle + take_off_throttle_boost - (accErrY * gain_throttle_acc + throttleErrI * gain_throttle_i*0.1f);
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
        auto_throttle = hoverthrottle + take_off_throttle_boost - (accErrY * gain_throttle_acc + throttleErrI * gain_throttle_i*0.1f);
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

    std::cout << roll << " , "  << pitch << " , " << throttle << std::endl;

    if (!_fromfile) {
        _rc->queue_commands(throttle,roll,pitch,yaw);
    }

    control_data c(Throttle(),Roll(),Pitch(),state_drone.time);
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

void DroneController::ludwig_burn (track_data state_drone_start_1g,track_data target,track_data drone, float t_offset) {

    cv::Point3f drone_vel_stageI;
    float dt =  drone.time - state_drone_start_1g.time;
    drone_vel_stageI.x = (drone.posX - state_drone_start_1g.posX) / dt;
    drone_vel_stageI.y = (drone.posY - state_drone_start_1g.posY) / dt;
    drone_vel_stageI.z = (drone.posZ - state_drone_start_1g.posZ) / dt;

    cv::Point3f est_target_pos,est_drone_pos;

    //estimated drone location when it is ready for the second burn (drone has finished to turn)
    est_drone_pos.x = drone.posX + drone_vel_stageI.x*t_offset;
    est_drone_pos.y = drone.posY + drone_vel_stageI.y*t_offset;
    est_drone_pos.z = drone.posZ + drone_vel_stageI.z*t_offset;
    //estimated insect location at the same time for tti estimation
    est_target_pos.x = target.posX + target.svelX*t_offset;
    est_target_pos.y = target.posY + target.svelY*t_offset;
    est_target_pos.z = target.posZ + target.svelZ*t_offset;
    // estimate time-to-intercept based on the current positions and drone velocity
    float tti = norm(est_target_pos-est_drone_pos)/norm(drone_vel_stageI);
    //Overwrite target position with the position of the interception:
    est_target_pos.x += target.svelX*tti;
    est_target_pos.y += target.svelY*tti;
    est_target_pos.z += target.svelZ*tti;
    // Direction the drone shall continue to fly to
    cv::Point3f targetDirectionStageII = cv::Point3f(est_target_pos.x - est_drone_pos.x,
                                                     est_target_pos.y - est_drone_pos.y,
                                                     est_target_pos.z - est_drone_pos.z);
    targetDirectionStageII /=norm(targetDirectionStageII);
    // Assuming to go in the new direction with the same speed as before:
    // -> Velocity correction
    cv::Point3f vel_commandedCorrectionStageII = targetDirectionStageII*norm(drone_vel_stageI) - drone_vel_stageI;
    // Generate artificial point that generates the corrected control values:
    cv::Point3f artifical_target_pos = Point3f(est_drone_pos.x + tti*vel_commandedCorrectionStageII.x,
                                               est_drone_pos.y + tti*vel_commandedCorrectionStageII.y,
                                               est_drone_pos.z + tti*vel_commandedCorrectionStageII.z);
    // Determine control inputs..
    calc_burn_direction(artifical_target_pos, est_drone_pos);
}

void DroneController::kevin_burn(track_data state_drone_start_1g, track_data state_drone,track_data state_insect, float t_offset) {

    //This function calcs a directional correction burn such that the drone, which is on a path close to the insect,
    //obtains a new optimal path precisly hitting the insect. Basically:
    //drone_vel_opti = drone_vel + drone_vel_correction ->
    //drone_vel_correction = drone_vel_opti - drone_vel

    //drone_vel: (calculate the exact speed of the drone assuming it started from the start position, some dt ago)
    cv::Point3f drone_vel;
    float dt =  state_drone.time - state_drone_start_1g.time;
    drone_vel.x = (state_drone.posX - state_drone_start_1g.posX) / dt;
    drone_vel.y = (state_drone.posY - state_drone_start_1g.posY) / dt;
    drone_vel.z = (state_drone.posZ - state_drone_start_1g.posZ) / dt;

    //drone_vel_opti: (the velocity we would like the drone to have had)
    cv::Point3f drone_vel_opti;
    //small problem, the drone will have a delayed reaction, so we need to calcuate this from that point on,
    //which means we need to do a small prediction step and acquire the new positions at that time
    cv::Point3f drone_pos_after_delay;
    drone_pos_after_delay.x = state_drone.posX + t_offset * drone_vel.x;
    drone_pos_after_delay.y = state_drone.posY + t_offset * drone_vel.y;
    drone_pos_after_delay.z = state_drone.posZ + t_offset * drone_vel.z;
    //same for insect:
    cv::Point3f insect_pos_after_delay;
    insect_pos_after_delay.x = state_insect.posX + t_offset * state_insect.svelX;
    insect_pos_after_delay.y = state_insect.posY + t_offset * state_insect.svelY;
    insect_pos_after_delay.z = state_insect.posZ + t_offset * state_insect.svelZ;

    //Use the delta position for the direction of the vector...
    cv::Point3f delta_pos = cv::Point3f(insect_pos_after_delay.x - drone_pos_after_delay.x,
                                   insect_pos_after_delay.y - drone_pos_after_delay.y,
                                   insect_pos_after_delay.z - drone_pos_after_delay.z);
    cv::Point3f delta_pos_norm = delta_pos / norm(delta_pos); // normalize to unit
    //...and use the known magnitude of drone_vel for the new drone_vel_opti magnitude as well (we assume the magnitude was fine already)
    drone_vel_opti = delta_pos_norm * norm(drone_vel);
    //calculate the tti if the current speed vector was directly towards the insect:
    float virtual_tti = norm(delta_pos) / norm(drone_vel_opti) ; // -> t = x / v
    //virtual_tti += t_offset ; // because delta_pos is predicted with t_offset into the future

    //drone_vel_correction:
    cv::Point3f drone_vel_correction = drone_vel_opti-drone_vel;

    //optimization 1, calculate a more precise magnitude of the correction.
    //calc the distance we actually need to fix, which is the point on our current path closest to the insect to the insect
    //TODO: insect is a moving target as well
    cv::Point3f x0 = delta_pos;
    cv::Point3f x2 = drone_vel*100,f; //just elongate the drone speed direction with a large number to get the line far enough out
    float dist = norm(x0.cross(-x2))/norm(x2);

    //this distance needs to be covered in the time to impact, which means the correction speed magnitude must be roughly:
    float mag_v_cor = dist/virtual_tti;
    drone_vel_correction = (drone_vel_correction / norm(drone_vel_correction)) * mag_v_cor;

    //this means the resulting speed will become:
    cv::Point3f drone_vel_result = drone_vel_correction + drone_vel;
    //which means the actual tti becomes:
    float new_dist = norm(x0.cross(-drone_vel_result*100))/norm(drone_vel_result*100); // hmm, I guess this is not exactly zero, because virtual_tti is not exactly to the target
    float new_tti =  static_cast<float>(norm(delta_pos) / norm(drone_vel_result)); // + t_offset;

    //optimization 2, assuming we are still before the target, we want to do a correctional boost without braking,
    //so for each v axis check if the sign is the same and calculate a boost factor towards the optimal speed to get that to zero
    //TODO: not sure if the boost calculation makes sense
    float boostf = 0;
    if (drone_vel_correction.x * drone_vel_opti.x < 0.0f){
        boostf = drone_vel_opti.x / (drone_vel_opti.x-drone_vel_correction.x);
    }
    if (drone_vel_correction.y * drone_vel_opti.y < 0.0f){
        float tmp_boostf = drone_vel_opti.y / (drone_vel_opti.y-drone_vel_correction.y);
        if (fabs(tmp_boostf) > fabs(boostf))
            boostf = tmp_boostf;
    }
    if (drone_vel_correction.z * drone_vel_opti.z < 0.0f){
        float tmp_boostf = drone_vel_correction.z / drone_vel_opti.z;
        if (fabs(tmp_boostf) > fabs(boostf))
            boostf = tmp_boostf;
    }

    cv::Point3f drone_vel_correction_boosted;
    drone_vel_correction_boosted =  drone_vel_correction -  drone_vel_opti * boostf;

    cv::Point3f drone_vel_result_boosted = drone_vel_correction_boosted + drone_vel;

    float new_dist_boosted = norm(x0.cross(-drone_vel_result_boosted*100))/norm(drone_vel_result_boosted*100);
    float new_tti_boosted =  static_cast<float>(norm(delta_pos) / norm(drone_vel_result_boosted)); // + t_offset;

    //put a constraint on the boost, such that new_dist_boosted does not get bigger than the drone:
    if (new_dist_boosted > 0.1f && new_dist < 0.1f) {
        drone_vel_correction_boosted = drone_vel_correction; // use the original value and hope for the best...
    }
    //in theory the boost should always be faster then the non-boost:
    if (new_tti_boosted > new_tti) {
        drone_vel_correction_boosted = drone_vel_correction; // use the original value and hope for the best...
    }

    //TODO: calcuate how long the burn should be. Or even better, what throttle we can apply such that we gradually
    //approach the insect (and we may even regulate a bit underway)

    //TODO: below is largely similar to calc_burn_direction and can be streamlined.
    float insect_angle_roll =  -atan2f(drone_vel_correction_boosted.x,drone_vel_correction_boosted.y);
    float insect_angle_pitch=  -atan2f(drone_vel_correction_boosted.z,drone_vel_correction_boosted.y);

    float commanded_roll ,commanded_pitch;
    commanded_roll = ((M_PIf32/2.f - fabs(insect_angle_roll))* (drone_acc+GRAVITY)+0.5f*M_PIf32 * GRAVITY)/(drone_acc + 2 * GRAVITY);
    commanded_pitch =((M_PIf32/2.f - fabs(insect_angle_pitch))*(drone_acc+GRAVITY)+0.5f*M_PIf32 * GRAVITY)/(drone_acc + 2 * GRAVITY);

    commanded_roll = M_PIf32/2.f - commanded_roll;
    commanded_pitch= M_PIf32/2.f - commanded_pitch;

    if (insect_angle_roll < 0)
        commanded_roll = -commanded_roll;
    if (insect_angle_pitch < 0)
        commanded_pitch = -commanded_pitch;

    max_burn.x = commanded_roll / M_PIf32*180.f;
    max_burn.y = commanded_pitch / M_PIf32*180.f;


    float bank_angle = 180; // todo: move to xml (betaflight setting)
    if (max_burn.x>bank_angle)
        max_burn.x=bank_angle;
    else if (max_burn.x<-bank_angle)
        max_burn.x=-bank_angle;
    if (max_burn.y>bank_angle)
        max_burn.y=bank_angle;
    else if (max_burn.y<-bank_angle)
        max_burn.y=-bank_angle;

    auto_roll_burn =  ((max_burn.x/bank_angle+1) / 2.f) * JOY_MAX;
    auto_pitch_burn = ((max_burn.y/bank_angle+1) / 2.f) * JOY_MAX;

    //we know that the burn time during take off resulted in the current state, so we want to have another burn with exactly the same magnitude, which means again taking that same burn time:
    auto_interception_time_burn = auto_takeoff_time_burn;

}


float DroneController::calc_tti(track_data state_drone,track_data state_insect){

    cv::Point3f insect_pos,insect_vel,drone_pos,drone_vel;
    insect_pos.x = state_insect.posX;
    insect_pos.y = state_insect.posY;
    insect_pos.z = state_insect.posZ;
    insect_vel.x = state_insect.svelX;
    insect_vel.y = state_insect.svelY;
    insect_vel.z = state_insect.svelZ;

    drone_pos.x = state_drone.posX;
    drone_pos.y = state_drone.posY;
    drone_pos.z = state_drone.posZ;
    drone_vel.x = state_drone.svelX;
    drone_vel.y = state_drone.svelY;
    drone_vel.z = state_drone.svelZ;

    //basic physics:
    //x = 0.5at² t = sqrt((2x)/a)
    //x = vt t = x/v
    //v = at t = v/a

    float ic_dx = norm(insect_pos-drone_pos);
    float ic_dv = norm(insect_vel-drone_vel);

    float tti = ic_dx / ic_dv;
    return tti;
}
float DroneController::calc_1g_tti(track_data state_drone_start_1g, track_data state_drone,track_data state_insect){

    cv::Point3f insect_pos,insect_vel,drone_pos,drone_vel;
    insect_vel.x = state_insect.svelX;
    insect_vel.y = state_insect.svelY;
    insect_vel.z = state_insect.svelZ;

    insect_pos.x = state_insect.posX + state_insect.svelX * tti_early_bird;
    insect_pos.y = state_insect.posY + state_insect.svelY * tti_early_bird;
    insect_pos.z = state_insect.posZ + state_insect.svelZ * tti_early_bird;

    float dt =  state_drone.time - state_drone_start_1g.time;
    if (dt < 4.f/pparams.fps)
        return 999; // to prevent unfiltered output

    drone_vel.x = (state_drone.posX - state_drone_start_1g.posX) / dt;
    drone_vel.y = (state_drone.posY - state_drone_start_1g.posY) / dt;
    drone_vel.z = (state_drone.posZ - state_drone_start_1g.posZ) / dt;

    //std::cout << "v 1g: " << drone_vel.x << ", "  << drone_vel.y << ", " << drone_vel.z <<std::endl;


    drone_pos.x = state_drone.posX + tti_early_bird*drone_vel.x;
    drone_pos.y = state_drone.posY + tti_early_bird*drone_vel.y;
    drone_pos.z = state_drone.posZ + tti_early_bird*drone_vel.z;

	// UPDATE DRONE_ACC aka THRST-FORCE-EQUIVILENT with current velocity measurements:
	for(int i=0; i<2; i++){ // find drone_acc over multiple iterations
		float resulting_acc = sqrt(	pow(sin(max_burn.x*M_PIf32/180)*drone_acc,2)
									+ pow(cos(max_burn.x*M_PIf32/180)*cos(max_burn.y*M_PIf32/180)*drone_acc-GRAVITY, 2)
									+ pow(-cos(max_burn.x*M_PIf32/180)*sin(max_burn.y*M_PIf32/180)*drone_acc, 2) );
        float vel_est = resulting_acc * auto_takeoff_time_burn;
        drone_acc = (static_cast<float>(norm(drone_vel))/vel_est) * drone_acc;
	}


    //basic physics:
    //x = 0.5at² t = sqrt((2x)/a)
    //x = vt t = x/v
    //v = at t = v/a

    float ic_dx = norm(insect_pos-drone_pos);
    float ic_dv = norm(insect_vel-drone_vel);

    float tti = ic_dx / ic_dv;
    return tti;
}


void DroneController::calc_burn_during_1g(track_data drone_start_1g, track_data drone,track_data target, float t_offset) {


    cv::Point3f insect_pos,insect_vel,drone_pos,drone_vel;
    insect_vel.x = target.svelX;
    insect_vel.y = target.svelY;
    insect_vel.z = target.svelZ;

    insect_pos.x = target.posX + target.svelX * t_offset;
    insect_pos.y = target.posY + target.svelY * t_offset;
    insect_pos.z = target.posZ + target.svelZ * t_offset;

    float dt =  drone.time - drone_start_1g.time;

    drone_vel.x = (drone.posX - drone_start_1g.posX) / dt;
    drone_vel.y = (drone.posY - drone_start_1g.posY) / dt;
    drone_vel.z = (drone.posZ - drone_start_1g.posZ) / dt;

    drone_pos.x = drone.posX + t_offset*drone_vel.x;
    drone_pos.y = drone.posY + t_offset*drone_vel.y;
    drone_pos.z = drone.posZ + t_offset*drone_vel.z;

    float ic_dx = norm(insect_pos-drone_pos);
    float ic_dv = norm(insect_vel-drone_vel);

    float tti = ic_dx / ic_dv; // basically this is the max time the drone has do a correction burn, otherwise we overshoot for sure


    cv::Point3f est_target_pos,est_drone_pos;
    //estimated location of the insect after tti_tresh seconds, assuming constant v
    est_target_pos.x = target.posX + target.svelX*tti;
    est_target_pos.y = target.posY + target.svelY*tti;
    est_target_pos.z = target.posZ + target.svelZ*tti;

    //estimated location of the drone after tti_tresh seconds, with constant v
    est_drone_pos.x = drone.posX + drone_vel.x*tti;
    est_drone_pos.y = drone.posY + drone_vel.y*tti;
    est_drone_pos.z = drone.posZ + drone_vel.z*tti;


//    std::cout << "v: " << drone.svelX << ", "  << drone.svelY << ", " << drone.svelZ <<std::endl;

    calc_burn_direction(est_target_pos,est_drone_pos);
}

void DroneController::calc_burn_direction(track_data target,track_data drone, float tti,float t_offset) {

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


//    std::cout << "v: " << drone.svelX << ", "  << drone.svelY << ", " << drone.svelZ <<std::endl;

    calc_burn_direction(est_target_pos,est_drone_pos);
}

void DroneController::calc_burn_direction(cv::Point3f target,cv::Point3f drone) {

    float insect_angle_roll =  -atan2f((target.x - drone.x),(target.y - drone.y));
    float insect_angle_pitch=  -atan2f((target.z - drone.z),(target.y - drone.y));

    float commanded_roll ,commanded_pitch;
	commanded_roll = ((M_PIf32/2.f - fabs(insect_angle_roll))* (drone_acc+GRAVITY)+0.5f*M_PIf32 * GRAVITY)/(drone_acc + 2 * GRAVITY);
	commanded_pitch =((M_PIf32/2.f - fabs(insect_angle_pitch))*(drone_acc+GRAVITY)+0.5f*M_PIf32 * GRAVITY)/(drone_acc + 2 * GRAVITY);

    commanded_roll = M_PIf32/2.f - commanded_roll;
    commanded_pitch= M_PIf32/2.f - commanded_pitch;

    if (insect_angle_roll < 0)
        commanded_roll = -commanded_roll;
    if (insect_angle_pitch < 0)
        commanded_pitch = -commanded_pitch;

    max_burn.x = commanded_roll / M_PIf32*180.f;
    max_burn.y = commanded_pitch / M_PIf32*180.f;


    float bank_angle = 180; // todo: move to xml (betaflight setting)
    if (max_burn.x>bank_angle)
        max_burn.x=bank_angle;
    else if (max_burn.x<-bank_angle)
        max_burn.x=-bank_angle;
    if (max_burn.y>bank_angle)
        max_burn.y=bank_angle;
    else if (max_burn.y<-bank_angle)
        max_burn.y=-bank_angle;


    //std::cout << "commanded angle: " << max_burn.x << ", "  << max_burn.y  <<std::endl;


    float max_angle = commanded_roll;
    if (commanded_pitch > commanded_roll)
        max_angle = commanded_pitch;
    float dist = norm(target-drone);

    auto_interception_time_burn = sqrtf((2*dist)/drone_acc);
    auto_takeoff_time_burn = (0.05f+sqrtf(dist)/8 + (fabs(max_angle))/360) * 0.65f;
    auto_roll_burn =  ((max_burn.x/bank_angle+1) / 2.f) * JOY_MAX;
    auto_pitch_burn = ((max_burn.y/bank_angle+1) / 2.f) * JOY_MAX;

    //        auto_roll = JOY_MIDDLE; //TMP WAYPOINT MODE
    //        auto_pitch = JOY_MIDDLE;
    //        auto_burn_time = 0.05;

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
                    joy_roll = (event.value >> 5)*0.4 + JOY_MIDDLE;
                    break;
                case 1: // pitch
                    joy_pitch = (event.value >> 5)*0.4 + JOY_MIDDLE;
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

void DroneController::recalibrateHover() {
    hoverthrottle = hoverthrottle - (throttleErrI*gain_throttle_i*0.1f);
    throttleErrI = 0;
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
