#include "dronecontroller.h"
#include "defines.h"

#ifdef HASSCREEN
//#define TUNING
#endif

using namespace cv;

const std::string paramsFile = "../controlParameters.dat";

// Create an instance of Joystick
Joystick joystick("/dev/input/js0");
JoystickEvent event;

bool DroneController::joystick_ready(){
    return joystick.isFound();
}
void DroneController::init(std::ofstream *logger,bool fromfile, MultiModule * rc) {
    _rc = rc;
    _logger = logger;
    _fromfile = fromfile;
    (*_logger) << "valid; posErrX; posErrY; posErrZ; velX; velY; velZ; accX; accY; accZ; hoverthrottle; autoThrottle; autoRoll; autoPitch; autoYaw; joyThrottle; joyRoll; joyPitch; joyYaw; joySwitch; throttleP; throttleI; throttleD; dt; dx; dy; dz; velx_sp; vely_sp; velz_sp;";
    std::cout << "Initialising control." << std::endl;

    // Load saved control paremeters
    if (checkFileExist(paramsFile)) {
        std::ifstream is(paramsFile, std::ios::binary);
        cereal::BinaryInputArchive archive( is );
        try {
            archive(params);
        }catch (cereal::Exception e) {
            std::cout << "Dronecontroller settings file error: " << e.what() << std::endl;
            std::cout << "Maybe delete the file: " << paramsFile << std::endl;
            exit (1);
        }
        controlParameters tmp;
        if (params.version < tmp.version){
            std::cout << "Dronecontroller settings version too low!" << std::endl;
            std::cout << "Maybe delete the file: " << paramsFile << std::endl;
            throw my_exit(1);
        }
    }

#ifdef TUNING
    std::cout << "Creating control tuning window." << std::endl;
    // create GUI to set control parameters
    namedWindow("Control", WINDOW_NORMAL);

    // throttle control
    createTrackbar("Throttle Pos", "Control", &params.throttle_Pos, 3000);
    createTrackbar("Throttle Vel", "Control", &params.throttle_Vel, 1000);
    createTrackbar("Throttle Acc", "Control", &params.throttle_Acc, 100);
    createTrackbar("Throttle I", "Control", &params.throttleI, 100);

    createTrackbar("Take off factor", "Control", &params.autoTakeoffFactor, 255);
    createTrackbar("Hover offset", "Control", &params.hoverOffset, 100);

    // roll control
    createTrackbar("Roll Pos", "Control", &params.roll_Pos, 3000);
    createTrackbar("Roll Vel", "Control", &params.roll_Vel, 2000);
    createTrackbar("Roll Acc", "Control", &params.roll_Acc, 100);
    createTrackbar("Roll I", "Control", &params.rollI, 100);

    // pitch control
    createTrackbar("Pitch Pos", "Control", &params.pitch_Pos, 3000);
    createTrackbar("Pitch Vel", "Control", &params.pitch_Vel, 2000);
    createTrackbar("Pitch Acc", "Control", &params.pitch_Acc, 100);
    createTrackbar("Pitch I", "Control", &params.pitchI, 100);


    //    // yaw control
    //    createTrackbar("Yaw P", "Control", &params.yawP, 255);
    //    createTrackbar("Yaw I", "Control", &params.yawI, 255);
    //    createTrackbar("Yaw D", "Control", &params.yawD, 255);


#endif

}

int bound_joystick_value(int v) {
    if ( v < JOY_BOUND_MIN )
        v = JOY_BOUND_MIN;
    if ( v > JOY_BOUND_MAX )
        v = JOY_BOUND_MAX;
    return v;
}
void DroneController::queue_commands(int throttle,int roll, int pitch, int yaw) {
    if (!_fromfile) {
        _rc->g_lockData.lock();
        _rc->throttle = throttle;
        _rc->roll = roll;
        _rc->pitch = pitch;
        _rc->yaw = yaw;
        _rc->g_lockData.unlock();
    }
}

void DroneController::control(trackData data,cv::Point3f setpoint, cv::Point3f setpoint_v) {

    if (!_fromfile)
        readJoystick();
#ifndef INSECT_LOGGING_MODE
    process_joystick();
#endif


    // Roll Control - X
    posErrX = data.sposX - setpoint.x;              // position error
    velx_sp = posErrX*params.roll_Pos/1000.f;       // desired velocity
    velErrX = data.svelX + velx_sp + setpoint_v.x;  // velocity error
    accx_sp = velErrX*params.roll_Vel/100;          // desired acceleration
    accErrX = data.saccX + accx_sp;                 // acceleration error

    // Altitude Control - Y
    posErrY = data.sposY - setpoint.y;              // position error
    vely_sp = posErrY*params.throttle_Pos/1000.f;   // desired velocity
    velErrY = data.svelY + vely_sp + setpoint_v.y;  // velocity error
    accy_sp = velErrY*params.throttle_Vel/100;      // desired acceleration
    accErrY = data.saccY + accy_sp;                 // acceleration error

    // Pitch Control - Z
    posErrZ = data.sposZ - setpoint.z;              // position error
    velz_sp = posErrZ*params.pitch_Pos/1000.f;      // desired velocity
    velErrZ = data.svelZ + velz_sp + setpoint_v.z;  // velocity error
    accz_sp = velErrZ*params.pitch_Vel/100;         // desired acceleration
    accErrZ = data.saccZ + accz_sp;                 // acceleration error

    //when tuning, reset integrators when I gain set to 0:
    if (params.throttleI < 1)
        throttleErrI = 0;
    if (params.rollI < 1)
        rollErrI = 0;
    if (params.pitchI < 1)
        pitchErrI = 0;

    int throttle,roll,pitch,yaw;
    switch(_flight_mode) {
    case fm_manual : {
        //reset integrators (prevent ever increasing error)
        rollErrI = 0;
        pitchErrI = 0;
        throttleErrI = 0;

        throttle = joyThrottle;
        roll = joyRoll;
        pitch = joyPitch;
        yaw = joyYaw;
        break;
    } case fm_taking_off : {
        //reset integrators
        rollErrI = 0;
        pitchErrI = 0;
        throttleErrI = 0;

        autoThrottle = hoverthrottle;
        if (!hoverthrottleInitialized)
            hoverthrottle  +=params.autoTakeoffFactor;
        if (hoverthrottle < min_throttle)
            hoverthrottle = min_throttle;

        //only control throttle:
        throttle = autoThrottle ;
        roll = JOY_MIDDLE;
        pitch = PITCH_MIDDLE;
        break;
    } case fm_flying : {
        //update integrators
        if (abs(posErrX)<integratorThresholdDistance && abs(posErrZ)<integratorThresholdDistance)
            throttleErrI += velErrY; //posErrY;
        if (abs(posErrX)<integratorThresholdDistance)
            rollErrI += posErrX;
        if (abs(posErrZ)<integratorThresholdDistance)
            pitchErrI += posErrZ;

        autoThrottle =  hoverthrottle + take_off_throttle_boost - (accErrY * params.throttle_Acc + throttleErrI * params.throttleI*0.1f);
        autoRoll =  accErrX * params.roll_Acc;

        float depth_gain = 1;
        if (-data.sposZ - 2 > 0) // if further then 2 meters
            depth_gain  = 1 + powf((-data.sposZ-2),2) * depth_precision_gain;
        else // closer then 2 meters, do nothing:
            depth_gain = 1;

        autoPitch = accErrZ * params.pitch_Acc / depth_gain;

        autoThrottle += abs(autoRoll*throttleBankFactor)+abs(autoPitch*throttleBankFactor);

        autoRoll    += JOY_MIDDLE + (params.rollI*rollErrI);
        autoPitch   += PITCH_MIDDLE + (params.pitchI*pitchErrI);

        //int minThrottle = 1300 + min(abs(autoRoll-1500)/10,50) + min(abs(autoPitch-1500)/10,50);
        if (autoThrottle<min_throttle)
            autoThrottle = min_throttle;

        throttle = autoThrottle ;
        roll = autoRoll;
        pitch = autoPitch;
        break;
    } case fm_landing : {

        //fix integrators

        //slowly decrease throttle
        autoThrottle = hoverthrottle + take_off_throttle_boost - (accErrY * params.throttle_Acc + throttleErrI * params.throttleI*0.1f);

        //same as fm_flying:
        autoRoll =  JOY_MIDDLE + (accErrX * params.roll_Acc +  params.rollI*rollErrI);
        autoPitch = PITCH_MIDDLE + (accErrZ * params.pitch_Acc +  params.pitchI*pitchErrI);

        throttle = autoThrottle ;
        roll = autoRoll;
        pitch = autoPitch;
        break;
    } case fm_disarmed: {
        //reset integrators (prevent ever increasing error)
        rollErrI = 0;
        pitchErrI = 0;
        throttleErrI = 0;

        throttle = INITIALTHROTTLE;
        roll = JOY_MIDDLE;
        pitch = JOY_MIDDLE;
        yaw = JOY_MIDDLE;
        break;
    } case fm_inactive: {
        //reset integrators (prevent ever increasing error)
        rollErrI = 0;
        pitchErrI = 0;
        throttleErrI = 0;

        throttle = INITIALTHROTTLE;
        roll = JOY_MIDDLE;
        pitch = JOY_MIDDLE;
        yaw = JOY_MIDDLE;
        break;
    }
    }

    yaw = joyYaw; // tmp until auto yaw control is fixed

    throttle = bound_joystick_value(throttle);
    roll = bound_joystick_value(roll);
    pitch = bound_joystick_value(pitch);
    yaw = bound_joystick_value(yaw);

    queue_commands(throttle,roll,pitch,yaw);

    (*_logger) << static_cast<int>(data.valid)  << "; " << posErrX << "; " << posErrY  << "; " << posErrZ << "; " << setpoint.x << "; " << setpoint.y  << "; " << setpoint.z << "; " << accx_sp << "; " << accy_sp  << "; " << accx_sp << "; " << hoverthrottle << "; " << autoThrottle << "; " << autoRoll << "; " << autoPitch << "; " << autoYaw <<  "; " << joyThrottle <<  "; " << joyRoll <<  "; " << joyPitch <<  "; " << joyYaw << "; " << static_cast<int>(_joy_arm_switch) << "; " << params.throttle_Pos << "; " << params.throttleI << "; " << params.throttle_Acc << "; " << data.dt << "; " << data.dx << "; " << data.dy << "; " << data.dz << "; " << velx_sp << "; " << vely_sp << "; " << velz_sp << "; ";
}

int first_joy_time = 1;
void DroneController::readJoystick(void) {
    while (joystick.sample(&event))
    {
        if (event.isAxis()) {
            if (JOYSTICK_TYPE == RC_DEVO) {
                switch ( event.number ) {
                case 0: // roll
                    joyRoll = (event.value >> 5) + JOY_MIDDLE;
                    break;
                case 1: // pitch
                    joyPitch =  (event.value >> 5) + JOY_MIDDLE;
                    break;
                case 2: //throttle
                    joyThrottle =  (event.value >> 5) + JOY_MIDDLE - 100;
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
                    joyYaw =  (event.value >> 5) + JOY_MIDDLE;
                    break;
                default:
                    std::cout << "Unkown joystick event: " << std::to_string(event.number) << ". Value: " << std::to_string(event.value) << std::endl;
                    break;
                }
            } else if (JOYSTICK_TYPE == RC_USB_HOBBYKING) {
                switch ( event.number ) {
                case 0: // roll
                    joyRoll = JOY_MIDDLE + (event.value*(JOY_MAX-JOY_MIN)/33000);
                    break;
                case 1: // pitch
                    joyPitch = JOY_MIDDLE + (event.value*(JOY_MAX-JOY_MIN)/33000);
                    break;
                case 2: //throttle
                    joyThrottle = JOY_MIDDLE - (event.value*(JOY_MAX-JOY_MIN)/44000);
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
                    joyYaw = JOY_MIDDLE + (event.value*(JOY_MAX-JOY_MIN)/33000);
                    break;
                default:
                    std::cout << "Unkown joystick event: " << std::to_string(event.number) << ". Value: " << std::to_string(event.value) << std::endl;
                    break;
                }
            } else if (JOYSTICK_TYPE == RC_PLAYSTATION) {
                switch ( event.number ) {
                case 2: // roll
                    joyRoll = JOY_MIDDLE + (event.value >> 5);
                    std::cout << "roll" << std::endl;
                    break;
                case 3: // pitch
                    joyPitch = JOY_MIDDLE - (event.value >> 5);
                    std::cout << "pitch" << std::endl;
                    break;
                case 1: //throttle
                    joyThrottle = JOY_MIN - (event.value >> 5);
                    break;
                case 0: //yaw
                    joyYaw = JOY_MIDDLE + (event.value >> 5);
                    std::cout << "yaw" << std::endl;
                    break;
                default:
                    std::cout << "Unkown joystick event: " << std::to_string(event.number) << ". Value: " << std::to_string(event.value) << std::endl;
                    break;
                }
            } else if (JOYSTICK_TYPE == RC_XLITE) {
                switch ( event.number ) {
                case 0: // roll
                    joyRoll = (event.value >> 5)*0.8 + JOY_MIDDLE;
                    break;
                case 1: // pitch
                    joyPitch = (event.value >> 5)*0.8 + JOY_MIDDLE;
                    break;
                case 2: //throttle
                    joyThrottle =  (event.value >> 5)*0.8 + JOY_MIDDLE;
                    break;
                case 3: //yaw
                    joyYaw = (event.value >> 5)*0.8 + JOY_MIDDLE;
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
                    break;
                case 7: //switch (2 way)
                    if (_flight_mode == fm_inactive){
                        manual_override_take_off_now = event.value>0;
                    } else if (_flight_mode == fm_flying) {
                        manual_override_land_now = event.value<0;
                    }
                    break;
                default:
                    //this joystick seems to have 16 extra buttons... weird whatever
                    //                std::cout << "Unkown joystick event: " << std::to_string(event.number) << ". Value: " << std::to_string(event.value) << std::endl;
                    break;
                }
            } else if (event.isButton() && JOYSTICK_TYPE == RC_PLAYSTATION) {
                switch ( event.number ) {
                case 2: //bind
                    if (event.value>0) {
                        _joy_arm_switch = 1;
                        joyPitch = JOY_MAX_THRESH;
                    } else {
                        _joy_arm_switch = 0;
                        joyPitch = JOY_MIDDLE;
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
    if (!joystick.isFound())
        return;
    if (first_joy_time > 0) {
        if (_joy_arm_switch || joyThrottle > JOY_MIN_THRESH ) {
            if (_joy_arm_switch)
                std::cout << "Joystick Arm Switch ACTIVE!" << std::endl;
            if (joyThrottle > JOY_MIN_THRESH )
                std::cout << "Joystick Throttle NON-ZERO!" << std::endl;
            _flight_mode = fm_disarmed;
            _joy_state = js_disarmed;
            _rc->arm(false);
        } else {
            first_joy_time--;
        }
    } else {

        if  (JOYSTICK_TYPE == RC_USB_HOBBYKING) {
            //check switch functions
            static bool joySwitch_prev = _joy_arm_switch;
            if (_joy_arm_switch && !joySwitch_prev) {
                if (joyPitch > JOY_MAX_THRESH) {
                    _rc->bind(true);
                    _joy_arm_switch = false;
                } else if(joyThrottle < JOY_MIN + 100) {
                    if (joyPitch < JOY_MIN + 200) {
                        _flight_mode = fm_inactive;
                        if (joyRoll > JOY_MAX_THRESH)
                            manual_override_take_off_now = true;
                    }
                }
            } else if (!_joy_arm_switch && joySwitch_prev && !_fromfile) {
                _joy_mode_switch = jmsm_manual;
            }
            joySwitch_prev = _joy_arm_switch;
        } else if  (JOYSTICK_TYPE == RC_XLITE) {
            if (!_joy_arm_switch){
                _rc->arm(false);
                _joy_state = js_disarmed;
                _flight_mode = fm_disarmed;
            }else {
                _rc->arm(true);
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
    hoverthrottle = hoverthrottle - (throttleErrI*params.throttleI*0.1f);
    throttleErrI = 0;
    hoverthrottleInitialized = true;
}

void DroneController::close () {
    std::cout << "closing controller" << std::endl;
    std::ofstream os(paramsFile, std::ios::binary);
    cereal::BinaryOutputArchive archive( os );
    archive( params );
}
