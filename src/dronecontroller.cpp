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

    if (pparams.drone == drone_trashcan || pparams.drone == drone_none)
        settings_file = "../control_tc.xml";
    else if (pparams.drone == drone_tinywhoop_black || pparams.drone == drone_tinywhoop_green)
        settings_file = "../control_tw.xml";
    else
        throw my_exit("No settings file found for drone type");

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
    if ( v < JOY_BOUND_MIN )
        v = JOY_BOUND_MIN;
    if ( v > JOY_BOUND_MAX )
        v = JOY_BOUND_MAX;
    return v;
}

void DroneController::control(track_data data,cv::Point3f setpoint_pos, cv::Point3f setpoint_vel, cv::Point3f setpoint_acc) {

    if (!_fromfile)
        readJoystick();
    if (!pparams.insect_logging_mode)
        process_joystick();


    // Roll Control - X
    posErrX = data.sposX - setpoint_pos.x;              // position error
    if (posErrX>3.0f)
        posErrX = 3.0f;
    if (posErrX<-3.0f)
        posErrX = -3.0f;
    velx_sp = posErrX*gain_roll_pos/1000.f;           // desired velocity
    velErrX = data.svelX + velx_sp - setpoint_vel.x;    // velocity error
    accx_sp = velErrX*gain_roll_vel/100;              // desired acceleration
    accErrX = data.saccX + accx_sp - setpoint_acc.x;    // acceleration error

    // Altitude Control - Y
    posErrY = data.sposY - setpoint_pos.y;              // position error
    vely_sp = posErrY*gain_throttle_pos/1000.f;       // (inversed) desired velocity
    velErrY = data.svelY + vely_sp - setpoint_vel.y;    // velocity error
    accy_sp = velErrY*gain_throttle_vel/100;          // (inversed) desired acceleration
    accErrY = data.saccY + accy_sp;// - setpoint_acc.y;    // acceleration error

    float tmptbf = dparams.throttle_bank_factor; // if we are higher then the target, use the fact more attitude makes us go down
    if (posErrY< 0) {
        tmptbf = 0;
    }

    // Pitch Control - Z
    posErrZ = data.sposZ - setpoint_pos.z;              // position error
    if (posErrZ>1.5f)
        posErrZ = 1.5f;
    if (posErrZ<-1.5f)
        posErrZ = -1.5f;
    velz_sp = posErrZ*gain_pitch_pos/1000.f;          // desired velocity
    velErrZ = data.svelZ + velz_sp - setpoint_vel.z;    // velocity error
    accz_sp = velErrZ*gain_pitch_vel/100;             // desired acceleration
    accErrZ = data.saccZ + accz_sp - setpoint_acc.z;    // acceleration error

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
        throttle = joyThrottle;
        autoThrottle = JOY_BOUND_MIN;
        _rc->arm(_joy_arm_switch);
        roll = joyRoll;
        pitch = joyPitch;
        yaw = joyYaw;
        break;
    } case fm_start_takeoff : {
        take_off_start_time = data.time;
        //reset integrators
        rollErrI = 0;
        pitchErrI = 0;
        throttleErrI = 0;
        _rc->arm(true);
        throttle = JOY_BOUND_MIN;
        _flight_mode = fm_take_off_max_burn;
        break;
    } FALLTHROUGH_INTENDED; case fm_take_off_max_burn : {
        //only control throttle:
        autoThrottle = JOY_BOUND_MAX;
        throttle = autoThrottle;
        roll = JOY_MIDDLE;
        pitch = JOY_MIDDLE;
        if (data.time - take_off_start_time > dparams.max_burn_time)
            _flight_mode = fm_take_off_idle;
        break;
    } case fm_take_off_idle : {
        autoThrottle = dparams.min_throttle;
        //only control throttle:
        throttle = autoThrottle;
        roll = JOY_MIDDLE;
        pitch = JOY_MIDDLE;
        break;
    } case fm_abort_takeoff : {
        autoThrottle = JOY_BOUND_MIN;
        throttle = autoThrottle;
        roll = JOY_MIDDLE;
        pitch = JOY_MIDDLE;
        break;
    } case fm_flying : {
        //update integrators
        if (fabs(posErrZ)<integratorThresholdDistance)
            throttleErrI += velErrY; //posErrY;
        if (fabs(posErrX)<integratorThresholdDistance)
            rollErrI += posErrX;
        if (fabs(posErrZ)<integratorThresholdDistance)
            pitchErrI += posErrZ;

        autoThrottle =  hoverthrottle + take_off_throttle_boost - (accErrY * gain_throttle_acc + throttleErrI * gain_throttle_i*0.1f);
        autoRoll =  accErrX * gain_roll_acc;

        float depth_gain = 1;
        if (-data.sposZ - 2 > 0) // if further then 2 meters
            depth_gain  = 1 + powf((-data.sposZ-2),2) * depth_precision_gain;
        else // closer then 2 meters, do nothing:
            depth_gain = 1;

        autoPitch = accErrZ * static_cast<float>(gain_pitch_acc) / depth_gain;

        if (fabs(autoRoll) > fabs(autoPitch))
            autoThrottle += tmptbf*abs(autoRoll);
        else
            autoThrottle += tmptbf*abs(autoPitch);

        autoRoll    += JOY_MIDDLE + (gain_roll_i*rollErrI);
        autoPitch   += JOY_MIDDLE + (gain_pitch_i*pitchErrI);

        //int minThrottle = 1300 + min(abs(autoRoll-1500)/10,50) + min(abs(autoPitch-1500)/10,50);
        if (autoThrottle<dparams.min_throttle)
            autoThrottle = dparams.min_throttle;
        if (autoThrottle>JOY_BOUND_MAX)
            autoThrottle = JOY_BOUND_MAX;

        throttle = autoThrottle ;
        roll = autoRoll;
        pitch = autoPitch;
        break;
    } case fm_landing : {
        //slowly decrease throttle
        autoThrottle = hoverthrottle + take_off_throttle_boost - (accErrY * gain_throttle_acc + throttleErrI * gain_throttle_i*0.1f);
        //same as fm_flying:
        autoRoll =  JOY_MIDDLE + (accErrX * gain_roll_acc +  gain_roll_i*rollErrI);
        autoPitch = JOY_MIDDLE + (accErrZ * gain_pitch_acc +  gain_pitch_i*pitchErrI);
        throttle = autoThrottle ;
        roll = autoRoll;
        pitch = autoPitch;
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
        _rc->arm(false);
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

    yaw = joyYaw; // tmp until auto yaw control is fixed #10

    throttle = bound_throttle(throttle);
    roll = bound_joystick_value(roll);
    pitch = bound_joystick_value(pitch);
    yaw = bound_joystick_value(yaw);

    if (!_fromfile) {
        _rc->queue_commands(throttle,roll,pitch,yaw);
    }

    control_data c(Throttle(),Roll(),Pitch(),data.time);
    control_history.push_back(c);
    while (control_history.size() > control_history_max_size)
        control_history.erase(control_history.begin());

    (*_logger) << static_cast<int>(data.pos_valid)  << "; " <<
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
                  autoThrottle << "; " <<
                  autoRoll << "; " <<
                  autoPitch << "; " <<
                  autoYaw <<  "; " <<
                  joyThrottle <<  "; " <<
                  joyRoll <<  "; " <<
                  joyPitch <<  "; " <<
                  joyYaw << "; " <<
                  static_cast<int>(_joy_arm_switch) << "; " <<
                  static_cast<int>(_joy_mode_switch) << "; " <<
                  static_cast<int>(_joy_takeoff_switch) << "; " <<
                  data.dt << "; " <<
                  data.dx << "; " <<
                  data.dy << "; " <<
                  data.dz << "; " <<
                  velx_sp << "; " <<
                  vely_sp << "; " <<
                  velz_sp << "; ";
}

void DroneController::readJoystick(void) {
    while (joystick.sample(&event))
    {
        if (event.isAxis()) {
            if (pparams.joystick == rc_devo) {
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
            } else if (pparams.joystick == rc_usb_hobbyking) {
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
            } else if (pparams.joystick == rc_playstation) {
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
            } else if (pparams.joystick == rc_xlite) {
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
    if (pparams.joystick == rc_none)
        return;
    else if (_joy_state == js_none)
        _joy_state = js_checking;


    if (_joy_state == js_checking){
        if (!_joy_arm_switch &&
                joyThrottle <= JOY_MIN_THRESH &&
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
                if (joyPitch > JOY_MAX_THRESH) {
                    _rc->bind(true);
                    _joy_arm_switch = false;
                } else if(joyThrottle < JOY_MIN + 100) {
                    if (joyPitch < JOY_MIN + 200) {
                        _flight_mode = fm_inactive;
                        if (joyRoll > JOY_MAX_THRESH)
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
        serialize_settings();
        initialized = false;
    }
}
