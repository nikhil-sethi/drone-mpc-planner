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
void DroneController::init(std::ofstream *logger,bool fromfile, Arduino * arduino) {
    _arduino = arduino;
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
            exit(1);
        }
    }

#ifdef TUNING
    std::cout << "Creating control tuning window." << std::endl;
    // create GUI to set control parameters
    namedWindow("Control", WINDOW_NORMAL);

    createTrackbar("Rebind", "Control", &(_arduino->rebindValue), 1);
    //createTrackbar("AutoLand", "Control", &autoLand, 1);

    // throttle control
    createTrackbar("Throttle Pos", "Control", &params.throttle_Pos, 3000);
    createTrackbar("Throttle Vel", "Control", &params.throttle_Vel, 1000);
    createTrackbar("Throttle Acc", "Control", &params.throttle_Acc, 100);
    createTrackbar("Throttle I", "Control", &params.throttleI, 50);

    createTrackbar("Take off factor", "Control", &params.autoTakeoffFactor, 255);
    createTrackbar("Take off speed", "Control", &params.auto_takeoff_speed, 1000); // /100
    createTrackbar("Hover offset", "Control", &params.hoverOffset, 100);

    // roll control
    createTrackbar("Roll Pos", "Control", &params.roll_Pos, 3000);
    createTrackbar("Roll Vel", "Control", &params.roll_Vel, 2000);
    createTrackbar("Roll Acc", "Control", &params.roll_Acc, 100);
    createTrackbar("Roll I", "Control", &params.rollI, 10);

    // pitch control
    createTrackbar("Pitch Pos", "Control", &params.pitch_Pos, 3000);
    createTrackbar("Pitch Vel", "Control", &params.pitch_Vel, 2000);
    createTrackbar("Pitch Acc", "Control", &params.pitch_Acc, 100);
    createTrackbar("Pitch I", "Control", &params.pitchI, 10);


    //    // yaw control
    //    createTrackbar("Yaw P", "Control", &params.yawP, 255);
    //    createTrackbar("Yaw I", "Control", &params.yawI, 255);
    //    createTrackbar("Yaw D", "Control", &params.yawD, 255);


#endif

}

void DroneController::control(trackData data,cv::Point3f setpoint, cv::Point3f setpoint_v) {

    if (!_fromfile) {
        _arduino->rebind();
        readJoystick();
    }
    process_joystick();


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


    if(autoLand) {
        autoTakeOff=false;
    }

    if (autoTakeOff) {
        hoverthrottle  +=params.autoTakeoffFactor;
        beforeTakeOffFactor = 0.15;
        if (hoverthrottle < 1300)
            hoverthrottle = 1300;
    }

    if (data.svelY > ((float)params.auto_takeoff_speed) / 100.f && autoTakeOff) {
        autoTakeOff = false;
        hoverthrottle -= params.hoverOffset*params.autoTakeoffFactor; // to compensate for ground effect and delay
        alert("canberra-gtk-play -f /usr/share/sounds/ubuntu/notifications/Slick.ogg &");
    }

    if (params.throttleI < 1 || autoTakeOff || !autoControl)
        throttleErrI = 0;
    if (params.rollI < 1 || autoTakeOff  || !autoControl)
        rollErrI = 0;
    if (params.pitchI < 1 || autoTakeOff || !autoControl)
        pitchErrI = 0;


    if (autoTakeOff) {
        autoThrottle = hoverthrottle;
    } else if (autoLand) {
        autoThrottle = hoverthrottle - autoLandThrottleDecrease;
    } else {
            autoThrottle =  hoverthrottle + take_off_throttle_boost - (accErrY * params.throttle_Acc + throttleErrI * params.throttleI*beforeTakeOffFactor);
        if (autoThrottle < 1300)
            autoThrottle = 1300;
    }
    autoRoll =  1500 + (accErrX * params.roll_Acc +  params.rollI*rollErrI);
    autoPitch = 1500 + (accErrZ * params.pitch_Acc +  params.pitchI*pitchErrI);

    //TODO: Yaw    
    if (autoPitch > 1950) {
        autoPitch = 1950;
    } else if (autoPitch < 1050) {
        autoPitch = 1050;
    }
    if (autoRoll > 1950) {
        autoRoll = 1950;
    } else if (autoRoll < 1050) {
        autoRoll = 1050;
    }
    if (autoThrottle> 1950) {
        autoThrottle = 1950;
    } else if (autoThrottle < 1050) {
        autoThrottle = 1050;
    }

    int throttle,roll,pitch,yaw;

    if ( autoControl ) {
        if (landed)
        {
            throttle = INITIALTHROTTLE;
            roll = 1500;
            pitch = 1500;
        } else
        {
            throttle = autoThrottle ;
            if (!autoTakeOff) {
                roll = autoRoll;
                pitch = autoPitch;
                //yaw= autoYaw;
            } else {
                roll = 1500;
                pitch = 1530; //TODO: check why this is neces
                //yaw= 1500;
            }
        }

        //TMP:
        //roll = joyRoll;
        //pitch = joyPitch;
        yaw = joyYaw;

        //calc integrated errors
        throttleErrI += velErrY; //posErrY;
        rollErrI += posErrX;
        pitchErrI += posErrZ;

    } else {
        throttle = joyThrottle;
        roll = joyRoll;
        pitch = joyPitch;
        yaw = joyYaw;
    }

    if ( throttle < 1050 )
        throttle = 1050;
    if ( throttle > 1950 )
        throttle = 1950;

    if ( roll < 1050 )
        roll = 1050;
    if ( roll > 1950 )
        roll = 1950;

    if ( pitch < 1050 )
        pitch = 1050;
    if ( pitch > 1950 )
        pitch = 1950;

    if ( yaw < 1050 )
        yaw = 1050;
    if ( yaw > 1950 )
        yaw = 1950;

    if ((landed && autoTakeOff ) || (joyThrottle > 1070 && !autoControl))  {
        landed = false;
    } else if ((landed && !autoTakeOff )|| (joyThrottle <= 1070 && !autoControl)) {
        hoverthrottle  = INITIALTHROTTLE; //?
        landed = true;
    }

    if (!_fromfile) {
        _arduino->g_lockData.lock();

        _arduino->throttle = throttle;
        _arduino->roll = roll;
        _arduino->pitch = pitch;
        _arduino->yaw = yaw;

        _arduino->g_lockData.unlock();
    }

    (*_logger) << (int)data.valid  << "; " << posErrX << "; " << posErrY  << "; " << posErrZ << "; " << setpoint_v.x << "; " << setpoint_v.y  << "; " << setpoint_v.z << "; " << accx_sp << "; " << accy_sp  << "; " << accx_sp << "; " << hoverthrottle << "; " << autoThrottle << "; " << autoRoll << "; " << autoPitch << "; " << autoYaw <<  "; " << joyThrottle <<  "; " << joyRoll <<  "; " << joyPitch <<  "; " << joyYaw << "; " << (int)joySwitch << "; " << params.throttle_Pos << "; " << params.throttleI << "; " << params.throttle_Acc << "; " << data.dt << "; " << data.dx << "; " << data.dy << "; " << data.dz << "; " << velx_sp << "; " << vely_sp << "; " << velz_sp << "; ";
}

int firstTime = 3;
void DroneController::readJoystick(void) {
    while (joystick.sample(&event))
    {
        if (event.isAxis()) {
            if (JOYSTICK_TYPE == 0) {
                switch ( event.number ) {
                case 0: // roll
                    joyRoll = 1500 + (event.value >> 6);
                    break;
                case 1: // pitch
                    joyPitch = 1500 - (event.value >> 6);
                    break;
                case 2: //throttle
                    joyThrottle = 1500 + (event.value >> 6);
                    break;
                case 5: //switch
                    joySwitch = event.value>0; // goes between +/-32768
                    break;
                case 3: //dial
                    joyDial = event.value; // goes between +/-32768
                    scaledjoydial = joyDial+32767;
                    scaledjoydial = (scaledjoydial / 65536)*100+35;
                    break;
                case 4: //yaw
                    joyYaw = 1500 + (event.value >> 6);
                    break;
                default:
                    std::cout << "Unkown joystick event: " << std::to_string(event.number) << ". Value: " << std::to_string(event.value) << std::endl;
                    break;
                }
            } else if (JOYSTICK_TYPE == 1) {
		            switch ( event.number ) {
		            case 0: // roll
		                joyRoll = 1500 + (event.value >> 6);
		                break;
		            case 1: // pitch
                        joyPitch = 1500 + (event.value >> 5);
		                break;
		            case 2: //throttle
                        joyThrottle = 1500 - (event.value >> 5);
		                break;
		            case 3: //switch
		                joySwitch = event.value>0; // goes between +/-32768
		                break;
		            case 4: //dial
		                joyDial = event.value; // goes between +/-32768
		                scaledjoydial = joyDial+32767;
		                scaledjoydial = (scaledjoydial / 65536)*100+35;
		                break;
		            case 5: //yaw
		                joyYaw = 1500 + (event.value >> 6);
		                break;
		            default:
		                std::cout << "Unkown joystick event: " << std::to_string(event.number) << ". Value: " << std::to_string(event.value) << std::endl;
		                break;
		            }
            } else if (JOYSTICK_TYPE == 2) {
                switch ( event.number ) {
                case 2: // roll
                    joyRoll = 1500 + (event.value >> 6);
                    std::cout << "roll" << std::endl;
                    break;
                case 3: // pitch
                    joyPitch = 1500 - (event.value >> 6);
                    std::cout << "pitch" << std::endl;
                    break;
                case 1: //throttle
                    joyThrottle = 1000 - (event.value >> 6);
                    std::cout << "throttle" << std::endl;
                    break;
                case 0: //yaw
                    joyYaw = 1500 + (event.value >> 6);
                    std::cout << "yaw" << std::endl;
                    break;
                default:
                    std::cout << "Unkown joystick event: " << std::to_string(event.number) << ". Value: " << std::to_string(event.value) << std::endl;
                    break;
                }
            }
        } else if (event.isButton() && JOYSTICK_TYPE == 2) {
            switch ( event.number ) {
            case 2: //bind
                if (event.value>0) {
                    joySwitch = 1;
                    joyPitch = 1900;
                } else {
                    joySwitch = 0;
                    joyPitch = 1500;
                }
                break;
            case 9: //auto mode
                joySwitch = 1;
                break;
            case 4: //manual mode
                joySwitch = 0;
                break;
            case 6: //manual mode
                joySwitch = 0;
                break;
            default:
                std::cout << "Unkown joystick button: " << std::to_string(event.number) << ". Value: " << std::to_string(event.value) << std::endl;
                break;
            }
        }
    }
}


void DroneController::process_joystick() {
    // prevent accidental take offs at start up
    if (firstTime > 0) {
        if (joySwitch || joyThrottle > 1080 ) {
            std::cout << "Joystick not centered warning!" << std::endl;
            joySwitch = false;
            joyThrottle = 1050;
            firstTime++;
        }
        firstTime--;
    }

    //check switch functions
    static bool joySwitch_prev = joySwitch;
    if (joySwitch && !joySwitch_prev) {
        if (joyPitch > 1800) {
            _arduino->rebindValue = 1;
            alert("canberra-gtk-play -f /usr/share/sounds/ubuntu/notifications/Amsterdam.ogg &");
            joySwitch = false;
        } else if(joyThrottle < 1100) {
            autoTakeOff = true;
            autoLand = false;
            hoverthrottle = INITIALTHROTTLE;
            autoControl = true;
        } else {
            autoControl = true;
        }
    } else if (!joySwitch && joySwitch_prev) {
        autoTakeOff = false;
        autoControl = false;
        autoLand = false;
    }
    joySwitch_prev = joySwitch;

#if CAMMODE == CAMMODE_GENERATOR
   if (!autoControl)
       autoTakeOff = true;
    autoControl = true;
   joyPitch = 1500;
#endif

    if (autoControl && joyPitch < 1150) {
        autoLand=true;
        alert("canberra-gtk-play -f /usr/share/sounds/ubuntu/stereo/desktop-logout.ogg &");
    }
}

void DroneController::recalibrateHover() {
    hoverthrottle = hoverthrottle - throttleErrI;
    throttleErrI = 0;
    beforeTakeOffFactor =0.1f;
}

void DroneController::close () {
    std::cout << "closing controller" << std::endl;
    std::ofstream os(paramsFile, std::ios::binary);
    cereal::BinaryOutputArchive archive( os );
    archive( params );
}
