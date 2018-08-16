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

void DroneController::init(std::ofstream *logger,bool fromfile, Arduino * arduino) {
    _arduino = arduino;
    _logger = logger;
    (*_logger) << "valid; posErrX; posErrY; posErrZ; velX; velY; velZ; hoverthrottle; autoThrottle; autoRoll; autoPitch; autoYaw; joyThrottle; joyRoll; joyPitch; joyYaw; joySwitch; throttleP; throttleI; throttleD; dt; dx; dy; dz;";
    std::cout << "Initialising control." << std::endl;

    // Ensure that joystick was found and that we can use it
    if (!joystick.isFound() && !fromfile) {
        std::cout << "joystick failed." << std::endl;
        //exit(1);
    }

    // Load saved control paremeters
    if (checkFileExist(paramsFile)) {
        std::ifstream is(paramsFile, std::ios::binary);
        cereal::BinaryInputArchive archive( is );
        archive(params);
    }

#ifdef TUNING
    std::cout << "Creating control tuning window." << std::endl;
    // create GUI to set control parameters
    namedWindow("Control", WINDOW_NORMAL);

    createTrackbar("Rebind", "Control", &(_arduino->rebindValue), 1);
    //createTrackbar("AutoLand", "Control", &autoLand, 1);

    // throttle control
    createTrackbar("Throttle P", "Control", &params.throttleP, 2000);
    createTrackbar("Throttle I", "Control", &params.throttleI, 255);
    createTrackbar("Throttle D", "Control", &params.throttleD, 2000);

    createTrackbar("Take off factor", "Control", &params.autoTakeoffFactor, 255);
    createTrackbar("Take off speed", "Control", &params.auto_takeoff_speed, 1000); // /100
    createTrackbar("Hover offset", "Control", &params.hoverOffset, 100);

    // roll control
    createTrackbar("Roll P", "Control", &params.rollP, 5000);
    createTrackbar("Roll I", "Control", &params.rollI, 255);
    createTrackbar("Roll D", "Control", &params.rollD, 1000);
    // pitch control
    createTrackbar("Pitch P", "Control", &params.pitchP, 5000);
    createTrackbar("Pitch I", "Control", &params.pitchI, 255);
    createTrackbar("Pitch D", "Control", &params.pitchD, 1000);




    //    // yaw control
    //    createTrackbar("Yaw P", "Control", &params.yawP, 255);
    //    createTrackbar("Yaw I", "Control", &params.yawI, 255);
    //    createTrackbar("Yaw D", "Control", &params.yawD, 255);


#endif



}

void DroneController::control(trackData data) {

    _arduino->rebind();
    readJoystick();

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


    //static int autoLandThrottleDecrease_prev = 0;

    if (autoTakeOff) {
        autoThrottle = hoverthrottle;
//        autoRoll = 1500 ;
//        autoPitch =1500;
    //} //else if (autoLandThrottleDecrease > 0 && autoLandThrottleDecrease_prev == 0) {
     //   hoverthrottle = hoverthrottle + throttleErrI * params.throttleI - autoLandThrottleDecrease;
     //   autoThrottle =hoverthrottle ;
    } else if (autoLandThrottleDecrease > 0 ) {
        hoverthrottle -= autoLandThrottleDecrease;
        autoThrottle =hoverthrottle ;
    } else {
        autoThrottle =  hoverthrottle  - (data.posErrY * params.throttleP + data.svelY * params.throttleD + throttleErrI * params.throttleI*beforeTakeOffFactor);
        if (autoThrottle < 1300)
            autoThrottle = 1300;
    }
    autoRoll = 1500 + (data.posErrX * params.rollP + data.svelX * (params.rollD) +  params.rollI*rollErrI);
    autoPitch =1500 + (data.posErrZ * params.pitchP + data.svelZ * (params.pitchD) +  params.pitchI*pitchErrI);
    //TODO: Yaw
    //autoLandThrottleDecrease_prev = autoLandThrottleDecrease;
    //tmp only for vizs
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

    //std::cout << autoTakeOff << " " << autoThrottle << std::endl;
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
                pitch = 1500;
                //yaw= 1500;
            }
        }


        //TMP:
        //roll = joyRoll;
        //pitch = joyPitch;
        yaw = joyYaw;

        //calc integrated errors
        throttleErrI += data.posErrY;
        rollErrI += data.posErrX;
        pitchErrI += data.posErrZ;

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

    _arduino->g_lockData.lock();

    _arduino->throttle = throttle;
    _arduino->roll = roll;
    _arduino->pitch = pitch;
    _arduino->yaw = yaw;


    _arduino->g_lockData.unlock();

    (*_logger) << (int)data.valid  << "; " << data.posErrX << "; " << data.posErrY  << "; " << data.posErrZ << "; " << data.velX << "; " << data.velY  << "; " << data.velZ << "; " << hoverthrottle << "; " << autoThrottle << "; " << autoRoll << "; " << autoPitch << "; " << autoYaw <<  "; " << joyThrottle <<  "; " << joyRoll <<  "; " << joyPitch <<  "; " << joyYaw << "; " << (int)joySwitch << "; " << params.throttleP << "; " << params.throttleI << "; " << params.throttleD << "; " << data.dt << "; " << data.dx << "; " << data.dy << "; " << data.dz << "; ";
    //    if (!notconnected){
    //        params.throttleP = scaledjoydial;
    //        std::cout << "P:" << params.throttleP << " Throttle: " << throttle << " HT: " << hoverthrottle << std::endl;
    //        std::cout << "Roll: " << roll << " RollP: " << params.rollP << std::endl;
    //        std::cout << "AutoTakeOff:" << (int)autoTakeOff <<  " HT: " << hoverthrottle << " Valid: " << data->valid << "VelY: " << data->velY <<std::endl;
    //        std::cout << "AutoLand:" << autoLand <<  " HT: " << hoverthrottle << " Valid: " << data->valid << " PosY: " << data->posErrY << " startY:" << startY << " VelY: " << data->velY <<std::endl;
    //    }
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
                    joyPitch = 1500 - (event.value >> 6);
                    break;
                case 2: //throttle
                    joyThrottle = 1500 - (event.value >> 6);
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
            }

        }
    }

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
   autoControl = true;
   autoTakeOff = true;
   joyPitch = 1500;
#endif

    if (autoControl && joyPitch < 1100) {
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


    //    char buff[64];
    //    sprintf( (char*) buff,"1050,1500,1500,1500,0,0,0,0,0,0,0,2000\n");
    //    if (!notconnected) {
    //        RS232_SendBuf( (unsigned char*) buff, 63);
    //        RS232_CloseComport();
    //    }

    std::cout << "closing controller" << std::endl;

    std::ofstream os(paramsFile, std::ios::binary);
    cereal::BinaryOutputArchive archive( os );
    archive( params );

}
