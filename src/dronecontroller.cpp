#include "dronecontroller.h"
#include "defines.h"

#ifdef HASSCREEN
#if 1
#define TUNING
#endif
#endif

const string paramsFile = "../controlParameters.dat";

// Create an instance of Joystick
Joystick joystick("/dev/input/js0");
JoystickEvent event;

int notconnected;

bool DroneController::init(std::ofstream *logger,bool fromfile) {
    _logger = logger;
    (*_logger) << "valid; posErrX; posErrY; posErrZ; velX; velY; velZ; hoverthrottle; autoThrottle; autoRoll; autoPitch; autoYaw; joyThrottle; joyRoll; joyPitch; joyYaw; joySwitch; throttleP; throttleI; throttleD; dt; dx; dy; dz;";
    std::cout << "Initialising control." << std::endl;
    // setup connection with Arduino
    baudrate = 115200;
    notconnected = RS232_OpenComport(baudrate);

    if (notconnected && !fromfile && !INSECT_DATA_LOGGING_MODE) {
        printf("Arduino failed.\n");
        exit(1);
    }

    // Ensure that joystick was found and that we can use it
    if (!joystick.isFound() && !fromfile && !INSECT_DATA_LOGGING_MODE) {
        printf("joystick failed.\n");
        exit(1);
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

    createTrackbar("Rebind", "Control", &rebindValue, 1);
    createTrackbar("AutoLand", "Control", &autoLand, 1);

    // throttle control
    createTrackbar("Throttle P", "Control", &params.throttleP, 2000);
    createTrackbar("Throttle I", "Control", &params.throttleI, 255);
    createTrackbar("Throttle D", "Control", &params.throttleD, 2000);

    createTrackbar("Take off", "Control", &params.autoTakeoffFactor, 255);

    // roll control
    createTrackbar("Roll P", "Control", &params.rollP, 5000);
    createTrackbar("Roll I", "Control", &params.rollI, 255);
    createTrackbar("Roll D", "Control", &params.rollD, 255);
    // pitch control
    createTrackbar("Pitch P", "Control", &params.pitchP, 5000);
    createTrackbar("Pitch I", "Control", &params.pitchI, 255);
    createTrackbar("Pitch D", "Control", &params.pitchD, 255);

    createTrackbar("LED", "Control", &ledpower, 255);


    //    // yaw control
    //    createTrackbar("Yaw P", "Control", &params.yawP, 255);
    //    createTrackbar("Yaw I", "Control", &params.yawI, 255);
    //    createTrackbar("Yaw D", "Control", &params.yawD, 255);


#endif

    thread_nrf = std::thread(&DroneController::workerThread,this);

    std::cout << "led power: " << ledpower << std::endl;
}

float startY = -100;

void DroneController::control(trackData * data) {

    rebind();
    readJoystick();

    if(autoLand) {
        autoTakeOff=false;

        if (fabs(data->posErrY - startY) < 0.1) {
            hoverthrottle  = INITIALTHROTTLE; //?
        }
        data->posErrY = -startY-0.2;
    }

    if (autoTakeOff) {
        hoverthrottle  +=params.autoTakeoffFactor;
        if (hoverthrottle < 1300)
            hoverthrottle = 1300;
    }

    if (data->svelY > AUTOTAKEOFF_SPEED && autoTakeOff) {
        autoTakeOff = false;
        hoverthrottle -= 5*params.autoTakeoffFactor; // to compensate for ground effect and delay
        startY = data->posErrY;
        alert("canberra-gtk-play -f /usr/share/sounds/ubuntu/notifications/Slick.ogg &");
        std::cout << "HHHHHHHHHHHHHHH"<< data->svelY << std::endl;
    }

    if (autoTakeOff)
        autoThrottle = hoverthrottle;
    else
        autoThrottle =  hoverthrottle  - (data->posErrY * params.throttleP + data->velY * (params.throttleD/1000.0f) + throttleErrI * params.throttleI);
    autoRoll = 1500 + (data->posErrX * params.rollP + data->velX * (params.rollD/1000.0f) +  params.rollI*rollErrI);
    autoPitch =1500 + (data->posErrZ * params.pitchP + data->velZ * (params.pitchD/1000.0f) +  params.pitchI*pitchErrI);
    //TODO: Yaw

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

    g_lockData.lock();
    if ( autoControl ) {
        throttle = autoThrottle;
        if (!autoTakeOff) {
            roll = autoRoll;
            pitch = autoPitch;
            //yaw= autoYaw;
        }
        //TMP:
        roll = joyRoll;
        pitch = joyPitch;
        yaw = joyYaw;

        //calc integrated errors
        throttleErrI += data->posErrY;
        rollErrI += data->posErrX;
        pitchErrI += data->posErrZ;

    } else {
        throttle = joyThrottle;
        roll = joyRoll;
        pitch = joyPitch;
        yaw = joyYaw;
    }

    if (params.throttleI <= 1 || autoTakeOff || autoLand || !autoControl)
        throttleErrI = 0;
    if (params.rollI <= 1 || autoTakeOff || autoLand || !autoControl)
        rollErrI = 0;
    if (params.pitchI <= 1 || autoTakeOff || autoLand || !autoControl)
        pitchErrI = 0;

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


    if ((autoThrottle <= 1050 && autoLand) || (joyThrottle <= 1050 && !autoControl)) {
        data->landed = true;
    } else
        data->landed = false;

    g_lockData.unlock();

    (*_logger) << (int)data->valid  << "; " << data->posErrX << "; " << data->posErrY  << "; " << data->posErrZ << "; " << data->velX << "; " << data->velY  << "; " << data->velZ << "; " << hoverthrottle << "; " << autoThrottle << "; " << autoRoll << "; " << autoPitch << "; " << autoYaw <<  "; " << joyThrottle <<  "; " << joyRoll <<  "; " << joyPitch <<  "; " << joyYaw << "; " << (int)joySwitch << "; " << params.throttleP << "; " << params.throttleI << "; " << params.throttleD << "; " << data->dt << "; " << data->dx << "; " << data->dy << "; " << data->dz << "; ";
//    if (!notconnected){
//        params.throttleP = scaledjoydial;
//        std::cout << "P:" << params.throttleP << " Throttle: " << throttle << " HT: " << hoverthrottle << std::endl;
//        std::cout << "Roll: " << roll << " RollP: " << params.rollP << std::endl;
//        std::cout << "AutoTakeOff:" << (int)autoTakeOff <<  " HT: " << hoverthrottle << " Valid: " << data->valid << "VelY: " << data->velY <<std::endl;
//        std::cout << "AutoLand:" << autoLand <<  " HT: " << hoverthrottle << " Valid: " << data->valid << " PosY: " << data->posErrY << " startY:" << startY << " VelY: " << data->velY <<std::endl;
//    }
}

void DroneController::workerThread(void) {
    std::cout << "Send nrf thread started!" << std::endl;
    while (!exitSendThread) {
        g_lockData.lock();
        sendData();
        g_lockData.unlock();
        usleep(20000);
    }
}

int firstTime = 3;
void DroneController::readJoystick(void) {
    while (joystick.sample(&event))
    {
        if (event.isAxis()) {
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
            rebindValue = 1;
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
        if (joyPitch < 1100) {
            autoLand=1;
            alert("canberra-gtk-play -f /usr/share/sounds/ubuntu/stereo/desktop-logout.ogg &");
        } else {
            autoTakeOff = false;
            autoControl = false;
            autoLand = false;
        }
    }
    joySwitch_prev = joySwitch;
}

void DroneController::sendData(void) {

    if (false) {
        //tmp led power hack
        char buff[3];
        buff[0] = 'B';
        buff[1] = 't';
        buff[2] = ledpower;
        if (!notconnected) {
            RS232_SendBuf( (unsigned char*) buff, 3);
        }
        usleep(100000);
    } else {

        char buff[64];
        sprintf( (char*) buff,"%u,%u,%u,%u,%u,%u,0,0,0,0,0,0\n",throttle,roll,pitch,yaw,mode,ledpower);
        if (!notconnected) {
            RS232_SendBuf( (unsigned char*) buff, 63);
        }

        //if ( data->valid ) {
        //std::setprecision(2);
        //std::cout << data->posY << " " << data->velY << " - ";
        //std::cout << "JoyCommands:" << std::string(buff) << std::flush;
        //    (*_logger) << "JoyCommands:" << std::string(buff) << std::flush;

        //}

        if (!notconnected) {
            unsigned char inbuf[1];
            inbuf[1] = 0;
            std::stringstream tmp;
            int n = 1;
            int totn = 0;
            while (n)    {
                n = RS232_PollComport(inbuf,1);
                if (n > 0) {
                    tmp << inbuf[0];
                    totn += n;
                }
            }
            if (totn > 0) {
                 //std::cout << "Arduino: " << totn << ": " << tmp.str() << std::endl;
            }
        }

        //TODO: disable uart polling after bind confirmed!
        //check if input string is the same as the commanded strning, if so bind was succesfull!
        //	for (int i = 0 to tmp.count;i++) {
        //if tmp.str.c_str[i] == buff[i] ...
        //	}

    }
}

void DroneController::rebind(void){

    if (rebindValue) {
        rebindValue=0;

        char buff[64];
        sprintf( (char*) buff,"1050,1500,1500,1500,0,0,0,0,0,0,0,2000\n");
        if (!notconnected) {
            RS232_SendBuf( (unsigned char*) buff, 63);
            RS232_CloseComport();
        }
        usleep(100000);
        notconnected = RS232_OpenComport(baudrate);

        if (notconnected)
            std::cout << "Bind failure" << std::endl;
    }
}

void DroneController::close () {
    exitSendThread = true;
    g_lockData.unlock();
    g_lockWaitForData2.unlock();
    thread_nrf.join();

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
