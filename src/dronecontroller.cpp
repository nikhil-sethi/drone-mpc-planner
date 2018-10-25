#include "dronecontroller.h"
#include "defines.h"

#ifdef HASSCREEN
#define TUNING
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
        archive(params);
    }

    predictTarget = {0,0,0};
    convergedTarget = false;
    timeToTarget = 0;


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

    createTrackbar("vref gain", "Control", &params.vref_gain, 10000);
    createTrackbar("vref max", "Control", &params.vref_max, 10000);
    createTrackbar("v vs pos gain", "Control", &params.v_vs_pos_control_gain, 10000);
#endif

}

void DroneController::control(trackData data,cv::Point3f setpoint_world, cv::Point3f setspeed_world) {

    if (!_fromfile) {
        _arduino->rebind();
        readJoystick();
    }
    process_joystick();

    //cv::Point3f predictTarget = {0,0,0};
    double posErr,rangeParam,targetSpeed;

    //targetSpeed = sqrt(setspeed_world.x*setspeed_world.x + setspeed_world.y*setspeed_world.y + setspeed_world.z*setspeed_world.z);
    rangeParam = 0.4;

    //bool convergedTarget = false;


    targetSpeed = norm(setspeed_world);

    if (targetSpeed > 0)
    {


        if (!convergedTarget) {

            timeToTarget = 1.0;
            convergedTarget = true;

        }
        else {

            posErr = norm(predictTarget-setpoint_world);
            //posErr = sqrt(posErrX*posErrX + posErrY*posErrY + posErrZ*posErrZ);

            timeToTarget = posErr/targetSpeed;
        }

//        std::cout << "\t\t\t\t\t\t\t\t\t\t\t\t\t timeToTarget: " << timeToTarget << std::endl;

        predictTarget = setpoint_world + (setspeed_world*timeToTarget);

        //predictTarget.x = setpoint_world.x + setspeed_world.x;
        //predictTarget.y = setpoint_world.y + setspeed_world.y;
        //predictTarget.z = setpoint_world.z + setspeed_world.z;

        setpoint_world = predictTarget;

    } else
        convergedTarget = false;





    //std::cout << "\t\t\t\t\t\t\t\t\t\t\t\t\t etspeed_world.x: " << setspeed_world.x << std::endl;
    //std::cout << "\t\t\t\t\t\t\t\t\t\t\t\t\t etspeed_world.z: " << setspeed_world.z << std::endl;

/*
    int convergeCount = 0;
    while ( !convergedTarget) {

        posErrX = data.sposX - setpoint_world.x - predictTarget.x;
        posErrY = data.sposY - setpoint_world.y - predictTarget.y;
        posErrZ = data.sposZ - setpoint_world.z - predictTarget.z;

        posErr = sqrt(posErrX*posErrX + posErrY*posErrY + posErrZ*posErrZ);

        if ( targetSpeed > 0 ) {
            posErrHor = sqrt(posErrX*posErrX + posErrZ*posErrZ);
            timeToTarget = posErrHor*1000/(params.vref_max*2);

            if (abs(timeToTarget-prevTimeToTarget) < 0.1) {
                convergedTarget = true;
                //std::cout << std::endl << "\t\t\t\t\t\t\t\t\t\t\t\t\t convergeCount: " << convergeCount << std::endl;
            } else if ( prevTimeToTarget < 1000 )
                timeToTarget = (timeToTarget+prevTimeToTarget)/2;
            else if (timeToTarget > 10 ) {
                convergedTarget = true;
                timeToTarget = 3;
            }

            if (convergedTarget) {
                //std::cout << "\t\t\t\t\t\t\t\t\t\t\t\t\t predictTarget.x: " << setpoint_world.x+predictTarget.x << std::endl;
                //std::cout << "\t\t\t\t\t\t\t\t\t\t\t\t\t predictTarget.y: " << setpoint_world.y+predictTarget.y << std::endl;
            }


            predictTarget.x = timeToTarget*setspeed_world.x;
            predictTarget.z = timeToTarget*setspeed_world.z;
            prevTimeToTarget = timeToTarget;
            convergeCount++;
            //std::cout << "\t\t\t\t\t\t\t\t\t\t\t\t\t timeToTarget: " << timeToTarget << std::endl;



        } else
            convergedTarget = true;

    }

    posErrX = data.sposX - setpoint_world.x - predictTarget.x;
    posErrY = data.sposY - setpoint_world.y - predictTarget.y;
    posErrZ = data.sposZ - setpoint_world.z - predictTarget.z;

*/
    posErrX = data.sposX - setpoint_world.x;
    posErrY = data.sposY - setpoint_world.y;
    posErrZ = data.sposZ - setpoint_world.z;

    posErr = sqrt(posErrX*posErrX + posErrY*posErrY + posErrZ*posErrZ);

    if (posErr < rangeParam && targetSpeed > 0 && !rangeAlert) {
        alert("canberra-gtk-play -f /usr/share/sounds/ubuntu/notifications/Rhodes.ogg &");
//        std::cout << "\t\t\t\t\t\t\t\t ALERT" << std::endl;
        rangeAlert = true;
    } else if (targetSpeed==0)
        rangeAlert = false;


    //setspeed_world.x = 0;
    //setspeed_world.y = 0;
    //setspeed_world.z = 0;

/*
    std::cout << "\t\t\t\t\t\t\t\t\t\t\t\t\t posErr: " << posErr << std::endl;
    std::cout << "\t\t\t\t\t\t\t\t\t\t\t\t\t posErrX: " << posErrX << " rollErrI: " << rollErrI <<  std::endl;
    std::cout << "\t\t\t\t\t\t\t\t\t\t\t\t\t posErrY: " << posErrY << " throttleErrI: " << throttleErrI  << std::endl;
    std::cout << "\t\t\t\t\t\t\t\t\t\t\t\t\t posErrZ: " << posErrZ << " pitchErrI: " << pitchErrI  << std::endl;
*/
    /*
    // Roll Control - X

    velx_sp = posErrX*params.vref_gain/1000.f;
    if (velx_sp > params.vref_max/1000.f)
        velx_sp = params.vref_max/1000.f;
    if (velx_sp < -params.vref_max/1000.f)
        velx_sp = -params.vref_max/1000.f;
    velErrX = data.svelX + velx_sp; //desired speed
    //if (posErr < rangeParam && targetSpeed > 0)
        posErrX = posErrX/(1+(abs(velErrX)*(params.v_vs_pos_control_gain/100.f)));
    //else
        //posErrX /= 10;

        */
    velx_sp = posErrX*params.pitchP/1000.f; //desired speed
    velErrX = data.svelX + velx_sp;
    float accx_sp = velErrX*params.rollP/100; // desired accelleration
    float accErrX = data.saccX + accx_sp;

    velErrX = accErrX;
    posErrX = 0;


    // Altitude Control - Y

    if (true || posErrY<0) {
        vely_sp = posErrY*params.vref_gain/1000.f;
        velErrY = data.svelY + vely_sp;
        float accy_sp = velErrY*params.pitchD/100; // desired accelleration
        float accErrY = data.saccY + accy_sp;

        velErrY = accErrY;

    } else {

        vely_sp = posErrY*1300/1000.f;
        if (vely_sp > 1000/1000.f)
            vely_sp = 1000/1000.f;
        if (vely_sp < -1000/1000.f)
            vely_sp = -1000/1000.f;
        velErrY = data.svelY + vely_sp;
        //if (posErr < rangeParam && targetSpeed > 0)
            posErrY = posErrY/(1+(abs(velErrY)*(params.v_vs_pos_control_gain/100.f)));
        //else
            //posErrY /= 4; //*= (30.0/params.throttleP);
    }

/*
    vely_sp = posErrY*params.vref_gain/1000.f;
    if (vely_sp > params.vref_max/1000.f)
        vely_sp = params.vref_max/1000.f;
    if (vely_sp < -params.vref_max/1000.f)
        vely_sp = -params.vref_max/1000.f;
    velErrY = data.svelY + vely_sp;
    //if (posErr < rangeParam && targetSpeed > 0)
        posErrY = posErrY/(1+(abs(velErrY)*(params.v_vs_pos_control_gain/100.f)));
    //else
        //posErrY /= 4; //= (30.0/params.throttleP);
        */


        /*
    // Pitch Control - Z
    velz_sp = posErrZ*params.vref_gain/1000.f;
    if (velz_sp > params.vref_max/1000.f)
        velz_sp = params.vref_max/1000.f;
    if (velz_sp < -params.vref_max/1000.f)
        velz_sp = -params.vref_max/1000.f;
    velErrZ = data.svelZ + velz_sp;
    //if (posErr < rangeParam && targetSpeed > 0)
        posErrZ = posErrZ/(1+(abs(velErrZ)*(params.v_vs_pos_control_gain/100.f)));
    //else
        //posErrZ /=10;
        */

    velz_sp = posErrZ*params.pitchP/1000.f; //desired speed
    velErrZ = data.svelZ + velz_sp;
    float accz_sp = velErrZ*params.rollP/100; // desired accelleration
    float accErrZ = data.saccZ + accz_sp;

    velErrZ = accErrZ;
    posErrZ = 0;

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
    } else if (autoLandThrottleDecrease > 0 ) {
        hoverthrottle -= autoLandThrottleDecrease;
        autoThrottle =hoverthrottle ;
    } else {
            autoThrottle =  hoverthrottle + 100 - (velErrY * params.vref_max + throttleErrI * params.throttleI*beforeTakeOffFactor);
        if (autoThrottle < 1300)
            autoThrottle = 1300;
    }
    autoRoll = 1500 + (velErrX * params.rollD +  params.rollI*rollErrI);
    autoPitch =1500 + (velErrZ * params.rollD +  params.pitchI*pitchErrI);

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

    (*_logger) << (int)data.valid  << "; " << posErrX << "; " << posErrY  << "; " << posErrZ << "; " << setspeed_world.x << "; " << setspeed_world.y  << "; " << setspeed_world.z << "; " << predictTarget.x << "; " << predictTarget.y  << "; " << predictTarget.z << "; " << hoverthrottle << "; " << autoThrottle << "; " << autoRoll << "; " << autoPitch << "; " << autoYaw <<  "; " << joyThrottle <<  "; " << joyRoll <<  "; " << joyPitch <<  "; " << joyYaw << "; " << (int)joySwitch << "; " << params.throttleP << "; " << params.throttleI << "; " << params.throttleD << "; " << timeToTarget << "; " << data.dx << "; " << data.dy << "; " << data.dz << "; " << velx_sp << "; " << vely_sp << "; " << velz_sp << "; ";
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
