#ifndef DRONECONTROLLER_H
#define DRONECONTROLLER_H


#include "dronetracker.h"
#include "joystick.hpp"
#include "common.h"
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/archives/binary.hpp>
#include <fstream>
#include <iomanip>
#include <unistd.h>

#include "arduino.h"

using namespace cv;

/*
 * This class will control a micro drone via a Serial link
 *
 */
class DroneController {


private:
    
    struct controlParameters{

        //height control
        int throttleP = 0;
        int throttleI = 0;
        int throttleD = 0;

        int autoTakeoffFactor = 10;
        int auto_takeoff_speed = 20; // /100

        //roll control
        int rollP = 0;
        int rollI = 0;
        int rollD = 0;

        //pitch control
        int pitchP = 0;
        int pitchI = 0;
        int pitchD = 0;

        //yaw control
        int yawP = 0;
        int yawI = 0;
        int yawD = 0;

        template <class Archive>
        void serialize( Archive & ar )
        {
            ar( throttleP,throttleI,throttleD,rollP,rollI,rollD,pitchP,pitchI,pitchD,yawP,yawI,yawD,autoTakeoffFactor,auto_takeoff_speed);
        }

    };

    float throttleErrI = 0;
    float rollErrI = 0;
    float pitchErrI = 0;

    #define INITIALTHROTTLE 1050

    bool autoTakeOff = false;
    bool autoLand = false;
    bool autoControl = false;

    Arduino * _arduino;

    std::ofstream *_logger;
    void sendData(void);
    void readJoystick(void);
    void rebind(void);

public:

    controlParameters params;

    int autoThrottle = 1000;
    int autoRoll = 1500;
    int autoPitch = 1500;
    int autoYaw = 1500;

    float hoverthrottle = INITIALTHROTTLE;

    bool joySwitch = true;
    int joyDial = 0;
    float scaledjoydial = 0;
    int joyThrottle = 0;
    int joyRoll = 0;
    int joyPitch = 0;
    int joyYaw = 0;

    void close (void);
    bool init(std::ofstream *logger, bool fromfile, Arduino * arduino);
    void control(trackData *data);
    bool getAutoControl() {return autoControl;}
    bool getAutoTakeOff() {return autoTakeOff;}
    bool getAutoLand() {return autoLand;}

};

#endif //DRONECONTROLLER_H
