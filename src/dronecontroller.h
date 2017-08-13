#ifndef DRONECONTROLLER_H
#define DRONECONTROLLER_H

#include "rs232.h"
#include "dronetracker.h"
#include "joystick.hpp"
#include "common.h"
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/archives/binary.hpp>
#include <fstream>
#include <iomanip>
#include <unistd.h>

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

        int autoTakeoffFactor = 1;

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
            ar( throttleP,throttleI,throttleD,rollP,rollI,rollD,pitchP,pitchI,pitchD,yawP,yawI,yawD,autoTakeoffFactor);
        }

    };

    controlParameters params;
    int baudrate;
    std::ofstream *_logger;
    void sendData(int throttle,int roll,int pitch,int yaw,int mode);
    void readJoystick(void);

public:

    int autoThrottle = 1000;
    int autoRoll = 1500;
    int autoPitch = 1500;
    int autoYaw = 1500;

    bool joySwitch = true;
    int joyDial = 0;
    float scaledjoydial = 0;
    int joyThrottle = 0;
    int joyRoll = 0;
    int joyPitch = 0;
    int joyYaw = 0;



    void close (void);
    bool init(std::ofstream *logger);
    void control(trackData data);

    int roll,pitch,yaw = 1500;
    int throttle = 1000;

};




#endif //DRONECONTROLLER_H
