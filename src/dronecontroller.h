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

/*
 * This class will control a micro drone via a Serial link
 *
 */
class DroneController {


private:
    
    struct controlParameters{

        //height control
        int throttleP = 150;
        int throttleI = 1;
        int throttleD = 180;

        int autoTakeoffFactor = 4;
        int auto_takeoff_speed = 10; // /100
        int hoverOffset = 30;

        //roll control
        int rollP = 550;
        int rollI = 3;
        int rollD = 350;

        //pitch control
        int pitchP = 660;
        int pitchI = 3;
        int pitchD = 450;

        //yaw control
        int yawP = 0;
        int yawI = 0;
        int yawD = 0;

        int vref_gain = 1400;
        int vref_max = 2000;
        int v_vs_pos_control_gain = 0;

        template <class Archive>
        void serialize( Archive & ar )
        {
            ar( throttleP,throttleI,throttleD,rollP,rollI,rollD,pitchP,pitchI,pitchD,yawP,yawI,yawD,autoTakeoffFactor,auto_takeoff_speed,hoverOffset,vref_gain,vref_max);
        }

    };

    float throttleErrI = 0;
    float rollErrI = 0;
    float pitchErrI = 0;
    int autoLandThrottleDecrease = 0;
    float beforeTakeOffFactor = 1.0f;

    #define INITIALTHROTTLE 1050

    bool autoTakeOff = false;
    bool autoLand = false;
    bool autoControl = false;

    cv::Point3f predictTarget;
    bool convergedTarget;
    double timeToTarget;

    bool _fromfile;

    Arduino * _arduino;

    std::ofstream *_logger;
    void sendData(void);
    void readJoystick(void);
    void process_joystick();
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

    bool landed,rangeAlert;
    float posErrX,posErrY,posErrZ;
    float velErrX,velErrY,velErrZ;
    float velx_sp,vely_sp,velz_sp;

    void close (void);
    void init(std::ofstream *logger, bool fromfile, Arduino * arduino);
    void control(trackData data, cv::Point3f setpoint_world,cv::Point3f setspeed_world);
    bool getAutoControl() {return autoControl;}
    bool getDroneIsActive() {return (autoControl && autoThrottle > INITIALTHROTTLE) || (!autoControl && joyThrottle > INITIALTHROTTLE);}
    bool getAutoTakeOff() {return autoTakeOff;}
    bool getAutoLand() {return autoLand;}
    void setAutoLand(bool b) {autoLand = b;}
    void setAutoLandThrottleDecrease(int value) {autoLandThrottleDecrease = value;}
    void recalibrateHover();
    bool joystick_ready();

};

#endif //DRONECONTROLLER_H
