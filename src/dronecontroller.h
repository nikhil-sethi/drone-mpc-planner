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
        int throttle_Pos = 1300;
        int throttle_Vel = 800;
        int throttle_Acc = 22;
        int throttleI = 15;

        int autoTakeoffFactor = 2;
        int auto_takeoff_speed = 3; // /100
        int hoverOffset = 30;

        //roll control
        int roll_Pos = 1750;
        int roll_Vel = 1100;
        int roll_Acc = 75;
        int rollI = 10;


        //pitch control
        int pitch_Pos = 1750;
        int pitch_Vel = 1100;
        int pitch_Acc = 60;
        int pitchI = 10;

        //yaw control
        int yawP = 0;
        int yawI = 0;
        int yawD = 0;


        template <class Archive>
        void serialize( Archive & ar )
        {
            ar( throttle_Pos,throttle_Vel,throttle_Acc,throttleI,roll_Pos,roll_Vel,roll_Acc,rollI,pitch_Pos,pitch_Vel,pitch_Acc,pitchI,yawP,yawI,yawD,autoTakeoffFactor,auto_takeoff_speed,hoverOffset);
        }

    };

    float throttleErrI = 0;
    float rollErrI = 0;
    float pitchErrI = 0;
    int autoLandThrottleDecrease = 0;
    float beforeTakeOffFactor = 1.0f;

    #define INITIALTHROTTLE 1050
    const int take_off_throttle_boost = 100;

    bool autoTakeOff = false;
    bool autoLand = false;
    bool autoControl = false;

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
    float accErrX,accErrY,accErrZ;
    float velx_sp,vely_sp,velz_sp;
    float accx_sp,accy_sp,accz_sp;

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
