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

public:
    enum flight_mode{
        fm_inactive,
        fm_taking_off,
        fm_flying,
        fm_landing,
        fm_manual
    };
private:
    
    struct controlParameters{

        //height control
        int throttle_Pos = 1300;
        int throttle_Vel = 800;
        int throttle_Acc = 22;
        int throttleI = 15;

        int autoTakeoffFactor = 2;

        int hoverOffset = 30;

        //roll control
        int roll_Pos = 1750;
        int roll_Vel = 1100;
        int roll_Acc = 75;
        int rollI = 1;


        //pitch control
        int pitch_Pos = 1750;
        int pitch_Vel = 1100;
        int pitch_Acc = 60;
        int pitchI = 1;

        //yaw control
        int yawP = 0;
        int yawI = 0;
        int yawD = 666;

        float version = 2.0f;

        template <class Archive>
        void serialize( Archive & ar )
        {
            ar(version,throttle_Pos,throttle_Vel,throttle_Acc,throttleI,roll_Pos,roll_Vel,roll_Acc,rollI,pitch_Pos,pitch_Vel,pitch_Acc,pitchI,yawP,yawI,yawD,autoTakeoffFactor,hoverOffset);
        }

    };

    float throttleErrI = 0;
    float rollErrI = 0;
    float pitchErrI = 0;
    int autoLandThrottleDecrease = 0;


#define INITIALTHROTTLE 1050
    const int take_off_throttle_boost = 100;

    bool _fromfile;
    controlParameters params;
    flight_mode _flight_mode;

    int autoThrottle = 1000;
    int autoRoll = 1500;
    int autoPitch = 1500;
    int autoYaw = 1500;

    float hoverthrottle = INITIALTHROTTLE;

    Arduino * _arduino;

    std::ofstream *_logger;
    void sendData(void);
    void queue_commands(int throttle,int roll, int pitch, int yaw);
    void readJoystick(void);
    void process_joystick();
    void rebind(void);

public:   
    flight_mode get_flight_mode() {
        return _flight_mode;
    }
    void set_flight_mode(flight_mode f){
        if (_flight_mode != fm_manual)
            _flight_mode = f;
    }

    bool joySwitch = true;
    int joyDial = 0;
    float scaledjoydial = 0;
    int joyThrottle = 0;
    int joyRoll = 0;
    int joyPitch = 0;
    int joyYaw = 0;

    bool manual_override_take_off_now;
    bool manual_override_land_now;

    float posErrX,posErrY,posErrZ;
    float velErrX,velErrY,velErrZ;
    float accErrX,accErrY,accErrZ;
    float velx_sp,vely_sp,velz_sp;
    float accx_sp,accy_sp,accz_sp;

    void close (void);
    void init(std::ofstream *logger, bool fromfile, Arduino * arduino);
    void control(trackData data, cv::Point3f setpoint_world,cv::Point3f setspeed_world);
    bool getDroneIsActive() {
        if (_flight_mode != fm_manual && joyThrottle > INITIALTHROTTLE)
            return true;
        return (_flight_mode != fm_inactive && autoThrottle > INITIALTHROTTLE);
    }
    void setAutoLandThrottleDecrease(int value) {autoLandThrottleDecrease = value;}
    void recalibrateHover();
    bool joystick_ready();
    void init_ground_effect_compensation(){
        hoverthrottle -= params.hoverOffset*params.autoTakeoffFactor; // to compensate for ground effect and delay
    }

};

#endif //DRONECONTROLLER_H
