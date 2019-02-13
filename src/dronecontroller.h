#ifndef DRONECONTROLLER_H
#define DRONECONTROLLER_H

#include "dronetracker.h"
#include "joystick.hpp"
#include "arduino.h"
#include "multimodule.h"

static const char* joy_states_names[] = { "js_manual",
                                          "js_waypoint",
                                          "js_slider",
                                          "js_hunt",
                                          "js_disarmed",
                                          "js_none"
                                        };
static const char* flight_mode_names[] = { "fm_disarmed",
                                          "fm_inactive",
                                          "fm_manual",
                                          "fm_taking_off",
                                          "fm_flying",
                                          "fm_landing"
                                        };
/*
 * This class will control a micro drone via a Serial link
 *
 */
class DroneController {

public:
    enum flight_modes{
        fm_disarmed,
        fm_inactive,
        fm_manual,
        fm_taking_off,
        fm_flying,
        fm_landing
    };
    enum joy_mode_switch_modes{ // raw switch modes
        jmsm_manual,
        jmsm_waypoint,
        jmsm_slider,
        jmsm_hunt,
        jmsm_none, // in case the joystick does not have this switch
    };
    enum joy_states{ // end result after checking and processing
        js_manual,
        js_waypoint,
        js_slider,
        js_hunt,
        js_disarmed,
        js_none // in case of no joystick
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
        int pitch_Acc = 22;
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

    float depth_precision_gain = 0.2;
    float throttleErrI = 0;
    float rollErrI = 0;
    float pitchErrI = 0;
    int autoLandThrottleDecrease = 0;

    const int forward_pitch_take_off_boost = 0; // CX10 - 60
    const int min_throttle = JOY_BOUND_MIN+1; //Whoop - 600    CX10 - 800
    const float integratorThresholdDistance = 0.2f;
    float throttleBankFactor = 0.33; // Whoop 0.33


#if TX_TYPE == TX_CX10
#define INITIALTHROTTLE 200
#define PITCH_MIDDLE JOY_MIDDLE
#endif
#if TX_TYPE == TX_FRSKYD
#define INITIALTHROTTLE 200
#define INITIAL_HOVER_THROTTLE 800
#define PITCH_MIDDLE JOY_MIDDLE
#endif
#ifndef INITIALTHROTTLE
#define INITIALTHROTTLE 200
#define PITCH_MIDDLE JOY_MIDDLE
#endif
#ifndef INITIAL_HOVER_THROTTLE
#define INITIAL_HOVER_THROTTLE INITIALTHROTTLE
#endif

    const int take_off_throttle_boost = 0;

    bool _fromfile;
    controlParameters params;
    flight_modes _flight_mode = fm_disarmed; // only set externally (except for disarming), used internally
    joy_mode_switch_modes _joy_mode_switch = jmsm_none;
    bool _joy_arm_switch = true;
    bool _joy_takeoff_switch = false;
    joy_states _joy_state = js_none;
    int joyDial = 0;
    float scaledjoydial = 0;

    MultiModule * _rc;

    std::ofstream *_logger;
    void sendData(void);
    void queue_commands(int throttle,int roll, int pitch, int yaw);
    void readJoystick(void);
    void process_joystick();

public:
    //    flight_modes Flight_mode() {
    //        return _flight_mode;
    //    }
    void set_flight_mode(flight_modes f){
        _flight_mode = f;
    }
    joy_states Joy_State() {
        return _joy_state;
    }
    std::string Joy_State_str() {
        return joy_states_names[_joy_state];
    }
    std::string Flight_Mode() {
        return flight_mode_names[_flight_mode];
    }

    bool joy_arm_swtich(){
        return _joy_arm_switch;
    }
    bool joy_mode_swtich(){
        return _joy_mode_switch;
    }
    void insert_log(int joy_roll, int joy_pitch, int joy_yaw, int joy_throttle, int joyArmSwitch, int joyModeSwitch, int joyTakeOffSwitch){
        joyRoll = joy_roll;
        joyPitch= joy_pitch;
        joyYaw = joy_yaw;
        joyThrottle = joy_throttle;
        _joy_arm_switch = joyArmSwitch;
        _joy_mode_switch = static_cast<joy_mode_switch_modes>(joyModeSwitch);
        _joy_takeoff_switch = joyTakeOffSwitch;
    }


    int joyThrottle = JOY_BOUND_MIN;
    int joyRoll = JOY_MIDDLE;
    int joyPitch = JOY_MIDDLE;
    int joyYaw = JOY_MIDDLE;

    int autoThrottle = JOY_BOUND_MIN;
    int autoRoll = JOY_MIDDLE;
    int autoPitch = JOY_MIDDLE;
    int autoYaw = JOY_MIDDLE;

    float hoverthrottle = INITIAL_HOVER_THROTTLE;
    bool hoverthrottleInitialized = false;

    bool manual_override_take_off_now;
    bool manual_override_land_now;

    float posErrX,posErrY,posErrZ;
    float velErrX,velErrY,velErrZ;
    float accErrX,accErrY,accErrZ;
    float velx_sp,vely_sp,velz_sp;
    float accx_sp,accy_sp,accz_sp;

    void close (void);
    void init(std::ofstream *logger, bool fromfile, MultiModule *rc);
    void control(trackData data, cv::Point3f setpoint_world,cv::Point3f setspeed_world);
    bool getDroneIsActive() {
        if ( _flight_mode == fm_inactive || _flight_mode == fm_disarmed)
            return false;
        else if (_joy_mode_switch == jmsm_manual && joyThrottle > JOY_BOUND_MIN)
            return true;
        else
            return (autoThrottle > JOY_BOUND_MIN); //FIXME: check if this goes well if due to extreme control throttle is set to 0
    }
    void setAutoLandThrottleDecrease(int value) {autoLandThrottleDecrease = value;}
    void recalibrateHover();
    bool joystick_ready();
    void init_ground_effect_compensation(){
        hoverthrottle -= params.hoverOffset*params.autoTakeoffFactor; // to compensate for ground effect and delay
    }

    void blink_drone(bool b) {
        _rc->bind(b); // tmp trick until we create a dedicated feature for this
    }

};

#endif //DRONECONTROLLER_H
