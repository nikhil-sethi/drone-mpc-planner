#ifndef DRONECONTROLLER_H
#define DRONECONTROLLER_H

#include "dronetracker.h"
#include "joystick.hpp"
#include "multimodule.h"
#include "common.h"

static const char* joy_states_names[] = { "js_manual",
                                          "js_waypoint",
                                          "js_slider",
                                          "js_hunt",
                                          "js_disarmed",
                                          "js_checking",
                                          "js_none"
                                        };
static const char* flight_mode_names[] = { "fm_joystick_check",
                                           "fm_disarmed",
                                           "fm_inactive",
                                           "fm_manual",
                                           "fm_start_takeoff",
                                           "fm_take_off_max",
                                           "fm_take_off_min",
                                           "fm_abort_takeoff",
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
        fm_joystick_check,
        fm_disarmed,
        fm_inactive,
        fm_manual,
        fm_start_takeoff,
        fm_take_off_max_burn,
        fm_take_off_idle,
        fm_abort_takeoff,
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
        js_checking, // waiting for sticks to be reset
        js_none // in case of no joystick
    };
private:

    int gain_throttle_pos,gain_throttle_vel,gain_throttle_acc,gain_throttle_i;
    int gain_roll_pos,gain_roll_vel,gain_roll_acc,gain_roll_i;
    int gain_pitch_pos,gain_pitch_vel,gain_pitch_acc,gain_pitch_i;

    class ControlParameters: public xmls::Serializable
    {
    public:
        xmls::xInt gain_throttle_pos,gain_throttle_vel,gain_throttle_acc,gain_throttle_i;
        xmls::xInt gain_roll_pos,gain_roll_vel,gain_roll_acc,gain_roll_i;
        xmls::xInt gain_pitch_pos,gain_pitch_vel,gain_pitch_acc,gain_pitch_i;

        ControlParameters() {
            // Set the XML class name.
            // This name can differ from the C++ class name
            setClassName("ControlParameters");

            // Set class version
            setVersion("1.0");

            // Register members. Like the class name, member names can differ from their xml depandants
            Register("gain_throttle_pos", &gain_throttle_pos);
            Register("gain_throttle_vel", &gain_throttle_vel);
            Register("gain_throttle_acc", &gain_throttle_acc);
            Register("gain_throttle_i", &gain_throttle_i);
            Register("gain_roll_pos", &gain_roll_pos);
            Register("gain_roll_vel", &gain_roll_vel);
            Register("gain_roll_acc", &gain_roll_acc);
            Register("gain_roll_i", &gain_roll_i);
            Register("gain_pitch_pos", &gain_pitch_pos);
            Register("gain_pitch_vel", &gain_pitch_vel);
            Register("gain_pitch_acc", &gain_pitch_acc);
            Register("gain_pitch_i", &gain_pitch_i);
        }
    };

    float depth_precision_gain = 0.4;
    float throttleErrI = 0;
    float rollErrI = 0;
    float pitchErrI = 0;
    int autoLandThrottleDecrease = 0;

    const int forward_pitch_take_off_boost = 0; // CX10 - 60
    std::string settings_file;

    double take_off_start_time = 0;

    const float integratorThresholdDistance = 0.2f;

    bool initialized = false;

    const int take_off_throttle_boost = 0;

    bool _fromfile;
    flight_modes _flight_mode = fm_joystick_check; // only set externally (except for disarming), used internally
    joy_mode_switch_modes _joy_mode_switch = jmsm_none;
    bool _joy_arm_switch = true;
    bool _joy_takeoff_switch = false;
    joy_states _joy_state = js_none;
    int joyDial = 0;
    float scaledjoydial = 0;

    MultiModule * _rc;
    DroneTracker * _dtrk;

    std::ofstream *_logger;
    void sendData(void);
    void readJoystick(void);
    void process_joystick();
    void deserialize_settings();
    void serialize_settings();

public:
    void flight_mode(flight_modes f){
        _flight_mode = f;
    }
    joy_states Joy_State() {
        return _joy_state;
    }
    std::string Joy_State_str() {
        return joy_states_names[_joy_state];
    }
    std::string flight_mode() {
        return flight_mode_names[_flight_mode];
    }

    bool joy_arm_switch(){
        return _joy_arm_switch;
    }
    bool joy_mode_switch(){
        return _joy_mode_switch;
    }
    void insert_log(int joy_roll, int joy_pitch, int joy_yaw, int joy_throttle, int joyArmSwitch, int joyModeSwitch, int joyTakeOffSwitch,int auto_roll, int auto_pitch, int auto_throttle){
        joyRoll = joy_roll;
        joyPitch= joy_pitch;
        joyYaw = joy_yaw;
        joyThrottle = joy_throttle;
        _joy_arm_switch = joyArmSwitch;
        _joy_mode_switch = static_cast<joy_mode_switch_modes>(joyModeSwitch);
        _joy_takeoff_switch = joyTakeOffSwitch;
        _log_auto_roll = auto_roll;
        _log_auto_pitch= auto_pitch;
        _log_auto_throttle = auto_throttle;
    }

    int joyThrottle = JOY_BOUND_MIN;
    int joyRoll = JOY_MIDDLE;
    int joyPitch = JOY_MIDDLE;
    int joyYaw = JOY_MIDDLE;

    int autoThrottle = JOY_BOUND_MIN;
    int autoRoll = JOY_MIDDLE;
    int autoPitch = JOY_MIDDLE;
    int autoYaw = JOY_MIDDLE;

    //Normalized throttle, between [-1 .. 1].
    //0 equals hoverthrottle
    float _log_auto_throttle;
    float Throttle(){
        float throttle = _rc->throttle;
        if (_fromfile)
            throttle  = _log_auto_throttle;
        throttle -= hoverthrottle;
        throttle /= static_cast<float>(JOY_BOUND_MAX - JOY_BOUND_MIN);
        return throttle;
    }
    //Normalized roll, between [-1 .. 1].
    float _log_auto_roll;
    float Roll() {
        float roll = _rc->roll;
        if (_fromfile)
            roll  = _log_auto_roll;
        roll -= JOY_MIDDLE;
        roll /= static_cast<float>(JOY_BOUND_MAX - JOY_BOUND_MIN);
        return roll;
    }
    //Normalized pitch, between [0 .. 1].
    float _log_auto_pitch;
    float Pitch() {
        float pitch = _rc->pitch;
        if (_fromfile)
            pitch = _log_auto_pitch;
        pitch -= JOY_MIDDLE;
        pitch /= static_cast<float>(JOY_BOUND_MAX - JOY_BOUND_MIN);
        return pitch;
    }

    const int initial_throttle = 200;
    float hoverthrottle;

    bool _manual_override_take_off_now;
    bool manual_override_take_off_now() { return _manual_override_take_off_now;}
    void reset_manual_override_take_off_now() {
        _manual_override_take_off_now = false;
        _joy_takeoff_switch = false;
    }

    float posErrX,posErrY,posErrZ;
    float velErrX,velErrY,velErrZ;
    float accErrX,accErrY,accErrZ;
    float velx_sp,vely_sp,velz_sp;
    float accx_sp,accy_sp,accz_sp;

    uint control_history_max_size;
    std::vector<control_data> control_history;

    void close (void);
    void init(std::ofstream *logger, bool fromfile, MultiModule *rc, DroneTracker *dtrk);
    void control(track_data data, cv::Point3f setpoint_pos_world, cv::Point3f setpoint_vel_world, cv::Point3f setpoint_acc_world);
    bool drone_is_active() {
        if ( _flight_mode == fm_inactive || _flight_mode == fm_disarmed)
            return false;
        else if (_joy_mode_switch == jmsm_manual && joyThrottle > JOY_BOUND_MIN)
            return true;
        else if (_joy_mode_switch == jmsm_manual && joyThrottle <= JOY_BOUND_MIN)
            return false;
        else
            return (autoThrottle > JOY_BOUND_MIN || _flight_mode == fm_start_takeoff || _flight_mode == fm_take_off_max_burn); //FIXME: check if this goes well if due to extreme control throttle is set to 0
    }
    void setAutoLandThrottleDecrease(int value) {autoLandThrottleDecrease = value;}
    void recalibrateHover();
    bool joystick_ready();

    void blink_by_binding(bool b) {
        _rc->bind(b); // tmp trick until we create a dedicated feature for this
    }

    void blink(double time) {
        static double last_blink_time = time;
        static bool blink_state;

        if (time-last_blink_time>bind_blink_time) {
            if (blink_state)
                blink_state = false;
            else
                blink_state = true;
            last_blink_time = time;
        }
        _rc->LED(blink_state);
    }

    void LED(bool b){
        _rc->LED(b);
    }
};

#endif //DRONECONTROLLER_H
