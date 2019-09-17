#ifndef DRONECONTROLLER_H
#define DRONECONTROLLER_H

#include "dronetracker.h"
#include "joystick.hpp"
#include "multimodule.h"
#include "common.h"
#define GRAVITY 9.81f

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
                                          "fm_take_off_aim",
                                          "fm_max_burn",
                                          "fm_max_burn_spin_down",
                                          "fm_1g",
                                          "fm_interception_aim_start",
                                          "fm_interception_aim",
                                          "fm_interception_burn_start",
                                          "fm_interception_burn",
                                          "fm_interception_burn_spin_down",
                                          "fm_interception_1g",
                                          "fm_retry_aim_start",
                                          "fm_abort_takeoff",
                                          "fm_flying_pid_init",
                                          "fm_flying_pid",
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
        fm_take_off_aim,
        fm_max_burn,
        fm_max_burn_spin_down,
        fm_1g,
        fm_interception_aim_start,
        fm_interception_aim,
        fm_interception_burn_start,
        fm_interception_burn,
        fm_interception_burn_spin_down,
        fm_interception_1g,
        fm_retry_aim_start,
        fm_abort_takeoff,
        fm_flying_pid_init,
        fm_flying_pid,
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

    std::string settings_file;

    const float max_bank_angle = 180; // TODO: move to dparams (betaflight setting)
    const float aim_duration = 0.2f; // TODO: move to dparams, slightly related to full_bat_and_throttle_spinup_time. Should be 1/(bf_strenght/10) seconds
    const float transmission_delay_duration = 0.03f;
    const float effective_burn_spin_up_duration = 0.1f; // the time to spin up from hover to max
    const float effective_burn_spin_down_duration = 0.1f; // the time to spin down from max to hover
    float thrust = 3.0f*GRAVITY;
    float ground_effect = 1.0f;
    const float lift_off_dist_take_off_aim = 0.02f;

    double take_off_start_time = 0;
    double interception_start_time = 0;
    track_data drone_1g_start_pos;

    const float integratorThresholdDistance = 0.2f;
    cv::Point3f _burn_direction_for_thrust_approx = {0};

    bool initialized = false;
    bool _fromfile;
    flight_modes _flight_mode = fm_joystick_check; // only set externally (except for disarming), used internally
    joy_mode_switch_modes _joy_mode_switch = jmsm_none;
    bool _joy_arm_switch = true;
    bool _joy_takeoff_switch = false;
    joy_states _joy_state = js_none;
    int joyDial = 0;
    float scaledjoydial = 0;

    void approx_effective_thrust(track_data state_drone, cv::Point3f burn_direction, float burn_duration, float dt);
    std::tuple<cv::Point3f, cv::Point3f, cv::Point3f> predict_drone_state_after_burn(cv::Point3f current_drone_pos, cv::Point3f drone_vel, cv::Point3f burn_direction, cv::Point3f burn_accelleration_max, float remaining_aim_duration, float burn_duration);
    std::tuple<cv::Point3f, cv::Point3f> predict_drone_state_after_spindown(cv::Point3f integrated_pos, cv::Point3f integrated_vel, cv::Point3f burn_accelleration);
    std::tuple<int, int, float> calc_directional_burn(track_data state_drone, track_data state_insect, double aim_start_time, double time);
    std::tuple<int, int, float> calc_directional_burn(track_data state_drone_start_1g, track_data state_drone, track_data state_insect, double aim_start_time, double time);
    std::tuple<int, int, float> calc_directional_burn(cv::Point3f drone_vel, track_data state_drone,track_data state_insect, double aim_start_time, double time);
    std::tuple<float,float> convert_acc_to_deg(cv::Point3f acc);

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
    bool ff_interception() {
        return _flight_mode == fm_take_off_aim || _flight_mode == fm_max_burn || _flight_mode == fm_max_burn_spin_down || _flight_mode == fm_1g ||
               _flight_mode == fm_interception_aim_start  || _flight_mode == fm_interception_aim  || _flight_mode == fm_interception_burn_spin_down  ||
               _flight_mode == fm_interception_1g || _flight_mode == fm_interception_burn || _flight_mode == fm_interception_burn_start || _flight_mode == fm_retry_aim_start;
    }

    bool ff_completed() {
        return _flight_mode != fm_take_off_aim &&  _flight_mode != fm_max_burn && _flight_mode != fm_1g;
    }

    bool joy_arm_switch(){
        return _joy_arm_switch;
    }
    bool joy_mode_switch(){
        return _joy_mode_switch;
    }
    void insert_log(int log_joy_roll, int log_joy_pitch, int log_joy_yaw, int log_joy_throttle, int log_joy_arm_switch, int log_joy_mode_switch, int log_joy_take_off_switch,int log_auto_roll, int log_auto_pitch, int log_auto_throttle){
        joy_roll = log_joy_roll;
        joy_pitch= log_joy_pitch;
        joy_yaw = log_joy_yaw;
        joy_throttle = log_joy_throttle;
        _joy_arm_switch = log_joy_arm_switch;
        _joy_mode_switch = static_cast<joy_mode_switch_modes>(log_joy_mode_switch);
        _joy_takeoff_switch = log_joy_take_off_switch;
        _log_auto_roll = log_auto_roll;
        _log_auto_pitch= log_auto_pitch;
        _log_auto_throttle = log_auto_throttle;
    }

    int joy_throttle = JOY_BOUND_MIN;
    int joy_roll = JOY_MIDDLE;
    int joy_pitch = JOY_MIDDLE;
    int joy_yaw = JOY_MIDDLE;

    int auto_throttle = JOY_BOUND_MIN;
    int auto_roll = JOY_MIDDLE;
    int auto_pitch = JOY_MIDDLE;
    int auto_yaw = JOY_MIDDLE;
    float auto_burn_duration = 0;

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
    const float tmp_hover_throttle = 650; // TODO: LUDWIG HELP initial hover throttle...

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

    cv::Point3f viz_pos_after_aim = {0};
    double viz_time_after_aim = {0};
    cv::Point3f viz_pos_after_burn = {0};
    std::vector<cv::Point3f> viz_drone_trajectory;

    double viz_time_after_burn = {0};

    uint control_history_max_size;
    std::vector<control_data> control_history;

    void close (void);
    void init(std::ofstream *logger, bool fromfile, MultiModule *rc, DroneTracker *dtrk);
    void control(track_data data, track_data state_insect, cv::Point3f setpoint_pos_world, cv::Point3f setpoint_vel_world, cv::Point3f setpoint_acc_world, double time);
    bool drone_is_active() {
        if ( _flight_mode == fm_inactive || _flight_mode == fm_disarmed)
            return false;
        else if (_joy_mode_switch == jmsm_manual && joy_throttle > JOY_BOUND_MIN)
            return true;
        else if (_joy_mode_switch == jmsm_manual && joy_throttle <= JOY_BOUND_MIN)
            return false;
        else
            return (auto_throttle > JOY_BOUND_MIN || _flight_mode == fm_start_takeoff || _flight_mode == fm_take_off_aim || _flight_mode == fm_max_burn || _flight_mode == fm_1g ); //FIXME: check if this goes well if due to extreme control throttle is set to 0
    }
    void setAutoLandThrottleDecrease(int value) {autoLandThrottleDecrease = value;}
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
