#pragma once
#include "dronetracker.h"
#include "joystick.hpp"
#include "multimodule.h"
#include "common.h"
#define GRAVITY 9.81f

static const char* joy_states_names[] = { "js_manual",
                                          "js_waypoint",
                                          "js_hunt",
                                          "js_disarmed",
                                          "js_checking",
                                          "js_none"
                                        };
static const char* flight_mode_names[] = { "fm_joystick_check",
                                           "fm_disarmed",
                                           "fm_inactive",
                                           "fm_spinup",
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
                                           "fm_retry_aim_start",
                                           "fm_abort_flight",
                                           "fm_flying_pid_init",
                                           "fm_flying_pid",
                                           "fm_initial_reset_yaw",
                                           "fm_reset_yaw",
                                           "fm_landing_start",
                                           "fm_landing"
                                         };

class DroneController {

public:
    enum flight_modes {
        fm_joystick_check,
        fm_disarmed,
        fm_inactive,
        fm_spinup,
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
        fm_retry_aim_start,
        fm_abort_flight,
        fm_flying_pid_init,
        fm_flying_pid,
        fm_initial_reset_yaw,
        fm_reset_yaw,
        fm_landing_start,
        fm_landing
    };
    enum joy_mode_switch_modes { // raw switch modes
        jmsm_manual,
        jmsm_waypoint,
        jmsm_hunt,
        jmsm_none, // in case the joystick does not have this switch
    };
    enum joy_states { // end result after checking and processing
        js_manual,
        js_waypoint,
        js_hunt,
        js_disarmed,
        js_checking, // waiting for sticks to be reset
        js_none // in case of no joystick
    };
private:

    uint16_t kill_cnt_down = 0;
    double spin_up_start_time = 0;
    double start_takeoff_burn_time = 0;

    class ControlParameters: public xmls::Serializable
    {
    public:
        xmls::xInt kp_pos_roll,kp_pos_pitch,kp_pos_throttle;
        xmls::xInt ki_pos_roll,ki_pos_pitch,ki_pos_throttle;
        xmls::xInt kd_pos_roll,kd_pos_pitch,kd_pos_throttle;
        xmls::xInt kp_v_roll,kp_v_pitch,kp_v_throttle;
        xmls::xInt kd_v_roll,kd_v_pitch,kd_v_throttle;

        ControlParameters() {
            // Set the XML class name.
            // This name can differ from the C++ class name
            setClassName("ControlParameters");

            // Set class version
            setVersion("1.2");

            // Register members. Like the class name, member names can differ from their xml depandants
            Register("kp_pos_roll", &kp_pos_roll);
            Register("kp_pos_pitch", &kp_pos_pitch);
            Register("kp_pos_throttle", &kp_pos_throttle);
            Register("ki_pos_roll", &ki_pos_roll);
            Register("ki_pos_pitch", &ki_pos_pitch);
            Register("ki_pos_throttle", &ki_pos_throttle);
            Register("kd_pos_roll", &kd_pos_roll);
            Register("kd_pos_pitch", &kd_pos_pitch);
            Register("kd_pos_throttle", &kd_pos_throttle);
            Register("kp_v_roll", &kp_v_roll);
            Register("kp_v_pitch", &kp_v_pitch);
            Register("kp_v_throttle", &kp_v_throttle);
            Register("kd_v_roll", &kd_v_roll);
            Register("kd_v_pitch", &kd_v_pitch);
            Register("kd_v_throttle", &kd_v_throttle);

        }
    };

    float depth_precision_gain = 0.4f;
    float throttleErrI = 0;
    float rollErrI = 0;
    float pitchErrI = 0;
    int autoLandThrottleDecrease = 0;

    float time_spent_spinning_up(double time) {
        if (spin_up_start_time> 0)
            return static_cast<float>(time - spin_up_start_time );
        else
            return 0;
    }

    std::string settings_file;

    uint16_t initial_hover_throttle_guess_non3d;
    uint16_t initial_hover_throttle_guess() {
        if (dparams.mode3d)
            return initial_hover_throttle_guess_non3d / 2 + JOY_MIDDLE;
        else {
            return initial_hover_throttle_guess_non3d;
        }
    }
    uint16_t spinup_throttle() {
        if (dparams.mode3d)
            return JOY_MIDDLE +1;
        else {
            return dparams.spinup_throttle_non3d;
        }
    }
    uint16_t min_bound_throttle() {
        if (dparams.mode3d)
            return JOY_BOUND_MIN;
        else {
            return static_cast<uint16_t>(dparams.min_throttle);
        }
    }

    const float max_bank_angle = 180; // TODO: move to dparams (betaflight setting)
    const float aim_duration = 0.0833333333333f; // TODO: move to dparams, slightly related to full_bat_and_throttle_spinup_time. Should be 1/(bf_strenght/10) seconds
    const float transmission_delay_duration = 0.04f;
    float effective_burn_spin_up_duration = 0.15f; // the time to spin up from hover to max
    const float effective_burn_spin_down_duration = 0.1f; // the time to spin down from max to hover
    const float initial_thrust_guess = 50;
    float thrust = initial_thrust_guess; //60, 55, 50, 48, 45
    cv::Point3f drone_vel_after_takeoff = {0};
    float ground_effect = 1.0f;
    const float lift_off_dist_take_off_aim = 0.02f;
    const float take_off_burn_duration = 0.16f;
    float min_takeoff_angle = 30.f/180.f*static_cast<float>(M_PI);

    double take_off_start_time = 0;
    double interception_start_time = 0;
    track_data data_drone_1g_start;

    const float integratorThresholdDistance = 0.3f;
    cv::Point3f _burn_direction_for_thrust_approx = {0};

    float _dist_to_setpoint = 999;

    std::vector<cv::Point3f> aim_direction_history;

    bool initialized = false;
    bool _fromfile;
    flight_modes _flight_mode = fm_joystick_check; // only set externally (except for disarming), used internally
    joy_mode_switch_modes _joy_mode_switch = jmsm_none;
    betaflight_arming _joy_arm_switch = bf_armed;
    bool _joy_takeoff_switch = false;
    joy_states _joy_state = js_none;
    int joyDial = 0;
    float scaledjoydial = 0;

    int control_yaw(track_data data_drone, float gain_yaw);

    bool recovery_mode = false;
    cv::Point3f recovery_pos = {0};
    bool first_directional_burn = false;
    void approx_effective_thrust(track_data data_drone, cv::Point3f burn_direction, float burn_duration, float dt_burn);
    float thrust_to_throttle(float thrust_ratio);
    std::tuple<cv::Point3f, cv::Point3f, cv::Point3f> predict_drone_after_burn(state_data state_drone, cv::Point3f burn_direction, float remaining_aim_duration, float burn_duration);
    std::tuple<cv::Point3f, cv::Point3f> predict_drone_state_after_spindown(cv::Point3f integrated_pos, cv::Point3f integrated_vel, cv::Point3f burn_accelleration);
    std::tuple<int, int, float, cv::Point3f, std::vector<state_data> > calc_burn(state_data state_drone, state_data state_target, float remaining_aim_duration);
    std::tuple<int, int, float, cv::Point3f> calc_directional_burn(state_data state_drone, state_data state_target, float remaining_aim_duration);
    std::vector<state_data> predict_trajectory(float burn_duration, float remaining_aim_duration, cv::Point3f burn_direction, state_data state_drone);
    void draw_viz(state_data state_drone, state_data state_target, double time, cv::Point3f burn_direction, float burn_duration, float remaining_aim_duration, std::vector<state_data> traj);
    bool trajectory_in_view(std::vector<state_data> traj, CameraVolume::view_volume_check_mode c);

    std::tuple<cv::Point3f, cv::Point3f> keep_in_volume_check(track_data data_drone, cv::Point3f setpoint_pos, cv::Point3f setpoint_vel);
    void adapt_reffilter_dynamic(track_data data_drone, track_data data_target);

    std::tuple<float,float> acc_to_deg(cv::Point3f acc);
    std::tuple<float,float> acc_to_quaternion(cv::Point3f acc);

    void check_emergency_kill(track_data);
    void land(track_data data_drone, track_data data_target_new, bool headless_mode_disabled);
    void update_landing_yoffset(track_data data_drone, track_data data_target_new);
    void calc_ff_landing();
    void update_thrust_during_hovering(track_data data_drone, double time);

    cv::Point3f pos_err_i;
    int kp_pos_roll, kp_pos_throttle, kp_pos_pitch, ki_pos_roll, ki_pos_throttle, ki_pos_pitch, kd_pos_roll, kd_pos_throttle, kd_pos_pitch;
    int kp_v_roll, kp_v_throttle, kp_v_pitch, kd_v_roll, kd_v_throttle, kd_v_pitch;
    filtering::Tf_PT1_f filter_pos_err_x, filter_pos_err_y, filter_pos_err_z;
    filtering::Tf_PT1_f filter_vel_err_x, filter_vel_err_y, filter_vel_err_z;
    filtering::Tf_D_f d_pos_err_x, d_pos_err_y, d_pos_err_z;
    filtering::Tf_D_f d_vel_err_x, d_vel_err_y, d_vel_err_z;
    filtering::Tf_PT2_3f pos_reference_filter;
    void control_model_based(track_data data_drone, cv::Point3f setpoint_pos, cv::Point3f setpoint_vel, bool headless_mode_disabled);
    std::tuple<int,int,int> calc_feedforward_control(cv::Point3f desired_acceleration);

    MultiModule * _rc;
    tracking::DroneTracker * _dtrk;
    CameraVolume * _camvol;

    std::ofstream *_logger;
    void send_data_joystick(void);
    void read_joystick(void);
    void process_joystick();
    void deserialize_settings();
    void serialize_settings();

    float landing_yoffset = 0.f;
    float landing_velocity = -.15f;
    bool feedforward_landing = false;
    track_data previous_drone_data;
    double feedforward_land_time;
    double landing_time;

    inline state_data set_recoveryState(cv::Point3f position) {
        state_data rt;
        rt.pos = position;
        rt.vel = {0};
        rt.acc = {0};

        return rt;
    }

public:
    std::string flight_submode_name = "";
    void flight_mode(flight_modes f) {
        _flight_mode = f;
    }

    bool abort_take_off() {
        //check if the take off is not yet too far progressed to abort, if not go to spin up else return true

        if (_flight_mode == fm_take_off_aim) {
            float remaing_spinup_duration = dparams.full_bat_and_throttle_spinup_duration - aim_duration - time_spent_spinning_up(take_off_start_time);
            if (remaing_spinup_duration  < 0.05f)
                return false;
            _flight_mode = fm_spinup ;
            return true;

        } else if (_flight_mode == fm_spinup || _flight_mode==fm_start_takeoff) {
            _flight_mode = fm_spinup;
            return true;
        } else
            return false;
    }

    joy_states Joy_State() {
        return _joy_state;
    }
    std::string Joy_State_str() {
        return joy_states_names[_joy_state];
    }
    std::string flight_mode() {
        if(flight_submode_name.empty ()) {
            return flight_mode_names[_flight_mode];
        } else {
            return flight_submode_name;
        }
    }

    bool ff_interception() {
        return _flight_mode == fm_take_off_aim || _flight_mode == fm_max_burn || _flight_mode == fm_max_burn_spin_down || _flight_mode == fm_1g ||
               _flight_mode == fm_interception_aim_start  || _flight_mode == fm_interception_aim  || _flight_mode == fm_interception_burn_spin_down  ||
               _flight_mode == fm_interception_burn || _flight_mode == fm_interception_burn_start || _flight_mode == fm_retry_aim_start;
    }

    bool ff_completed() {
        return _flight_mode != fm_take_off_aim &&  _flight_mode != fm_max_burn && _flight_mode != fm_1g;
    }

    betaflight_arming joy_arm_switch() {
        return _joy_arm_switch;
    }
    joy_mode_switch_modes joy_mode_switch() {
        return _joy_mode_switch;
    }
    void insert_log(int log_joy_roll, int log_joy_pitch, int log_joy_yaw, int log_joy_throttle, int log_joy_arm_switch, int log_joy_mode_switch, int log_joy_take_off_switch,int log_auto_roll, int log_auto_pitch, int log_auto_throttle) {
        joy_roll = log_joy_roll;
        joy_pitch= log_joy_pitch;
        joy_yaw = log_joy_yaw;
        joy_throttle = log_joy_throttle;
        _joy_arm_switch = static_cast<betaflight_arming>(log_joy_arm_switch);
        _joy_mode_switch = static_cast<joy_mode_switch_modes>(log_joy_mode_switch);
        _joy_takeoff_switch = log_joy_take_off_switch;
        _log_auto_roll = log_auto_roll;
        _log_auto_pitch= log_auto_pitch;
        _log_auto_throttle = log_auto_throttle;
    }

    bool flight_aborted() {
        return _flight_mode == fm_abort_flight;
    }
    bool spinup() {
        return _flight_mode == fm_spinup;
    }

    float duration_spent_taking_off(double time) {
        if (start_takeoff_burn_time< 0.01)
            return 0;
        return  static_cast<float>(time - start_takeoff_burn_time) ;
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
    float Throttle() {
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
    /** @brief Determines the corresponding roll/pitch angle for a given command */
    float angle_of_command(int command) {
        command -= JOY_MIDDLE;
        float commandf = static_cast<float>(command)/static_cast<float>(JOY_BOUND_MAX - JOY_BOUND_MIN);
        return commandf*max_bank_angle;
    }

    float hoverthrottle;

    bool _manual_override_take_off_now;
    bool manual_override_take_off_now() { return _manual_override_take_off_now;}
    void reset_manual_override_take_off_now() {
        _manual_override_take_off_now = false;
        _joy_takeoff_switch = false;
    }

    cv::Point3f viz_drone_pos_after_burn = {0};
    cv::Point3f viz_target_pos_after_burn = {0};
    std::vector<state_data> viz_drone_trajectory;

    double viz_time_after_burn = {0};

    uint control_history_max_size;
    std::vector<control_data> control_history;

    float dist_to_setpoint() {
        return _dist_to_setpoint;
    }

    void close (void);
    void init(std::ofstream *logger, bool fromfile, MultiModule *rc, tracking::DroneTracker *dtrk, CameraVolume* camvol);
    void control(track_data, track_data, track_data, double);
    bool drone_is_active() {
        if ( _flight_mode == fm_inactive || _flight_mode == fm_disarmed)
            return false;
        else if (_joy_mode_switch == jmsm_manual && joy_throttle > JOY_BOUND_MIN && _joy_arm_switch == bf_armed)
            return true;
        else if ((_joy_mode_switch == jmsm_manual && joy_throttle <= JOY_BOUND_MIN) || _joy_arm_switch == bf_disarmed)
            return false;
        else
            return ((auto_throttle > JOY_BOUND_MIN && _flight_mode != fm_spinup) || _flight_mode == fm_start_takeoff || _flight_mode == fm_take_off_aim || _flight_mode == fm_max_burn || _flight_mode == fm_1g ); //FIXME: check if this goes well if due to extreme control throttle is set to 0
    }

    bool drone_state_inactive() {
        return _flight_mode == fm_inactive;
    }

    bool joystick_ready();

    void blink_by_binding(bool b) {
        _rc->bind(b); // tmp trick until we create a dedicated feature for this
    }

    void blink(double time) {
        static double last_blink_time = time;
        static bool blink_state;

        if (static_cast<float>(time-last_blink_time)>dparams.blink_period) {
            if (blink_state)
                blink_state = false;
            else
                blink_state = true;
            last_blink_time = time;
        }
        _rc->LED(blink_state);
    }

    void LED(bool b) {
        _rc->LED(b);
    }
};
