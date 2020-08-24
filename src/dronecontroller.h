#pragma once
#include "dronetracker.h"
#include "joystick.hpp"
#include "multimodule.h"
#include "common.h"
#include "cameraview.h"
#define GRAVITY 9.81f
#define DRONECONTROLLER_DEBUG false
#define ENABLE_SPINUP true

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
                                           "fm_blink",
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
                                           "fm_abort_takeoff",
                                           "fm_tracking_lost",
                                           "fm_model_error",
                                           "fm_abort",
                                           "fm_pid_init",
                                           "fm_pid",
                                           "fm_headed_pid",
                                           "fm_initial_reset_yaw",
                                           "fm_reset_yaw",
                                           "fm_ff_landing_start",
                                           "fm_shake_it_baby",
                                           "fm_ff_landing"
                                         };

class DroneController {

public:
    enum flight_modes {
        fm_joystick_check,
        fm_disarmed,
        fm_inactive,
        fm_blink,
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
        fm_abort_takeoff,
        fm_abort_tracking_lost,
        fm_abort_model_error,
        fm_abort,
        fm_flying_pid_init,
        fm_flying_pid,
        fm_flying_headed_pid,
        fm_initial_reset_yaw,
        fm_reset_yaw,
        fm_ff_landing_start,
        fm_shake_it_baby,
        fm_ff_landing
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
        xmls::xInt ki_pos_roll,ki_pos_pitch,ki_thrust;
        xmls::xInt kd_pos_roll,kd_pos_pitch,kd_pos_throttle;
        xmls::xInt kp_pos_roll_hover,kp_pos_pitch_hover,kp_pos_throttle_hover;
        xmls::xInt ki_pos_roll_hover,ki_pos_pitch_hover,ki_thrust_hover;
        xmls::xInt kd_pos_roll_hover,kd_pos_pitch_hover,kd_pos_throttle_hover;
        xmls::xInt kp_v_roll,kp_v_pitch,kp_v_throttle;
        xmls::xInt kd_v_roll,kd_v_pitch,kd_v_throttle;

        ControlParameters() {
            // Set the XML class name.
            // This name can differ from the C++ class name
            setClassName("ControlParameters");

            // Set class version
            setVersion("1.4");

            // Register members. Like the class name, member names can differ from their xml depandants
            Register("kp_pos_roll", &kp_pos_roll);
            Register("kp_pos_pitch", &kp_pos_pitch);
            Register("kp_pos_throttle", &kp_pos_throttle);
            Register("ki_pos_roll", &ki_pos_roll);
            Register("ki_pos_pitch", &ki_pos_pitch);
            Register("ki_thrust", &ki_thrust);
            Register("kd_pos_roll", &kd_pos_roll);
            Register("kd_pos_pitch", &kd_pos_pitch);
            Register("kd_pos_throttle", &kd_pos_throttle);

            Register("kp_pos_roll_hover", &kp_pos_roll_hover);
            Register("kp_pos_pitch_hover", &kp_pos_pitch_hover);
            Register("kp_pos_throttle_hover", &kp_pos_throttle_hover);
            Register("ki_pos_roll_hover", &ki_pos_roll_hover);
            Register("ki_pos_pitch_hover", &ki_pos_pitch_hover);
            Register("ki_thrust_hover", &ki_thrust_hover);
            Register("kd_pos_roll_hover", &kd_pos_roll_hover);
            Register("kd_pos_pitch_hover", &kd_pos_pitch_hover);
            Register("kd_pos_throttle_hover", &kd_pos_throttle_hover);

            Register("kp_v_roll", &kp_v_roll);
            Register("kp_v_pitch", &kp_v_pitch);
            Register("kp_v_throttle", &kp_v_throttle);
            Register("kd_v_roll", &kd_v_roll);
            Register("kd_v_pitch", &kd_v_pitch);
            Register("kd_v_throttle", &kd_v_throttle);

        }
    };

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
    void set_led_strength(float exposure);

    std::string settings_file;

    uint16_t initial_hover_throttle_guess_non3d;
    uint16_t initial_hover_throttle_guess() {
        if (dparams.mode3d)
            return initial_hover_throttle_guess_non3d / 2 + RC_MIDDLE;
        else {
            return initial_hover_throttle_guess_non3d;
        }
    }
    uint16_t spinup_throttle() {
        if (dparams.mode3d)
            return RC_MIDDLE +1;
        else {
            return dparams.spinup_throttle_non3d;
        }
    }
    uint16_t min_bound_throttle() {
        if (dparams.mode3d)
            return RC_BOUND_MIN;
        else {
            return static_cast<uint16_t>(dparams.min_throttle);
        }
    }

    const float max_bank_angle = 180; // TODO: move to dparams (betaflight setting)
    const float aim_duration = 0.0833333333333f; // TODO: move to dparams, slightly related to full_bat_and_throttle_spinup_time. Should be 1/(bf_strenght/10) seconds
    const float transmission_delay_duration = 0.04f;
    float effective_burn_spin_up_duration = 0.15f; // the time to spin up from hover to max
    const float effective_burn_spin_down_duration = 0.1f; // the time to spin down from max to hover
    float thrust;
    cv::Point3f drone_vel_after_takeoff = {0};
    float ground_effect = 1.0f;
    const float lift_off_dist_take_off_aim = 0.02f;
    const float take_off_burn_duration = 0.08f;
    float min_takeoff_angle = 45.f/180.f*static_cast<float>(M_PI);

    double take_off_start_time = 0;
    double interception_start_time = 0;
    double in_flight_start_time = -1;
    double ff_land_start_time = 0;
    int ff_auto_throttle_start;
    track_data data_drone_1g_start;

    const float integratorThresholdDistance = 0.3f;
    cv::Point3f _burn_direction_for_thrust_approx = {0};

    float _dist_to_setpoint = 999;
    double _time;
    double time_waypoint_changed = 0;

    std::vector<cv::Point3f> aim_direction_history;

    bool initialized = false;
    bool log_replay_mode = false;
    bool generator_mode = false;
    flight_modes _flight_mode = fm_joystick_check; // only set externally (except for disarming), used internally
    joy_mode_switch_modes _joy_mode_switch = jmsm_none;
    betaflight_arming _joy_arm_switch = bf_armed;
    bool _joy_takeoff_switch = false;
    joy_states _joy_state = js_none;
    int joyDial = 0;
    float scaledjoydial = 0;
    bool _hover_mode = false;

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
    bool trajectory_in_view(std::vector<state_data> traj, CameraView::view_volume_check_mode c);

    void correct_yaw(float deviation_angle);

    cv::Point3f keep_in_volume_correction_acceleration(track_data data_drone);
    cv::Point3f kiv_acceleration(std::array<bool, N_PLANES> violated_planes_inview, std::array<bool, N_PLANES> violated_planes_brakedistance);

    void adapt_reffilter_dynamic(track_data data_drone, track_data data_target);

    std::tuple<float,float> acc_to_deg(cv::Point3f acc);
    std::tuple<float,float> acc_to_quaternion(cv::Point3f acc);

    void check_emergency_kill(track_data data_drone);
    void check_tracking_lost(track_data data_drone);
    void check_control_and_tracking_problems(track_data data_drone);
    void update_thrust_during_hovering(track_data data_drone, double time);

    void blink(double time);
    void blink_motors(double time);

    cv::Point3f pos_err_i;
    int kp_pos_roll, kp_pos_throttle, kp_pos_pitch, ki_pos_roll, ki_thrust, ki_pos_pitch, kd_pos_roll, kd_pos_throttle, kd_pos_pitch;
    int kp_pos_roll_hover, kp_pos_throttle_hover, kp_pos_pitch_hover, ki_pos_roll_hover, ki_thrust_hover, ki_pos_pitch_hover, kd_pos_roll_hover, kd_pos_throttle_hover, kd_pos_pitch_hover;
    int kp_v_roll, kp_v_throttle, kp_v_pitch, kd_v_roll, kd_v_throttle, kd_v_pitch;
    filtering::Tf_D_f d_pos_err_x, d_pos_err_y, d_pos_err_z;
    filtering::Tf_D_f d_vel_err_x, d_vel_err_y, d_vel_err_z;
    filtering::Tf_PT2_3f pos_reference_filter;

    filtering::Tf_PT2_f pos_modelx, pos_modely, pos_modelz;

    std::array<float, N_PLANES> pos_err_kiv= {0}, vel_err_kiv= {0};
    std::array<filtering::Tf_D_f, N_PLANES> d_pos_err_kiv, d_vel_err_kiv;
    void control_model_based(track_data data_drone, cv::Point3f setpoint_pos, cv::Point3f setpoint_vel);

    std::tuple<cv::Point3f, cv::Point3f, cv::Point3f, cv::Point3f, cv::Point3f> adjust_control_gains(track_data drone_data, cv::Point3f setpoint_pos, cv::Point3f setpoint_vel);
    std::tuple<cv::Point3f, cv::Point3f, cv::Point3f, cv::Point3f> control_error(track_data data_drone, cv::Point3f setpoint_pos, cv::Point3f setpoint_vel, cv::Point3f ki_pos);
    std::tuple<int,int,int> calc_feedforward_control(cv::Point3f desired_acceleration);

    MultiModule * _rc;
    tracking::DroneTracker * _dtrk;
    CameraView * _camview;

    std::ofstream *_logger;
    void send_data_joystick(void);
    void read_joystick(void);
    void process_joystick();
    void deserialize_settings();
    void serialize_settings();

    track_data previous_drone_data;

    inline state_data set_recoveryState(cv::Point3f position) {
        state_data rt;
        rt.pos = position;
        rt.vel = {0};
        rt.acc = {0};

        return rt;
    }
public:
    float model_error;
    void flight_mode(flight_modes f) {
        if (f != fm_abort || !flight_aborted())
            _flight_mode = f;
    }
    void hover_mode(bool value) { _hover_mode = value;}
    void double_led_strength() {
        dparams.drone_led_strength = std::clamp(dparams.drone_led_strength*2,5,100);
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
        std::string ME = " ME: " + to_string_with_precision(model_error,0);
        return flight_mode_names[_flight_mode] + ME;
    }

    bool ff_interception() {
        return _flight_mode == fm_take_off_aim || _flight_mode == fm_max_burn || _flight_mode == fm_max_burn_spin_down || _flight_mode == fm_1g ||
               _flight_mode == fm_interception_aim_start  || _flight_mode == fm_interception_aim  || _flight_mode == fm_interception_burn_spin_down  ||
               _flight_mode == fm_interception_burn || _flight_mode == fm_interception_burn_start || _flight_mode == fm_retry_aim_start;
    }

    bool at_base() {
        return _flight_mode<=fm_take_off_aim;
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
    bool joy_takeoff_switch() {
        return _joy_takeoff_switch;
    }
    void joy_takeoff_switch_file_trigger(bool value) {
        _joy_takeoff_switch = value;
    }
    void insert_log(int log_joy_roll, int log_joy_pitch, int log_joy_yaw, int log_joy_throttle, int log_joy_arm_switch, int log_joy_mode_switch, int log_joy_take_off_switch,int log_auto_roll, int log_auto_pitch, int log_auto_throttle, float log_acc_z, uint16_t log_throttle, float log_throttle_s, float log_max_thrust, float log_thrust_rpm) {
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
        _log_acc_z = log_acc_z;
        _log_throttle = log_throttle;
        _log_throttle_s = log_throttle_s;
        _log_max_thrust = log_max_thrust;
        _log_thrust_rpm = log_thrust_rpm;
    }

    bool flight_aborted() {
        return _flight_mode == fm_abort || _flight_mode == fm_abort_model_error || _flight_mode == fm_abort_tracking_lost || _flight_mode == fm_abort_takeoff;
    }

    float in_flight_duration(double time) {
        if ((_flight_mode == fm_flying_pid || _flight_mode == fm_reset_yaw || _flight_mode == fm_ff_landing || _flight_mode == fm_initial_reset_yaw)
                && in_flight_start_time < 0)
            in_flight_start_time = time;
        else if (_flight_mode == fm_disarmed || _flight_mode == fm_inactive || _flight_mode == fm_spinup || _flight_mode == fm_manual)
            in_flight_start_time = -1;

        if (in_flight_start_time > 0)
            return static_cast<float>(time - in_flight_start_time);
        else
            return 0;
    }
    bool spinup() {
        return _flight_mode == fm_spinup;
    }
    bool landing() {
        return _flight_mode == fm_ff_landing || _flight_mode ==fm_ff_landing_start;
    }

    float duration_spent_taking_off(double time) {
        if (start_takeoff_burn_time< 0.01)
            return 0;
        return  static_cast<float>(time - start_takeoff_burn_time) ;
    }

    void nav_waypoint_changed(double time) {
        time_waypoint_changed = time;
    }
    float duration_since_waypoint_changed(double time) {
        return  static_cast<float>(time - time_waypoint_changed) ;
    }

    int joy_throttle = RC_BOUND_MIN;
    int joy_roll = RC_MIDDLE;
    int joy_pitch = RC_MIDDLE;
    int joy_yaw = RC_MIDDLE;

    int auto_throttle = RC_BOUND_MIN;
    int auto_roll = RC_MIDDLE;
    int auto_pitch = RC_MIDDLE;
    int auto_yaw = RC_MIDDLE;
    float auto_burn_duration = 0;

    //Normalized throttle, between [-1 .. 1].
    //0 equals hoverthrottle
    float _log_auto_throttle;
    float Throttle() {
        float throttle = _rc->throttle;
        if (log_replay_mode)
            throttle  = _log_auto_throttle;
        throttle /= static_cast<float>(RC_BOUND_MAX - RC_BOUND_MIN);
        return throttle;
    }
    //Normalized roll, between [-1 .. 1].
    float _log_auto_roll;
    float Roll() {
        float roll = _rc->roll;
        if (log_replay_mode)
            roll  = _log_auto_roll;
        roll -= RC_MIDDLE;
        roll /= static_cast<float>(RC_BOUND_MAX - RC_BOUND_MIN);
        return roll;
    }
    //Normalized pitch, between [0 .. 1].
    float _log_auto_pitch;
    float Pitch() {
        float pitch = _rc->pitch;
        if (log_replay_mode)
            pitch = _log_auto_pitch;
        pitch -= RC_MIDDLE;
        pitch /= static_cast<float>(RC_BOUND_MAX - RC_BOUND_MIN);
        return pitch;
    }
    float _log_acc_z;
    float telem_acc_z() {
        float acc_z = _rc->sensor.acc.z;
        if (log_replay_mode)
            acc_z = _log_acc_z;
        return acc_z;
    }
    uint16_t _log_throttle;
    uint16_t telem_throttle() {
        uint16_t throttle = _rc->sensor.throttle;
        if (log_replay_mode)
            throttle = _log_throttle;
        return throttle;
    }
    float _log_throttle_s;
    float telem_throttle_s() {
        float throttle_s = _rc->sensor.throttle_scaled;
        if (log_replay_mode)
            throttle_s = _log_throttle_s;
        return throttle_s;
    }
    float _log_max_thrust;
    float telem_max_thrust() {
        // max_thrust = _rc->sensor.thrust_max;
        // if (log_replay_mode)
        //     max_thrust = _log_max_thrust;
        return dparams.thrust+5; // tmp
    }
    float _log_thrust_rpm;
    float telem_thrust_rpm() {
        float telem_thrust_rpm = _rc->sensor.thrust_rpm;
        if (log_replay_mode)
            telem_thrust_rpm = _log_thrust_rpm;
        return telem_thrust_rpm;
    }


    /** @brief Determines the corresponding roll/pitch angle for a given command */
    float angle_of_command(int command) {
        command -= RC_MIDDLE;
        float commandf = static_cast<float>(command)/static_cast<float>(RC_BOUND_MAX - RC_BOUND_MIN);
        return commandf*max_bank_angle;
    }

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
    void init(std::ofstream *logger, bool fromfile, bool generator_mode, MultiModule *rc, tracking::DroneTracker *dtrk, CameraView* camvol,float exposure);
    void control(track_data, track_data, track_data, double);
    bool drone_is_active() {
        if ( _flight_mode == fm_inactive || _flight_mode == fm_disarmed || _flight_mode == fm_joystick_check)
            return false;
        else if (flight_aborted())
            return false;
        else if (_joy_mode_switch == jmsm_manual && joy_throttle > RC_BOUND_MIN && _joy_arm_switch == bf_armed)
            return true;
        else if ((_joy_mode_switch == jmsm_manual && joy_throttle <= RC_BOUND_MIN) || _joy_arm_switch == bf_disarmed)
            return false;
        else
            return ((auto_throttle > RC_BOUND_MIN && _flight_mode != fm_spinup) || _flight_mode == fm_start_takeoff || _flight_mode == fm_take_off_aim || _flight_mode == fm_max_burn || _flight_mode == fm_1g ); //FIXME: check if this goes well if due to extreme control throttle is set to 0
    }

    bool drone_state_inactive() {
        return _flight_mode == fm_inactive;
    }
    bool drone_state_disarmed() {
        return _flight_mode == fm_disarmed;
    }

    bool joystick_ready();

    void LED(bool b) {
        _rc->LED_drone(b,dparams.drone_led_strength);
    }
    void LED(bool b, int value) {
        _rc->LED_drone(b,value);
    }
    void stop_rc() {
        _rc->close();
    }

    void beep(bool b) {
        _rc->beep(b);
    }

    bool blocked();
};
