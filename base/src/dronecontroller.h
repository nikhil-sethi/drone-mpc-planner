#pragma once
#include "dronetracker.h"
#include "joystick.hpp"
#include "multimodule.h"
#include "common.h"
#include "flightarea/flightarea.h"
#include "tracking.h"
#include "landingcontroller.h"
#include "keepinviewcontroller.h"

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
                                           "fm_pid_init",
                                           "fm_pid",
                                           "fm_long_range_forth",
                                           "fm_long_range_back",
                                           "fm_headed",
                                           "fm_reset_headless_yaw",
                                           "fm_correct_yaw",
                                           "fm_calib_thrust",
                                           "fm_prep_to_land",
                                           "fm_ff_landing_start",
                                           "fm_ff_landing",
                                           "fm_shake_it_baby",
                                           "fm_monitoring",
                                           "fm_abort"
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
        fm_flying_pid_init,
        fm_flying_pid,
        fm_long_range_forth,
        fm_long_range_back,
        fm_headed,
        fm_reset_headless_yaw,
        fm_correct_yaw,
        fm_calib_thrust,
        fm_prep_to_land,
        fm_ff_landing_start,
        fm_ff_landing,
        fm_shake_it_baby,
        fm_monitoring,
        fm_abort
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

    Rc * _rc;
    tracking::DroneTracker * _dtrk;
    FlightArea *_flight_area;
    std::ofstream *_logger;

    std::string control_parameters_rfn;
    const std::string calib_fn = "drone_calibration.xml";

    xmls::DroneCalibration calibration;

    const int required_pad_att_calibration_cnt = 15;
    const cv::Point2f allowed_pad_att_calibration_range = cv::Point2f(7.5f,7.5f);
    const cv::Point2f allowed_att_calibration_range = cv::Point2f(1.0f,1.0f); // the max difference between the current att and the att measured during the last blink detect
    bool pat_att_calibration_valid = false;
    filtering::Smoother pad_att_calibration_roll;
    filtering::Smoother pad_att_calibration_pitch;

    const float transmission_delay_duration = 0.04f;
    const float max_bank_angle = 180;
    const float aim_duration = 0.0833333333333f; //Slightly related to full_bat_and_throttle_spinup_time. Should be 1/(bf_strenght/10) seconds
    float effective_burn_spin_up_duration = 0.15f; // the time to spin up from hover to max
    const float effective_burn_spin_down_duration = 0.1f; // the time to spin down from max to hover
    cv::Point3f vel_after_takeoff = {0};
    float burn_thrust = -1;

    const float lift_off_dist_take_off_aim = 0.02f;
    float min_takeoff_angle = 45.f/180.f*static_cast<float>(M_PI);

    double take_off_start_time = 0;
    float remaining_spinup_duration_t0 = 0;
    double interception_start_time = 0;
    double in_flight_start_time = -1;
    double ff_land_start_time = 0;
    int ff_auto_throttle_start;

    cv::Point3f _burn_direction_for_thrust_approx = {0};

    float _dist_to_setpoint = 999;
    double _time;
    double time_waypoint_moved = 0;

    uint16_t kill_cnt_down = 0;
    double spin_up_start_time = 0;
    double start_takeoff_burn_time = 0;

    std::vector<cv::Point3f> aim_direction_history;

    bool initialized = false;
    bool log_replay_mode = false;
    bool airsim_mode = false;
    bool generator_mode = false;
    bool thrust_calibration = false;
    flight_modes _flight_mode = fm_joystick_check; // only set externally (except for disarming), used internally
    joy_mode_switch_modes _joy_mode_switch = jmsm_none;
    betaflight_arming _joy_arm_switch = bf_armed;
    bool _joy_takeoff_switch = false;
    joy_states _joy_state = js_none;
    int joyDial = 0;
    float scaledjoydial = 0;

    bool _hover_mode = false;

    cv::Point3f pos_err_i;
    int kp_pos_roll, kp_pos_throttle, kp_pos_pitch, ki_pos_roll, ki_thrust, ki_pos_pitch, kd_pos_roll, kd_pos_throttle, kd_pos_pitch;
    int kp_pos_roll_hover, kp_pos_throttle_hover, kp_pos_pitch_hover, ki_pos_roll_hover, ki_thrust_hover, ki_pos_pitch_hover, kd_pos_roll_hover, kd_pos_throttle_hover, kd_pos_pitch_hover;
    int kp_v_roll, kp_v_throttle, kp_v_pitch, kd_v_roll, kd_v_throttle, kd_v_pitch;
    filtering::Tf_D_f d_vel_err_x, d_vel_err_y, d_vel_err_z;
    filtering::Tf_PT2_f pos_modelx, pos_modely, pos_modelz;

    float time_spent_spinning_up(double time) {
        if (spin_up_start_time> 0)
            return static_cast<float>(time - spin_up_start_time );
        else
            return 0;
    }
    void set_led_strength(float exposure);

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

    void save_calibration();
    void approx_effective_thrust(tracking::TrackData data_drone, cv::Point3f burn_direction, float burn_duration, float dt_burn);
    float thrust_to_throttle(float thrust_ratio);
    std::tuple<cv::Point3f, cv::Point3f, cv::Point3f> predict_drone_after_burn(tracking::StateData state_drone, cv::Point3f burn_direction, float remaining_aim_duration, float burn_duration);
    std::tuple<int, int, float, cv::Point3f, std::vector<tracking::StateData> > calc_burn(tracking::StateData state_drone, tracking::StateData state_target, float remaining_aim_duration);
    std::tuple<int, int, float, cv::Point3f> calc_directional_burn(tracking::StateData state_drone, tracking::StateData state_target, float remaining_aim_duration);
    std::vector<tracking::StateData> predict_trajectory(float burn_duration, float remaining_aim_duration, cv::Point3f burn_direction, tracking::StateData state_drone);
    void draw_viz(tracking::StateData state_drone, tracking::StateData state_target, double time, cv::Point3f burn_direction, float burn_duration, float remaining_aim_duration, std::vector<tracking::StateData> traj);
    void calibrate_pad_attitude();
    cv::Point3f keep_in_volume_correction_acceleration(tracking::TrackData data_drone);
    float duration_since_waypoint_moved(double time) { return  static_cast<float>(time - time_waypoint_moved); }
    bool horizontal_integrators(cv::Point3f setpoint_vel,double time);

    std::tuple<float,float> acc_to_deg(cv::Point3f acc);
    std::tuple<float,float> acc_to_quaternion(cv::Point3f acc);

    void correct_yaw(float deviation_angle);

    void blink(double time);
    void blink_motors(double time);

    std::tuple<cv::Point3f, cv::Point3f, cv::Point3f, cv::Point3f, cv::Point3f> adjust_control_gains(tracking::TrackData drone_data, cv::Point3f setpoint_pos, cv::Point3f setpoint_vel,bool enable_horizontal_integrators);
    std::tuple<cv::Point3f, cv::Point3f> control_error(tracking::TrackData data_drone, cv::Point3f setpoint_pos, bool enable_horizontal_integrators, bool dry_run);
    std::tuple<int,int,int> calc_feedforward_control(cv::Point3f desired_acceleration);
    cv::Point3f compensate_gravity_and_crop_to_limit(cv::Point3f des_acc, float thrust);
    void control_model_based(tracking::TrackData data_drone, cv::Point3f setpoint_pos, cv::Point3f setpoint_vel);

    void send_data_joystick(void);
    void read_joystick(void);
    void process_joystick();
    void load_calibration(std::string replay_dir);
    void load_control_parameters();
    void serialize_settings();

public:
    LandingController land_ctrl;
    KeepInViewController kiv_ctrl;

    cv::Point3f pid_error(tracking::TrackData data_drone, cv::Point3f setpoint_pos, cv::Point3f setpoint_vel, bool choosing_insect);
    void flight_mode(flight_modes f) { _flight_mode = f; }
    void hover_mode(bool value) { _hover_mode = value;}
    void double_led_strength() { dparams.drone_led_strength = std::clamp(dparams.drone_led_strength*2,5,100); }

    bool abort_take_off() {
        //check if the take off is not yet too far progressed to abort, if not go to spin up else return true

        if (_flight_mode == fm_take_off_aim) {
            float remaining_spinup_duration = dparams.full_bat_and_throttle_spinup_duration - aim_duration - time_spent_spinning_up(_time);
            if (remaining_spinup_duration  < 0.05f)
                return false;
            _flight_mode = fm_spinup ;
            if (spin_up_start_time < take_off_start_time)
                spin_up_start_time = take_off_start_time;
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
        return flight_mode_names[_flight_mode];
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
    void insert_log(int log_joy_roll, int log_joy_pitch, int log_joy_yaw, int log_joy_throttle, int log_joy_arm_switch, int log_joy_mode_switch, int log_joy_take_off_switch,int log_auto_roll, int log_auto_pitch, int log_auto_throttle, float log_acc_z) {
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
    }

    float in_flight_duration(double time) {
        if ((_flight_mode == fm_flying_pid || _flight_mode == fm_correct_yaw || _flight_mode == fm_ff_landing || _flight_mode == fm_reset_headless_yaw)
                && in_flight_start_time < 0)
            in_flight_start_time = time;
        else if (_flight_mode == fm_disarmed || _flight_mode == fm_inactive || _flight_mode == fm_spinup || _flight_mode == fm_manual || _flight_mode == fm_abort)
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

    void nav_waypoint_moved(double time) { time_waypoint_moved = time; }

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
        float acc_z = _rc->telemetry.acc.z;
        if (log_replay_mode)
            acc_z = _log_acc_z;
        return acc_z;
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
    std::vector<tracking::StateData> viz_trajectory;

    double viz_time_after_burn = {0};

    uint control_history_max_size;
    std::vector<ControlData> control_history;

    float dist_to_setpoint() { return _dist_to_setpoint; }

    void close (void);
    void init(std::ofstream *logger, std::string replay_dir, bool generator_mode, bool airsim, Rc *rc, tracking::DroneTracker *dtrk, FlightArea* flight_area,float exposure);
    void control(tracking::TrackData, tracking::TrackData, tracking::TrackData, double);
    bool active() {
        if (_flight_mode == fm_inactive || _flight_mode == fm_disarmed || _flight_mode == fm_joystick_check)
            return false;
        else if (_flight_mode == fm_abort || _flight_mode == fm_shake_it_baby || _flight_mode == fm_blink)
            return false;
        else if (_joy_mode_switch == jmsm_manual && joy_throttle > RC_BOUND_MIN && _joy_arm_switch == bf_armed)
            return true;
        else if ((_joy_mode_switch == jmsm_manual && joy_throttle <= RC_BOUND_MIN) || _joy_arm_switch == bf_disarmed)
            return false;
        else
            return ((auto_throttle > RC_BOUND_MIN && _flight_mode != fm_spinup) || _flight_mode == fm_start_takeoff || _flight_mode == fm_take_off_aim || _flight_mode == fm_max_burn || _flight_mode == fm_1g ); //FIXME: check if this goes well if due to extreme control throttle is set to 0
    }
    bool state_inactive() { return _flight_mode == fm_inactive; }
    bool state_disarmed() { return _flight_mode == fm_disarmed; }
    bool ready_for_first_arm(double time) {return time - _rc->time_disarmed() > 1.5 && _rc->time_disarmed() >= 0;}
    bool telemetry_OK() {
        if (initialized)
            return _rc->bf_telem_OK();
        return false;
    }
    bool arming_problem() {
        if (initialized)
            return _rc->telemetry.arming_state != 0;
        return true;
    }

    bool joystick_ready();

    void LED(bool b) {
        if (initialized)
            _rc->LED_drone(b,dparams.drone_led_strength);
    }
    void LED(bool b, int value) {
        if (initialized)
            _rc->LED_drone(b,value);
    }
    void stop_rc() {
        if (initialized)
            _rc->close();
    }
    void beep(bool b) {
        if (initialized)
            _rc->beep(b);
    }
    Telemetry telemetry() {
        if (initialized)
            return _rc->telemetry;
        else {
            Telemetry t = {0};
            return t;
        }
    }

    void init_thrust_calibration();
    void save_thrust_calibration();
    bool attitude_on_pad_OK();
    void invalidize_blink();
    bool pad_calibration_done();
    bool takeoff_calib_valid();
    bool thrust_calib_valid();

};
