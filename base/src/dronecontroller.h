#pragma once
#include "dronetracker.h"
#include "joystick.hpp"
#include "multimodule.h"
#include "common.h"
#include "flightarea/flightarea.h"
#include "tracking.h"
#include "landingcontroller.h"
#include "keepinviewcontroller.h"
#include "accelerometertrim.h"
#include "dronereader.h"


#define DRONECONTROLLER_DEBUG false
#define ENABLE_SPINUP true

static const char *joy_states_names[] = { "js_manual",
                                          "js_waypoint",
                                          "js_hunt",
                                          "js_disarmed",
                                          "js_checking",
                                          "js_none"
                                        };
static const char *flight_mode_names[] = { "fm_joystick_check",
                                           "fm_disarmed",
                                           "fm_inactive",
                                           "fm_wait",
                                           "fm_manual",
                                           "fm_spinup",
                                           "fm_start_takeoff",
                                           "fm_take_off_aim",
                                           "fm_max_burn",
                                           "fm_max_burn_spin_down",
                                           "fm_1g",
                                           "fm_pid_init",
                                           "fm_pid",
                                           "fm_long_range_forth",
                                           "fm_long_range_back",
                                           "fm_headed",
                                           "fm_reset_yaw_on_pad",
                                           "fm_reset_headless_yaw",
                                           "fm_correct_yaw",
                                           "fm_calib_thrust",
                                           "fm_prep_to_land",
                                           "fm_ff_landing_start",
                                           "fm_ff_landing",
                                           "fm_start_shake",
                                           "fm_shake_it_baby",
                                           "fm_trim_accelerometer",
                                           "fm_abort"
                                         };




class DroneController {

public:
    enum flight_modes {
        fm_joystick_check,
        fm_disarmed,
        fm_inactive,
        fm_wait,
        fm_manual,
        fm_spinup,
        fm_start_takeoff,
        fm_take_off_aim,
        fm_max_burn,
        fm_max_burn_spin_down,
        fm_1g,
        fm_flying_pid_init,
        fm_flying_pid,
        fm_long_range_forth,
        fm_long_range_back,
        fm_headed,
        fm_reset_yaw_on_pad,
        fm_reset_headless_yaw,
        fm_correct_yaw,
        fm_calib_thrust,
        fm_prep_to_land,
        fm_ff_landing_start,
        fm_ff_landing,
        fm_start_shake,
        fm_shake_it_baby,
        fm_trim_accelerometer,
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
    enum integrator_state {
        reset,
        hold,
        running
    };
private:

    RC *_rc;
    tracking::DroneTracker *_dtrk;
    FlightArea *_flight_area;
    std::ofstream *_logger;

    const std::string calib_fn = "drone_calibration.xml";
    std::string calib_rfn = "../../../../pats/xml/" + calib_fn;

    xmls::DroneCalibration calibration;

    const cv::Point2f att_drone_calibration_range = cv::Point2f(1.f, 1.f); // the max difference between the current att and the att measured during the last blink detect
    const cv::Point2f att_pad_calibration_range = cv::Point2f(7.5f, 7.5f);
    const cv::Point2f att_precisely_on_pad_range = cv::Point2f(3.f, 3.f);
    const cv::Point2f att_somewhere_on_pad_range = cv::Point2f(30.f, 30.f);
    cv::Point2f att_reset_yaw_on_pad;
    int att_somewhere_on_pad_cnt = 0, att_precisely_on_pad_cnt = 0, att_pad_calibration_ok_cnt = 0;
    int _att_telemetry_samples_cnt = 0;

    const float transmission_delay_duration = 0.04f;
    const float max_bank_angle = 180;
    const float aim_duration = 0.0833333333333f; //Slightly related to full_bat_and_throttle_spinup_time. Should be 1/(bf_strenght/10) seconds
    const float effective_burn_spin_up_duration = 0.15f; // the time to spin up from hover to max
    const float effective_burn_spin_down_duration = 0.1f; // the time to spin down from max to hover
    cv::Point3f vel_after_takeoff = {0};
    float burn_thrust = -1;

    const float lift_off_dist_take_off_aim = 0.02f;
    const float min_takeoff_angle = 60.f / 180.f * static_cast<float>(M_PI); //A takeoff angle of ~45 is possible if the ir-led is just above the pad (such that the drone enters the light beam even with a "horizontal" takeoff)

    const float min_yaw_deviation = 0.5f;

    double take_off_start_time = 0;
    double state_start_time = 0;
    float remaining_spinup_duration_t0 = 0;
    double interception_start_time = 0;
    double in_flight_start_time = -1;
    double ff_land_start_time = 0;
    int ff_auto_throttle_start = RC_BOUND_MIN;
    float auto_burn_duration = 0;
    uint n_invalid_or_bad_telemetry_package = 0; // Counts when telemetry holds imposiible (invalid) data or (bad) data which will result in some undesired system state
    bool landing_att_calibration_msg_printed = false;

    cv::Point3f _burn_direction_for_thrust_approx = {0};

    float _dist_to_setpoint = 999;
    double _time;
    double time_waypoint_moved = 0;

    uint16_t kill_cnt_down = 0;
    double spin_up_start_time = 0;
    double start_takeoff_burn_time = 0;
    bool _shake_finished = false;

    bool initialized = false;
    bool log_replay_mode = false;
    bool thrust_calibration = false;
    flight_modes _flight_mode = fm_joystick_check; // only set externally (except for disarming), used internally
    joy_mode_switch_modes _joy_mode_switch = jmsm_none;
    betaflight_arming _joy_arm_switch = bf_armed;
    bool _joy_takeoff_switch = false;
    joy_states _joy_state = js_none;

    bool _hover_mode = false;
    bool _landed = false;
    bool _manual_override_take_off_now = false;

    cv::Point3f pos_err_i;

    filtering::Tf_D_f d_vel_err_x, d_vel_err_y, d_vel_err_z;
    filtering::Tf_PT2_f pos_modelx, pos_modely, pos_modelz;
    filtering::hold_filter control_mode_hold_filter;

    float time_spent_spinning_up(double time) {
        if (spin_up_start_time > 0)
            return static_cast<float>(time - spin_up_start_time);
        else
            return 0;
    }

    uint16_t initial_hover_throttle_guess_non3d;
    uint16_t spinup_throttle() {
        if (dparams.mode3d)
            return RC_MIDDLE + 1;
        else {
            return dparams.spinup_throttle_non3d;
        }
    }

    float thrust_to_throttle(float thrust);
    float duration_since_waypoint_moved(double time) { return  static_cast<float>(time - time_waypoint_moved); }
    DroneController::integrator_state horizontal_integrators(cv::Point3f setpoint_vel, double time);

    std::tuple<float, float> acc_to_deg(cv::Point3f acc);
    std::tuple<float, float> acc_to_quaternion(cv::Point3f acc);

    void correct_yaw(float deviation_angle);

    std::tuple<cv::Point3f, cv::Point3f, cv::Point3f> adjust_control_gains(tracking::TrackData drone_data, bool enable_horizontal_integrators);
    std::tuple<cv::Point3f, cv::Point3f> control_error(tracking::TrackData data_drone, cv::Point3f setpoint_pos, integrator_state enable_horizontal_integrators, bool dry_run);
    std::tuple<int, int, int> calc_feedforward_control(cv::Point3f desired_acceleration);
    cv::Point3f takeoff_acceleration(tracking::TrackData data_target, float target_acceleration_y);
    cv::Point3f compensate_gravity_and_crop_to_limit(cv::Point3f des_acc, float thrust);
    void control_model_based(tracking::TrackData data_drone, cv::Point3f setpoint_pos, cv::Point3f setpoint_vel);

    void send_data_joystick(void);
    void read_joystick(void);
    void process_joystick();
    void load_calibration();
    void save_calibration();
    void save_calibration_before_flight(int flight_id);
    void fine_tune_thrust(float integration_error);

public:
    LandingController land_ctrl;
    KeepInViewController kiv_ctrl;
    AccelerometerTrim accelerometer_trim;

    void led_strength(float light_level);

    cv::Point3f pid_error(tracking::TrackData data_drone, cv::Point3f setpoint_pos, cv::Point3f setpoint_vel, bool choosing_insect);
    void flight_mode(flight_modes f) { _flight_mode = f; }
    void hover_mode(bool value) { _hover_mode = value;}
    void double_led_strength() { dparams.drone_led_strength = std::clamp(dparams.drone_led_strength * 2, 5, 100); }
    void freeze_attitude_reset_yaw_on_pad() { att_reset_yaw_on_pad = cv::Point2f(_rc->telemetry.roll, _rc->telemetry.pitch); };

    int shake_finished() {return _shake_finished;}

    bool abort_take_off();

    void update_hover_integrators() {
        accelerometer_trim.intergrator_hovering(pos_err_i.x, pos_err_i.z);
        fine_tune_thrust(pos_err_i.y);
    }

    joy_states Joy_State() {
        return _joy_state;
    }
    std::string Joy_State_str() {
        return joy_states_names[_joy_state];
    }
    std::string flight_mode_str() {
        return flight_mode_names[_flight_mode];
    }
    flight_modes flight_mode() {
        return _flight_mode;
    }

    bool ff_interception() {
        return _flight_mode == fm_take_off_aim || _flight_mode == fm_max_burn || _flight_mode == fm_max_burn_spin_down || _flight_mode == fm_1g;
    }

    bool at_base() {
        return _flight_mode <= fm_take_off_aim;
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
    void inject_log(logging::LogEntryDrone entry) {
        joy_roll = entry.joy_roll;
        joy_pitch = entry.joy_pitch;
        joy_yaw = entry.joy_yaw;
        joy_throttle = entry.joy_throttle;
        _joy_arm_switch = static_cast<betaflight_arming>(entry.joy_arm_switch);
        _joy_mode_switch = static_cast<joy_mode_switch_modes>(entry.joy_mode_switch);
        _joy_takeoff_switch = entry.joy_takeoff_switch;
        _log_auto_roll = entry.auto_roll;
        _log_auto_pitch = entry.auto_pitch;
        _log_auto_throttle = entry.auto_throttle;
    }

    bool spinup() {  return _flight_mode == fm_spinup; }
    bool landing() { return _flight_mode == fm_ff_landing || _flight_mode == fm_ff_landing_start; }
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
    float duration_spent_taking_off(double time) {
        if (start_takeoff_burn_time < 0.01)
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

    /** @brief Determines the corresponding roll/pitch angle for a given command */
    float angle_of_command(int command) {
        command -= RC_MIDDLE;
        float commandf = static_cast<float>(command) / static_cast<float>(RC_BOUND_MAX - RC_BOUND_MIN);
        return commandf * max_bank_angle;
    }

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

    void close(void);
    void init(RC *rc, tracking::DroneTracker *dtrk, FlightArea *flight_area);
    void init_flight(std::ofstream *logger, int flight_id);
    void init_flight_replay(std::string replay_dir, int flight_id);
    void init_full_log_replay(std::string replay_dir);
    void control(tracking::TrackData data_drone, tracking::TrackData data_target, control_modes control_mode, cv::Point3f target_acceleration, double time, bool enable_logging);
    bool landed() {return _landed; }
    bool state_inactive() { return _flight_mode == fm_inactive; }
    bool state_disarmed() { return _flight_mode == fm_disarmed; }
    bool ready_for_arm(double time) {return time - _rc->time_disarmed() > 1.5 && _rc->time_disarmed() >= 0;}
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
    bool rc_ok(double time) { return _rc->ok(time); }

    void LED(bool b) {
        if (initialized)
            _rc->LED_drone(b, dparams.drone_led_strength);
    }
    void LED(bool b, int value) {
        if (initialized)
            _rc->LED_drone(b, value);
    }
    void stop_rc() {
        if (initialized)
            _rc->close();
    }
    Telemetry telemetry() {
        if (initialized)
            return _rc->telemetry;
        else {
            Telemetry t = {0};
            return t;
        }
    }

    bool att_somewhere_on_pad() { return att_somewhere_on_pad_cnt > 3;}
    bool att_precisely_on_pad() { return att_precisely_on_pad_cnt > 3;}
    bool att_ok_for_pad_calibration() {return att_pad_calibration_ok_cnt > 3;}
    bool att_telemetry_valid() {return _att_telemetry_samples_cnt > 6;}

    void init_thrust_calibration();
    void save_thrust_calibration();
    void update_attitude_pad_state();
    void reset_attitude_pad_state();
    void invalidize_blink();
    void save_pad_pos_and_att_calibration();
    bool pad_calib_valid();
    bool thrust_calib_valid();
    float *max_thrust() {
        return &(calibration.max_thrust);
    }

};
