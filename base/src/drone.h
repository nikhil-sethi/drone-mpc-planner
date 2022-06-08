#pragma once
#include "dronetracker.h"
#include "dronecontroller.h"
#include "navigation.h"
#include "dronenavigation.h"
#include "baseboardlink.h"
#include "interceptor.h"
#include "dronereader.h"

static const char *drone_state_names[] = {
    "pre_flight",
    "charging",
    "ready",
    "flight",
    "post_flight",
    "charge_fail",
    "rc loss",
    "beep"
};
static const char *pre_flight_state_names[] = {
    "init",
    "locate",
    "wait blink led",
    "locating...",
    "calibrating",
    "check_telem",
    "check_pad",
    "wait to arm",
    "arming",
    "wait x led",
    "wait_charge",
    "locate_fail"
};
static const char *post_flight_state_names[] = {
    "init",
    "yaw",
    "shake",
    "shaking",
    "wait_init",
    "shaked...",
    "trim_acc",
    "lost",
    "crash",
    "crashed"
};
class Drone {
public:
    enum drone_states {
        ds_pre_flight = 0,
        ds_charging,
        ds_ready,
        ds_flight,
        ds_post_flight,
        ds_charging_failure,
        ds_rc_loss,
        ds_beep
    };
    enum pre_flight_states {
        pre_init = 0,
        pre_locate_drone_init,
        pre_locate_drone_wait_led,
        pre_locate_drone,
        pre_calibrating_pad,
        pre_check_telemetry,
        pre_check_pad_att,
        pre_wait_to_arm,
        pre_arming,
        pre_wait_init_led,
        pre_wait_charging,
        pre_locate_time_out
    };
    enum post_flight_states {
        post_init = 0,
        post_reset_yaw_on_pad,
        post_start_shaking,
        post_shaking_drone,
        post_wait_after_shake_init,
        post_wait_after_shake,
        post_trim_accelerometer,
        post_lost,
        post_init_crashed,
        post_crashed,
    };
private:
    Interceptor *_interceptor;
    BaseboardLink *_baseboard_link;
    VisionData *_visdat;
    tracking::TrackerManager *_trackers;

    bool initialized = false;
    drone_states _state = ds_pre_flight;
    pre_flight_states pre_flight_state = pre_init;
    post_flight_states post_flight_state = post_init;
    uint _rc_id = 0;
    RC *_rc;

    //pre flight
    int n_locate_drone_attempts = 0;

    bool confirm_drone_on_pad = false;
    double time_start_locating_drone = 0;
    double time_start_locating_drone_attempt = 0;
    double time_located_drone = 0;
    double time_waiting_for_charge = 0;
    double time_led_init = 0;
    double time_start_att_wait_pad = 0;

    uint n_detected_pad_locations = 0;

    // flight
    bool trigger_waypoint_flight = false;
    std::string flightplan_fn = "";
    int _n_take_offs = 0;
    int _n_landings = 0;
    int _n_drone_detects = 0;
    int _n_wp_flights = 0;
    int _n_hunt_flights = 0;

    //post flight:
    const float duration_post_shake_wait = 5;
    const float duration_shake = 3;
    const float duration_reset_yaw_on_pad = 1.5f;
    const float duration_wait_before_shake = 1;
    const float led_response_duration = 1;
    const float wait_charging_response_duration = 10;
    const float att_wait_pad_timeout = 10;
    float confirm_drone_on_pad_delta_distance;
    double time_reset_yaw_on_pad = 0;
    double time_start_shaking = 0;
    double time_post_shake = 0;
    double time_shake_start = 0;
    double time_crashed = 0;
    int n_shakes_sessions_after_landing = 0;

    const float max_safe_charging_telemetry_voltage = 4.35f; // safety overcharge flip happens at 4.4v

    std::ofstream *main_logger;
    std::ofstream flight_logger;
    time_t take_off_datetime;
    time_t land_datetime;

    void pre_flight(double time);
    void post_flight(double time);
    void take_off(bool hunt, double time);
    void save_flight_results();
    void blink(double time);

public:
    DroneController control;
    navigation::DroneNavigation nav;
    tracking::DroneTracker tracker;

    bool in_flight() {return _state == ds_flight;}

    std::string drone_state_str() {
        if (_state == ds_pre_flight)
            return std::string(drone_state_names[_state]) + " " + std::string(pre_flight_state_names[pre_flight_state]);
        else if (_state == ds_post_flight)
            return std::string(drone_state_names[_state]) + " " + std::string(post_flight_state_names[post_flight_state]);
        else if (_state == ds_flight)
            return nav.navigation_status();
        else
            return std::string(drone_state_names[_state]);
    }
    drone_states state() { return _state;}
    RC *rc() { return _rc;}
    uint rc_id() { return _rc_id;}

    void waypoint_flight(std::string flightplan_fn_) {
        flightplan_fn = flightplan_fn_;
        trigger_waypoint_flight = true;
    }

    int n_take_offs() {return _n_take_offs;}
    int n_landings() {return _n_landings;}
    int n_drone_detects() {return _n_drone_detects;}
    int n_wp_flights() {return _n_wp_flights;}
    int n_hunt_flights() {return _n_hunt_flights;}

    void shake_drone() {_state = ds_post_flight; post_flight_state = post_start_shaking;}
    bool drone_flying() {return _state == ds_flight;}
    bool drone_ready_and_waiting() {return _state == ds_ready;}
    bool program_restart_allowed() {return _state != ds_flight && (_state != ds_post_flight || post_flight_state == post_crashed || post_flight_state == post_lost);}
    void beep_drone() {_state = ds_beep;}
    void redetect_drone_location() {
        control.invalidize_blink();
        _state = ds_pre_flight;
    }

    void init(std::ofstream *logger, int rc_id, RC *rc, tracking::TrackerManager *trackers, VisionData *visdat, FlightArea *flight_area, Interceptor *interceptor, BaseboardLink *baseboard);
    void init_flight_replay(std::string replay_dir, int flight_id);
    void init_full_log_replay(std::string replay_dir);
    void update(double time);
    void dummy_log() {(*main_logger) << "NA;";}
    void inject_log(logging::LogEntryDrone entry, unsigned long long rs_id);
    void close();

};
