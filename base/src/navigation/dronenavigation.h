#pragma once
#include "common.h"
#include "dronetracker.h"
#include "dronecontroller.h"
#include "insecttracker.h"
#include "trackermanager.h"
// #include "interceptor.h" // recipe for disaster
#include "navigation.h"
#include "flightplan.h"
#include "baseboardlink.h"
#include <string>


class Interceptor;

namespace navigation {

class DroneNavigation {

private:
    navigation_states _navigation_status = ns_takeoff;
    nav_flight_modes _nav_flight_mode = nfm_none;

    bool initialized = false;
    bool low_battery_triggered = false;
    bool force_thrust_calib = false;

    uint wpid = 0;
    std::vector<Waypoint> waypoints;
    Waypoint *current_waypoint = new Waypoint_Landing();

    float _flight_time = -1;

    double time_start_reset_headless_yaw = 0;
    double time_start_thrust_calibration = 0;
    double time_start_wait_after_landing = -1;
    double time_prev_wp_reached = -1;
    double time_wp_reached = -1;
    double time_start_landing = -1;
    double time_landed = 0;
    double time_drone_problem = -1;
    double time_take_off = 0;
    float time_out_after_landing = 10;
    double time_start_wait_expected_attitude = -1;
    const float duration_correct_yaw = 6;
    const float duration_trigger_bowling = 2.4f;
    const float duration_reset_headless_yaw = 2;
    const float duration_wait_after_landing = 0.8;
    const float duration_wait_expected_attitude = 10.;

    std::ofstream *_logger;
    DroneController *_control;
    tracking::DroneTracker *_tracker;
    Interceptor *_iceptor;
    VisionData *_visdat;
    FlightArea *_flight_area;
    BaseboardLink *_baseboard_link;

    cv::Point3f setpoint_pos_world = {0};
    cv::Point3f setpoint_pos_world_landing = {0};
    cv::Point3f setpoint_vel_world = {0};
    cv::Point3f setpoint_acc_world = {0};

    string settings_file = "../xml/navigation.xml";
    void deserialize_settings();
    void serialize_settings();
    void next_waypoint(Waypoint wp, double time);
    bool drone_at_wp();
    bool drone_close_to_wp();
    void check_abort_autonomous_flight_conditions();
    float calibration_offset(Waypoint wp);
    bool exec_thrust_calib() {return !_control->thrust_calib_valid() || force_thrust_calib;}

public:
    void close(void);
    void init(tracking::DroneTracker *tracker, DroneController *control, VisionData *visdat, FlightArea *flight_area, Interceptor *interceptor, BaseboardLink *baseboard);
    void init_flight(bool hunt, std::ofstream *logger);
    void update(double time);
    void flightplan(std::string flightplan_fn);

    nav_flight_modes nav_flight_mode() {return _nav_flight_mode;}
    void nav_flight_mode(nav_flight_modes m) {_nav_flight_mode = m;}
    std::string navigation_status() {
        if (_navigation_status == ns_approach_waypoint || _navigation_status == ns_landing) {
            return static_cast<string>(navigation_status_names[_navigation_status]) + " " + current_waypoint->name;
        } else
            return navigation_status_names[_navigation_status];
    }

    tracking::TrackData setpoint() {
        tracking::TrackData data_setpoint;
        data_setpoint.state.pos = setpoint_pos_world;
        data_setpoint.state.vel = setpoint_vel_world;
        data_setpoint.state.acc = setpoint_acc_world;
        data_setpoint.pos_valid = true;
        data_setpoint.vel_valid = true;
        data_setpoint.acc_valid = true;
        return data_setpoint;
    }

    void manual_trigger_next_wp() {
        if (wpid < waypoints.size() - 1 && _nav_flight_mode == nfm_waypoint) {
            wpid++;
            _navigation_status = ns_set_waypoint;
        }
    }
    void manual_trigger_prev_wp() {
        if (wpid > 0 && _nav_flight_mode == nfm_waypoint) {
            wpid--;
            _navigation_status = ns_set_waypoint;
        }
    }

    bool drone_hunting() { return _nav_flight_mode == nfm_hunt && (_navigation_status == ns_chasing_insect || _navigation_status ==  ns_start_the_chase); }
    bool drone_resetting_yaw() {return _navigation_status == ns_correct_yaw || _navigation_status == ns_reset_headless_yaw;}
    bool flight_done() {return _navigation_status == ns_flight_done;}
    bool flight_aborted() {return _navigation_status == ns_flight_aborted;}
    int distance_threshold_mm() { return current_waypoint->threshold_mm; }

    float flight_time() {return _flight_time;}
    double takeoff_time() {return time_take_off;}
    bool drone_problem() {return time_drone_problem >= 0;}
};
}
