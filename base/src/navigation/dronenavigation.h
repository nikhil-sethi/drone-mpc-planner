#pragma once
#include "common.h"
#include "dronetracker.h"
#include "dronecontroller.h"
#include "insecttracker.h"
#include "trackermanager.h"
#include "interceptor.h"
#include "navigation.h"
#include "flightplan.h"
#include <string>

namespace navigation {

class DroneNavigation {

private:
    navigation_states _navigation_status = ns_init;
    nav_flight_modes _nav_flight_mode = nfm_none;

    bool initialized = false;
    bool low_battery_triggered = false;
    bool force_pad_redetect = false;
    bool force_thrust_calib = false;

    uint wpid = 0;
    std::vector<Waypoint> waypoints;
    Waypoint * current_waypoint = new Waypoint_Landing();

    double time_first_frame = -1;
    double time_start_motion_calibration = 0;
    double time_start_reset_headless_yaw = 0;
    double time_start_thrust_calibration = 0;
    double time_prev_wp_reached = -1;
    double time_wp_reached = -1;
    double time_start_landing = -1;
    double time_landed = 0;
    double time_start_locating_drone = 0;
    int locate_drone_attempts = 0;
    double time_last_led_doubler = 0;
    double time_drone_problem = -1;
    float duration_drone_problem = 0;
    double time_located_drone = 0;
    double time_take_off = 0;
    double time_shake_start = 0;
    float time_out_after_landing = 0;
    const float duration_shake = 5;
    const float duration_motion_calibration = 2;
    const double duration_correct_yaw = 6;
    const double duration_reset_headless_yaw = 2;

    std::ofstream *_logger;
    DroneController * _dctrl;
    tracking::TrackerManager * _trackers;
    Interceptor * _iceptor;
    VisionData *_visdat;
    FlightArea *_flight_area;

    int _n_take_offs = 0;
    int _n_landings = 0;
    int _n_shakes_sessions_after_landing = 0;
    int _n_drone_detects = 0;
    int _n_drone_readys = 0;
    int _n_wp_flights = 0;
    int _n_hunt_flights = 0;
    float _flight_time = -1;

    cv::Point3f setpoint_pos_world = {0};
    cv::Point3f setpoint_pos_world_landing = {0};
    cv::Point3f setpoint_vel_world = {0};
    cv::Point3f setpoint_acc_world = {0};
    int setpoint_slider_X = 250;
    int setpoint_slider_Y = 250;
    int setpoint_slider_Z = 250;

    int v_crcl1 = 250;
    int v_crcl2 = 500;
    int r_crcl1 = 5;
    int r_crcl2 = 15;
    int w_sqr = 600;
    int v_sqr = 100;

    void deserialize_flightplan(string replay_dir);
    string settings_file = "../xml/navigation.xml";
    void deserialize_settings();
    void serialize_settings();
    void next_waypoint(Waypoint wp, double time);
    bool drone_at_wp();
    bool drone_close_to_wp();
    cv::Point3f square_point(cv::Point3f center, float width, float s);
    void check_abort_autonomus_flight_conditions();
    void maintain_motion_map(double time);

public:

    void close (void);
    void init(std::ofstream *logger, tracking::TrackerManager * imngr, DroneController *dctrl, VisionData *visdat, FlightArea *flight_area, string replay_dir, Interceptor *iceptor);
    void update(double time);
    void redetect_drone_location() {_navigation_status = ns_locate_drone_init; force_pad_redetect=true; locate_drone_attempts=0;}
    void replay_detect_drone_location() { force_pad_redetect=true;}
    void shake_drone() {_navigation_status = ns_start_shaking;}
    void set_drone_problem() {_navigation_status = ns_drone_problem;}
    void demo_flight(std::string flightplan_fn);

    void render_now_override() {
        _nav_flight_mode = nfm_none;
        _navigation_status = ns_init_render;
        _dctrl->flight_mode(DroneController::fm_monitoring);
        _trackers->mode(tracking::TrackerManager::mode_idle);
        _visdat->reset_motion_integration();
        _visdat->enable_noise_map_calibration();
    }

    nav_flight_modes nav_flight_mode() {return _nav_flight_mode;}
    void nav_flight_mode(nav_flight_modes m) {
        if (m == nfm_manual)
            _navigation_status = ns_manual;
        _nav_flight_mode = m;
    }
    std::string navigation_status() {
        if (_navigation_status == ns_approach_waypoint || _navigation_status == ns_landing) {
            return static_cast<string>(navigation_status_names[_navigation_status]) + " " + current_waypoint->name + " " + to_string_with_precision(_dctrl->dist_to_setpoint(),2);
        } else if (_navigation_status == ns_wait_locate_drone || _navigation_status == ns_locate_drone_wait_led_on) {
            return static_cast<string>(navigation_status_names[_navigation_status]) + " " + std::to_string(locate_drone_attempts);
        } else if (_navigation_status == ns_shaking_drone || _navigation_status == ns_wait_after_shake) {
            return static_cast<string>(navigation_status_names[_navigation_status]) + " " + std::to_string(_n_shakes_sessions_after_landing);
        } else
            return navigation_status_names[_navigation_status];
    }

    tracking::TrackData setpoint() {
        tracking::TrackData data_target;
        data_target.state.pos = setpoint_pos_world;
        data_target.state.vel = setpoint_vel_world;
        data_target.state.acc = setpoint_acc_world;
        data_target.pos_valid = true;
        data_target.vel_valid = true;
        data_target.acc_valid = true;
        return data_target;
    }

    void manual_trigger_next_wp() {
        if (wpid < waypoints.size()-1 && _nav_flight_mode == nfm_waypoint ) {
            wpid++;
            _navigation_status = ns_set_waypoint;
        }
    }
    void manual_trigger_prev_wp() {
        if (wpid > 0 && _nav_flight_mode == nfm_waypoint ) {
            wpid--;
            _navigation_status = ns_set_waypoint;
        }
    }

    cv::Point2i drone_setpoint_im() {
        //transform to image coordinates:

        cv::Point3f tmp = setpoint_pos_world;
        if (!drone_flying()) {
            tmp.x = _trackers->dronetracker()->pad_location().x;
            tmp.y = _trackers->dronetracker()->pad_location().y+0.5f;
            tmp.z = _trackers->dronetracker()->pad_location().z;
        }

        cv::Point3f resf = world2im_3d(tmp,_visdat->Qfi,_visdat->camera_roll,_visdat->camera_pitch);
        return cv::Point2i(roundf(resf.x),round(resf.y));
    }

    bool drone_ready_and_waiting() {return _navigation_status == ns_wait_for_insect || _navigation_status == ns_wait_for_takeoff_command;}
    bool drone_hunting() {
        if (_nav_flight_mode == nfm_hunt) {
            return _navigation_status == ns_chasing_insect || _navigation_status ==  ns_start_the_chase;
        } else {
            return false;
        }
    }
    bool drone_resetting_yaw() {return _navigation_status == ns_correct_yaw || _navigation_status == ns_reset_headless_yaw;}
    bool drone_flying() {return _navigation_status < ns_landing && _navigation_status >  ns_take_off_completed;}
    bool drone_manual() {return _navigation_status == ns_manual;}

    int distance_threshold_mm() { return current_waypoint->threshold_mm; }

    int n_take_offs() {return _n_take_offs;}
    int n_landings() {return _n_landings;}
    int n_drone_detects() {return _n_drone_detects;}
    int n_drone_readys() {return _n_drone_readys;}
    int n_wp_flights() {return _n_wp_flights;}
    int n_hunt_flights() {return _n_hunt_flights;}
    float flight_time() {return _flight_time;}
    bool drone_problem() {return time_drone_problem>=0;}
    bool drone_problem(float min_problem_duration) {return duration_drone_problem>=min_problem_duration;}
};
}
