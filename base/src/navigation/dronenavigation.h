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
public:

private:
    int v_crcl1 = 250;
    int v_crcl2 = 500;
    int r_crcl1 = 5;
    int r_crcl2 = 15;
    int w_sqr = 600;
    int v_sqr = 100;

    float time_out_after_landing = 0;
    int setpoint_slider_X = 250;
    int setpoint_slider_Y = 250;
    int setpoint_slider_Z = 250;

    void deserialize_flightplan(string replay_dir);
    string settings_file = "../xml/navigation.xml";
    void deserialize_settings();
    void serialize_settings();

    float motion_calibration_duration = 2;
    double time_motion_calibration_started = 0;
    double time_initial_reset_yaw = 0;
    double time_wp_reached = -1;
    double landing_start_time = -1;
    double landed_time = 0;
    nav_flight_modes _nav_flight_mode = nfm_none;

    void next_waypoint(Waypoint wp, double time);

    bool force_pad_redetect = false;

    navigation_states _navigation_status = ns_init;
    double locate_drone_start_time = 0;
    int locate_drone_attempts = 0;
    double last_led_doubler_time = 0;

    uint wpid = 0;
    std::vector<Waypoint> waypoints;
    Waypoint * current_waypoint = new Waypoint_Landing();

    const double yaw_reset_duration = 6;

    double time_drone_problem = -1;
    float duration_drone_problem = 0;
    double time_located_drone = 0;
    double time_take_off = 0;
    double time_shake_start = 0;
    float shake_duration = 15;

    std::ofstream *_logger;
    DroneController * _dctrl;
    tracking::TrackerManager * _trackers;
    Interceptor * _iceptor;
    VisionData *_visdat;
    CameraView *_camview;

    cv::Point3f setpoint_pos_world;
    cv::Point3f setpoint_pos_world_landing;
    cv::Point3f setpoint_vel_world;
    cv::Point3f setpoint_acc_world;

    bool initialized = false;
    bool low_battery_triggered = false;
    cv::Point3f square_point(cv::Point3f center, float width, float s);

    int _n_take_offs = 0;
    int _n_landings = 0;
    int _n_drone_detects = 0;
    int _n_drone_readys = 0;
    int _n_wp_flights = 0;
    int _n_hunt_flights = 0;
    float _flight_time = -1;

    void check_abort_autonomus_flight_conditions();

public:

    void close (void);
    void init(std::ofstream *logger, tracking::TrackerManager * imngr, DroneController *dctrl, VisionData *visdat, CameraView *camview, string replay_dir, Interceptor *iceptor);
    void update(double time);
    void redetect_drone_location() {_navigation_status = ns_locate_drone_init; force_pad_redetect=true; locate_drone_attempts=0;}
    void replay_detect_drone_location() { force_pad_redetect=true;}
    void shake_drone() {_navigation_status = ns_start_shaking;}
    void set_drone_problem() {_navigation_status = ns_drone_problem;}
    void demo_flight(std::string flightplan_fn);

    void render_now_override() {
        _nav_flight_mode = nfm_none;
        _navigation_status = ns_monitoring;
        _trackers->mode(tracking::TrackerManager::mode_wait_for_insect);
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

    cv::Point2i drone_v_setpoint_im() {

        cv::Point3f tmp = 0.1*setpoint_vel_world + setpoint_pos_world;
        if (!drone_flying())
            tmp = {0};

        std::vector<cv::Point3d> world_length,camera_length;
        cv::Point3d tmpd;
        float theta = -_visdat->camera_angle * deg2rad;
        float temp_y = tmp.y * cosf(theta) + tmp.z * sinf(theta);
        tmpd.z = static_cast<double>(-tmp.y * sinf(theta) + tmp.z * cosf(theta));
        tmpd.y = static_cast<double>(temp_y);
        tmpd.x = static_cast<double>(tmp.x);

        world_length.push_back(tmpd);
        cv::perspectiveTransform(world_length,camera_length,_visdat->Qfi);

        if (camera_length[0].x > IMG_W)
            camera_length[0].x = IMG_W;
        if (camera_length[0].y > IMG_H)
            camera_length[0].y = IMG_H;

        if (camera_length[0].x < 0)
            camera_length[0].x = 0;
        if (camera_length[0].y < 0)
            camera_length[0].y = 0;

        return cv::Point2i(camera_length[0].x,camera_length[0].y);
    }
    cv::Point2i drone_setpoint_im() {
        //transform to image coordinates:

        cv::Point3f tmp = setpoint_pos_world;
        if (!drone_flying()) {
            tmp.x = _trackers->dronetracker()->pad_location().x;
            tmp.y = _trackers->dronetracker()->pad_location().y+0.5f;
            tmp.z = _trackers->dronetracker()->pad_location().z;
        }

        cv::Point3f resf  =world2im_3d(tmp,_visdat->Qfi,_visdat->camera_angle);
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
    bool drone_resetting_yaw() {return _navigation_status == ns_wait_reset_yaw || _navigation_status == ns_initial_reset_yaw;}
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
