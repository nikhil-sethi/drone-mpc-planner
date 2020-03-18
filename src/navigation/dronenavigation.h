#pragma once
#include "defines.h"

#include "common.h"
#include "dronetracker.h"
#include "dronecontroller.h"
#include "insecttracker.h"
#include "trackermanager.h"
#include "interceptor.h"
#include "navigation.h"
#include "flightplan.h"

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

    int distance_threshold_f;
    int setpoint_slider_X = 250;
    int setpoint_slider_Y = 250;
    int setpoint_slider_Z = 250;

    void deserialize_flightplan(string replay_dir);
    string settings_file = "../../xml/navigation.xml";
    void deserialize_settings();
    void serialize_settings();

    double time_initial_reset_yaw = 0;
    double landed_time = 0;
    nav_flight_modes _nav_flight_mode;

    void next_waypoint(Waypoint wp, double time);

    navigation_states _navigation_status = ns_init;
    double locate_drone_start_time = 0;

    uint wpid = 0;
    std::vector<Waypoint> waypoints;
    Waypoint * current_waypoint = new Waypoint_Landing();


    double time_drone_problem = -1;
    double time_located_drone = 0;
    double time_take_off = 0;

    std::ofstream *_logger;
    DroneController * _dctrl;
    tracking::TrackerManager * _trackers;
    Interceptor _iceptor;
    VisionData *_visdat;
    CameraView *_camview;

    cv::Point3f setpoint_pos_world;
    cv::Point3f setpoint_vel_world;
    cv::Point3f setpoint_acc_world;

    bool initialized = false;
    cv::Point3f square_point(cv::Point3f center, float width, float s);

    bool first_takeoff = true;

    bool drone_is_blocked(float speed_threshold);

public:
    float time_out_after_landing;

    nav_flight_modes nav_flight_mode() {
        return _nav_flight_mode;
    }
    void nav_flight_mode(nav_flight_modes m) {
        if (m == nfm_manual)
            _navigation_status = ns_manual;
        _nav_flight_mode = m;
    }
    std::string navigation_status() {
        if (_navigation_status == ns_approach_waypoint) {
            return static_cast<string>(navigation_status_names[_navigation_status]) + " " + current_waypoint->name + " " + to_string_with_precision(_dctrl->dist_to_setpoint(),2);
        } else

            return navigation_status_names[_navigation_status];
    }

    track_data setpoint() {
        track_data data_target;
        data_target.state.pos = setpoint_pos_world;
        data_target.state.vel = setpoint_vel_world;
        data_target.state.acc = setpoint_acc_world;
        data_target.pos_valid = true;
        data_target.vel_valid = true;
        data_target.acc_valid = true;
        return data_target;
    }

    int distance_threshold_mm() {
        return current_waypoint->threshold_mm;
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

    void close (void);
    void init(std::ofstream *logger, tracking::TrackerManager * imngr, DroneController *dctrl, VisionData *visdat, CameraView *camview, string replay_dir);
    void update(double time);
    bool disable_insect_detection() {
        return _navigation_status < ns_wait_for_takeoff_command;
    }

    void redetect_drone_location() {
        _navigation_status = ns_locate_drone_init;
    }

    cv::Point2i drone_v_setpoint_im() {

        cv::Point3f tmp = 0.1*setpoint_vel_world + setpoint_pos_world;
        if (!drone_is_flying())
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
        if (!drone_is_flying()) {
            tmp.x = _trackers->dronetracker()->drone_startup_location().x;
            tmp.y = _trackers->dronetracker()->drone_startup_location().y+0.5f;
            tmp.z = _trackers->dronetracker()->drone_startup_location().z;
        }

        cv::Point3f resf  =world2im_3d(tmp,_visdat->Qfi,_visdat->camera_angle);
        return cv::Point2i(roundf(resf.x),round(resf.y));
    }
    bool drone_is_hunting() {
        if (_nav_flight_mode == nfm_hunt) {
            return _navigation_status == ns_chasing_insect || _navigation_status ==  ns_start_the_chase;
        } else {
            return false;
        }
    }
    bool drone_is_flying() {
        return _navigation_status < ns_landing && _navigation_status >  ns_take_off_completed;
    }
    bool drone_is_manual() {
        return _navigation_status == ns_manual;
    }
    bool time_for_restart() { // tmp function to signal restart so that another drone may fly
        return false; //navigation_status == ns_drone_problem;
    }

    void demo_flight(std::string flightplan_fn);

    Interceptor get_Interceptor() {return _iceptor;}
};
}
