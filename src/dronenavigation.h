#ifndef DRONENAVIGATION_H
#define DRONENAVIGATION_H
#include "defines.h"

#include "common.h"
#include "dronetracker.h"
#include "dronecontroller.h"
#include "insecttracker.h"
#include "interceptor.h"

/*
 * This class will navigate a micro drone
 *
 */
static const char* navigation_status_names[] = {"ns_init",
                                                "ns_locate_drone",
                                                "ns_wait_locate_drone",
                                                "ns_located_drone",
                                                "ns_wait_for_takeoff",
                                                "ns_wait_for_insect",
                                                "ns_init_calibrate_hover",
                                                "ns_takeoff",
                                                "ns_taking_off",
                                                "ns_take_off_completed",
                                                "ns_calibrate_hover",
                                                "ns_start_the_chase",
                                                "ns_chasing_insect",
                                                "ns_set_waypoint",
                                                "ns_approach_waypoint",
                                                "ns_stay_waypoint",
                                                "ns_goto_landing",
                                                "ns_land",
                                                "ns_landing",
                                                "ns_landed",
                                                "ns_wait_after_landing",
                                                "ns_manual",
                                                "ns_drone_problem"};

class DroneNavigation {
public:
    enum nav_flight_modes {
        nfm_manual,
        nfm_waypoint,
        nfm_slider,
        nfm_hunt
    };
private:

    float landed_time = 0;
    nav_flight_modes _nav_flight_mode;

    enum waypoint_flight_modes {
        //todo: replace this enum with a type check https://stackoverflow.com/questions/351845/finding-the-type-of-an-object-in-c
        fm_takeoff,
        fm_hover_calib,
        fm_flying,
        fm_slider,
        fm_landing
    };

    struct waypoint{
        waypoint(cv::Point3f p, int distance_threshold_mm) {
            xyz = p;
            threshold_mm = distance_threshold_mm;
            mode  = fm_flying;
        }
        cv::Point3f xyz;
        int threshold_mm;
        waypoint_flight_modes mode;
    protected:
        waypoint(){}
    };
    struct landing_waypoint : waypoint{
        landing_waypoint(){
            xyz = cv::Point3f(0,.5f,0); // 1 meter over, relative to the startup location
            threshold_mm = 20;
            mode = fm_landing;
        }
    };
    struct takeoff_waypoint : waypoint{
        takeoff_waypoint(){
            mode = fm_takeoff;
        }
    };
    struct hovercalib_waypoint : waypoint{
        hovercalib_waypoint(){
            xyz = cv::Point3f(0,.5f,0); // 1 meter over, relative to the startup location
            threshold_mm = 20;
            mode = fm_hover_calib;
        }
    };
    struct slider_waypoint : waypoint{
        slider_waypoint(){
            //todo: implement
            mode = fm_slider;
        }
    };
    void set_next_waypoint(waypoint wp);

    enum navigation_states {
        ns_init=0,
        ns_locate_drone,
        ns_wait_locate_drone,
        ns_located_drone,
        ns_wait_for_takeoff_command,
        ns_wait_for_insect,
        ns_init_calibrate_hover,
        ns_takeoff,
        ns_taking_off,
        ns_take_off_completed,
        ns_calibrate_hover,
        ns_start_the_chase,
        ns_chasing_insect,
        ns_set_waypoint,
        ns_approach_waypoint,
        ns_stay_waypoint,
        ns_goto_landing_waypoint,
        ns_land,
        ns_landing,
        ns_landed,
        ns_wait_after_landing,
        ns_manual, // also for disarmed
        ns_drone_problem
    };
    navigation_states _navigation_status = ns_init;

    float land_incr = 0;
    uint wpid = 0;
    std::vector<waypoint> setpoints;
    waypoint * current_setpoint = new landing_waypoint();

    int autoLandThrottleDecrease = 0;

    float time_located_drone = 0;

    std::ofstream *_logger;
    DroneTracker * _dtrk;
    DroneController * _dctrl;
    Interceptor _iceptor;
    VisionData *_visdat;

    bool _calibrating_hover = false;
    bool initialized = false;

    float _dist_to_wp = 0;
    cv::Mat Qfi;

public:

    nav_flight_modes Nav_Flight_Mode(){
        return _nav_flight_mode;
    }
    void set_nav_flight_mode(nav_flight_modes m){
        if (m == nfm_manual)
            _navigation_status = ns_manual;
        _nav_flight_mode = m;
    }
    std::string Navigation_Status() {
        if (_navigation_status == ns_approach_waypoint) {
            return static_cast<string>(navigation_status_names[_navigation_status]) + " " + to_string_with_precision(_dist_to_wp,2);
        } else

            return navigation_status_names[_navigation_status];
    }

    struct navigationParameters{
        int distance_threshold_f = 1;

        int setpoint_slider_X = 0 ; //SETPOINTXMAX / 2;
        int setpoint_slider_Y = 600;
        int setpoint_slider_Z = 1000;

        int land_incr_f_mm = 5;
        int autoLandThrottleDecreaseFactor = 3;
        int auto_takeoff_speed = 3;
        float time_out_after_landing = 0.5f;

        float version = 2.3f;

        template <class Archive>
        void serialize( Archive & ar )
        {
            ar( version, distance_threshold_f,setpoint_slider_X,setpoint_slider_Y,setpoint_slider_Z,land_incr_f_mm,autoLandThrottleDecreaseFactor,auto_takeoff_speed,time_out_after_landing);
        }
    };

    navigationParameters params;

    cv::Point3d setpoint;
    cv::Point3f setpoint_pos_world;
    cv::Point3f setpoint_vel_world;
    cv::Point3f setpoint_acc_world;
    int distance_threshold_mm() {
        return current_setpoint->threshold_mm;
    }

    float dist_to_wp() {
        return _dist_to_wp;
    }

    void close (void);
    void init(std::ofstream *logger, DroneTracker *dtrk, DroneController *dctrl, InsectTracker *itrkr, VisionData *visdat);
    void update(float time);
    bool disable_insect_detection() {
        return _navigation_status < ns_wait_for_takeoff_command;
    }

    void redetect_drone_location(){
        _navigation_status = ns_locate_drone;
    }

    cv::Point2i drone_setpoint_im(){
        //transform to image coordinates:

        cv::Point3f tmp = setpoint_pos_world;
        if (_navigation_status == ns_takeoff || _navigation_status == ns_taking_off){
            tmp.x = _dtrk->Drone_Startup_Location().x;
            tmp.y = _dtrk->Drone_Startup_Location().y+0.5f;
            tmp.z = _dtrk->Drone_Startup_Location().z;
        }

        std::vector<cv::Point3d> world_coordinates,camera_coordinates;
        cv::Point3d tmpd;
        float theta = -_visdat->camera_angle * deg2rad;
        float temp_y = tmp.y * cosf(theta) + tmp.z * sinf(theta);
        tmpd.z = -tmp.y * sinf(theta) + tmp.z * cosf(theta);
        tmpd.y = temp_y;
        tmpd.x = tmp.x;

        world_coordinates.push_back(tmpd);
        cv::perspectiveTransform(world_coordinates,camera_coordinates,Qfi);
        return cv::Point2i(camera_coordinates[0].x,camera_coordinates[0].y);

    }
    bool drone_is_hunting(){
        if (_nav_flight_mode == nfm_hunt){
            return _navigation_status == ns_chasing_insect || _navigation_status ==  ns_start_the_chase;
        } else {
            return false;
        }
    }
    bool drone_is_flying(){
         return _navigation_status < ns_landing && _navigation_status >  ns_takeoff;

    }


};

#endif //DRONENAVIGATION
