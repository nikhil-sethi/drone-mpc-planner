#ifndef DRONENAVIGATION_H
#define DRONENAVIGATION_H
#include "defines.h"

#include "common.h"
#include "dronetracker.h"
#include "dronecontroller.h"
#include "insecttracker.h"
#include "trackermanager.h"
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
                                                "ns_takeoff",
                                                "ns_taking_off",
                                                "ns_take_off_completed",
                                                "ns_start_the_chase",
                                                "ns_chasing_insect_ff",
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
        nfm_none,
        nfm_manual,
        nfm_waypoint,
        nfm_slider,
        nfm_hunt
    };
private:

    class navigationParameters: public xmls::Serializable
    {
    public:
        xmls::xInt distance_threshold_f,setpoint_slider_X,setpoint_slider_Y,setpoint_slider_Z;
        xmls::xInt land_incr_f_mm, autoLandThrottleDecreaseFactor;
        xmls::xFloat time_out_after_landing;

        navigationParameters() {
            // Set the XML class name.
            // This name can differ from the C++ class name
            setClassName("navigationParameters");

            // Set class version
            setVersion("1.0");

            // Register members. Like the class name, member names can differ from their xml depandants
            Register("distance_threshold_f",&distance_threshold_f);
            Register("setpoint_slider_X",&setpoint_slider_X);
            Register("setpoint_slider_Y",&setpoint_slider_Y);
            Register("setpoint_slider_Z",&setpoint_slider_Z);
            Register("land_incr_f_mm",&land_incr_f_mm);
            Register("autoLandThrottleDecreaseFactor",&autoLandThrottleDecreaseFactor);
            Register("time_out_after_landing",&time_out_after_landing);
        }
    };

    int distance_threshold_f;
    int setpoint_slider_X ;
    int setpoint_slider_Y;
    int setpoint_slider_Z;
    int land_incr_f_mm;
    int autoLandThrottleDecreaseFactor;
    double time_out_after_landing;

    string settings_file = "../../xml/navigation.xml";
    void deserialize_settings();
    void serialize_settings();

    double landed_time = 0;
    nav_flight_modes _nav_flight_mode;

    enum waypoint_flight_modes {
        fm_takeoff,
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
    struct slider_waypoint : waypoint{
        slider_waypoint(){
            //todo: implement
            mode = fm_slider;
        }
    };
    void next_waypoint(waypoint wp);


    enum navigation_states {
        ns_init=0,
        ns_locate_drone,
        ns_wait_locate_drone,
        ns_located_drone,
        ns_wait_for_takeoff_command,
        ns_wait_for_insect,
        ns_takeoff,
        ns_taking_off,
        ns_take_off_completed,
        ns_start_the_chase,
        ns_chasing_insect_ff,
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

    double time_located_drone = 0;
    double time_taken_off = 0;

    std::ofstream *_logger;
    DroneController * _dctrl;
    TrackerManager * _trackers;
    Interceptor _iceptor;
    VisionData *_visdat;

    bool initialized = false;

    float _dist_to_wp = 0;

public:

    nav_flight_modes nav_flight_mode(){
        return _nav_flight_mode;
    }
    void nav_flight_mode(nav_flight_modes m){
        if (m == nfm_manual)
            _navigation_status = ns_manual;
        _nav_flight_mode = m;
    }
    std::string navigation_status() {
        if (_navigation_status == ns_approach_waypoint) {
            return static_cast<string>(navigation_status_names[_navigation_status]) + " " + to_string_with_precision(_dist_to_wp,2);
        } else

            return navigation_status_names[_navigation_status];
    }

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
    void init(std::ofstream *logger, TrackerManager * imngr, DroneController *dctrl, VisionData *visdat);
    void update(double time);
    bool disable_insect_detection() {
        return _navigation_status < ns_wait_for_takeoff_command;
    }

    void redetect_drone_location(){
        _navigation_status = ns_locate_drone;
    }

    cv::Point2i drone_v_setpoint_im(){

        cv::Point3f tmp = 0.1*setpoint_vel_world + setpoint_pos_world;
        if (_navigation_status == ns_takeoff || _navigation_status == ns_taking_off || _navigation_status == ns_take_off_completed)
            tmp = {0};

        std::vector<cv::Point3d> world_length,camera_length;
        cv::Point3d tmpd;
        float theta = -_visdat->camera_angle * deg2rad;
        float temp_y = tmp.y * cosf(theta) + tmp.z * sinf(theta);
        tmpd.z = -tmp.y * sinf(theta) + tmp.z * cosf(theta);
        tmpd.y = temp_y;
        tmpd.x = tmp.x;

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
    cv::Point2i drone_setpoint_im(){
        //transform to image coordinates:

        cv::Point3f tmp = setpoint_pos_world;
        if (_navigation_status == ns_takeoff || _navigation_status == ns_taking_off || _navigation_status == ns_take_off_completed){
            tmp.x = _trackers->dronetracker()->drone_startup_location().x;
            tmp.y = _trackers->dronetracker()->drone_startup_location().y+0.5f;
            tmp.z = _trackers->dronetracker()->drone_startup_location().z;
        }

        cv::Point3f resf  =world2im_3d(tmp,_visdat->Qfi,_visdat->camera_angle);
        return cv::Point2i(roundf(resf.x),round(resf.y));
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
    bool drone_is_manual(){
        return _navigation_status == ns_manual;
    }

    Interceptor get_Interceptor(){return _iceptor;}
};

#endif //DRONENAVIGATION
