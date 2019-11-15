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
                                                "ns_flower_of_fire",
                                                "ns_brick_of_fire",
                                                "ns_goto_landing",
                                                "ns_initial_reset_heading",
                                                "ns_reset_heading",
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
        nfm_hunt
    };
private:
    int v_crcl1 = 250;
    int v_crcl2 = 500;
    int r_crcl1 = 5;
    int r_crcl2 = 15;
    int w_sqr = 500;
    int v_sqr = 50;

    int enable_vel_control_x = 0;
    int enable_vel_control_y = 0;
    int enable_vel_control_z = 0;

    class navigationParameters: public xmls::Serializable
    {
    public:
        xmls::xInt distance_threshold_f;
        xmls::xInt land_incr_f_mm, autoLandThrottleDecreaseFactor;
        xmls::xFloat time_out_after_landing;

        navigationParameters() {
            // Set the XML class name.
            // This name can differ from the C++ class name
            setClassName("navigationParameters");

            // Set class version
            setVersion("1.1");

            // Register members. Like the class name, member names can differ from their xml depandants
            Register("distance_threshold_f",&distance_threshold_f);
            Register("land_incr_f_mm",&land_incr_f_mm);
            Register("autoLandThrottleDecreaseFactor",&autoLandThrottleDecreaseFactor);
            Register("time_out_after_landing",&time_out_after_landing);
        }
    };

    int distance_threshold_f;
    int setpoint_slider_X = 250;
    int setpoint_slider_Y = 250;
    int setpoint_slider_Z = 250;
    float time_out_after_landing;

    string settings_file = "../../xml/navigation.xml";
    void deserialize_settings();
    void serialize_settings();

    double time_initial_reset_heading = 0;
    double time_reset_heading = 0;
    double landed_time = 0;
    nav_flight_modes _nav_flight_mode;

    enum waypoint_flight_modes {
        fm_takeoff,
        fm_flying,
        fm_flower,
        fm_brick,
        fm_wp_stay,
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
            threshold_mm = 50;
            mode = fm_landing;
        }
    };
    struct takeoff_waypoint : waypoint{
        takeoff_waypoint(){
            mode = fm_takeoff;
        }
    };
    struct flower_waypoint : waypoint{
        flower_waypoint(cv::Point3f p) : waypoint(p,0) {
            mode = fm_flower;
        }
    };
    struct brick_waypoint : waypoint{
        brick_waypoint(cv::Point3f p) : waypoint(p,0) {
            mode = fm_brick;
        }
    };
    struct stay_waypoint : waypoint{
        stay_waypoint(cv::Point3f p) : waypoint(p,0) {
            mode = fm_wp_stay;
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
        ns_flower_waypoint,
        ns_brick_waypoint,
        ns_goto_landing_waypoint,
        ns_initial_reset_heading,
        ns_reset_heading,
        ns_land,
        ns_landing,
        ns_landed,
        ns_wait_after_landing,
        ns_manual, // also for disarmed
        ns_drone_problem
    };
    navigation_states _navigation_status = ns_init;

    uint wpid = 0;
    std::vector<waypoint> setpoints;
    waypoint * current_setpoint = new landing_waypoint();


    double time_located_drone = 0;
    double time_taken_off = 0;

    std::ofstream *_logger;
    DroneController * _dctrl;
    TrackerManager * _trackers;
    Interceptor _iceptor;
    VisionData *_visdat;

    bool initialized = false;
    cv::Point3f square_point(cv::Point3f center, float width, float s);

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
            return static_cast<string>(navigation_status_names[_navigation_status]) + " " + to_string_with_precision(_dctrl->dist_to_setpoint(),2);
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

    void manual_trigger_next_wp(){
        if (wpid < setpoints.size()-1 && _nav_flight_mode == nfm_waypoint ) {
            wpid++;
            _navigation_status = ns_set_waypoint;
        }
    }
    void manual_trigger_prev_wp(){
        if (wpid > 0 && _nav_flight_mode == nfm_waypoint ) {
            wpid--;
            _navigation_status = ns_set_waypoint;
        }
    }

    void close (void);
    void init(std::ofstream *logger, TrackerManager * imngr, DroneController *dctrl, VisionData *visdat, CameraVolume *camvol);
    void update(double time);
    bool disable_insect_detection() {
        return _navigation_status < ns_wait_for_takeoff_command;
    }

    void redetect_drone_location(){
        _navigation_status = ns_locate_drone;
    }

    cv::Point2i drone_v_setpoint_im(){

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
    cv::Point2i drone_setpoint_im(){
        //transform to image coordinates:

        cv::Point3f tmp = setpoint_pos_world;
        if (!drone_is_flying()){
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
        return _navigation_status < ns_landing && _navigation_status >  ns_take_off_completed;
    }
    bool drone_is_manual(){
        return _navigation_status == ns_manual;
    }
    bool time_for_restart() { // tmp function to signal restart so that another drone may fly
        return _navigation_status == ns_drone_problem ||  _navigation_status == ns_wait_after_landing;
    }

    Interceptor get_Interceptor(){return _iceptor;}
};

#endif //DRONENAVIGATION
