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
class DroneNavigation {

private:

#define SETPOINTXMAX 3000 // in mm
#define SETPOINTYMAX 3000 // in mm
#define SETPOINTZMAX 5000 // in mm

    struct waypoint{
        waypoint(cv::Point3i xyz, int distance_threshold_mm) {
            _xyz = xyz;
            _distance_threshold_mm = distance_threshold_mm;
        }
        cv::Point3i _xyz;
        int _distance_threshold_mm;
    };

    enum Navigation_Status {
        navigation_status_init = 0,
        navigation_status_calibrating_motion_background = 1,
        navigation_status_locate_drone = 2,
        navigation_status_wait_locate_drone = 3,
        navigation_status_located_drone = 4,
        navigation_status_wait_for_insect=5,
        navigation_status_takeoff=6,
        navigation_status_taking_off=7,
        navigation_status_take_off_completed=8,
        navigation_status_start_the_chase=9,
        navigation_status_chasing_insect=10,
        navigation_status_set_waypoint_in_flightplan=11,
        navigation_status_approach_waypoint_in_flightplan=12,
        navigation_status_stay_waypoint_in_flightplan=13,
        navigation_status_stay_slider_waypoint=14,
        navigation_status_goto_landing_waypoint=15,
        navigation_status_land=16,
        navigation_status_landing=17,
        navigation_status_landed=18,
        navigation_status_manual=19,
        navigation_status_drone_problem=99
    };
    Navigation_Status navigation_status = navigation_status_init;

    float land_incr = 0;
    uint wpid = 0;
    std::vector<waypoint> setpoints;

    int autoLandThrottleDecrease = 0;


    std::ofstream *_logger;
    DroneTracker * _dtrk;
    DroneController * _dctrl;
    Interceptor _iceptor;
    VisionData *_visdat;

public:

    struct navigationParameters{
        int distance_threshold_f = 1;

        int setpoint_slider_X = SETPOINTXMAX / 2;
        int setpoint_slider_Y = 600;
        int setpoint_slider_Z = 1000;

        int land_incr_f_mm = 50;
        int autoLandThrottleDecreaseFactor = 10;
        int auto_takeoff_speed = 3;

        float version = 2.0f;

        template <class Archive>
        void serialize( Archive & ar )
        {
            ar( version, distance_threshold_f,setpoint_slider_X,setpoint_slider_Y,setpoint_slider_Z,land_incr_f_mm,autoLandThrottleDecreaseFactor,auto_takeoff_speed);
        }
    };

    navigationParameters params;

    cv::Point3d setpoint;
    cv::Point3f setpoint_world;
    cv::Point3f setspeed_world;
    int distance_threshold_mm;

    void close (void);
    bool init(std::ofstream *logger, DroneTracker *dtrk, DroneController *dctrl, InsectTracker *itrkr, VisionData *visdat);
    void update();
    bool disable_insect_detection() {
        return navigation_status < navigation_status_wait_for_insect;
    }


};

#endif //DRONENAVIGATION
