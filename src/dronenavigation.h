#ifndef DRONENAVIGATION_H
#define DRONENAVIGATION_H
#include "defines.h"

#include "common.h"
#include "dronetracker.h"
#include "dronecontroller.h"

#include <cereal/types/unordered_map.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/archives/binary.hpp>
#include <fstream>
#include <iomanip>
#include <unistd.h>

/*
 * This class will navigate a micro drone
 *
 */
class DroneNavigation {

private:

#define SETPOINTXMAX 3000 // in mm
#define SETPOINTYMAX 3000 // in mm
#define SETPOINTZMAX 5000 // in mm

    struct navigationParameters{
        int distance_threshold_mm = 40;

        int setpoint_slider_X = SETPOINTXMAX / 2;
        int setpoint_slider_Y = 600;
        int setpoint_slider_Z = 1000;

        int land_incr_f_mm = 50;
        int autoLandThrottleDecreaseFactor = 10;

        template <class Archive>
        void serialize( Archive & ar )
        {
            ar( distance_threshold_mm,setpoint_slider_X,setpoint_slider_Y,setpoint_slider_Z,land_incr_f_mm,autoLandThrottleDecreaseFactor);
        }
    };


    float land_incr = 0;
    int wpid = 0;
    std::vector<cv::Point3i> setpoints;

    int autoLandThrottleDecrease = 0;


    std::ofstream *_logger;
    DroneTracker * _dtrk;
    DroneController * _dctrl;

    navigationParameters params;
public:

    cv::Point3d setpoint;
    cv::Point3f setpoint_world;

    void close (void);
    bool init(std::ofstream *logger, DroneTracker *dtrk, DroneController *dctrl);
    void update();

};

#endif //DRONENAVIGATION
