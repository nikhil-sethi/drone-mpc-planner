#ifndef DRONENAVIGATION_H
#define DRONENAVIGATION_H
#include "defines.h"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>

#include "smoother.h"
#include "common.h"
#include "dronetracker.h"
#include "dronecontroller.h"


/*
 * This class will navigate a micro drone
 *
 */
class DroneNavigation {

private:

#define SETPOINTXMAX 3000 // in mm
#define SETPOINTYMAX 3000 // in mm
#define SETPOINTZMAX 5000 // in mm

    float land_incr = 0;
    int land_incr_f_mm = 10;
    int setpoint_slider_X = SETPOINTXMAX / 2;
    int setpoint_slider_Y = 600;
    int setpoint_slider_Z = 1000;
    int wpid = 0;


    int distance_threshold_mm = 40;
    std::vector<cv::Point3i> setpoints;

    std::ofstream *_logger;

    DroneTracker * _dtrk;
    DroneController * _dctrl;



public:


    cv::Point3d setpoint;
    cv::Point3f setpoint_world;

    void close (void);
    bool init(std::ofstream *logger, DroneTracker *dtrk, DroneController *dctrl);
    void update();

};

#endif //DRONENAVIGATION
