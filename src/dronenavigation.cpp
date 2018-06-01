#include <iostream>
#include "dronenavigation.h"

using namespace cv;
using namespace std;

#define POSITIONTRACKBARS

bool DroneNavigation::init(std::ofstream *logger, DroneTracker * dtrk, DroneController * dctrl) {
    _logger = logger;
    _dtrk = dtrk;
    _dctrl = dctrl;

    //(*_logger) << "imLx; imLy; disparity;";


    setpoints.push_back(cv::Point3i(SETPOINTXMAX / 2,SETPOINTYMAX / 2,1000)); // this is overwritten by position trackbars!!!
    setpoints.push_back(cv::Point3i(1500,600,1000));
    setpoints.push_back(cv::Point3i(1800,600,1200));
    setpoints.push_back(cv::Point3i(1200,600,1000));
    setpoints.push_back(cv::Point3i(1000,600,2000));
    setpoints.push_back(cv::Point3i(2000,600,2000));
    setpoints.push_back(cv::Point3i(1800,600,1200));

    //far
    setpoints.push_back(cv::Point3i(1800,1000,1200));
    setpoints.push_back(cv::Point3i(1200,1000,1000));
    setpoints.push_back(cv::Point3i(1000,1000,2000));
    setpoints.push_back(cv::Point3i(2000,1000,2000));
    setpoints.push_back(cv::Point3i(1800,1000,1200));


#ifdef POSITIONTRACKBARS
    namedWindow("Setpoint", WINDOW_NORMAL);
    createTrackbar("X [mm]", "Setpoint", &setpointX, SETPOINTXMAX);
    createTrackbar("Y [mm]", "Setpoint", &setpointY, SETPOINTYMAX);
    createTrackbar("Z [mm]", "Setpoint", &setpointZ, SETPOINTZMAX);
    createTrackbar("WP id", "Setpoint", &wpid, setpoints.size()-1);
#endif

}

void DroneNavigation::update() {
    cv::Point3i tmps;
    if (wpid > 0)
        tmps = setpoints[wpid];
    else { // read from position trackbars
        tmps.x = setpointX;
        tmps.y = setpointY;
        tmps.z = setpointZ;
    }

    setpoint_world.x = (tmps.x - SETPOINTXMAX/2) / 1000.0f;
    setpoint_world.y = (tmps.y - SETPOINTYMAX/2) / 1000.0f;
    setpoint_world.z = -(tmps.z) / 1000.0f;
}
