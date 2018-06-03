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
    setpoints.push_back(cv::Point3i(1000,600,2000));
    setpoints.push_back(cv::Point3i(1500,600,1000));


    /* // fly squares
    setpoints.push_back(cv::Point3i(1800,600,1200));
    setpoints.push_back(cv::Point3i(1200,600,1000));
    setpoints.push_back(cv::Point3i(1000,600,2000));
    setpoints.push_back(cv::Point3i(2000,600,2000));
    setpoints.push_back(cv::Point3i(1800,600,1200));
    setpoints.push_back(cv::Point3i(1800,1000,1200));
    setpoints.push_back(cv::Point3i(1200,1000,1000));
    setpoints.push_back(cv::Point3i(1000,1000,2000));
    setpoints.push_back(cv::Point3i(2000,1000,2000));
    setpoints.push_back(cv::Point3i(1800,1000,1200));
    */


#ifdef POSITIONTRACKBARS
    namedWindow("Setpoint", WINDOW_NORMAL);
    createTrackbar("X [mm]", "Setpoint", &setpoint_slider_X, SETPOINTXMAX);
    createTrackbar("Y [mm]", "Setpoint", &setpoint_slider_Y, SETPOINTYMAX);
    createTrackbar("Z [mm]", "Setpoint", &setpoint_slider_Z, SETPOINTZMAX);
    createTrackbar("WP id", "Setpoint", &wpid, setpoints.size()-1);
    createTrackbar("d threshold", "Setpoint", &distance_threshold_mm, 1000);
    createTrackbar("land_incr_f_mm", "Setpoint", &land_incr_f_mm, 50);

#endif

}

void DroneNavigation::update() {
    float dis = sqrtf(_dtrk->data.posErrX*_dtrk->data.posErrX + _dtrk->data.posErrY*_dtrk->data.posErrY + _dtrk->data.posErrZ*_dtrk->data.posErrZ);
    if (dis *1000 < distance_threshold_mm && !_dctrl->getAutoLand() && _dctrl->getAutoControl() && !_dctrl->getAutoTakeOff() && _dtrk->n_frames_tracking>5) {
        wpid++;
        if (wpid >= setpoints.size()-1) {
            wpid = setpoints.size()-1;
            _dctrl->doAutoLand();
        }
    }

    if (_dctrl->getAutoLand() && _dtrk->data.reset_filters){
        _dtrk->data.landed = true;
        wpid = 0;
    }


    cv::Point3i tmps;
    if (wpid > 0)
        tmps = setpoints[wpid];
    else { // read from position trackbars
        tmps.x = setpoint_slider_X;
        tmps.y = setpoint_slider_Y;
        tmps.z = setpoint_slider_Z;
    }

    setpoint_world.x = (tmps.x - SETPOINTXMAX/2) / 1000.0f;
    setpoint_world.y = (tmps.y - SETPOINTYMAX/2) / 1000.0f;
    setpoint_world.z = -(tmps.z) / 1000.0f;

    if (_dctrl->getAutoLand()) {
        land_incr += ((float)land_incr_f_mm)/1000.f;
        setpoint_world.y -= land_incr;
    } else {
        land_incr = 0;
    }

    cout << "WP ID: " <<  wpid << " distance: " <<  dis*1000 << endl;

}
