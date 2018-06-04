#include <iostream>
#include "dronenavigation.h"

using namespace cv;
using namespace std;

#define TUNING

const string paramsFile = "../navigationParameters.dat";

bool DroneNavigation::init(std::ofstream *logger, DroneTracker * dtrk, DroneController * dctrl) {
    _logger = logger;
    _dtrk = dtrk;
    _dctrl = dctrl;

    // Load saved control paremeters
    if (checkFileExist(paramsFile)) {
        std::ifstream is(paramsFile, std::ios::binary);
        cereal::BinaryInputArchive archive( is );
        archive(params);
    }

    //(*_logger) << "imLx; imLy; disparity;";



    setpoints.push_back(cv::Point3i(SETPOINTXMAX / 2,SETPOINTYMAX / 2,1000)); // this is overwritten by position trackbars!!!
    setpoints.push_back(cv::Point3i(1000,600,2000));
    setpoints.push_back(cv::Point3i(1500,600,1300));
    setpoints.push_back(cv::Point3i(1500,300,1300));


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


#ifdef TUNING
    namedWindow("Setpoint", WINDOW_NORMAL);
    createTrackbar("X [mm]", "Setpoint", &params.setpoint_slider_X, SETPOINTXMAX);
    createTrackbar("Y [mm]", "Setpoint", &params.setpoint_slider_Y, SETPOINTYMAX);
    createTrackbar("Z [mm]", "Setpoint", &params.setpoint_slider_Z, SETPOINTZMAX);
    createTrackbar("WP id", "Setpoint", &wpid, setpoints.size()-1);
    createTrackbar("d threshold", "Setpoint", &params.distance_threshold_mm, 1000);
    createTrackbar("land_incr_f_mm", "Setpoint", &params.land_incr_f_mm, 50);
    createTrackbar("Land Decrease  ", "Setpoint", &params.autoLandThrottleDecreaseFactor, 50);

#endif

}

void DroneNavigation::update() {
    float dis = sqrtf(_dtrk->data.posErrX*_dtrk->data.posErrX + _dtrk->data.posErrY*_dtrk->data.posErrY + _dtrk->data.posErrZ*_dtrk->data.posErrZ);
    if (dis *1000 < params.distance_threshold_mm && !_dctrl->getAutoLand() && _dctrl->getAutoControl() && !_dctrl->getAutoTakeOff() && _dtrk->n_frames_tracking>5) {
        if (wpid < setpoints.size()-1) {
            wpid++;
            alert("canberra-gtk-play -f /usr/share/sounds/ubuntu/stereo/window-slide.ogg &");
        }
        else if (wpid == setpoints.size()-1)
            _dctrl->setAutoLand(true);
    }

    if (_dctrl->getAutoLand() && !_dtrk->data.landed && ((_dtrk->data.sposY < -(DRONE_MAX_BORDER_Y-0.1f) && fabs(_dtrk->data.svelY) < 0.2) || autoLandThrottleDecrease > 0)){
        if (autoLandThrottleDecrease == 0)
            alert("canberra-gtk-play -f /usr/share/sounds/ubuntu/notifications/Slick.ogg &");
        autoLandThrottleDecrease = params.autoLandThrottleDecreaseFactor;
        _dctrl->setAutoLandThrottleDecrease(autoLandThrottleDecrease);
    }
    if (autoLandThrottleDecrease >1000 || (autoLandThrottleDecrease > 0 && !_dctrl->getAutoControl()) || (autoLandThrottleDecrease > 500 && _dtrk->data.reset_filters)) {
        _dtrk->data.landed = true;
        wpid = 0;
        autoLandThrottleDecrease = 0;
        _dctrl->setAutoLandThrottleDecrease(0);
        _dctrl->setAutoLand(false);
    }


    cv::Point3i tmps;
    if (wpid > 0)
        tmps = setpoints[wpid];
    else { // read from position trackbars
        tmps.x = params.setpoint_slider_X;
        tmps.y = params.setpoint_slider_Y;
        tmps.z = params.setpoint_slider_Z;
    }

    setpoint_world.x = (tmps.x - SETPOINTXMAX/2) / 1000.0f;
    setpoint_world.y = (tmps.y - SETPOINTYMAX/2) / 1000.0f;
    setpoint_world.z = -(tmps.z) / 1000.0f;

    if (_dctrl->getAutoLand()) {
        if ( setpoint_world.y - land_incr> -(DRONE_MAX_BORDER_Y-0.2f))
            land_incr += ((float)params.land_incr_f_mm)/1000.f;
        setpoint_world.y -= land_incr;

    } else {
        land_incr = 0;
    }

    cout << "WP ID: " <<  wpid << " distance: " <<  dis*1000 << endl;

}

void DroneNavigation::close() {
    std::ofstream os(paramsFile, std::ios::binary);
    cereal::BinaryOutputArchive archive( os );
    archive( params );
}
