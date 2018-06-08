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



    setpoints.push_back(waypoint(cv::Point3i(SETPOINTXMAX / 2,SETPOINTYMAX / 2,1000),40)); // this is overwritten by position trackbars!!!
    setpoints.push_back(waypoint(cv::Point3i(1000,600,2000),150));
    setpoints.push_back(waypoint(cv::Point3i(1500,600,1300),40));
    setpoints.push_back(waypoint(cv::Point3i(1500,300,1300),60));


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
    namedWindow("Nav", WINDOW_NORMAL);
    createTrackbar("X [mm]", "Nav", &params.setpoint_slider_X, SETPOINTXMAX);
    createTrackbar("Y [mm]", "Nav", &params.setpoint_slider_Y, SETPOINTYMAX);
    createTrackbar("Z [mm]", "Nav", &params.setpoint_slider_Z, SETPOINTZMAX);
    createTrackbar("WP id", "Nav", &wpid, setpoints.size()-1);
    createTrackbar("d threshold factor", "Nav", &params.distance_threshold_f, 10);
    createTrackbar("land_incr_f_mm", "Nav", &params.land_incr_f_mm, 50);
    createTrackbar("Land Decrease  ", "Nav", &params.autoLandThrottleDecreaseFactor, 50);

#endif

}

void DroneNavigation::update() {
    float dis = sqrtf(_dtrk->data.posErrX*_dtrk->data.posErrX + _dtrk->data.posErrY*_dtrk->data.posErrY + _dtrk->data.posErrZ*_dtrk->data.posErrZ);
    if (dis *1000 < setpoints[wpid].distance_threshold_mm * params.distance_threshold_f && !_dctrl->getAutoLand() && _dctrl->getAutoControl() && !_dctrl->getAutoTakeOff() && _dtrk->n_frames_tracking>5) {
        if (wpid < setpoints.size()-1) {
            wpid++;
            alert("canberra-gtk-play -f /usr/share/sounds/ubuntu/stereo/window-slide.ogg &");
        }
        else if (wpid == setpoints.size()-1)
            _dctrl->setAutoLand(true);
    }

    if (_dctrl->getAutoLand() && !_dtrk->data.landed && ((_dtrk->data.sposY < -(DRONE_MAX_BORDER_Y-0.15f) && fabs(_dtrk->data.svelY) < 0.2) || autoLandThrottleDecrease > 0)){
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

    if (!_dctrl->getAutoControl())
        wpid = 0;


    waypoint * wp;
    if (wpid > 0)
        wp = &setpoints[wpid];
    else { // read from position trackbars
        wp = &setpoints[wpid];
        wp->xyz.x = params.setpoint_slider_X;
        wp->xyz.y = params.setpoint_slider_Y;
        wp->xyz.z = params.setpoint_slider_Z;
    }

    setpoint = setpoints[wpid].xyz;

    setpoint_world.x = (wp->xyz.x - SETPOINTXMAX/2) / 1000.0f;
    setpoint_world.y = (wp->xyz.y - SETPOINTYMAX/2) / 1000.0f;
    setpoint_world.z = -(wp->xyz.z) / 1000.0f;

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
