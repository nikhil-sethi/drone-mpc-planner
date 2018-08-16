#include <iostream>
#include "dronenavigation.h"

using namespace cv;
using namespace std;

#ifdef HASSCREEN
//#define TUNING
#endif

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


    //waypoints work as follows:
    //(cv::Point3i(x,y,z),
    // x = 1500 means the middle of the camera, x = 0 means 1.5m right!!! of the middle. (counter intuitive)
    // y = 1500 is on the same height as the camera position itself, y = 0 means 1.5m below the cam position
    //z = 0 means distance from the camera is zero. z = 1500 means 1.5m from the camera

    // large scale flight plan
    //setpoints.push_back(waypoint(cv::Point3i(SETPOINTXMAX / 2,SETPOINTYMAX / 2,1000),40)); // this is overwritten by position trackbars!!!
    setpoints.push_back(waypoint(cv::Point3i(SETPOINTXMAX / 2,1,1000),40)); // this is overwritten by position trackbars!!!
    setpoints.push_back(waypoint(cv::Point3i(1000,400,1500),25));
    setpoints.push_back(waypoint(cv::Point3i(2000,1000,1500),25));
    setpoints.push_back(waypoint(cv::Point3i(1000,400,1500),25));
    setpoints.push_back(waypoint(cv::Point3i(2000,1000,1500),25));
    setpoints.push_back(waypoint(cv::Point3i(1000,400,1500),25));
    setpoints.push_back(waypoint(cv::Point3i(2000,1000,1500),25));
    setpoints.push_back(waypoint(cv::Point3i(1000,400,1500),25));
    setpoints.push_back(waypoint(cv::Point3i(2000,1000,1500),25));
    setpoints.push_back(waypoint(cv::Point3i(1000,400,1500),25));
    setpoints.push_back(waypoint(cv::Point3i(2000,1000,1500),25));
    setpoints.push_back(waypoint(cv::Point3i(1000,400,1500),25));
    setpoints.push_back(waypoint(cv::Point3i(2000,1000,1500),25));
    setpoints.push_back(waypoint(cv::Point3i(1000,400,1500),25));
    setpoints.push_back(waypoint(cv::Point3i(2000,1000,1500),25));
    setpoints.push_back(waypoint(cv::Point3i(1000,400,1500),25));
    setpoints.push_back(waypoint(cv::Point3i(2000,1000,1500),25));
    setpoints.push_back(waypoint(cv::Point3i(1000,400,1500),25));
    setpoints.push_back(waypoint(cv::Point3i(2000,1000,1500),25));
    setpoints.push_back(waypoint(cv::Point3i(1000,400,1500),25));
    setpoints.push_back(waypoint(cv::Point3i(2000,1000,1500),25));
    setpoints.push_back(waypoint(cv::Point3i(1000,400,1500),25));
    setpoints.push_back(waypoint(cv::Point3i(2000,1000,1500),25));
    setpoints.push_back(waypoint(cv::Point3i(1000,400,1500),25));
    setpoints.push_back(waypoint(cv::Point3i(2000,1000,1500),25));

    setpoints.push_back(waypoint(cv::Point3i(1500,200,1300),40)); // landing waypoint (=last one), must be 1 meter above the ground in world coordinatates
    //setpoints.push_back(waypoint(cv::Point3i(1500,300,1300),60));


    /* // small scale flight plan
    setpoints.push_back(waypoint(cv::Point3i(1500,900,1000),40)); // this is overwritten by position trackbars!!!
    setpoints.push_back(waypoint(cv::Point3i(1200,900,1000),40));
    setpoints.push_back(waypoint(cv::Point3i(1500,900,1000),40));
    */

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
    createTrackbar("WP id", "Nav", (int*)&wpid, setpoints.size()-1);
    createTrackbar("d threshold factor", "Nav", &params.distance_threshold_f, 10);
    createTrackbar("land_incr_f_mm", "Nav", &params.land_incr_f_mm, 50);
    createTrackbar("Land Decrease  ", "Nav", &params.autoLandThrottleDecreaseFactor, 50);

#endif

    return false;
}

void DroneNavigation::update() {
    trackData data = _dtrk->get_last_track_data();
    float dis = sqrtf(data.posErrX*data.posErrX + data.posErrY*data.posErrY + data.posErrZ*data.posErrZ);
    if (dis *1000 < setpoints[wpid]._distance_threshold_mm * params.distance_threshold_f && !_dctrl->getAutoLand() && _dctrl->getAutoControl() && !_dctrl->getAutoTakeOff() && _dtrk->n_frames_tracking>5) {
        if (wpid < setpoints.size()-1) {
            wpid++;
            alert("canberra-gtk-play -f /usr/share/sounds/ubuntu/stereo/window-slide.ogg &");
        } else if (wpid == setpoints.size()-1)
            _dctrl->setAutoLand(true);
        if (wpid == 1)
            _dctrl->recalibrateHover();
    }

    if (_dctrl->getAutoLand() && !_dctrl->landed && ((data.sposY < -(_dtrk->drone_max_border_y-0.18f)) || autoLandThrottleDecrease > 0)){
        if (autoLandThrottleDecrease == 0)
            alert("canberra-gtk-play -f /usr/share/sounds/ubuntu/notifications/Slick.ogg &");
        autoLandThrottleDecrease = params.autoLandThrottleDecreaseFactor;
        _dctrl->setAutoLandThrottleDecrease(autoLandThrottleDecrease);
        _dtrk->drone_max_border_y = 0;
    }
    if (autoLandThrottleDecrease >1000 || (_dctrl->autoThrottle <= 1050 && _dctrl->getAutoControl()) || (autoLandThrottleDecrease > 0 && !_dctrl->getAutoControl()) || (autoLandThrottleDecrease > 500 && _dtrk->n_frames_tracking>0)) {
        _dctrl->landed = true;
        wpid = 0;
        autoLandThrottleDecrease = 0;
        _dctrl->setAutoLandThrottleDecrease(0);
        _dctrl->setAutoLand(false);
        _dtrk->drone_max_border_y = MAX_BORDER_Y_DEFAULT;
    }

    if (!_dctrl->getAutoControl())
        wpid = 0;


    waypoint * wp;
    if (wpid > 0)
        wp = &setpoints[wpid];
    else { // read from position trackbars
        wp = &setpoints[wpid];
        wp->_xyz.x = params.setpoint_slider_X;
        wp->_xyz.y = params.setpoint_slider_Y;
        wp->_xyz.z = params.setpoint_slider_Z;
    }

    setpoint = setpoints[wpid]._xyz;

    setpoint_world.x = (wp->_xyz.x - SETPOINTXMAX/2) / 1000.0f;
    setpoint_world.y = (wp->_xyz.y - SETPOINTYMAX/2) / 1000.0f;
    setpoint_world.z = -(wp->_xyz.z) / 1000.0f;
    distance_threshold_mm = wp->_distance_threshold_mm;

    if (_dctrl->getAutoLand()) {
        if ( setpoint_world.y - land_incr> -(_dtrk->drone_max_border_y+100000.0f))
            land_incr += ((float)params.land_incr_f_mm)/1000.f;
        setpoint_world.y -= land_incr;

    } else {
        land_incr = 0;
    }

}

void DroneNavigation::close() {
    std::ofstream os(paramsFile, std::ios::binary);
    cereal::BinaryOutputArchive archive( os );
    archive( params );
}
