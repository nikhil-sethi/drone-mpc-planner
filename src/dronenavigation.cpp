#include <iostream>
#include "dronenavigation.h"

using namespace cv;
using namespace std;

#ifdef HASSCREEN
//#define TUNING
#endif

const string paramsFile = "../navigationParameters.dat";

bool DroneNavigation::init(std::ofstream *logger, DroneTracker * dtrk, DroneController * dctrl, InsectTracker * itrkr, VisionData *visdat) {
    _logger = logger;
    _dtrk = dtrk;
    _dctrl = dctrl;
    _visdat = visdat;
    _iceptor.init(dtrk,itrkr);

    // Load saved control paremeters
    if (checkFileExist(paramsFile)) {
        std::ifstream is(paramsFile, std::ios::binary);
        cereal::BinaryInputArchive archive( is );
        try {
            archive(params);
        }catch (cereal::Exception e) {
            std::cout << "Dronecontroller settings file error: " << e.what() << std::endl;
            std::cout << "Maybe delete the file: " << paramsFile << std::endl;
            exit (1);
        }
        navigationParameters tmp;
        if (params.version < tmp.version){
            std::cout << "Dronecontroller settings version too low!" << std::endl;
            std::cout << "Maybe delete the file: " << paramsFile << std::endl;
            exit(1);
        }
    }

    //(*_logger) << "imLx; imLy; disparity;";


    //waypoints work as follows:
    //(cv::Point3i(x,y,z),
    // x = 1500 means the middle of the camera, x = 0 means 1.5m right!!! of the middle. (counter intuitive)
    // y = 1500 is on the same height as the camera position itself, y = 0 means 1.5m below the cam position
    //z = 0 means distance from the camera is zero. z = 1500 means 1.5m from the camera

    // large scale flight plan
    //setpoints.push_back(waypoint(cv::Point3i(SETPOINTXMAX / 2,SETPOINTYMAX / 2,1000),40)); // this is overwritten by position trackbars!!!
    setpoints.push_back(waypoint(cv::Point3i(1700,600,2300),30)); // this is overwritten by position trackbars!!!

    //setpoints.push_back(waypoint(cv::Point3i(1500,300,1500),0));
    //setpoints.push_back(waypoint(cv::Point3i(1500,-200,1500),0));

    //setpoints.push_back(waypoint(cv::Point3i(1000,-200,1500),0));
    //setpoints.push_back(waypoint(cv::Point3i(2000,-200,1500),0));

    //setpoints.push_back(waypoint(cv::Point3i(1500,400,1370),30));


    //setpoints.push_back(waypoint(cv::Point3i(2200,400,2000),50));
    //setpoints.push_back(waypoint(cv::Point3i(2200,400,3000),50));
    //setpoints.push_back(waypoint(cv::Point3i(2200,400,4000),50));
    //setpoints.push_back(waypoint(cv::Point3i(2200,400,5000),50));
    //setpoints.push_back(waypoint(cv::Point3i(1000,600,1500),30));
    //setpoints.push_back(waypoint(cv::Point3i(2000,600,1500),30));
    //setpoints.push_back(waypoint(cv::Point3i(1500,600,3500),0));
    //setpoints.push_back(waypoint(cv::Point3i(1500,600,4000),0));



    //setpoints.push_back(waypoint(cv::Point3i(1000,-125,1500),0));
    //setpoints.push_back(waypoint(cv::Point3i(2000,-125,1500),0));



    //setpoints.push_back(waypoint(cv::Point3i(1500,-250,1500),5));
    //setpoints.push_back(waypoint(cv::Point3i(1500,0,1500),5));






    setpoints.push_back(waypoint(cv::Point3i(1750,600,2310),10)); // landing waypoint (=last one), must be 1 meter above the ground in world coordinatates
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
    createTrackbar("WP id", "Nav", reinterpret_cast<int*>(wpid), setpoints.size()-1);
    createTrackbar("d threshold factor", "Nav", &params.distance_threshold_f, 10);
    createTrackbar("land_incr_f_mm", "Nav", &params.land_incr_f_mm, 50);
    createTrackbar("Land Decrease  ", "Nav", &params.autoLandThrottleDecreaseFactor, 50);
    createTrackbar("Take off speed threshold  ", "Nav", &params.auto_takeoff_speed, 50);

#endif

    return false;
}

void DroneNavigation::update() {

    waypoint * wp;

    _iceptor.update(navigation_status == navigation_status_wait_for_insect);

    switch (navigation_status) {
    case navigation_status_init: {
        navigation_status = navigation_status_calibrating_motion_background;
        break;
    } case navigation_status_calibrating_motion_background: {
        //wait until motion background done
        if (_visdat->background_calibrated()) {
            navigation_status = navigation_status_locate_drone;
#ifdef BEEP
            system("canberra-gtk-play -f /usr/share/sounds/ubuntu/notifications/Mallet.ogg &");
#endif
        }
        break;
    } case navigation_status_locate_drone: {
        _dctrl->blink_drone(true);
        _dtrk->Locate_Startup_Location();
        navigation_status = navigation_status_wait_locate_drone;
        break;
    } case navigation_status_wait_locate_drone: {
        if (_dtrk->blinking_drone_located())
            navigation_status = navigation_status_located_drone;
        break;
    } case navigation_status_located_drone: {
        _dctrl->blink_drone(false);
        navigation_status = navigation_status_wait_for_insect;
        break;
    } case navigation_status_wait_for_insect: {
        if (_dctrl->get_flight_mode() == DroneController::fm_manual)
            navigation_status=navigation_status_manual;
        else if (_iceptor.get_insect_in_range() || _dctrl->manual_override_take_off_now )
            navigation_status = navigation_status_takeoff;
        break;
    } case navigation_status_takeoff: {
        _dctrl->manual_override_take_off_now = false;
        _dctrl->manual_override_land_now = false;

        _dctrl->set_flight_mode(DroneController::fm_taking_off);
        navigation_status=navigation_status_taking_off;
        break;
    } case navigation_status_taking_off: {
        trackData data = _dtrk->get_last_track_data();
        if (data.svelY > static_cast<float>(params.auto_takeoff_speed) / 100.f ) {
            navigation_status = navigation_status_take_off_completed;
        }
        if (_dctrl->get_flight_mode() == DroneController::fm_manual)
            navigation_status=navigation_status_manual;
        break;
    } case navigation_status_take_off_completed: {
        _dctrl->init_ground_effect_compensation();
        alert("canberra-gtk-play -f /usr/share/sounds/ubuntu/notifications/Slick.ogg &");
        _dctrl->set_flight_mode(DroneController::fm_flying);
        //TODO: choose whether to fly waypoints (e.g. for testing or demos) or to start the chase
        //for now, just chase always
        navigation_status = navigation_status_start_the_chase;
        //OR fly waypoints

        //if(_dctrl->hoverthrottleInitialized)
        //navigation_status = navigation_status_start_the_chase;
        //else
        //navigation_status = navigation_status_set_waypoint_in_flightplan;
        break;
    } case navigation_status_start_the_chase: {
        _iceptor.reset_insect_cleared();
        navigation_status = navigation_status_chasing_insect;

        if (_iceptor.get_insect_in_range())
            navigation_status = navigation_status_chasing_insect;
        else
            navigation_status = navigation_status_goto_landing_waypoint;


    } FALLTHROUGH_INTENDED; case navigation_status_chasing_insect: {

        //update target chasing waypoint and speed
        if (_iceptor.get_insect_in_range()) {
            setpoint_world = _iceptor.get_intercept_position();
            if (setpoint_world.y < -1.2f)
                setpoint_world.y = -1.20f;
        }

        if (setpoint_world.z == 0) { // fly to landing waypoint (but do not land)
            setpoint_world.x = -1.0f;
            setpoint_world.y = -1.20f;
            setpoint_world.z = -2.2f;
        }

        if (_dctrl->get_flight_mode() == DroneController::fm_manual)
            navigation_status=navigation_status_manual;
        // TODO: return to landing waypoint after the insect was lost for several frames
        else if (_iceptor.get_insect_cleared())
            navigation_status = navigation_status_goto_landing_waypoint;
        break;
    } case navigation_status_goto_landing_waypoint: {
        wpid = static_cast<uint>(setpoints.size())-1; // last waypoint is the landing waypoint
        navigation_status = navigation_status_set_waypoint_in_flightplan;
    } FALLTHROUGH_INTENDED; case navigation_status_set_waypoint_in_flightplan: {
        wp = &setpoints[wpid];

        setpoint_world.x = (wp->_xyz.x - SETPOINTXMAX/2) / 1000.0f;
        setpoint_world.y = (wp->_xyz.y - SETPOINTYMAX/2) / 1000.0f;
        setpoint_world.z = -(wp->_xyz.z) / 1000.0f;
        distance_threshold_mm = wp->_distance_threshold_mm;

        setspeed_world.x = 0;
        setspeed_world.y = 0;
        setspeed_world.z = 0;

        navigation_status = navigation_status_approach_waypoint_in_flightplan;
        break;
    } case navigation_status_approach_waypoint_in_flightplan: {
        float dis = sqrtf(_dctrl->posErrX*_dctrl->posErrX + _dctrl->posErrY*_dctrl->posErrY + _dctrl->posErrZ*_dctrl->posErrZ);
        if (dis *1000 < setpoints[wpid]._distance_threshold_mm * params.distance_threshold_f && _dtrk->n_frames_tracking>5) {
            if (wpid < setpoints.size()-1) {
                wpid++;
                alert("canberra-gtk-play -f /usr/share/sounds/ubuntu/stereo/window-slide.ogg &");
                navigation_status = navigation_status_set_waypoint_in_flightplan;
            } else if (wpid == setpoints.size()-1)
                navigation_status = navigation_status_land;
            if (wpid == 1)
                _dctrl->recalibrateHover();
        } else
            navigation_status = navigation_status_set_waypoint_in_flightplan;

        if (_dctrl->get_flight_mode() == DroneController::fm_manual)
            navigation_status=navigation_status_manual;
        break;
    } case navigation_status_stay_waypoint_in_flightplan: {
        //TODO: implement
        if (_dctrl->get_flight_mode() == DroneController::fm_manual)
            navigation_status=navigation_status_manual;
        break;
    } case navigation_status_stay_slider_waypoint: {
        wp = &setpoints[wpid];
        wp->_xyz.x = params.setpoint_slider_X;
        wp->_xyz.y = params.setpoint_slider_Y;
        wp->_xyz.z = params.setpoint_slider_Z;
        if (_dctrl->get_flight_mode() == DroneController::fm_manual)
            navigation_status=navigation_status_manual;
        break;
    } case navigation_status_land: {
        _dtrk->drone_max_border_y = 9999; // keep tracking to the last possible end. TODO: earlier in the descend this may be disturbed by ground shadows
        _dctrl->set_flight_mode(DroneController::fm_landing);
        _dctrl->recalibrateHover();
        alert("canberra-gtk-play -f /usr/share/sounds/ubuntu/notifications/Slick.ogg &");
        navigation_status = navigation_status_landing;
    } FALLTHROUGH_INTENDED; case navigation_status_landing: {
        trackData data = _dtrk->get_last_track_data();
        if (data.sposY < -(MAX_BORDER_Y_DEFAULT-0.18f) || autoLandThrottleDecrease >1000)
            navigation_status = navigation_status_landed;

        autoLandThrottleDecrease += params.autoLandThrottleDecreaseFactor;
        _dctrl->setAutoLandThrottleDecrease(autoLandThrottleDecrease);

        if ( setpoint_world.y - land_incr> -(_dtrk->drone_max_border_y+100000.0f))
            land_incr = static_cast<float>(params.land_incr_f_mm)/1000.f;
        setpoint_world.y -= land_incr;
        if (_dctrl->get_flight_mode() == DroneController::fm_manual)
            navigation_status=navigation_status_manual;
        break;
    } case navigation_status_landed: {
        alert("canberra-gtk-play -f /usr/share/sounds/ubuntu/notifications/Slick.ogg &");
        wpid = 0;
        autoLandThrottleDecrease = 0;
        _dctrl->setAutoLandThrottleDecrease(0);
        _dctrl->set_flight_mode(DroneController::fm_inactive);
        _dtrk->drone_max_border_y = MAX_BORDER_Y_DEFAULT;
        land_incr = 0;
        navigation_status = navigation_status_wait_for_insect;
    } FALLTHROUGH_INTENDED; case navigation_status_manual: {
        wpid = 0;
        if (_dctrl->get_flight_mode() == DroneController::fm_inactive) {
            navigation_status=navigation_status_wait_for_insect;
            alert("canberra-gtk-play -f /usr/share/sounds/ubuntu/notifications/Rhodes.ogg &");
        }
        break;
    } case navigation_status_drone_problem: {
        break;
    }
    }
}

void DroneNavigation::close() {
    std::ofstream os(paramsFile, std::ios::binary);
    cereal::BinaryOutputArchive archive( os );
    archive( params );
}
