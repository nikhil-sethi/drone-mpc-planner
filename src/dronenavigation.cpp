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
            std::cout << "Drone navigation settings file error: " << e.what() << std::endl;
            std::cout << "Maybe delete the file: " << paramsFile << std::endl;
            exit (1);
        }
        navigationParameters tmp;
        if (tmp.version-params.version > 0.001f){
            std::cout << "Drone navigation settings version too low!" << std::endl;
            std::cout << "Maybe delete the file: " << paramsFile << std::endl;
            throw my_exit(1);
        }
    }

    //Waypoints are relative to the camera position. The camera is 0,0,0.
    //X goes from negative left, to positive right.
    //Everything below the camera is negative Y, heigher than the camera is positive
    //Farther away from the camera is negative Z, positive Z should be impossible because the camera can't see that.

    //The flight plan will be repeated indefinetely, unless there is a landing waypoint somewhere in the list.
    setpoints.push_back(waypoint(cv::Point3f(0,-1.0f,-1.5f),30));
    setpoints.push_back(landing_waypoint());

#ifdef TUNING
    namedWindow("Nav", WINDOW_NORMAL);
    //    createTrackbar("X [mm]", "Nav", &params.setpoint_slider_X, SETPOINTXMAX);
    //    createTrackbar("Y [mm]", "Nav", &params.setpoint_slider_Y, SETPOINTYMAX);
    //    createTrackbar("Z [mm]", "Nav", &params.setpoint_slider_Z, SETPOINTZMAX);
    createTrackbar("WP id", "Nav", reinterpret_cast<int*>(wpid), setpoints.size()-1);
    createTrackbar("d threshold factor", "Nav", &params.distance_threshold_f, 10);
    createTrackbar("land_incr_f_mm", "Nav", &params.land_incr_f_mm, 50);
    createTrackbar("Land Decrease  ", "Nav", &params.autoLandThrottleDecreaseFactor, 50);
    createTrackbar("Take off speed threshold  ", "Nav", &params.auto_takeoff_speed, 50);

#endif

#ifdef INSECT_LOGGING_MODE
    _navigation_status = ns_wait_for_insect;
#endif

    return false;
}

void DroneNavigation::update(float time) {

    if (_dctrl->Joy_State() != DroneController::js_none) {
        //copy the (processed) joy mode switch, but only if the switch is available
        //(otherwise assume _nav_flight_mode is set in another way externally)
        if (_dctrl->Joy_State() != DroneController::js_disarmed)
            _nav_flight_mode = static_cast<nav_flight_modes>(_dctrl->Joy_State());
        else { // if disarmed use the manual state:
            _nav_flight_mode = nfm_manual;
        }

    }
    _iceptor.update(_navigation_status == ns_wait_for_insect);

    switch (_navigation_status) {
    case ns_init: {
        _navigation_status = ns_locate_drone;
        break;
    } case ns_locate_drone: {
#if TX_TYPE == TX_FRSKYD || TX_TYPE == TX_FRSKYX
        _dctrl->blink_drone(true);
#endif
        _dtrk->Locate_Startup_Location();
        _navigation_status = ns_wait_locate_drone;
        break;
    } case ns_wait_locate_drone: {
        static float prev_time = time;
#if TX_TYPE == TX_FRSKYD || TX_TYPE == TX_FRSKYX
        if (time - prev_time > 7 && time - prev_time < 8) {
            _dctrl->blink_drone(false); // refresh the blinking
        } else if (time - prev_time > 8) {
            prev_time = time;
            _dctrl->blink_drone(true);
        }
#endif
        if (_dtrk->blinking_drone_located()) {
            _navigation_status = ns_located_drone;
            time_located_drone = time;
        }
#if TX_TYPE == TX_FRSKYX_TC
        if (time - prev_time > _dtrk->bind_blink_time)
            _dctrl->blink_drone(true,time);
#endif
        break;
    } case ns_located_drone: {
#if TX_TYPE == TX_FRSKYD || TX_TYPE == TX_FRSKYX
        _dctrl->blink_drone(false);
#endif
#if TX_TYPE == TX_FRSKYX_TC
        _dctrl->blink_drone(false,time);
#endif
        if (time-time_located_drone>1.0f) { // delay until blinking stopped
            if (_nav_flight_mode == nfm_hunt)
                _navigation_status = ns_wait_for_insect;
            else if (_nav_flight_mode == nfm_manual)
                _navigation_status = ns_manual;
            else
                _navigation_status = ns_wait_for_takeoff_command;
        }
        break;
    } case ns_wait_for_takeoff_command: {
        _dctrl->set_flight_mode(DroneController::fm_inactive);
        if (_nav_flight_mode == nfm_hunt)
            _navigation_status = ns_wait_for_insect;
        else if (_nav_flight_mode == nfm_manual)
            _navigation_status = ns_manual;
        else if (_dctrl->manual_override_take_off_now() )
            _navigation_status = ns_takeoff;
        break;
    } case ns_wait_for_insect: {
        _dctrl->set_flight_mode(DroneController::fm_inactive);
        if (_nav_flight_mode == nfm_manual)
            _navigation_status = ns_manual;
        else if (_nav_flight_mode == nfm_hunt) {
            if (!_dctrl->hoverthrottleInitialized) {
                _navigation_status = ns_init_calibrate_hover;
            } else if (_iceptor.get_insect_in_range()) {
                _navigation_status = ns_takeoff;
            }
        } else //waypoint modes
            _navigation_status = ns_wait_for_takeoff_command;
        break;
    } case ns_init_calibrate_hover: {
        _navigation_status = ns_takeoff;
        _calibrating_hover = true;
        break;
    } case ns_takeoff: {
        _dctrl->reset_manual_override_take_off_now();
        _dctrl->set_flight_mode(DroneController::fm_taking_off);
        _navigation_status=ns_taking_off;
        break;
    } case ns_taking_off: {
        track_data data = _dtrk->Last_track_data();
        if (data.svelY > static_cast<float>(params.auto_takeoff_speed) / 100.f ) {
            _navigation_status = ns_take_off_completed;
        }
        if (_nav_flight_mode == nfm_manual)
            _navigation_status = ns_manual;
        break;
    } case ns_take_off_completed: {
        _dctrl->init_ground_effect_compensation();
        alert("canberra-gtk-play -f /usr/share/sounds/ubuntu/notifications/Slick.ogg &");
        _dctrl->set_flight_mode(DroneController::fm_flying);
#ifdef MANUAL_DRONE_LOCATE
        _dtrk->do_post_takeoff_detection();
#endif
        if (_calibrating_hover)
            _navigation_status = ns_calibrate_hover;
        else if (_nav_flight_mode == nfm_hunt)
            _navigation_status = ns_start_the_chase;
        else if (_nav_flight_mode == nfm_manual)
            _navigation_status = ns_manual;
        else
            _navigation_status = ns_set_waypoint;
        break;
    } case ns_calibrate_hover: {
        set_next_waypoint(hovercalib_waypoint());
        _navigation_status = ns_approach_waypoint;
        if (_nav_flight_mode == nfm_manual)
            _navigation_status = ns_manual;
        break;
    } case ns_start_the_chase: {
        _iceptor.reset_insect_cleared();
        _navigation_status = ns_chasing_insect;


        if (_nav_flight_mode == nfm_manual)
            _navigation_status = ns_manual;
    } FALLTHROUGH_INTENDED; case ns_chasing_insect: {

        //update target chasing waypoint and speed
        if (_iceptor.get_insect_in_range()) {
            setpoint_world = _iceptor.get_intercept_position();
            setspeed_world = _iceptor.get_target_speed();
        }

        if (setpoint_world.z == 0) { // fly to landing waypoint (but do not land)
            //FIXME, use set_new_waypoint
            setpoint_world = _iceptor.get_prev_intercept_position();

            setspeed_world.x = 0;
            setspeed_world.y = 0;
            setspeed_world.z = 0;
        }

        if (_nav_flight_mode == nfm_manual)
            _navigation_status=ns_manual;
        else if (_iceptor.get_insect_cleared())
            _navigation_status = ns_goto_landing_waypoint;

        // TODO: return to landing waypoint after the insect was lost for several frames

        break;
    } case ns_goto_landing_waypoint: {
        set_next_waypoint(landing_waypoint());
        _navigation_status = ns_approach_waypoint;
        break;
    } FALLTHROUGH_INTENDED; case ns_set_waypoint: {
        set_next_waypoint(setpoints[wpid]);
        _navigation_status = ns_approach_waypoint;
        break;
    } case ns_approach_waypoint: {
#ifdef MANUAL_DRONE_LOCATE

        if (current_setpoint->mode == fm_landing){
            set_next_waypoint(landing_waypoint()); // re-update the location, important when manually setting take off location
        } else if (current_setpoint->mode == fm_hover_calib){
            set_next_waypoint(hovercalib_waypoint()); // re-update the location, important when manually setting take off location
        } else if (current_setpoint->mode == fm_takeoff){
            set_next_waypoint(takeoff_waypoint()); // re-update the location, important when manually setting take off location
        }

#endif

        float dis = sqrtf(_dctrl->posErrX*_dctrl->posErrX + _dctrl->posErrY*_dctrl->posErrY + _dctrl->posErrZ*_dctrl->posErrZ);
        if (dis *1000 < current_setpoint->threshold_mm * params.distance_threshold_f && _dtrk->n_frames_tracking>5) {
            if (current_setpoint->mode == fm_landing) {
                _navigation_status = ns_landing;
            } else if (current_setpoint->mode == fm_hover_calib) {
                _dctrl->recalibrateHover();
                _calibrating_hover = false;
                _navigation_status = ns_goto_landing_waypoint;
            } else if (wpid < setpoints.size()) { // next waypoint in flight plan
                wpid++;
                alert("canberra-gtk-play -f /usr/share/sounds/ubuntu/stereo/window-slide.ogg &");
                if (wpid == 1)
                    _dctrl->recalibrateHover();
                _navigation_status = ns_set_waypoint;
            } else if (wpid == setpoints.size()){
                wpid = 0; // another round
                alert("canberra-gtk-play -f /usr/share/sounds/ubuntu/stereo/window-slide.ogg &");
                _navigation_status = ns_set_waypoint;
            }
        }
        if (_nav_flight_mode == nfm_manual)
            _navigation_status=ns_manual;
        break;
    } case ns_stay_waypoint: {
        //TODO: implement
        if (_nav_flight_mode == nfm_manual)
            _navigation_status=ns_manual;
        break;
        //    } case navigation_status_stay_slider_waypoint: {
        //        wp = &setpoints[wpid];
        //        wp->_xyz.x = params.setpoint_slider_X;
        //        wp->_xyz.y = params.setpoint_slider_Y;
        //        wp->_xyz.z = params.setpoint_slider_Z;
        //        if (_dctrl->get_flight_mode() == DroneController::fm_manual)
        //            navigation_status=navigation_status_manual;
        //        break;
    } case ns_land: {
        _dctrl->set_flight_mode(DroneController::fm_landing);
        _dctrl->recalibrateHover();
        alert("canberra-gtk-play -f /usr/share/sounds/ubuntu/notifications/Slick.ogg &");
        _navigation_status = ns_landing;
    } FALLTHROUGH_INTENDED; case ns_landing: {
        track_data data = _dtrk->Last_track_data();
        if (data.sposY < _dtrk->Drone_Startup_Location().y+0.1f || autoLandThrottleDecrease >1000)
            _navigation_status = ns_landed;

        autoLandThrottleDecrease += params.autoLandThrottleDecreaseFactor;
        _dctrl->setAutoLandThrottleDecrease(autoLandThrottleDecrease);

        if ( setpoint_world.y - land_incr> -(_dtrk->Drone_Startup_Location().y+100000.0f))
            land_incr = static_cast<float>(params.land_incr_f_mm)/1000.f;
        setpoint_world.y -= land_incr;
        if (_nav_flight_mode == nfm_manual)
            _navigation_status=ns_manual;
        break;
    } case ns_landed: {
        alert("canberra-gtk-play -f /usr/share/sounds/ubuntu/notifications/Slick.ogg &");
        wpid = 0;
        autoLandThrottleDecrease = 0;
        _dctrl->setAutoLandThrottleDecrease(0);
        _dctrl->set_flight_mode(DroneController::fm_inactive);
        land_incr = 0;
        _navigation_status = ns_wait_after_landing;
        landed_time = time;
    } FALLTHROUGH_INTENDED; case ns_wait_after_landing: {
        _visdat->delete_from_motion_map(_dtrk->Drone_Startup_Im_Location()*IMSCALEF,DRONE_IM_START_SIZE);
        if (time - landed_time > params.time_out_after_landing )
            _navigation_status = ns_locate_drone;
        break;
    } case ns_manual: { // also used for disarmed
        wpid = 0;
        if (_nav_flight_mode == nfm_hunt) {
            _navigation_status=ns_wait_for_insect;
            alert("canberra-gtk-play -f /usr/share/sounds/ubuntu/notifications/Rhodes.ogg &");
        } else if (_nav_flight_mode != nfm_manual) { // waypoint mode
            _navigation_status=ns_wait_for_takeoff_command;
        }
        break;
    } case ns_drone_problem: {
        break;
    }
    }
}

void DroneNavigation::set_next_waypoint(waypoint wp) {
    current_setpoint = new waypoint(wp);
    if (wp.mode == fm_landing || wp.mode == fm_hover_calib || wp.mode == fm_takeoff) {
        cv::Point3f p = _dtrk->Drone_Startup_Location();
        setpoint_world =  p + wp.xyz;
    } else {
        setpoint_world =  wp.xyz;
    }

    setspeed_world.x = 0;
    setspeed_world.y = 0;
    setspeed_world.z = 0;

}

void DroneNavigation::close() {
    std::ofstream os(paramsFile, std::ios::binary);
    cereal::BinaryOutputArchive archive( os );
    archive( params );
}
