#include <iostream>
#include "dronenavigation.h"

using namespace cv;
using namespace std;

#ifdef HASSCREEN
//#define TUNING
#endif

const string paramsFile = "../navigationParameters.dat";

void DroneNavigation::init(std::ofstream *logger, ItemManager * trackers, DroneController * dctrl, VisionData *visdat) {
    _logger = logger;
    _trackers = trackers;
    _dctrl = dctrl;
    _visdat = visdat;

    _iceptor.init(_trackers,visdat);
    cv::invert(visdat->Qf,Qfi);

    // Load saved control paremeters
    if (checkFileExist(paramsFile)) {
        std::ifstream is(paramsFile, std::ios::binary);
        cereal::BinaryInputArchive archive( is );
        try {
            archive(params);
        }catch (cereal::Exception e) {
            std::stringstream serr;
            serr << "cannot read drone navigation settings file: " << e.what() << ". Maybe delete the file: " << paramsFile;
            throw my_exit(serr.str());
        }
        navigationParameters tmp;
        if (tmp.version-params.version > 0.001f){
            std::stringstream serr;
            serr << "dronenavigation settings version too low! Maybe delete the file: " << paramsFile;
            throw my_exit(serr.str());
        }
    }

    //Waypoints are relative to the camera position. The camera is 0,0,0.
    //X goes from negative left, to positive right.
    //Everything below the camera is negative Y, heigher than the camera is positive
    //Farther away from the camera is negative Z, positive Z should be impossible because the camera can't see that.

    //The flight plan will be repeated indefinetely, unless there is a landing waypoint somewhere in the list.

    //    setpoints.push_back(waypoint(cv::Point3f(-2,-1.0f,-3.5f),100));
    //    setpoints.push_back(waypoint(cv::Point3f(2,-1.0f,-3.5f),100));
    //    setpoints.push_back(waypoint(cv::Point3f(-2,-1.0f,-3.5f),100));
    //    setpoints.push_back(waypoint(cv::Point3f(2,-1.0f,-3.5f),100));


    setpoints.push_back(waypoint(cv::Point3f(0,-1.3f,-1.5f),30));

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

    initialized = true;
}

void DroneNavigation::update(double time) {

    if (_dctrl->Joy_State() != DroneController::js_none) {
        if (_dctrl->Joy_State() == DroneController::js_checking ||
                _dctrl->Joy_State() == DroneController::js_none ||
                _dctrl->Joy_State() == DroneController::js_disarmed )
            _nav_flight_mode = nfm_none;
        else if(_dctrl->Joy_State() == DroneController::js_hunt)
            _nav_flight_mode = nfm_hunt;
        else if(_dctrl->Joy_State() == DroneController::js_waypoint)
            _nav_flight_mode = nfm_waypoint;
        else if(_dctrl->Joy_State() == DroneController::js_manual)
            _nav_flight_mode = nfm_manual;
        else if(_dctrl->Joy_State() == DroneController::js_slider)
            _nav_flight_mode = nfm_slider;
    } else {
        _nav_flight_mode = nfm_none;
    }
    _iceptor.update(_navigation_status != ns_chasing_insect);
    bool repeat = true;
    while (repeat) {
        repeat  = false;
        switch (_navigation_status) {
        case ns_init: {
#if DRONE_TYPE == DRONE_TRASHCAN
            _dctrl->LED(true);
#endif
            if (time > 1) // skip the first second or so, e.g. auto exposure may give heavily changing images
                _navigation_status = ns_locate_drone;
            _trackers->mode(ItemManager::mode_idle);
            break;
        } case ns_locate_drone: {
#if DRONE_TYPE != DRONE_TRASHCAN
            _dctrl->blink_by_binding(true);
#endif
            _trackers->mode(ItemManager::mode_locate_drone);
            _visdat->reset_motion_integration();
            _visdat->disable_fading = true;
            _navigation_status = ns_wait_locate_drone;
            break;
        } case ns_wait_locate_drone: {
            static double __attribute__((unused)) prev_time = time;
#if DRONE_TYPE != DRONE_TRASHCAN
            if (time - prev_time > 7 && time - prev_time < 8) {
                _dctrl->blink_by_binding(false); // refresh the blinking
            } else if (abs(time - prev_time) > 8.5) {
                prev_time = time;
                _dctrl->blink_by_binding(true);
            }
#else
            if (time - prev_time > bind_blink_time)
                _dctrl->blink(time);
#endif
            if (_trackers->mode() != ItemManager::mode_locate_drone) {
                _navigation_status = ns_located_drone;
                time_located_drone = time;
            }

            break;
        } case ns_located_drone: {
#if DRONE_TYPE != DRONE_TRASHCAN
            _dctrl->blink_by_binding(false);
#else
            _dctrl->LED(true);
#endif
            _trackers->mode(ItemManager::mode_idle);
            _visdat->disable_fading = false;
            if (time-time_located_drone>1.0) { // delay until blinking stopped
                if (_nav_flight_mode == nfm_hunt)
                    _navigation_status = ns_wait_for_insect;
                else if (_nav_flight_mode == nfm_manual)
                    _navigation_status = ns_manual;
                else
                    _navigation_status = ns_wait_for_takeoff_command;
            }
            break;
        } case ns_wait_for_takeoff_command: {
            _trackers->mode(ItemManager::mode_idle);
            _dctrl->flight_mode(DroneController::fm_inactive);
            if (_nav_flight_mode == nfm_hunt)
                _navigation_status = ns_wait_for_insect;
            else if (_nav_flight_mode == nfm_manual)
                _navigation_status = ns_manual;
            else if (_dctrl->manual_override_take_off_now() ){
                _navigation_status = ns_takeoff;
                repeat = true;
            }
            break;
        } case ns_wait_for_insect: {
            _trackers->mode(ItemManager::mode_wait_for_insect);
            _dctrl->flight_mode(DroneController::fm_inactive);
            if (_nav_flight_mode == nfm_manual)
                _navigation_status = ns_manual;
            else if (_nav_flight_mode == nfm_hunt && _iceptor.insect_in_range()) {
                _navigation_status = ns_takeoff;
                repeat = true;
            } else if (_nav_flight_mode == nfm_none) { // e.g. insect logging mode
            } else if (_nav_flight_mode == nfm_waypoint || _nav_flight_mode == nfm_slider)
                _navigation_status = ns_wait_for_takeoff_command;
            break;
        } case ns_init_calibrate_hover: {
            _navigation_status = ns_takeoff;
            _calibrating_hover = true;
            break;
        } case ns_takeoff: {
            _dctrl->reset_manual_override_take_off_now();
            _dctrl->flight_mode(DroneController::fm_start_takeoff);
            time_taken_off = time;
            if (_nav_flight_mode == nfm_hunt)
                _trackers->mode(ItemManager::mode_hunt);
            else
                _trackers->mode(ItemManager::mode_drone_only);
            _navigation_status=ns_taking_off;
            break;
        } case ns_taking_off: {
            if (_nav_flight_mode == nfm_manual){
                _navigation_status = ns_manual;
                break;
            }
            if (time - time_taken_off > 0.7){
                std::cout << "Drone was not detected during max burn take off manoeuvre, aborting." << std::endl;
                _dctrl->flight_mode(DroneController::fm_abort_takeoff);
                _navigation_status = ns_drone_problem;
                break;
            }

            if (!_trackers->dronetracker()->taking_off())
                _navigation_status = ns_take_off_completed;
            else
                break;
        } FALLTHROUGH_INTENDED; case ns_take_off_completed: {
            _dctrl->init_ground_effect_compensation();
            _dctrl->flight_mode(DroneController::fm_flying);
            _dctrl->hoverthrottle = _trackers->dronetracker()->hover_throttle_estimation;
#ifdef MANUAL_DRONE_LOCATE
            _dtrk->do_post_takeoff_detection();
#endif
            if (_calibrating_hover) {
                _navigation_status = ns_calibrate_hover;
            } else if (_nav_flight_mode == nfm_hunt) {
                _navigation_status = ns_start_the_chase;
                repeat = true;
            } else if (_nav_flight_mode == nfm_manual) {
                _navigation_status = ns_manual;
            } else {
                _navigation_status = ns_set_waypoint;
            }
            break;
        } case ns_calibrate_hover: {
            next_waypoint(hovercalib_waypoint());
            _navigation_status = ns_approach_waypoint;
            if (_nav_flight_mode == nfm_manual)
                _navigation_status = ns_manual;
            break;
        } case ns_start_the_chase: {
            _iceptor.reset_insect_cleared();
            _navigation_status = ns_chasing_insect;


            if (_nav_flight_mode == nfm_manual) {
                _navigation_status = ns_manual;
                break;
            }
        } FALLTHROUGH_INTENDED; case ns_chasing_insect: {

            //update target chasing waypoint and speed
            if (_iceptor.insect_in_range()) {
                setpoint_pos_world = _iceptor.target_position();
                setpoint_vel_world = _iceptor.target_speed();
                setpoint_acc_world = _iceptor.target_accelleration();
            }

            if (_nav_flight_mode == nfm_manual)
                _navigation_status=ns_manual;
            else if (_iceptor.insect_cleared())
                _navigation_status = ns_goto_landing_waypoint;
            break;
        } case ns_goto_landing_waypoint: {
            next_waypoint(landing_waypoint());
            _navigation_status = ns_approach_waypoint;
            break;
        } FALLTHROUGH_INTENDED; case ns_set_waypoint: {
            next_waypoint(setpoints[wpid]);
            _navigation_status = ns_approach_waypoint;
            break;
        } case ns_approach_waypoint: {
#ifdef MANUAL_DRONE_LOCATE

            if (current_setpoint->mode == fm_landing){
                next_waypoint(landing_waypoint()); // re-update the location, important when manually setting take off location
            } else if (current_setpoint->mode == fm_hover_calib){
                next_waypoint(hovercalib_waypoint()); // re-update the location, important when manually setting take off location
            } else if (current_setpoint->mode == fm_takeoff){
                next_waypoint(takeoff_waypoint()); // re-update the location, important when manually setting take off location
            }

#endif

            float dis = sqrtf(_dctrl->posErrX*_dctrl->posErrX + _dctrl->posErrY*_dctrl->posErrY + _dctrl->posErrZ*_dctrl->posErrZ);
            _dist_to_wp = dis;
            if (dis *1000 < current_setpoint->threshold_mm * params.distance_threshold_f && _trackers->dronetracker()->n_frames_tracking>5) {
                if (current_setpoint->mode == fm_landing) {
                    _navigation_status = ns_land;
                } else if (current_setpoint->mode == fm_hover_calib) {
                    _dctrl->recalibrateHover();
                    _calibrating_hover = false;
                    _navigation_status = ns_goto_landing_waypoint;
                } else if (wpid < setpoints.size()) { // next waypoint in flight plan
                    wpid++;
                    if (wpid == 1)
                        _dctrl->recalibrateHover();
                    _navigation_status = ns_set_waypoint;
                } else if (wpid == setpoints.size()){
                    wpid = 0; // another round
                    _navigation_status = ns_set_waypoint;
                }
            }

            if (_nav_flight_mode == nfm_hunt && _iceptor.insect_in_range())
                _navigation_status = ns_start_the_chase;
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
            _dctrl->flight_mode(DroneController::fm_landing);
            _dctrl->recalibrateHover();
            _trackers->dronetracker()->land();
            _navigation_status = ns_landing;
        } FALLTHROUGH_INTENDED; case ns_landing: {
            track_data data = _trackers->dronetracker()->Last_track_data();
            if (data.sposY < _trackers->dronetracker()->drone_landing_location().y+0.1f || autoLandThrottleDecrease >1000)
                _navigation_status = ns_landed;

            autoLandThrottleDecrease += params.autoLandThrottleDecreaseFactor;
            _dctrl->setAutoLandThrottleDecrease(autoLandThrottleDecrease);

            if ( setpoint_pos_world.y - land_incr> -(_trackers->dronetracker()->drone_landing_location().y+100000.0f))
                land_incr = static_cast<float>(params.land_incr_f_mm)/1000.f;
            setpoint_pos_world.y -= land_incr;
            if (_nav_flight_mode == nfm_hunt && _iceptor.insect_in_range())
                _navigation_status = ns_start_the_chase;
            if (_nav_flight_mode == nfm_manual)
                _navigation_status=ns_manual;
            break;
        } case ns_landed: {
            wpid = 0;
            autoLandThrottleDecrease = 0;
            _dctrl->setAutoLandThrottleDecrease(0);
            _dctrl->flight_mode(DroneController::fm_inactive);
            land_incr = 0;
            _navigation_status = ns_wait_after_landing;
            landed_time = time;
        } FALLTHROUGH_INTENDED; case ns_wait_after_landing: {
            _visdat->delete_from_motion_map(_trackers->dronetracker()->drone_startup_im_location()*IMSCALEF,_trackers->dronetracker()->drone_startup_im_disparity(),_trackers->dronetracker()->drone_startup_im_size()*IMSCALEF,1);
            if (time - landed_time > params.time_out_after_landing )
                _navigation_status = ns_locate_drone;
            break;
        } case ns_manual: { // also used for disarmed
            wpid = 0;
            _trackers->mode(ItemManager::mode_drone_only);
            if (_nav_flight_mode == nfm_hunt) {
                _navigation_status=ns_wait_for_insect;
            } else if (_nav_flight_mode != nfm_manual) { // waypoint mode
                _navigation_status=ns_wait_for_takeoff_command;
            }
            break;
        } case ns_drone_problem: {
            break;
        }
        }
    }
}

void DroneNavigation::next_waypoint(waypoint wp) {
    current_setpoint = new waypoint(wp);
    if ( wp.mode == fm_hover_calib || wp.mode == fm_takeoff) {
        cv::Point3f p = _trackers->dronetracker()->drone_startup_location();
        setpoint_pos_world =  p + wp.xyz;
    }else if (wp.mode == fm_landing ) {
        cv::Point3f p = _trackers->dronetracker()->drone_landing_location();
        setpoint_pos_world =  p + wp.xyz;
    } else {
        setpoint_pos_world =  wp.xyz;
    }

    setpoint_vel_world = {0,0,0};
    setpoint_acc_world = {0,0,0};


}

void DroneNavigation::close() {
    if (initialized) {
        std::cout << "Closing drone navigation" << std::endl;
        std::ofstream os(paramsFile, std::ios::binary);
        cereal::BinaryOutputArchive archive( os );
        archive( params );
        initialized = false;
    }
}
