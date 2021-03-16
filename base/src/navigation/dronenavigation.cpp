#include <iostream>
#include "dronenavigation.h"
#include <experimental/filesystem>

using namespace cv;
using namespace std;

namespace navigation {


void DroneNavigation::init(std::ofstream *logger, tracking::TrackerManager * trackers, DroneController * dctrl, VisionData *visdat, CameraView *camview,std::string replay_dir, Interceptor *iceptor) {
    _logger = logger;
    _trackers = trackers;
    _dctrl = dctrl;
    _visdat = visdat;
    _camview = camview;
    _iceptor = iceptor;

    deserialize_settings();
    deserialize_flightplan(replay_dir);

    if (pparams.navigation_tuning) {
        namedWindow("Nav", WINDOW_NORMAL);
        createTrackbar("X [cm", "Nav", &setpoint_slider_X, 500);
        createTrackbar("Y off", "Nav", &setpoint_slider_Y, 500);
        createTrackbar("Z center]", "Nav", &setpoint_slider_Z, 500);

        createTrackbar("v_crcl1", "Nav", &v_crcl1, 1000);
        createTrackbar("v_crcl2", "Nav", &v_crcl2, 1000);
        createTrackbar("r_crcl1", "Nav", &r_crcl1, 1000);
        createTrackbar("r_crcl2", "Nav", &r_crcl2, 1000);

        createTrackbar ("w_sqr", "Nav", &w_sqr, 2500);
        createTrackbar ("v_sqr", "Nav", &v_sqr, 500);
    }

    (*_logger) << "nav_state;nav_state_str;";

    initialized = true;
}

void DroneNavigation::deserialize_flightplan(std::string replay_dir) {
    navigation::XML_FlightPlan fp;
    if (replay_dir == "") {
        fp.deserialize(pparams.flightplan);
        if (file_exist("./logging/flightplan.xml")) {
            std::string rmcmd = "rm ./logging/flightplan.xml";
            auto res [[maybe_unused]] = std::system(rmcmd.c_str());
        }
        std::experimental::filesystem::copy(pparams.flightplan, "./logging/flightplan.xml");
    } else {
        fp.deserialize(replay_dir + "/flightplan.xml");
    }
    waypoints = fp.waypoints();
}

void DroneNavigation::update(double time) {

    if (_dctrl->Joy_State() != DroneController::js_none) {
        if (_dctrl->Joy_State() == DroneController::js_checking ||
                _dctrl->Joy_State() == DroneController::js_none ||
                _dctrl->Joy_State() == DroneController::js_disarmed )
            _nav_flight_mode = nfm_manual;
        else if(_dctrl->Joy_State() == DroneController::js_hunt)
            _nav_flight_mode = nfm_hunt;
        else if(_dctrl->Joy_State() == DroneController::js_waypoint)
            _nav_flight_mode = nfm_waypoint;
        else if(_dctrl->Joy_State() == DroneController::js_manual)
            _nav_flight_mode = nfm_manual;
    } else if (pparams.op_mode == op_mode_monitoring ) {
        _nav_flight_mode = nfm_none;
    } else if (pparams.op_mode == op_mode_waypoint && _navigation_status == ns_init ) {
        _nav_flight_mode = nfm_waypoint;
    } else if (_navigation_status == ns_init) {
        _nav_flight_mode = nfm_hunt;
    }

    bool repeat = true;
    while (repeat) {
        repeat  = false;
        switch (_navigation_status) {
        case ns_init: {
            _dctrl->LED(true);
            locate_drone_attempts = 0;
            _visdat->reset_motion_integration();
            _trackers->mode(tracking::TrackerManager::mode_idle);
            if (time > 1.5) { // skip first second or so due to auto exposure
                if (pparams.op_mode==op_mode_monitoring) {
                    _dctrl->flight_mode(DroneController::fm_disarmed);
                    _dctrl->LED(false);
                    _navigation_status = ns_start_calibrating_motion;
                } else
                    _navigation_status = ns_locate_drone_init;
            }
            break;
        } case ns_locate_drone_init: {
            _dctrl->LED(true);
            _dctrl->flight_mode(DroneController::fm_disarmed);
            locate_drone_start_time = time;
            locate_drone_attempts++;
            if (dparams.led_type == led_none && (! _dctrl->takeoff_calib_valid() || force_pad_redetect)) {
                std::cout << "Error: The drone has no led, and no valid drone calibration (with takeoff location) was found..." << std::endl;
                _navigation_status = ns_unable_to_locate;
            } else if (_dctrl->takeoff_calib_valid() && !force_pad_redetect)
                _navigation_status = ns_start_calibrating_motion;
            else
                _navigation_status = ns_locate_drone_wait_led_on;
            force_pad_redetect = false;
            [[fallthrough]];
        } case ns_locate_drone_wait_led_on: {
            if (time - locate_drone_start_time > 1.5) {
                _visdat->reset_motion_integration();
                _visdat->disable_fading = true;
                _dctrl->flight_mode(DroneController::fm_blink);
                _trackers->mode(tracking::TrackerManager::mode_locate_drone);
                _navigation_status = ns_wait_locate_drone;
                last_led_doubler_time = time;
            }
            break;
        } case ns_wait_locate_drone: {
            static double prev_time = time;
            if (static_cast<float>(time - prev_time) > dparams.blink_period)
                _dctrl->flight_mode(DroneController::fm_blink);
            if (_trackers->mode() != tracking::TrackerManager::mode_locate_drone) {
                _dctrl->flight_mode(DroneController::fm_disarmed);
                time_located_drone = time;
                locate_drone_attempts = 0;
                _navigation_status = ns_located_drone;
            }
            if (time - last_led_doubler_time > 3 ) { // if the blink detect takes too long, it may be that the led is not bright enough to be detected
                last_led_doubler_time = time;
                // _dctrl->double_led_strength();
            }
            if (time - locate_drone_start_time > 15 )
                _navigation_status = ns_locate_drone_init;
            if (time - locate_drone_start_time > 45 && pparams.op_mode != op_mode_waypoint)
                _navigation_status = ns_drone_problem;
            break;
        } case ns_located_drone: {
            _dctrl->LED(true);
            _trackers->mode(tracking::TrackerManager::mode_idle);
            _visdat->disable_fading = false;
            _visdat->enable_collect_no_drone_frames = true;
            if (time-time_located_drone>5 && (_dctrl->state_disarmed() || pparams.joystick != rc_none)) { // delay until blinking stopped. Drone must be 1.5s disarmed to get out of a possible (rx) failsafe
                _navigation_status = ns_calibrating_pad;
                _n_drone_detects++;
            }
            break;
        } case ns_calibrating_pad: {
            if (_dctrl->pad_calibration_done())
                _navigation_status = ns_start_calibrating_motion;
            break;
        } case ns_start_calibrating_motion: {
            _visdat->enable_background_motion_map_calibration(motion_calibration_duration);
            _visdat->create_overexposed_removal_mask(_trackers->dronetracker()->pad_location(),_trackers->dronetracker()->pad_im_size());
            time_motion_calibration_started = time;
            if (pparams.op_mode==op_mode_monitoring) {
                _navigation_status = ns_calibrating_motion;
                _trackers->mode(tracking::TrackerManager::mode_wait_for_insect);
                _dctrl->stop_rc();
            } else
                _navigation_status = ns_check_telemetry;
            break;
        } case ns_check_telemetry: {
            if (_dctrl->telemetry_OK() || ! dparams.Telemetry())
                _navigation_status = ns_check_pad_att;
            break;
        } case ns_check_pad_att: {
            if (_dctrl->attitude_on_pad_OK() || ! dparams.Telemetry())
                _navigation_status = ns_wait_to_arm;
            break;
        } case ns_wait_to_arm: {
            if (_dctrl->ready_for_first_arm(time)) {
                _navigation_status = ns_calibrating_motion;
                _dctrl->flight_mode(DroneController::fm_inactive);
            }
            break;
        } case ns_calibrating_motion: {
            if (_dctrl->arming_problem()) {
                _dctrl->flight_mode(DroneController::fm_disarmed);
                time_motion_calibration_started = time;
                _navigation_status = ns_wait_to_arm;
            }
            if (static_cast<float>(time-time_motion_calibration_started) > motion_calibration_duration) {
                _navigation_status= ns_calibrating_motion_done;
            }
            break;
        } case ns_calibrating_motion_done: {
            _n_drone_readys++;
            if (pparams.op_mode==op_mode_monitoring)
                _navigation_status = ns_monitoring;
            else if (_nav_flight_mode == nfm_manual)
                _navigation_status = ns_manual;
            else if (_nav_flight_mode == nfm_waypoint)
                _navigation_status = ns_wait_for_takeoff_command;
            else if (_nav_flight_mode == nfm_hunt)
                _navigation_status = ns_wait_for_insect;
            else if (pparams.op_mode == op_mode_waypoint)
                _navigation_status = ns_wait_for_takeoff_command;
            else if (pparams.op_mode == op_mode_hunt )
                _navigation_status = ns_wait_for_insect;
            break; // don't fallthrough because we need an entry in the log
        } case ns_wait_for_takeoff_command: {
            int itime = time;
            _dctrl->LED(true,itime*5 % 45+5);
            _trackers->mode(tracking::TrackerManager::mode_idle);
            _dctrl->flight_mode(DroneController::fm_inactive);
            if (_nav_flight_mode == nfm_hunt) {
                _navigation_status = ns_wait_for_insect;
                _dctrl->LED(true);
            } else if (_nav_flight_mode == nfm_manual) {
                _navigation_status = ns_manual;
                _dctrl->LED(true);
            } else if ( _dctrl->manual_override_take_off_now()) {
                wpid = 0;
                next_waypoint(waypoints[wpid],time);
                _navigation_status = ns_takeoff;
                _dctrl->LED(true);
                repeat = true;
            }
            break;
        } case ns_monitoring: {

            break;
        } case ns_wait_for_insect: {
            if (_nav_flight_mode == nfm_manual) {
                _navigation_status = ns_manual;
            } else if (_nav_flight_mode == nfm_hunt) {

                if (_dctrl->telemetry().batt_cell_v > 2 && _dctrl->telemetry().batt_cell_v < dparams.min_hunt_cell_v) {
                    _navigation_status = ns_batlow;
                    break;
                } else if (_trackers->too_many_false_positives()) {
                    _navigation_status = ns_tracker_problem;
                    break;
                }

                _trackers->mode(tracking::TrackerManager::mode_wait_for_insect);

                auto itrkr = _iceptor->target_insecttracker();

                if (!itrkr) {
                    _dctrl->flight_mode(DroneController::fm_inactive);
                } else if(_iceptor->trigger_takeoff() && _visdat->no_recent_large_brightness_events(time)) {
                    _navigation_status = ns_takeoff;
                    repeat = true;
                } else if(itrkr->properly_tracking() && !itrkr->false_positive() && _visdat->no_recent_large_brightness_events(time)) {
                    _dctrl->flight_mode (DroneController::fm_spinup);
                } else {
                    _dctrl->flight_mode(DroneController::fm_inactive);
                }
            } else if (_nav_flight_mode == nfm_none) {
                _dctrl->flight_mode(DroneController::fm_inactive);
            } else if (_nav_flight_mode == nfm_waypoint)
                _navigation_status = ns_wait_for_takeoff_command;
            break;
        } case ns_takeoff: {
            _dctrl->reset_manual_override_take_off_now();
            _dctrl->flight_mode(DroneController::fm_start_takeoff);
            _visdat->enable_collect_no_drone_frames = false;
            _dctrl->hover_mode(false);
            _trackers->dronetracker()->hover_mode(false);
            time_take_off = time;
            if (_nav_flight_mode == nfm_hunt) {
                _n_hunt_flights++;
                _trackers->mode(tracking::TrackerManager::mode_hunt);
            } else {
                _n_wp_flights++;
                _trackers->mode(tracking::TrackerManager::mode_drone_only);
            }
            _navigation_status=ns_taking_off;
            break;
        } case ns_taking_off: {
            if (_nav_flight_mode == nfm_manual) {
                _navigation_status = ns_manual;
                break;
            }
            if (_trackers->dronetracker()->take_off_detection_failed()) {
                std::cout << "Drone was not detected during max burn take off manoeuvre, aborting." << std::endl;
                _dctrl->flight_mode(DroneController::fm_abort);
                _navigation_status = ns_drone_problem;
                break;
            }
            if (_dctrl->spinup()) {
                _navigation_status = ns_wait_for_insect;
                break;
            }

            if (_iceptor->trigger_takeoff() && _nav_flight_mode == nfm_hunt) {
                setpoint_pos_world = _iceptor->aim_pos();
                setpoint_vel_world = _iceptor->aim_vel();
                setpoint_acc_world = _iceptor->aim_acc();
            } else if (_nav_flight_mode == nfm_hunt && _dctrl->abort_take_off()) {
                _navigation_status = ns_wait_for_insect;
            }

            if (!_trackers->dronetracker()->taking_off() && time - time_take_off > dparams.max_burn_time)
                _navigation_status = ns_take_off_completed;
            else
                break;
            [[fallthrough]];
        } case ns_take_off_completed: {
            _n_take_offs++;
            if (_nav_flight_mode == nfm_hunt) {
                _navigation_status = ns_start_the_chase;
                repeat = true;
            } else {
                wpid++;
                _navigation_status = ns_set_waypoint;
            }
            check_abort_autonomus_flight_conditions();
            break;
        } case ns_start_the_chase: {
            _dctrl->hover_mode(false);
            _trackers->dronetracker()->hover_mode(false);
            _navigation_status = ns_chasing_insect_ff;
            [[fallthrough]];
        } case ns_chasing_insect_ff: {
            if (_dctrl->ff_completed())
                _navigation_status = ns_chasing_insect;
            check_abort_autonomus_flight_conditions();
            break;
        } case ns_chasing_insect: {
            setpoint_pos_world = _iceptor->aim_pos();
            setpoint_pos_world = _camview->setpoint_in_cameraview(setpoint_pos_world, _trackers->dronetracker()->last_track_data().pos(), CameraView::relaxed);
            setpoint_vel_world = _iceptor->aim_vel();
            setpoint_acc_world = _iceptor->aim_acc();

            if (_iceptor->target_cleared())
                _navigation_status = ns_goto_yaw_waypoint;
            check_abort_autonomus_flight_conditions();
            break;
        } case ns_goto_yaw_waypoint: {
            _dctrl->LED(true);
            _dctrl->hover_mode(true);
            _trackers->dronetracker()->hover_mode(true);
            next_waypoint(Waypoint_Yaw_Reset(),time);
            _navigation_status = ns_approach_waypoint;
            break;
        } case ns_goto_landing_waypoint: {
            _dctrl->hover_mode(true);
            _dctrl->enable_thrust_calibration = true;
            _trackers->dronetracker()->hover_mode(true);
            next_waypoint(Waypoint_Landing(),time);
            _navigation_status = ns_approach_waypoint;
            break;
        } case ns_set_waypoint: {
            next_waypoint(waypoints[wpid],time);
            if (current_waypoint->mode == wfm_flower)
                _navigation_status = ns_flower_waypoint;
            else if(current_waypoint->mode == wfm_brick)
                _navigation_status = ns_brick_waypoint;
            else
                _navigation_status = ns_approach_waypoint;

            if (current_waypoint->threshold_mm <30 && current_waypoint->threshold_mm > 0 ) {
                _dctrl->hover_mode(true);
                _trackers->dronetracker()->hover_mode(true);
            } else {
                _dctrl->hover_mode(false);
                _trackers->dronetracker()->hover_mode(false);
            }

            time_wp_reached = -1;
            break;
        } case ns_approach_waypoint: {
            if (pparams.navigation_tuning && current_waypoint->mode != wfm_landing && current_waypoint->mode != wfm_takeoff ) {
                setpoint_pos_world.x = waypoints[wpid].xyz.x + (250-setpoint_slider_X)/100.f;
                setpoint_pos_world.y = waypoints[wpid].xyz.y + (250-setpoint_slider_Y)/100.f;
                setpoint_pos_world.z = waypoints[wpid].xyz.z + (setpoint_slider_Z-250)/100.f;
                setpoint_pos_world = _camview->setpoint_in_cameraview(setpoint_pos_world, CameraView::relaxed);
            }

            if (_dctrl->dist_to_setpoint() *1000 < current_waypoint->threshold_mm
                    && normf(_trackers->dronetracker()->last_track_data().state.vel) < current_waypoint->threshold_v
                    && _trackers->dronetracker()->n_frames_tracking()>5)
            {
                if ((static_cast<float>(time - time_wp_reached) > current_waypoint->hover_pause && time_wp_reached > 0) || current_waypoint->hover_pause <= 0) {
                    if (current_waypoint->mode == wfm_landing)
                        _navigation_status = ns_land;
                    else if (current_waypoint->mode == wfm_yaw_reset) {
                        _navigation_status = ns_initial_reset_yaw;
                        time_initial_reset_yaw = time;
                    } else if (wpid < waypoints.size()) { // next waypoint in flight plan
                        wpid++;
                        _navigation_status = ns_set_waypoint;
                    } else if (wpid == waypoints.size()) {
                        wpid = 0; // another round
                        _navigation_status = ns_set_waypoint;
                    }
                } else if (time_wp_reached<0) {
                    time_wp_reached = time;
                }
            }

            if (_nav_flight_mode == nfm_hunt && _iceptor->aim_in_range() && !low_battery_triggered) {
                _dctrl->enable_thrust_calibration = false;
                _navigation_status = ns_start_the_chase;
            }

            check_abort_autonomus_flight_conditions();
            break;
        } case ns_flower_waypoint: {

            float timef = static_cast<float>(time);

            cv::Point3f new_pos_setpoint;
            cv::Point3f new_vel_setpoint;
            new_pos_setpoint.x = current_waypoint->xyz.x + (r_crcl1/100.f) * sinf((v_crcl1/100.f)*timef) + (r_crcl2/100.f) * cosf((v_crcl2/100.f)*timef);
            new_pos_setpoint.y = current_waypoint->xyz.y;
            new_pos_setpoint.z = current_waypoint->xyz.z + (r_crcl1/100.f) * cosf((v_crcl1/100.f)*timef) + (r_crcl2/100.f) * sinf((v_crcl2/100.f)*timef);
            new_vel_setpoint = (new_pos_setpoint - setpoint_pos_world)*static_cast<float>(pparams.fps);
            setpoint_acc_world = (new_vel_setpoint - setpoint_vel_world)*static_cast<float>(pparams.fps);
            setpoint_vel_world = new_vel_setpoint;
            setpoint_pos_world = new_pos_setpoint;

            check_abort_autonomus_flight_conditions();
            break;
        } case ns_brick_waypoint: {
            float timef = static_cast<float>(time);

            cv::Point3f new_pos_setpoint;
            cv::Point3f new_vel_setpoint;
            new_pos_setpoint = square_point (current_waypoint->xyz, static_cast<float>(w_sqr)/1000.f, static_cast<float>(v_sqr)/100.f*timef);
            std::cout << "brick_setpoint: " << new_pos_setpoint << std::endl;
            new_vel_setpoint = (new_pos_setpoint - setpoint_pos_world)*static_cast<float>(pparams.fps);
            setpoint_acc_world = (new_vel_setpoint - setpoint_vel_world)*static_cast<float>(pparams.fps);
            setpoint_vel_world = new_vel_setpoint;
            setpoint_pos_world = new_pos_setpoint;

            check_abort_autonomus_flight_conditions();
            break;
        } case ns_initial_reset_yaw: {
            _dctrl->flight_mode(DroneController::fm_initial_reset_yaw);
            if(time-time_initial_reset_yaw>2.0) {
                _trackers->dronetracker()->detect_yaw(time);
                _navigation_status = ns_wait_reset_yaw;
            }
            break;
        } case ns_wait_reset_yaw: {
            _dctrl->flight_mode(DroneController::fm_reset_yaw);
            bool drone_pos_ok = _dctrl->dist_to_setpoint() *1000 < current_waypoint->threshold_mm
                                && normf(_trackers->dronetracker()->last_track_data().state.vel) < current_waypoint->threshold_v
                                && _trackers->dronetracker()->n_frames_tracking()>5;

            if(!_trackers->dronetracker()->check_yaw(time) || (time-time_initial_reset_yaw > yaw_reset_duration && drone_pos_ok)) {
                _dctrl->flight_mode(DroneController::fm_flying_headed_pid);
                _navigation_status = ns_goto_landing_waypoint;
            }
            check_abort_autonomus_flight_conditions();
            break;
        } case ns_land: {
            landing_start_time = time;
            _trackers->dronetracker()->land();
            _navigation_status = ns_landing;
            [[fallthrough]];
        } case ns_landing: {
            float dt_land = static_cast<float>(time-landing_start_time);
            cv::Point3f pad_pos = _trackers->dronetracker()->pad_location(true);
            cv::Point3f new_pos_setpoint = _dctrl->land_ctrl.setpoint_cc_landing(pad_pos, current_waypoint, dt_land);
            _dctrl->enable_thrust_calibration = false;

            if(_dctrl->land_ctrl.switch_to_ff_landing(_trackers->dronetracker()->last_track_data(),
                    new_pos_setpoint, pad_pos, _dctrl->landing()))
                _dctrl->flight_mode(DroneController::fm_ff_landing_start);

            setpoint_pos_world = new_pos_setpoint;
            if (!_dctrl->active())
                _navigation_status = ns_landed;
            if (_nav_flight_mode == nfm_hunt && _iceptor->aim_in_range() && !_dctrl->landing()  && !low_battery_triggered)
                _navigation_status = ns_start_the_chase;
            if (_nav_flight_mode == nfm_manual)
                _navigation_status=ns_manual;
            break;
        } case ns_landed: {
            wpid = 0;
            _navigation_status = ns_start_shaking;
            landed_time = time;
            _trackers->dronetracker()->hover_mode(false);
            _trackers->dronetracker()->delete_landing_motion(time_out_after_landing);
            _dctrl->hover_mode(false);
            _dctrl->save_calibration();
            _flight_time+= static_cast<float>(time-time_take_off);
            _n_landings++;
            [[fallthrough]];
        } case ns_start_shaking: {
            if (static_cast<float>(time - landed_time) > 1.0f) {
                _navigation_status = ns_shaking_drone;
                time_shake_start = time;
            }
            break;
        } case ns_shaking_drone: {
            _dctrl->flight_mode(DroneController::fm_shake_it_baby);
            if (static_cast<float>(time - time_shake_start) > shake_duration) {
                _navigation_status = ns_wait_after_landing;
                _dctrl->flight_mode(DroneController::fm_disarmed);
            }
            break;
        } case ns_wait_after_landing: {
            if (static_cast<float>(time - landed_time) > time_out_after_landing + shake_duration )
                _navigation_status = ns_start_calibrating_motion;
            break;
        } case ns_manual: { // also used for disarmed
            wpid = 0;
            _trackers->mode(tracking::TrackerManager::mode_drone_only);
            _trackers->dronetracker()->manual_flight_mode(true);
            if (_nav_flight_mode == nfm_hunt) {
                _navigation_status=ns_wait_for_insect;
                _trackers->dronetracker()->manual_flight_mode(false);
            } else if (_nav_flight_mode != nfm_manual) { // waypoint mode
                _navigation_status=ns_wait_for_takeoff_command;
                _trackers->dronetracker()->manual_flight_mode(false);
            }
            break;
        } case ns_batlow: {
            _dctrl->flight_mode(DroneController::fm_abort);
            _dctrl->LED(static_cast<int>((time - time_drone_problem) * 10.0) % 10 > 5, 5); // minimal blink every 5 seconds
            break;
        } case ns_tracker_problem: {
            _dctrl->flight_mode(DroneController::fm_abort);
            _dctrl->LED(static_cast<int>((time - time_drone_problem) * 2.0) % 2 > 1, 5); // faster blink every second
            break;
        } case ns_drone_lost: {
            _dctrl->flight_mode(DroneController::fm_abort);
            _dctrl->LED(static_cast<int>((time - time_drone_problem) * 2.0) % 2 > 1, 5); // faster blink every second
            break;
        } case ns_unable_to_locate: {
            _dctrl->flight_mode(DroneController::fm_abort);
            break;
        } case ns_drone_problem: {
            if (time_drone_problem < 0)
                time_drone_problem = time;
            duration_drone_problem = static_cast<float>(time - time_drone_problem);
            if (_flight_time<0)
                _flight_time+= static_cast<float>(time-time_take_off);
            _dctrl->flight_mode(DroneController::fm_abort);
            _dctrl->LED(static_cast<int>((time - time_drone_problem) * 10.0) % 10 > 5,100); // blink every half second
            _dctrl->beep(true);
            break;
        }
        }
    }
    (*_logger) << static_cast<int16_t>(_navigation_status) << ";" << navigation_status() << ";";
}

void DroneNavigation::check_abort_autonomus_flight_conditions() {
    if (_nav_flight_mode == nfm_manual)
        _navigation_status=ns_manual;
    if (_trackers->dronetracker()->lost())
        _navigation_status = ns_drone_lost;
    if (_dctrl->telemetry().batt_cell_v > 2 && _dctrl->telemetry().batt_cell_v  < dparams.land_cell_v && !low_battery_triggered) {
        _navigation_status = ns_goto_yaw_waypoint;
        low_battery_triggered = true;
    }
}

void DroneNavigation::next_waypoint(Waypoint wp, double time) {
    delete current_waypoint;
    current_waypoint = new Waypoint(wp);
    if (wp.mode == wfm_takeoff) {
        cv::Point3f p = _trackers->dronetracker()->pad_location(true);
        setpoint_pos_world =  p + wp.xyz;
        setpoint_pos_world = _camview->setpoint_in_cameraview(setpoint_pos_world, CameraView::relaxed);
    } else if (wp.mode == wfm_landing || wp.mode == wfm_yaw_reset) {
        cv::Point3f p = _trackers->dronetracker()->pad_location(true);
        setpoint_pos_world =  p + wp.xyz;
        if (setpoint_pos_world.y > -0.5f) // keep some margin to the top of the view, because atm we have an overshoot problem.
            setpoint_pos_world.y = -0.5f;
        setpoint_pos_world = _camview->setpoint_in_cameraview(setpoint_pos_world, CameraView::relaxed);
        setpoint_pos_world_landing = setpoint_pos_world;
    } else {
        setpoint_pos_world =  wp.xyz;
        setpoint_pos_world = _camview->setpoint_in_cameraview(setpoint_pos_world, CameraView::relaxed);
    }
    _dctrl->nav_waypoint_changed(time);

    setpoint_vel_world = {0,0,0};
    setpoint_acc_world = {0,0,0};
}

void DroneNavigation::deserialize_settings() {
    std::cout << "Reading settings from: " << settings_file << std::endl;
    navigationParameters params;
    if (file_exist(settings_file)) {
        std::ifstream infile(settings_file);
        std::string xmlData((std::istreambuf_iterator<char>(infile)),
                            std::istreambuf_iterator<char>());

        if (!xmls::Serializable::fromXML(xmlData, &params))
        {   // Deserialization not successful
            throw MyExit("Cannot read: " + settings_file);
        }
        navigationParameters tmp;
        auto v1 = params.getVersion();
        auto v2 = tmp.getVersion();
        if (v1 != v2) {
            throw MyExit("XML version difference detected from " + settings_file);
        }
        infile.close();
    } else {
        throw MyExit("File not found: " + settings_file);
    }

    time_out_after_landing = params.time_out_after_landing.value();
}

void DroneNavigation::serialize_settings() {
    navigationParameters params;
    params.time_out_after_landing = time_out_after_landing;

    std::string xmlData = params.toXML();
    std::ofstream outfile = std::ofstream (settings_file);
    outfile << xmlData ;
    outfile.close();
}

void DroneNavigation::close() {
    if (initialized) {
        std::cout << "Closing drone navigation" << std::endl;
        if (pparams.navigation_tuning)
            serialize_settings();
        initialized = false;
    }
}

cv::Point3f DroneNavigation::square_point(cv::Point3f center, float width, float s) {
    s = fmodf(s, 4*width);
    float si = fmodf(s, width);

    float x_offset, y_offset, z_offset;

    if(s>3*width) {
        x_offset = -width/2;
        z_offset = width/2 - si;
    } else if(s>2*width) {
        x_offset = width/2 - si;
        z_offset = width/2;
    } else if(s>width) {
        x_offset = width/2;
        z_offset = -width/2 + si;
    } else {
        x_offset = -width/2 + si;
        z_offset = -width/2;
    }
    y_offset = z_offset;
    z_offset = 0;
    // Miss use this function to tune the pid controller:
    //x_offset = 0; z_offset = 0;
    //float y_offset = width/2 * sin(si/width*2*M_PIf32);

    return {center.x+x_offset, center.y+y_offset, center.z+z_offset};
}

void DroneNavigation::demo_flight(std::string flightplan_fn) {
    if (_nav_flight_mode == nfm_waypoint) {
        navigation::XML_FlightPlan fp;
        fp.deserialize(flightplan_fn);
        fp.serialize("./logging/flightplan.xml"); // write a copy of the currently used flightplan to the logging dir
        waypoints = fp.waypoints();
        wpid = 0;
        next_waypoint(waypoints[wpid],_trackers->dronetracker()->last_track_data().time);
        _navigation_status = ns_takeoff;
    } else
    {
        std::cout << "Warning: received waypoint trigger, but not in waypoint mode!" << std::endl;
    }
}
}
