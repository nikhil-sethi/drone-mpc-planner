#include "drone.h"
#include <chrono>
#include <experimental/filesystem>
#include "navigation.h"
#include "hash.h"

void Drone::init(std::ofstream *logger, int rc_id, RC *rc, tracking::TrackerManager *trackers, VisionData *visdat, FlightArea *flight_area, Interceptor *interceptor, BaseboardLink *baseboard_link, safety_margin_types _safety_margin_type) {
    _rc_id = rc_id;
    _rc = rc;
    main_logger = logger;
    _baseboard_link = baseboard_link;
    _interceptor = interceptor;
    _visdat = visdat;
    _trackers = trackers;
    confirm_drone_on_pad_delta_distance = dparams.pad_radius * 2.f;

    tracker.init(visdat, _trackers->motion_thresh(), 1);
    nav.init(&tracker, &control, visdat, flight_area, interceptor, baseboard_link);
    control.init(_rc, &tracker, flight_area, _safety_margin_type);
    tracker.commanded_acceleration(&control.commanded_acceleration);
    tracker.takeoff_area(flight_area);

    executor_hash = getFileHash("executor");

    (*main_logger) << "drone_state_str;";
    initialized = true;
}

void Drone::init_full_log_replay(std::string replay_dir) {
    control.init_full_log_replay(replay_dir);
}
void Drone::init_flight_replay(std::string replay_dir, int flight_id) {
    _state = ds_ready;
    flightplan_fn = replay_dir + "/flightplan_flight" + to_string(flight_id) + ".xml";
    control.init_flight_replay(replay_dir, flight_id);
}

void Drone::update(double time) {
#ifdef PATS_PROFILING
    std::chrono::_V2::system_clock::time_point t_start_drone = std::chrono::high_resolution_clock::now();
#endif
#ifdef PATS_PROFILING
    std::chrono::_V2::system_clock::time_point t_start_attitude_pad_update = std::chrono::high_resolution_clock::now();
#endif
    control.update_attitude_pad_state();
#ifdef PATS_PROFILING
    std::chrono::_V2::system_clock::time_point t_end_attitude_pad_update = std::chrono::high_resolution_clock::now();
    std::cout << "timing (drone_attitude_pad_update): " << (t_end_attitude_pad_update - t_start_attitude_pad_update).count() * 1e-6 << "ms" << std::endl;
#endif
    switch (_state) {
        case ds_pre_flight: {
                communicate_state(es_pats_x);
                control.control(tracker.last_track_data(), nav.setpoint(), position_control, cv::Point3f(0, 0, 0), cv::Point3f(0, 0, 0), time, false); // TODO need better solution for this
                pre_flight(time);
                break;
        } case ds_charging: {
                communicate_state(es_pats_x);
                control.flight_mode(DroneController::fm_inactive);
                control.control(tracker.last_track_data(), nav.setpoint(), position_control, cv::Point3f(0, 0, 0), cv::Point3f(0, 0, 0), time, false);
                if (_baseboard_link->battery_ready_for_flight() || _baseboard_link->disabled())
                    _state = ds_ready;
                else if (_baseboard_link->charging_problem())
                    _state = ds_charging_failure;
                else if (_baseboard_link->contact_problem() || !_baseboard_link->drone_on_pad()) {
                    _state = ds_pre_flight;
                    require_confirmation_drone_on_pad = true;
                }

                if (_rc->telemetry.batt_cell_v > max_safe_charging_telemetry_voltage) {
                    _baseboard_link->allow_charging(false);
                    _state = ds_overcharged_failure;
                    break;
                }
                break;
        } case ds_ready: {
                communicate_state(es_pats_x);
                control.control(tracker.last_track_data(), nav.setpoint(), position_control, cv::Point3f(0, 0, 0), cv::Point3f(0, 0, 0), time, false);
                if (_rc->telemetry.batt_cell_v > max_safe_charging_telemetry_voltage) {
                    _baseboard_link->allow_charging(false);
                    _state = ds_overcharged_failure;
                    break;
                }
                if (_interceptor->intercepting()) {

                    take_off(true, time);
                    _state = ds_flight;
                    _interceptor->reset_hunt_error();

                    _baseboard_link->allow_charging(false);
                    break;
                } else if (_interceptor->target_detected(time)) {
                    control.flight_mode(DroneController::fm_init_spinup);
                    _baseboard_link->allow_charging(false);
                } else if (control.manual_override_take_off_now()) {
                    flightplan_fn = pparams.flightplan;
                    take_off(false, time);
                    _state = ds_flight;
                    _baseboard_link->allow_charging(false);
                    break;
                } else if (trigger_waypoint_flight && !_trackers->monster_alert() && _visdat->no_recent_brightness_events(time)) {
                    trigger_waypoint_flight = false;
                    take_off(false, time);
                    _state = ds_flight;
                    _baseboard_link->allow_charging(false);
                    break;
                } else {
                    control.flight_mode(DroneController::fm_inactive);
                    _baseboard_link->allow_charging(true);
                }
                if (_baseboard_link->charging_problem())
                    _state = ds_charging_failure;
                else if (_baseboard_link->contact_problem() || !_baseboard_link->drone_on_pad()) {
                    _state = ds_pre_flight;
                    require_confirmation_drone_on_pad = true;
                } else if (((_baseboard_link->drone_battery_voltage() < dparams.min_hunt_cell_v && !control.spinup()) || _baseboard_link->drone_battery_voltage() < dparams.min_hunt_cell_v - 0.5f) && !_baseboard_link->disabled()) { // alternative: charge if below min hunt cel v and not spinup or spinup and like 0.5 volts below hunt cell v
                    _state = ds_charging;
                    _baseboard_link->allow_charging(true);
                } else if (!control.rc_ok(time)) {
                    _state = ds_rc_loss;
                    _baseboard_link->allow_charging(true);
                }
                _has_been_ready = true;
                break;
        } case ds_flight: {
                communicate_state(es_pats_x);
                auto state = tracker.last_track_data();

                flight_logger << _visdat->frame_id << ";" << time << ";" << state.dt << ";" << drone_state_str() << ";";
                nav.update(time);
                if (_interceptor->control_mode() == acceleration_feedforward)
                    control.control(state, nav.setpoint(),  acceleration_feedforward, cv::Point3f(0, 0, 0), _interceptor->aim_acc(), time, true);
                else if (_interceptor->control_mode() == velocity_control)
                    control.control(state, nav.setpoint(), velocity_control, _interceptor->aim_vel(), cv::Point3f(0, 0, 0), time, true);
                else if (_interceptor->control_mode() == acceleration_control)
                    control.control(state, nav.setpoint(), acceleration_control,  cv::Point3f(0, 0, 0), _interceptor->aim_acc(), time, true);
                else
                    control.control(state, nav.setpoint(), position_control, cv::Point3f(0, 0, 0), cv::Point3f(0, 0, 0), time, true);

                _interceptor->log(&flight_logger);
                _interceptor->target_is_hunted(_n_take_offs);
                flight_logger << std::endl;
                if (nav.drone_problem()) {
                    post_flight_state = post_init_crashed;
                    _state = ds_post_flight;
                } else if (nav.flight_done()) {
                    post_flight_state = post_init;
                    _state = ds_post_flight;
                    _n_landings++;
                } else if (nav.flight_aborted()) {
                    post_flight_state = post_aborted;
                    _state = ds_post_flight;
                }
                break;
        } case ds_post_flight: {
                communicate_state(es_pats_x);
                control.control(tracker.last_track_data(), nav.setpoint(), position_control, cv::Point3f(0, 0, 0), cv::Point3f(0, 0, 0), time, false);
                post_flight(time);
                break;
        } case ds_charging_failure: {
                // #1177
                if (!_baseboard_link->charging_problem() &&  _baseboard_link->drone_on_pad()) {
                    _baseboard_link->allow_charging(true);
                    _state = ds_charging;
                }
                break;
        } case ds_overcharged_failure: {
                if (_rc->telemetry.batt_cell_v < max_safe_charging_telemetry_voltage - 0.1f) {
                    _baseboard_link->allow_charging(true);
                    _state = ds_charging;
                }
                break;

        } case ds_rc_loss: {
                // #1177
                break;
        } case ds_beep: {
                _rc->beep(true);
                break;
            }
    }

    (*main_logger) << drone_state_str() << ";";
#ifdef PATS_PROFILING
    std::chrono::_V2::system_clock::time_point t_end_drone = std::chrono::high_resolution_clock::now();
    std::cout << "timing (update_drone): " << (t_end_drone - t_start_drone).count() * 1e-6 << "ms" << std::endl;
#endif
}

void Drone::pre_flight(double time) {
    switch (pre_flight_state) {
        case pre_init: {
                n_locate_drone_attempts = 0;
                _trackers->mode(tracking::TrackerManager::t_idle);
                time_start_locating_drone = time;
                pre_flight_state = pre_locate_drone_init;
                _baseboard_link->allow_charging(true);
                [[fallthrough]];
        } case pre_locate_drone_init: {
                control.LED(true);
                control.led_strength(_visdat->light_level());
                control.flight_mode(DroneController::fm_disarmed);
                communicate_state(es_pats_x);
                time_start_locating_drone_attempt = time;
                time_low_voltage = 0;
                n_locate_drone_attempts++;
                if (dparams.led_type == led_none && !control.pad_calib_valid()) {
                    std::cout << "Error: The drone has no led, and no valid drone calibration (with takeoff location) was found..." << std::endl;
                    pre_flight_state = pre_locate_time_out;
                    _trackers->mode(tracking::TrackerManager::t_c);
                } else if (control.pad_calib_valid() && !require_confirmation_drone_on_pad)
                    pre_flight_state = pre_check_telemetry;
                else
                    pre_flight_state = pre_locate_drone_wait_led;
                break;
        } case pre_locate_drone_wait_led: {
                _visdat->reset_motion_integration();
                if (static_cast<float>(time - time_start_locating_drone) > led_response_duration) {
                    _visdat->disable_fading = true;
                    _trackers->mode(tracking::TrackerManager::t_locate_drone);
                    control.flight_mode(DroneController::fm_disarmed);
                    pre_flight_state = pre_locate_drone;
                    n_detected_blink_locations = _trackers->detected_blink_locations().size();
                }
                break;
        } case pre_locate_drone: {
                blink(time);

                if (require_confirmation_drone_on_pad && _baseboard_link->charging() && control.pad_calib_valid()) {
                    require_confirmation_drone_on_pad = false;
                    pre_flight_state = pre_check_telemetry;
                }
                if (_rc->telemetry_time_out()) {
                    pre_flight_state =  pre_telemetry_time_out;
                    _trackers->mode(tracking::TrackerManager::t_c);
                    communicate_state(es_pats_x);
                }

                auto detected_blink_locations = _trackers->detected_blink_locations();
                if (detected_blink_locations.size() > n_detected_blink_locations) {
                    std::cout << "Blink detected at " << detected_blink_locations.back() << std::endl;
                    n_detected_blink_locations = detected_blink_locations.size();
                    control.LED(true);
                    _visdat->disable_fading = false;
                    if (require_confirmation_drone_on_pad) {
                        require_confirmation_drone_on_pad = false;
                        float dist = normf(tracker.pad_location(false) - tracker.pad_location_from_blink(detected_blink_locations.back()));
                        std::cout << "Confirmed drone on " << dist << " from the pad... while charging is: " << _baseboard_link->charging() << std::endl;
                        if (dist < confirm_drone_on_pad_delta_distance && _baseboard_link->charging()) {
                            pre_flight_state = pre_check_telemetry;
                            _trackers->mode(tracking::TrackerManager::t_x);
                        } else if (_baseboard_link->charging()) {
                            control.invalidize_blink();
                            tracker.set_pad_location_from_blink(detected_blink_locations.back());
                            _trackers->mode(tracking::TrackerManager::t_x);
                            time_located_drone = time;
                            n_locate_drone_attempts = 0;
                            pre_flight_state = pre_check_telemetry;
                            _n_drone_detects++;
                        } else if (dist < confirm_drone_on_pad_delta_distance && control.att_somewhere_on_pad()) {
                            pre_flight_state = pre_init;
                            _state = ds_post_flight;
                            time_start_shaking = time;
                            post_flight_state = post_start_shaking;
                            _trackers->mode(tracking::TrackerManager::t_x);
                            std::cout << "Not charging, but blink and att confirm drone is on pad. Shake it." << std::endl;
                        } else {
                            pre_flight_state = pre_init;
                            _state = ds_post_flight;
                            post_flight_state = post_lost;
                            _trackers->mode(tracking::TrackerManager::t_c);
                        }
                    } else if (_baseboard_link->charging() || _baseboard_link->disabled()) {
                        tracker.set_pad_location_from_blink(detected_blink_locations.back());
                        _trackers->mode(tracking::TrackerManager::t_x);
                        time_located_drone = time;
                        n_locate_drone_attempts = 0;
                        pre_flight_state = pre_check_telemetry;
                        _n_drone_detects++;
                    } else if (control.att_ok_for_pad_calibration()) {
                        pre_flight_state = pre_init;
                        _state = ds_post_flight;
                        time_start_shaking = time;
                        post_flight_state = post_start_shaking;
                        _trackers->mode(tracking::TrackerManager::t_x);
                        std::cout << "Found the drone, it is reasonably up right, but its not charging so not sure if it is actually on the pad. Shake it to see if we can make it charge." << std::endl;
                    }
                }
                if (time - time_start_locating_drone_attempt > 15) {
                    pre_flight_state = pre_locate_drone_init;
                    _trackers->mode(tracking::TrackerManager::t_idle);
                }
                if (time - time_start_locating_drone > 300) {
                    pre_flight_state = pre_locate_time_out;
                    _trackers->mode(tracking::TrackerManager::t_c);
                    communicate_state(es_pats_x);
                }
                break;
        } case pre_check_telemetry: {
                if (control.telemetry_OK() || ! dparams.Telemetry()) {
                    time_start_att_wait_pad = time;
                    if (control.pad_calib_valid())
                        pre_flight_state = pre_check_pad_att;
                    else {
                        control.reset_attitude_pad_filter(); // avoids calibrating the pad attitude with corrupt telemetry
                        pre_flight_state = pre_calibrating_pad;
                    }
                }
                if (_rc->telemetry_time_out()) {
                    pre_flight_state =  pre_telemetry_time_out;
                    _trackers->mode(tracking::TrackerManager::t_c);
                    communicate_state(es_pats_x);
                }
                break;
        } case pre_calibrating_pad: {
                if (control.att_ok_for_pad_calibration() || !dparams.Telemetry()) {
                    control.save_pad_pos_and_att_calibration();
                    pre_flight_state = pre_wait_to_arm;
                }
                break;
        } case pre_check_pad_att: {
                if (control.att_telemetry_valid()) {
                    if (_baseboard_link->charging() || _baseboard_link->disabled()) {
                        if (control.att_precisely_on_pad()) {
                            tracker.drone_on_landing_pad(true);
                            pre_flight_state = pre_wait_to_arm;
                        }
                        if (!control.att_precisely_on_pad() && static_cast<float>(time - time_start_att_wait_pad) > att_wait_pad_timeout && dparams.Telemetry() && !_baseboard_link->disabled()) {
                            control.reset_attitude_pad_filter(); // avoids calibrating the pad attitude with corrupt telemetry
                            pre_flight_state = pre_calibrating_pad;
                        }
                    } else if (static_cast<float>(time - time_start_att_wait_pad) > att_wait_pad_timeout && control.att_somewhere_on_pad()) {
                        pre_flight_state = pre_init;
                        require_confirmation_drone_on_pad = true;
                    }
                }
                if (_rc->telemetry_time_out()) {
                    pre_flight_state =  pre_telemetry_time_out;
                    _trackers->mode(tracking::TrackerManager::t_c);
                    communicate_state(es_pats_x);
                }
                break;
        } case pre_wait_to_arm: {
                if (control.ready_for_arm(time)) {
                    control.flight_mode(DroneController::fm_inactive);
                    pre_flight_state = pre_arming;
                }
                break;
        } case pre_arming: {
                if (control.arming_problem()) {
                    control.flight_mode(DroneController::fm_disarmed);
                    pre_flight_state = pre_wait_to_arm;
                } else {
                    pre_flight_state = pre_wait_init_led;
                    time_led_init = time;
                    switch (dparams.led_type) {
                        case led_none:
                            control.LED(false);
                            break;
                        case led_strip:
                            control.LED(true);
                            break;
                        case led_fiber_ir:
                            control.LED(true);
                            break;
                        case led_fiber_uv:
                            control.LED(true);
                            break;
                        case led_top_uv:
                            control.LED(false);
                            break;
                    }
                }
                break;
        } case pre_wait_init_led: {
                if (static_cast<float>(time - time_led_init) > led_response_duration) {
                    _visdat->reset_motion_integration();
                    _trackers->mode(tracking::TrackerManager::t_x);
                    if (_baseboard_link->charging()) {
                        pre_flight_state = pre_init;
                        _state = ds_charging;
                    } else {
                        pre_flight_state = pre_wait_charging;
                        time_waiting_for_charge = time;
                    }
                }
                break;
        } case pre_wait_charging: {
                if (_baseboard_link->charging() || _baseboard_link->disabled()) {
                    pre_flight_state = pre_init;
                    _state = ds_charging;
                } else if (static_cast<float>(time - time_waiting_for_charge) > wait_charging_response_duration) {
                    require_confirmation_drone_on_pad = true;
                    pre_flight_state = pre_locate_drone_init;
                    std::cout << "Not charging, but drone att is within pad bounds. Confirming with blink...." << std::endl;
                }
                break;
        } case pre_locate_time_out: {
                if (((_baseboard_link->charging() && _baseboard_link->charging_duration() < 1) || _baseboard_link->disabled()) && (control.telemetry_OK() || ! dparams.Telemetry())) {
                    pre_flight_state =  pre_locate_drone_init;
                    time_start_locating_drone = time;
                }
                if (_rc->telemetry_time_out()) {
                    pre_flight_state =  pre_telemetry_time_out;
                    communicate_state(es_pats_x);
                } else if (control.telemetry_OK() && low_voltage_timeout(time, _rc->telemetry.batt_cell_v)) {
                    _state = ds_post_flight;
                    post_flight_state = post_init_deep_sleep;
                    communicate_state(es_pats_x);
                }
                break;
        } case pre_telemetry_time_out: {
                if (!_rc->telemetry_time_out()) {
                    pre_flight_state =  pre_locate_drone_init;
                    time_start_locating_drone = time;
                }
            }
    }
}
void Drone::post_flight(double time) {
    switch (post_flight_state) {
        case post_init: {
                n_shakes_sessions_after_landing = 0;
                time_low_voltage = 0;
                flight_logger.flush();
                flight_logger.close();
                land_datetime = chrono::system_clock::to_time_t(chrono::system_clock::now());
                _trackers->stop_drone_tracking(&tracker);
                save_flight_results();
                time_reset_yaw_on_pad = time;
                control.freeze_attitude_reset_yaw_on_pad();
                post_flight_state = post_reset_yaw_on_pad;
                break;
        } case post_reset_yaw_on_pad: {
                control.flight_mode(DroneController::fm_reset_yaw_on_pad);
                if (static_cast<float>(time - time_reset_yaw_on_pad) > duration_reset_yaw_on_pad) {
                    post_flight_state = post_start_shaking;
                    control.flight_mode(DroneController::fm_wait);
                    time_reset_yaw_on_pad = 0;
                    time_start_shaking = time;
                }
                break;
        } case post_start_shaking: {
                if (static_cast<float>(time - time_start_shaking) > duration_wait_before_shake) {
                    control.flight_mode(DroneController::fm_start_shake);
                    post_flight_state = post_shaking_drone;
                    time_shake_start = time;
                }
                if (dparams.static_shakeit_thrust < 0) {
                    post_flight_state = post_wait_after_shake;
                }
                break;
        } case post_shaking_drone: {
                if (control.shake_finished())  // require_confirmation_drone_on_pad = true;
                    post_flight_state = post_wait_after_shake_init;
                break;
        } case post_wait_after_shake_init: {
                control.flight_mode(DroneController::fm_disarmed);
                n_shakes_sessions_after_landing++;
                time_post_shake = time;
                post_flight_state = post_wait_after_shake;
                control.reset_attitude_pad_filter();
                [[fallthrough]];
        } case post_wait_after_shake: {
                _baseboard_link->allow_charging(true);
                if (static_cast<float>(time - time_post_shake) > duration_post_shake_wait && control.att_telemetry_valid()) {
                    if (_baseboard_link->charging_waits_until_drone_ready() && static_cast<float>(time - time_post_shake) < duration_post_shake_timeout) { /* wait some more until we receive new data the baseboard */ }
                    else if (control.att_precisely_on_pad() && _baseboard_link->charging()) {
                        if (control.accelerometer_trim.ready()) {
                            control.flight_mode(DroneController::fm_trim_accelerometer);
                            post_flight_state = post_trim_accelerometer;
                        } else {
                            post_flight_state = post_init;
                            _state = ds_charging;
                        }
                    } else if (control.att_precisely_on_pad() && _baseboard_link->disabled()) {
                        post_flight_state = post_init;
                        _state = ds_charging;
                    } else if (_baseboard_link->charging()) {
                        post_flight_state = post_init;
                        pre_flight_state = pre_init;
                        _state = ds_pre_flight;
                    } else if (n_shakes_sessions_after_landing <= 30 && control.att_somewhere_on_pad()) {
                        post_flight_state = post_start_shaking;
                        std::cout << "Shake unsucessfull. Attempt " << n_shakes_sessions_after_landing << " of 30" << std::endl;
                        _baseboard_link->allow_charging(false);
                    }
                    else {
                        post_flight_state = post_lost;
                    }
                }
                break;
        } case post_trim_accelerometer: {
                if (control.flight_mode() == DroneController::fm_wait) {
                    post_flight_state = post_init;
                    _state = ds_charging;
                }
                break;
        } case post_aborted: {
                flight_logger.flush();
                flight_logger.close();
                _trackers->stop_drone_tracking(&tracker);
                _baseboard_link->allow_charging(true);
                post_flight_state = post_init;
                _state = ds_ready;
                break;
        } case post_init_crashed: {
                time_crashed = time;
                flight_logger.flush();
                flight_logger.close();
                _trackers->stop_drone_tracking(&tracker);
                _baseboard_link->allow_charging(true);
                save_flight_results();
                post_flight_state = post_crashed;
                communicate_state(es_pats_x);
                break;
        } case post_crashed: {
                if (_baseboard_link->charging() && (time - time_crashed > 5)) { // baseboard charging detection can be slower a fast crash, so give it a few seconds
                    post_flight_state = post_init;
                    communicate_state(es_pats_x);
                    _state = ds_pre_flight;
                    pre_flight_state = pre_init;
                } else if (control.telemetry_OK() && low_voltage_timeout(time, _rc->telemetry.batt_cell_v)) {
                    post_flight_state = post_init_deep_sleep;
                    communicate_state(es_pats_x);
                }
                break;
        } case post_lost: {
                if (_baseboard_link->charging() || _baseboard_link->disabled()) {
                    post_flight_state = post_init;
                    pre_flight_state = pre_init;
                    _state = ds_pre_flight;
                } else if (control.telemetry_OK() && low_voltage_timeout(time, _rc->telemetry.batt_cell_v)) {
                    post_flight_state = post_init_deep_sleep;
                    communicate_state(es_pats_x);
                }
                break;
        } case post_init_deep_sleep: {
                if (_baseboard_link->charging() && (control.telemetry_OK())) {
                    post_flight_state = post_init;
                    pre_flight_state = pre_init;
                    _state = ds_pre_flight;
                } else {
                    control.LED(false);
                    post_flight_state = post_deep_sleep;
                }
                break;
        } case post_deep_sleep: {
                if (_baseboard_link->charging() && (control.telemetry_OK())) {
                    post_flight_state = post_init;
                    pre_flight_state = pre_init;
                    _state = ds_pre_flight;
                } else {
                    control.flight_mode(DroneController::fm_sleep);
                }
                break;
            }
    }
}

void Drone::take_off(bool hunt, double time) {
    _n_take_offs++;
    take_off_datetime = chrono::system_clock::to_time_t(chrono::system_clock::now());
    if (hunt) {
        _n_hunt_flights++;
        std::cout << "Taking off to hunt #" << _n_hunt_flights << std::endl;
    } else {
        _n_wp_flights++;
        std::cout << "Taking off for waypoint flight #" << _n_wp_flights << std::endl;
    }

    _visdat->save_maps(_n_take_offs, data_output_dir);

    flight_logger.open(data_output_dir  + "log_flight" + to_string(_n_take_offs) + ".csv", std::ofstream::out);
    tracker.init_flight(&flight_logger, time);
    flight_logger << "rs_id;elapsed;dt;drone_state_str;";
    _trackers->start_drone_tracking(&tracker);
    nav.init_flight(hunt, &flight_logger);
    if (!hunt) {
        nav.flightplan(flightplan_fn);
        std::experimental::filesystem::copy(flightplan_fn, data_output_dir  + "flightplan_flight" + to_string(_n_take_offs) + ".xml");
    }
    control.init_flight(&flight_logger, _n_take_offs);
    _interceptor->init_flight(&flight_logger);

    flight_logger << std::endl;
}

void Drone::save_flight_results() {
    std::ofstream results_log;
    results_log.open(data_output_dir  + "flight_results" + to_string(_n_take_offs) + ".txt", std::ofstream::out);
    results_log << "take_off_datetime:" << std::put_time(std::localtime(&take_off_datetime), "%Y/%m/%d %T") << '\n';
    results_log << "land_datetime:" <<  std::put_time(std::localtime(&land_datetime), "%Y/%m/%d %T") << '\n';
    results_log << "flight_time:" << nav.flight_time() << '\n';
    results_log << "crashed:" << nav.drone_problem() << '\n';
    results_log << "best_interception_distance:" << _interceptor->best_distance() << '\n';
    results_log << "time_to_best_interception:" << _interceptor->time_best_distance() << '\n';
    results_log << "pos_best_interception_xyz:" << _interceptor->pos_best_distance().x << "," << _interceptor->pos_best_distance().y << "," << _interceptor->pos_best_distance().z << '\n';
    results_log << "vel_best_interception_xyz:" << _interceptor->vel_best_distance().x << "," << _interceptor->vel_best_distance().y << "," << _interceptor->vel_best_distance().z << '\n';
    results_log << "acc_best_interception_xyz:" << _interceptor->acc_best_distance().x << "," << _interceptor->acc_best_distance().y << "," << _interceptor->acc_best_distance().z << '\n';
    results_log << "executor_hash:" << executor_hash << '\n';
    if (benchmark_len) {
        if (benchmark_entry_id <= benchmark_len) {
            results_log << "benchmark_timestamp:" << benchmark_time << '\n';
            results_log << "benchmark_hash:" << benchmark_hash << '\n';
            results_log << "benchmark_entry_id:" << benchmark_entry_id << '\n';
        }
        else {
            benchmark_len = 0;
            benchmark_entry_id = 0;
        }
    }
    results_log.close();
}

void Drone::blink(double time) {
    static double last_blink_time = time;
    static bool blink_state;

    if (static_cast<float>(time - last_blink_time) > dparams.blink_period) {
        if (blink_state)
            blink_state = false;
        else
            blink_state = true;
        last_blink_time = time;
    }
    control.LED(blink_state);
}

bool Drone::low_voltage_timeout(double time, float voltage) {
    if (voltage >= 4.0F) {
        time_low_voltage = 0;
        return false;
    }
    if (!time_low_voltage) {
        time_low_voltage = time;
        return false;
    }
    if (time - time_low_voltage > 60.0) {
        return true;
    }
    return false;
}

void Drone::inject_log(logging::LogEntryDrone entry, unsigned long long rs_id) {
    if (entry.nav_flight_mode == navigation::nfm_waypoint && nav.nav_flight_mode() != navigation::nfm_waypoint)
        trigger_waypoint_flight = true;
    if (rs_id >= entry.rs_id)
        control.inject_log(entry);
}

void Drone::close() {
    if (initialized) {
        initialized = false;
        control.close();
        nav.close();
    }
}
