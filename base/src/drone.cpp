#include "drone.h"
#include <experimental/filesystem>
#include "navigation.h"

void Drone::init(std::ofstream *logger, int rc_id, RC *rc, tracking::TrackerManager *trackers, VisionData *visdat, FlightArea *flight_area, Interceptor *interceptor, BaseboardLink *baseboard_link) {
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
    control.init(_rc, &tracker, flight_area);

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
    control.update_drone_attitude_pad_state();

    switch (_state) {
        case ds_pre_flight: {
                control.control(tracker.last_track_data(), nav.setpoint(), _interceptor->target_last_trackdata(), time, false); // TODO need better solution for this
                pre_flight(time);
                break;
        } case ds_charging: {
                control.flight_mode(DroneController::fm_inactive);
                control.control(tracker.last_track_data(), nav.setpoint(), _interceptor->target_last_trackdata(), time, false);
                if (_baseboard_link->battery_ready_for_flight() || _baseboard_link->disabled())
                    _state = ds_ready;
                else if (_baseboard_link->charging_problem())
                    _state = ds_charging_failure;
                else if (_baseboard_link->contact_problem() || !_baseboard_link->drone_on_pad()) {
                    _state = ds_pre_flight;
                    confirm_drone_on_pad = true;
                }

                if (_rc->telemetry.batt_cell_v > max_safe_charging_telemetry_voltage) {
                    _baseboard_link->allow_charging(false);
                    _state = ds_charging_failure;
                    break;
                }
                break;
        } case ds_ready: {
                control.control(tracker.last_track_data(), nav.setpoint(), _interceptor->target_last_trackdata(), time, false);
                if (_rc->telemetry.batt_cell_v > max_safe_charging_telemetry_voltage) {
                    _baseboard_link->allow_charging(false);
                    _state = ds_charging_failure;
                    break;
                }
                if (_interceptor->target_acquired(time)) {
                    take_off(true, time);
                    _state = ds_flight;
                    _baseboard_link->allow_charging(false);
                    break;
                } else if (_interceptor->target_detected(time)) {
                    control.flight_mode(DroneController::fm_spinup);
                    _baseboard_link->allow_charging(false);
                } else if (control.manual_override_take_off_now()) {
                    flightplan_fn = pparams.flightplan;
                    take_off(false, time);
                    _state = ds_flight;
                    _baseboard_link->allow_charging(false);
                    break;
                } else if (trigger_waypoint_flight && !_trackers->monster_alert() && !_visdat->no_recent_brightness_events(time)) {
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
                    confirm_drone_on_pad = true;
                } else if (_baseboard_link->drone_battery_voltage() < dparams.min_hunt_cell_v - 0.1f && !_baseboard_link->disabled()) {
                    _state = ds_charging;
                    _baseboard_link->allow_charging(true);
                } else if (!control.rc_ok(time)) {
                    _state = ds_rc_loss;
                    _baseboard_link->allow_charging(true);
                }
                break;
        } case ds_flight: {
                auto state = tracker.last_track_data();
                flight_logger << _visdat->frame_id << ";" << time << ";" << state.dt << ";" << drone_state_str() << ";";
                nav.update(time);
                control.control(state, nav.setpoint(), _interceptor->target_last_trackdata(), time, true);
                _interceptor->target_is_hunted(_n_take_offs);
                flight_logger << std::endl;
                if (nav.drone_problem()) {
                    post_flight_state = post_init_crashed;
                    _state = ds_post_flight;
                } else if (nav.flight_done()) {
                    post_flight_state = post_init;
                    _state = ds_post_flight;
                    _n_landings++;
                }
                break;
        } case ds_post_flight: {
                control.control(tracker.last_track_data(), nav.setpoint(), _interceptor->target_last_trackdata(), time, false);
                post_flight(time);
                break;
        } case ds_charging_failure: {
                // #1177
                if (!_baseboard_link->charging_problem() &&  _baseboard_link->drone_on_pad()) {
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
                time_start_locating_drone_attempt = time;
                n_locate_drone_attempts++;
                if (dparams.led_type == led_none && !control.pad_calib_valid()) {
                    std::cout << "Error: The drone has no led, and no valid drone calibration (with takeoff location) was found..." << std::endl;
                    pre_flight_state = pre_locate_time_out;
                } else if (control.pad_calib_valid() && !confirm_drone_on_pad)
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
                    n_detected_pad_locations = _trackers->detected_pad_locations().size();
                }
                break;
        } case pre_locate_drone: {
                blink(time);
                auto detected_pad_locations = _trackers->detected_pad_locations();
                if (detected_pad_locations.size() > n_detected_pad_locations) {
                    std::cout << "Blink detected at " << detected_pad_locations.back() << std::endl;
                    n_detected_pad_locations = detected_pad_locations.size();
                    control.LED(true);
                    _visdat->disable_fading = false;
                    if (confirm_drone_on_pad) {
                        confirm_drone_on_pad = false;
                        float dist = normf(tracker.pad_location(false) - detected_pad_locations.back());
                        std::cout << "Confirmed drone on " << dist << " from the pad... while charging is: " << _baseboard_link->charging() << std::endl;
                        if (dist < confirm_drone_on_pad_delta_distance && _baseboard_link->charging()) {
                            pre_flight_state = pre_init;
                            _state = ds_charging;
                        } else if (_baseboard_link->charging()) {
                            control.invalidize_blink();
                            tracker.set_pad_location_from_blink(detected_pad_locations.back());
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
                            std::cout << "Not charging, but blink and att confirm drone is on pad. Shake it." << std::endl;
                        } else {
                            pre_flight_state = pre_init;
                            _state = ds_post_flight;
                            post_flight_state = post_lost;
                        }
                    } else if (_baseboard_link->charging() || _baseboard_link->disabled()) {
                        tracker.set_pad_location_from_blink(detected_pad_locations.back());
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
                        std::cout << "Found the drone, it is reasonably up right, but its not charging so not sure if it is actually on the pad. Shake it to see if we can make it charge." << std::endl;
                    }
                }
                if (time - time_start_locating_drone_attempt > 15)
                    pre_flight_state = pre_locate_drone_init;
                if (time - time_start_locating_drone > 300) {
                    pre_flight_state = pre_locate_time_out;
                    _trackers->mode(tracking::TrackerManager::t_c);
                }
                break;
        } case pre_check_telemetry: {
                if (control.telemetry_OK() || ! dparams.Telemetry()) {
                    time_start_att_wait_pad = time;
                    if (control.pad_calib_valid())
                        pre_flight_state = pre_check_pad_att;
                    else
                        pre_flight_state = pre_calibrating_pad;
                }
                break;
        } case pre_calibrating_pad: {
                if (control.att_ok_for_pad_calibration() || !dparams.Telemetry()) {
                    control.save_pad_pos_and_att_calibration();
                    pre_flight_state = pre_wait_to_arm;
                }
                break;
        } case pre_check_pad_att: {
                if (_baseboard_link->charging() || _baseboard_link->disabled()) {
                    if (control.att_precisely_on_pad()) {
                        tracker.drone_on_landing_pad(true);
                        pre_flight_state = pre_wait_to_arm;
                    }
                    if (!control.att_precisely_on_pad() && static_cast<float>(time - time_start_att_wait_pad) > att_wait_pad_timeout && dparams.Telemetry() && !_baseboard_link->disabled())
                        pre_flight_state = pre_calibrating_pad;
                } else if (static_cast<float>(time - time_start_att_wait_pad) > att_wait_pad_timeout && control.att_somewhere_on_pad()) {
                    pre_flight_state = pre_init;
                    confirm_drone_on_pad = true;
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
                    confirm_drone_on_pad = true;
                    pre_flight_state = pre_locate_drone_init;
                    std::cout << "Not charging, but drone att is within pad bounds. Confirming with blink...." << std::endl;
                }
                break;
        } case pre_locate_time_out: {
                if ((_baseboard_link->charging() || _baseboard_link->disabled()) && (control.telemetry_OK() || ! dparams.Telemetry())) {
                    pre_flight_state =  pre_locate_drone_init;
                    time_start_locating_drone = time;
                }
                break;
            }
    }
}
void Drone::post_flight(double time) {
    switch (post_flight_state) {
        case post_init: {
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
                if (control.shake_finished())  // confirm_drone_on_pad = true;
                    post_flight_state = post_wait_after_shake_init;
                break;
        } case post_wait_after_shake_init: {
                control.flight_mode(DroneController::fm_disarmed);
                n_shakes_sessions_after_landing++;
                time_post_shake = time;
                post_flight_state = post_wait_after_shake;
                [[fallthrough]];
        } case post_wait_after_shake: {
                _baseboard_link->allow_charging(true);
                if (static_cast<float>(time - time_post_shake) > duration_post_shake_wait) {
                    if (_baseboard_link->charging_waits_until_drone_ready()) { /* wait some more until we receive new data the baseboard */ }
                    else if (control.att_precisely_on_pad() && _baseboard_link->charging()) {
                        post_flight_state = post_init;
                        _state = ds_charging;
                        control.flight_mode(DroneController::fm_trim_accelerometer);
                        post_flight_state = post_trim_accelerometer;
                    } else if (control.att_precisely_on_pad() && _baseboard_link->disabled()) {
                        post_flight_state = post_init;
                        _state = ds_charging;
                    } else if (_baseboard_link->charging()) {
                        post_flight_state = post_init;
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
        } case post_init_crashed: {
                _trackers->stop_drone_tracking(&tracker);
                _baseboard_link->allow_charging(true);
                save_flight_results();
                post_flight_state = post_crashed;
                break;
        } case post_crashed: {
                if (_baseboard_link->charging()) {
                    post_flight_state = post_init;
                    _state = ds_pre_flight;
                }
                break;
        } case post_lost: {
                if ((_baseboard_link->charging() || _baseboard_link->disabled())) {
                    post_flight_state = post_init;
                    _state = ds_pre_flight;
                }
                break;
            }
    }
}

void Drone::take_off(bool hunt, double time) {
    _n_take_offs++;
    take_off_datetime = chrono::system_clock::to_time_t(chrono::system_clock::now());
    if (hunt)
        _n_hunt_flights++;
    else
        _n_wp_flights++;
    _visdat->save_maps_before_flight(_n_take_offs, data_output_dir);

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
