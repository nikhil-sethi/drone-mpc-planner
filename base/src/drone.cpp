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

    tracker.init(visdat, _trackers->motion_thresh(), 1);
    nav.init(&tracker, &control, visdat, flight_area, interceptor, baseboard_link);
    control.init(_rc, &tracker, flight_area, visdat->camera_exposure);

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
    switch (_state) {
        case ds_pre_flight: {
                control.control(tracker.last_track_data(), nav.setpoint(), _interceptor->target_last_trackdata(), time, false); // TODO need better solution for this
                pre_flight(time);
                break;
        } case ds_charging: {
                control.control(tracker.last_track_data(), nav.setpoint(), _interceptor->target_last_trackdata(), time, false);
                if (_baseboard_link->battery_ready_for_flight() || _baseboard_link->disabled())
                    _state = ds_ready;
                else if (_baseboard_link->charging_problem() || !_baseboard_link->drone_on_pad())
                    _state = ds_charging_failure;

                if (_rc->telemetry.batt_cell_v > 4.7f) {
                    _baseboard_link->allow_charging(false);
                    _state = ds_charging_failure;
                    break;
                }
                break;
        } case ds_ready: {
                control.control(tracker.last_track_data(), nav.setpoint(), _interceptor->target_last_trackdata(), time, false);
                if (_rc->telemetry.batt_cell_v > 4.7f) {
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
                } else if (trigger_waypoint_flight) {
                    trigger_waypoint_flight = false;
                    take_off(false, time);
                    _state = ds_flight;
                    _baseboard_link->allow_charging(false);
                    break;
                } else {
                    control.flight_mode(DroneController::fm_inactive);
                    _baseboard_link->allow_charging(true);
                }
                if (_baseboard_link->contact_problem() || !_baseboard_link->drone_on_pad() || _baseboard_link->charging_problem())
                    _state = ds_pre_flight;
                else if (_baseboard_link->drone_battery_voltage() < dparams.min_hunt_cell_v - 0.1f && !_baseboard_link->disabled()) {
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
                _interceptor->target_is_hunted();
                flight_logger << std::endl;
                if (nav.drone_problem())
                    _state = ds_crashed;
                else if (nav.flight_done()) {
                    _state = ds_post_flight;
                    _n_landings++;
                    flight_logger.flush();
                    flight_logger.close();
                }
                break;
        } case ds_post_flight: {
                control.control(tracker.last_track_data(), nav.setpoint(), _interceptor->target_last_trackdata(), time, false);
                post_flight(time);
                break;
        } case ds_charging_failure: {
                // #1177
                if (!_baseboard_link->charging_problem() &&  _baseboard_link->drone_on_pad() && _rc->telemetry.batt_cell_v < 4.3f) {
                    _baseboard_link->allow_charging(true);
                    _state = ds_charging;
                }
                break;
        } case ds_rc_loss: {
                // #1177
                break;
        } case ds_crashed: {
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
    control.calibrate_pad_attitude();
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
                control.flight_mode(DroneController::fm_disarmed);
                time_start_locating_drone_attempt = time;
                n_locate_drone_attempts++;
                if (dparams.led_type == led_none && (! control.takeoff_calib_valid() || force_pad_redetect)) {
                    std::cout << "Error: The drone has no led, and no valid drone calibration (with takeoff location) was found..." << std::endl;
                    pre_flight_state = pre_locate_time_out;
                } else if (control.takeoff_calib_valid() && !force_pad_redetect)
                    pre_flight_state = pre_calibrating_pad;
                else {
                    control.invalidize_blink();
                    pre_flight_state = pre_locate_drone_wait_led;
                }
                force_pad_redetect = false;
                break;
        } case pre_locate_drone_wait_led: {
                _visdat->reset_motion_integration();
                if (static_cast<float>(time - time_start_locating_drone) > led_response_duration) {
                    _visdat->disable_fading = true;
                    _trackers->mode(tracking::TrackerManager::t_locate_drone);
                    control.flight_mode(DroneController::fm_disarmed);
                    pre_flight_state = pre_locate_drone;
                    time_last_led_doubler = time;
                }
                break;
        } case pre_locate_drone: {
                blink(time);

                auto detected_pad_locations = _trackers->detected_pad_locations();
                if (detected_pad_locations.size() > n_detected_pad_locations) {
                    n_detected_pad_locations = detected_pad_locations.size();
                    tracker.set_pad_location_from_blink(detected_pad_locations.back());
                    _trackers->mode(tracking::TrackerManager::t_x);

                    time_located_drone = time;
                    n_locate_drone_attempts = 0;
                    control.LED(true);
                    _visdat->disable_fading = false;
                    pre_flight_state = pre_calibrating_pad;
                    _n_drone_detects++;
                }
                if (time - time_last_led_doubler > 3) {  // if the blink detect takes too long, it may be that the led is not bright enough to be detected
                    time_last_led_doubler = time;
                    // control.double_led_strength();
                }
                if (time - time_start_locating_drone_attempt > 15)
                    pre_flight_state = pre_locate_drone_init;
                if (time - time_start_locating_drone > 300)
                    pre_flight_state = pre_locate_time_out;
                break;
        } case pre_calibrating_pad: {
                if (control.pad_calibration_done())
                    pre_flight_state = pre_check_telemetry;
                break;
        } case pre_check_telemetry: {
                if (control.telemetry_OK() || ! dparams.Telemetry())
                    pre_flight_state = pre_check_pad_att;
                break;
        } case pre_check_pad_att: {
                if (control.attitude_on_pad_OK() || ! dparams.Telemetry()) {
                    pre_flight_state = pre_wait_to_arm;
                    tracker.drone_on_landing_pad(true);
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
                    pre_flight_state = pre_init;
                    _state = ds_charging;
                }
                break;
        } case pre_locate_time_out: {
                // #1177
                break;
            }
    }
}
void Drone::post_flight(double time) {
    switch (post_flight_state) {
        case post_init: {
                land_datetime = chrono::system_clock::to_time_t(chrono::system_clock::now());
                _trackers->stop_drone_tracking(&tracker);
                save_flight_results();
                if (_baseboard_link->charging())
                    post_flight_state = post_wait_after_shake_init;
                else {
                    time_reset_yaw_on_pad = time;
                    control.freeze_attitude_reset_yaw_on_pad();
                    post_flight_state = post_reset_yaw_on_pad;
                }
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
                if (control.shake_finished())
                    post_flight_state = post_wait_after_shake_init;
                break;
        } case post_wait_after_shake_init: {
                control.flight_mode(DroneController::fm_disarmed);
                n_shakes_sessions_after_landing++;
                control.new_attitude_package_available(); // reset internal counter so that we can use it in ns_wait_after_shake
                time_post_shake = time;
                post_flight_state = post_wait_after_shake;
                [[fallthrough]];
        } case post_wait_after_shake: {
                _baseboard_link->allow_charging(true);
                if (static_cast<float>(time - time_post_shake) > duration_post_shake_wait) {
                    if (!control.new_attitude_package_available()) { /* wait some more until we receive new package */ }
                    else if (control.attitude_on_pad_OK() && (_baseboard_link->charging() || _baseboard_link->disabled())) {
                        post_flight_state = post_init;
                        _state = ds_charging;
                    } else if (n_shakes_sessions_after_landing <= 10 && control.drone_pad_state()) {
                        post_flight_state = post_start_shaking;
                        _baseboard_link->allow_charging(false);
                    }
                    else {
                        post_flight_state = post_lost;
                        _baseboard_link->allow_charging(false);
                    }
                }
                break;
        } case post_lost: {
                // #1177
                if ((_baseboard_link->charging() || _baseboard_link->disabled()) && control.attitude_on_pad_OK()) {
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