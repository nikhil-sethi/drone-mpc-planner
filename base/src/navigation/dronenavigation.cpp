#include <iostream>
#include "dronenavigation.h"
#include "interceptor.h"

using namespace cv;
using namespace std;

namespace navigation {

void DroneNavigation::init(tracking::DroneTracker *tracker, DroneController *control, VisionData *visdat, FlightArea *flight_area, Interceptor *interceptor, BaseboardLink *baseboard_link) {
    _control = control;
    _tracker = tracker;
    _visdat = visdat;
    _flight_area = flight_area;
    _iceptor = interceptor;
    _baseboard_link = baseboard_link;
    deserialize_settings();
    initialized = true;
}

void DroneNavigation::init_flight(bool hunt, std::ofstream *logger) {
    _logger = logger;
    (*_logger) << "nav_state_str;nav_flight_mode;insect_id;charging_state_str;charging_state;";

    _navigation_status = ns_takeoff;
    wpid = 0;
    time_start_reset_headless_yaw = 0;
    time_start_thrust_calibration = 0;
    time_start_wait_after_landing = -1;
    time_prev_wp_reached = -1;
    time_wp_reached = -1;
    time_start_landing = -1;
    time_landed = 0;
    time_drone_problem = -1;
    time_take_off = 0;
    time_out_after_landing = 8;
    low_battery_triggered = false;
    _flight_time = -1;

    setpoint_pos_world = {0};
    setpoint_pos_world_landing = {0};
    setpoint_vel_world = {0};
    setpoint_acc_world = {0};

    if (hunt) {
        _nav_flight_mode = nfm_hunt;
        _navigation_status = ns_takeoff;
        force_thrust_calib = false;
    } else {
        _nav_flight_mode = nfm_waypoint;
        _navigation_status = ns_takeoff;
    }
}

void DroneNavigation::update(double time) {
    switch (_navigation_status) {
        case ns_takeoff: {
                _control->reset_manual_override_take_off_now();
                _control->flight_mode(DroneController::fm_start_takeoff);
                _control->hover_mode(false);
                _control->nav_waypoint_moved(time);
                _tracker->hover_mode(false);
                time_take_off = time;
                _navigation_status = ns_taking_off;
                break;
        } case ns_taking_off: {
                if (_tracker->take_off_detection_failed()) {
                    _control->flight_mode(DroneController::fm_abort);
                    _navigation_status = ns_takeoff_failure;
                    break;
                } else if (!_tracker->taking_off())
                    _navigation_status = ns_take_off_completed;

                if (_iceptor->target_acquired(time) && _nav_flight_mode == nfm_hunt) {
                    setpoint_pos_world = _iceptor->aim_pos();
                    setpoint_pos_world = _flight_area->move_inside(setpoint_pos_world, relaxed, _tracker->pad_location(false));
                    setpoint_vel_world = _iceptor->aim_vel();
                    setpoint_acc_world = _iceptor->aim_acc();
                } else if (!_iceptor->target_acquired(time) && _nav_flight_mode == nfm_hunt && _tracker->drone_on_landing_pad()) {
                    if (_control->abort_take_off())
                        _navigation_status = ns_flight_done;
                }
                break;
        } case ns_take_off_completed: {
                _control->nav_waypoint_moved(time);
                if (_nav_flight_mode == nfm_hunt) {
                    _navigation_status = ns_start_the_chase;
                } else {
                    wpid++;
                    _navigation_status = ns_set_waypoint;
                    _control->LED(true);
                }
                time_prev_wp_reached = time;
                check_abort_autonomus_flight_conditions();
                break;
        } case ns_start_the_chase: {
                _control->hover_mode(false);
                _tracker->hover_mode(false);
                _control->flight_mode(DroneController::fm_flying_pid);
                _navigation_status = ns_chasing_insect_ff;
                [[fallthrough]];
        } case ns_chasing_insect_ff: {
                if (dparams.led_type != led_top_uv)
                    _control->LED(true);
                if (_control->ff_completed())
                    _navigation_status = ns_chasing_insect;
                check_abort_autonomus_flight_conditions();
                break;
        } case ns_chasing_insect: {
                setpoint_pos_world = _iceptor->aim_pos();
                setpoint_pos_world = _flight_area->move_inside(setpoint_pos_world, relaxed, _tracker->last_track_data().pos());
                setpoint_vel_world = _iceptor->aim_vel();
                setpoint_acc_world = _iceptor->aim_acc();

                if (_iceptor->target_cleared())
                    _navigation_status = ns_goto_yaw_waypoint;
                check_abort_autonomus_flight_conditions();
                break;
        } case ns_goto_yaw_waypoint: {
                _control->flight_mode(DroneController::fm_flying_pid);
                _control->LED(true);
                next_waypoint(Waypoint_Yaw_Reset(), time);
                _control->nav_waypoint_moved(time); // trigger wp movement. If we were hunting there was no previous wp, or if a flight abort was triggered the previous wp may not have been reached...
                _navigation_status = ns_approach_waypoint;
                break;
        } case ns_goto_thrust_calib_waypoint: {
                _control->flight_mode(DroneController::fm_headed);
                if (exec_thrust_calib()) {
                    next_waypoint(Waypoint_Thrust_Calibration(), time);
                    _navigation_status = ns_approach_waypoint;
                } else
                    _navigation_status = ns_goto_landing_waypoint;
                break;
        } case ns_goto_landing_waypoint: {
                _control->flight_mode(DroneController::fm_headed);
                next_waypoint(Waypoint_Landing(), time);
                _navigation_status = ns_approach_waypoint;
                break;
        } case ns_set_waypoint: {
                next_waypoint(waypoints[wpid], time);
                _navigation_status = ns_approach_waypoint;

                if (current_waypoint->mode == wfm_long_range)
                    _control->flight_mode(DroneController::fm_long_range_forth);
                else
                    _control->flight_mode(DroneController::fm_flying_pid);

                if (current_waypoint->mode != wfm_thrust_calib && current_waypoint->mode != wfm_yaw_reset && current_waypoint->mode != wfm_landing) {
                    _control->hover_mode(false);
                    _tracker->hover_mode(false);
                }

                time_prev_wp_reached = time;
                time_wp_reached = -1;
                break;
        } case ns_approach_waypoint: {
                if (current_waypoint->mode == wfm_long_range) {
                    if (static_cast<float>(time - time_prev_wp_reached) < static_cast<Waypoint_Long_Range *>(current_waypoint)->pitch_duration) {
                        _control->flight_mode(DroneController::fm_long_range_forth);
                    } else {
                        _control->flight_mode(DroneController::fm_long_range_back);
                        if (_tracker->world_item().valid && _tracker->world_item().distance < 6) {
                            wpid++;
                            _navigation_status = ns_set_waypoint;
                        }
                    }
                } else if (drone_at_wp()) {
                    if ((static_cast<float>(time - time_wp_reached) > current_waypoint->hover_pause && time_wp_reached > 0) || current_waypoint->hover_pause <= 0) {
                        if (current_waypoint->mode == wfm_landing) {
                            _navigation_status = ns_land;
                        } else if (current_waypoint->mode == wfm_yaw_reset) {
                            _navigation_status = ns_reset_headless_yaw;
                            time_start_reset_headless_yaw = time;
                        } else if (current_waypoint->mode == wfm_thrust_calib)
                            _navigation_status = ns_calibrate_thrust;
                        else if (wpid < waypoints.size()) { // next waypoint in flight plan
                            wpid++;
                            _navigation_status = ns_set_waypoint;
                        } else if (wpid == waypoints.size()) {
                            wpid = 0; // another round
                            _navigation_status = ns_set_waypoint;
                        }
                    } else if (time_wp_reached < 0) {
                        time_wp_reached = time;
                    }
                }
                if (drone_close_to_wp() && current_waypoint->threshold_mm <= hover_mode_wp_dist_threshold && current_waypoint->threshold_mm > 0) {
                    _control->hover_mode(true);
                    _tracker->hover_mode(true);
                }

                check_abort_autonomus_flight_conditions();
                break;
        } case ns_reset_headless_yaw: {
                _navigation_status = ns_resetting_headless_yaw;
                _control->flight_mode(DroneController::fm_reset_headless_yaw);
                [[fallthrough]];
        } case ns_resetting_headless_yaw: {
                if (drone_close_to_wp()) {
                    _control->hover_mode(true);
                    _tracker->hover_mode(true);
                }
                if (static_cast<float>(time - time_start_reset_headless_yaw) > duration_reset_headless_yaw)
                    _navigation_status = ns_correct_yaw;
                check_abort_autonomus_flight_conditions();
                break;
        } case ns_correct_yaw: {
                _tracker->detect_yaw(time);
                _control->flight_mode(DroneController::fm_correct_yaw);
                if (_tracker->bowl_nudge_needed(setpoint_pos_world))
                    _navigation_status = ns_bowling_nudge;
                else
                    _navigation_status = ns_correcting_yaw;
                [[fallthrough]];
        } case ns_bowling_nudge: {
                if (static_cast<float>(time - time_start_reset_headless_yaw) < duration_trigger_bowling) {
                    setpoint_pos_world = _tracker->last_track_data().pos() + cv::Point3f(0.2f, 0, 0);
                }
                else {
                    _navigation_status = ns_correcting_yaw;
                    setpoint_pos_world = _tracker->pad_location(true);
                    setpoint_pos_world += current_waypoint->xyz;
                }
                check_abort_autonomus_flight_conditions();
                break;
        } case ns_correcting_yaw: {
                if (!_tracker->check_yaw(time) || (static_cast<float>(time - time_start_reset_headless_yaw) > duration_correct_yaw && drone_at_wp()))
                    _navigation_status = ns_goto_thrust_calib_waypoint;
                check_abort_autonomus_flight_conditions();
                if (low_battery_triggered) {
                    std::cout << "Warning: skipping thrust calibration because battery low." << std::endl;
                    _navigation_status = ns_goto_landing_waypoint;
                }
                if (drone_close_to_wp()) {
                    _control->hover_mode(true);
                    _tracker->hover_mode(true);
                }
                break;
        } case ns_calibrate_thrust: {
                _control->init_thrust_calibration();
                _control->flight_mode(DroneController::fm_calib_thrust);
                time_start_thrust_calibration = time;
                _navigation_status = ns_calibrating_thrust;
                [[fallthrough]];
        } case ns_calibrating_thrust: {
                if (time - time_start_thrust_calibration > static_cast<double>(current_waypoint->hover_pause)) {
                    _control->save_thrust_calibration();
                    _navigation_status = ns_goto_landing_waypoint;
                }
                if (drone_close_to_wp()) {
                    _control->hover_mode(true);
                    _tracker->hover_mode(true);
                }
                check_abort_autonomus_flight_conditions();
                if (low_battery_triggered) {
                    std::cout << "Warning: skipping thrust calibration because battery low." << std::endl;
                    _navigation_status = ns_goto_landing_waypoint;
                }
                break;
        } case ns_land: {
                time_start_landing = time;
                _tracker->land();
                _control->flight_mode(DroneController::fm_prep_to_land);
                _navigation_status = ns_landing;
                [[fallthrough]];
        } case ns_landing: {
                float dt_land = static_cast<float>(time - time_start_landing);
                cv::Point3f pad_pos = _tracker->pad_location(true);
                cv::Point3f new_pos_setpoint = _control->land_ctrl.setpoint_cc_landing(pad_pos, current_waypoint, dt_land);
                if (drone_close_to_wp()) {
                    _control->hover_mode(true);
                    _tracker->hover_mode(true);
                }
                if (_control->landed())
                    _navigation_status = ns_landed;
                else if (!_control->landing() && _control->land_ctrl.switch_to_ff_landing(_tracker->last_track_data(), pad_pos))
                    _control->flight_mode(DroneController::fm_ff_landing_start);

                setpoint_pos_world = new_pos_setpoint;
                break;
        } case ns_landed: {
                wpid = 0;
                _navigation_status = ns_wait_after_landed;
                time_landed = time;
                _tracker->hover_mode(false);
                _tracker->delete_landing_motion(time_out_after_landing);
                _control->hover_mode(false);
                _control->kiv_ctrl.enable();
                _flight_time += static_cast<float>(time - time_take_off);
                [[fallthrough]];
        } case ns_wait_after_landed: {
                if (time_start_wait_after_landing < 0)
                    time_start_wait_after_landing = time;

                _control->flight_mode(DroneController::fm_wait);
                if (static_cast<float>(time - time_start_wait_after_landing) > duration_wait_after_landing && _control->new_attitude_package_available()) {
                    if (_control->drone_pad_state() == drone_on_pad) {
                        _navigation_status = ns_flight_done;
                        time_start_wait_after_landing = -1;
                    } else if (!_control->drone_pad_state())
                        _navigation_status = ns_landing_failure;
                }
                break;
        } case ns_flight_done: {
                break;
        } case ns_takeoff_failure:
        case ns_flight_failure:
        case ns_landing_failure: {
                if (time_drone_problem < 0)
                    time_drone_problem = time;
                if (_flight_time < 0)
                    _flight_time += static_cast<float>(time - time_take_off);
                _control->flight_mode(DroneController::fm_abort);
                _control->LED(static_cast<int>((time - time_drone_problem) * 10.0) % 10 > 5, 100); // blink every half second
                break;
            }
    }
    (*_logger) << navigation_status() << ";" << static_cast<uint16_t>(_nav_flight_mode)  << ";" << _iceptor->insect_id() << ";" << _baseboard_link->charging_state_str() << ";" << static_cast<uint16_t>(_baseboard_link->charging_state()) << ";";

}
void DroneNavigation::close() {
    if (initialized) {
        std::cout << "Closing drone navigation" << std::endl;
        initialized = false;
    }
}

bool DroneNavigation::drone_close_to_wp() {
    return (_control->dist_to_setpoint() < 0.1f
            && normf(_tracker->last_track_data().state.vel) < 0.5f
            && _tracker->properly_tracking());
}

bool DroneNavigation::drone_at_wp() {
    return (_control->dist_to_setpoint() * 1000 < current_waypoint->threshold_mm
            && normf(_tracker->last_track_data().state.vel) < current_waypoint->threshold_v
            && _tracker->properly_tracking());
}

void DroneNavigation::check_abort_autonomus_flight_conditions() {
    if (!_tracker->tracking())
        _navigation_status = ns_flight_failure;
    if (_control->telemetry().batt_cell_v > 2 && _control->telemetry().batt_cell_v  < dparams.land_cell_v && !low_battery_triggered) {
        if (current_waypoint->mode != wfm_landing && current_waypoint->mode != wfm_thrust_calib && current_waypoint->mode != wfm_yaw_reset)
            _navigation_status = ns_goto_yaw_waypoint;
        low_battery_triggered = true;
    }

}

void DroneNavigation::next_waypoint(Waypoint wp, double time) {
    float dist_to_new_wp = normf(wp.xyz - current_waypoint->xyz);
    delete current_waypoint;
    current_waypoint = new Waypoint(wp);
    if (wp.mode == wfm_takeoff) {
        cv::Point3f p = _tracker->pad_location(true);
        setpoint_pos_world =  p + wp.xyz;
        setpoint_pos_world = _flight_area->move_inside(setpoint_pos_world, relaxed);
    } else if (wp.mode == wfm_landing || wp.mode == wfm_yaw_reset || wp.mode == wfm_thrust_calib) {
        _control->kiv_ctrl.disable();
        cv::Point3f p = _tracker->pad_location(true);
        setpoint_pos_world =  p + wp.xyz;
        if (setpoint_pos_world.y > -0.5f) // keep some margin to the top of the view, because atm we have an overshoot problem.
            setpoint_pos_world.y = -0.5f;

        setpoint_pos_world.x += calibration_offset(wp); // don't calibrate in between camera and pad if drone is not calibrated

        setpoint_pos_world_landing = setpoint_pos_world;
    } else {
        setpoint_pos_world =  wp.xyz;
        setpoint_pos_world = _flight_area->move_inside(setpoint_pos_world, relaxed);
    }
    if (dist_to_new_wp > 0.1f)
        _control->nav_waypoint_moved(time);

    setpoint_vel_world = {0, 0, 0};
    setpoint_acc_world = {0, 0, 0};
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
            throw std::runtime_error("Cannot read: " + settings_file);
        }
        navigationParameters tmp;
        auto v1 = params.getVersion();
        auto v2 = tmp.getVersion();
        if (v1 != v2) {
            throw std::runtime_error("XML version difference detected from " + settings_file);
        }
        infile.close();
    } else {
        throw std::runtime_error("File not found: " + settings_file);
    }

    time_out_after_landing = params.time_out_after_landing.value();
}

void DroneNavigation::serialize_settings() {
    navigationParameters params;
    params.time_out_after_landing = time_out_after_landing;

    std::string xmlData = params.toXML();
    std::ofstream outfile = std::ofstream(settings_file);
    outfile << xmlData ;
    outfile.close();
}


void DroneNavigation::flightplan(std::string flightplan_fn) {
    navigation::XML_FlightPlan fp;
    fp.deserialize(flightplan_fn);
    fp.serialize(data_output_dir + "flightplan.xml"); // write a copy of the currently used flightplan to the logging dir
    waypoints = fp.waypoints();
    if (!pparams.long_range_mode)
        for (auto w : waypoints) {
            if (w.mode == wfm_long_range)
                throw std::runtime_error("Long range flightplan needs long_range_mode in pats xml enabled!");
        }
    wpid = 0;
    next_waypoint(waypoints[wpid], _tracker->last_track_data().time);
    if (!strcmp(fp.flightplan_name.c_str(), "thrust-calibration"))
        force_thrust_calib = true;
    else
        force_thrust_calib = false;
}


float DroneNavigation::calibration_offset(Waypoint wp) {
    if ((wp.mode == wfm_thrust_calib || wp.mode == wfm_yaw_reset) && exec_thrust_calib()) {

        // search the side which is closer to the drone to prevent the drone passing the pad if thrust calibration isn't done yet (potential crash).
        float candidate_sign = 1.f;
        if (normf(setpoint_pos_world + cv::Point3f(0.5, 0, 0) - _tracker->last_track_data().state.pos)
                > normf(setpoint_pos_world + cv::Point3f(-0.5, 0, 0) - _tracker->last_track_data().state.pos))
            candidate_sign = -1.f;

        if (_flight_area->inside(setpoint_pos_world + candidate_sign * cv::Point3f(0.5, 0, 0), relaxed))
            return candidate_sign * 0.5f;
        else if (_flight_area->inside(setpoint_pos_world + candidate_sign * cv::Point3f(-0.5, 0, 0), relaxed))
            return candidate_sign * -0.5f;
    }

    return 0;
}
}
