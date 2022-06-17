#include "interceptor.h"
#include "opencv2/imgproc.hpp"

using namespace tracking;

void Interceptor::init(tracking::TrackerManager *trackers, VisionData *visdat, FlightArea *flight_area, Drone *drone) {
    _trackers = trackers;
    _visdat = visdat;
    _flight_area = flight_area;
    _drone = drone;
    interception_max_thrust = *drone->control.max_thrust();

    n_frames_target_cleared_timeout = pparams.fps * 1.f;

    tti_optimizer.init(&interception_max_thrust);
    intercept_in_planes_optimizer.init(&interception_max_thrust, _flight_area, relaxed);

    initialized = true;
}

void Interceptor::init_flight(std::ofstream *logger) {
    (*logger) << "tti;tti_iip;aimx;aimy;aimz;accx_iip;accy_iip;accz_iip;";
}

void Interceptor::log(std::ostream *logger) {
    (*logger) << _tti << ";" << _tti_iip << ";" << _aim_pos.x << ";" << _aim_pos.y << ";" << _aim_pos.z  << ";" << _aim_acc.x << ";" << _aim_acc.y << ";" << _aim_acc.z << ";";
}

void Interceptor::update(bool drone_at_base, double time[[maybe_unused]]) {
    _tti = -1;
    _tti_iip = -1;
    aim_in_flightarea = false;
    _aim_acc = cv::Point3f(0, 0, 0);
    _control_mode = position_control;
    interception_max_thrust = *_drone->control.max_thrust();
    auto target_trkr = update_target_insecttracker();

    switch (_interceptor_state) {
        case  is_init: {
                _interceptor_state = is_waiting_for_target;
                _aim_pos = _flight_area->move_inside(cv::Point3f(0, 0, 0), strict);
                FlightAreaConfig *relaxed_flightareaconfig = _flight_area->flight_area_config(relaxed);
                interception_center = cv::Point3f(0, _drone->tracker.pad_location().y / 2, _drone->tracker.pad_location().z + (relaxed_flightareaconfig->active_back_plane().support.z - _drone->tracker.pad_location().z) / 2);
                _tti = -1;
                [[fallthrough]];
        } case is_waiting_for_target: {
                _n_frames_aim_not_in_range++;

                if (!target_trkr)
                    break;
                else  if (target_trkr->tracking() && !target_trkr->false_positive() && !_trackers->monster_alert() && target_trkr->go_for_terminate()) {
                    _interceptor_state = is_waiting_in_reach_zone;
                } else
                    break;

                [[fallthrough]];
        } case is_waiting_in_reach_zone: {
                if (!target_trkr) {
                    _interceptor_state = is_waiting_for_target;
                    break;
                } if (!target_trkr->tracking() || target_trkr->false_positive() || _trackers->monster_alert() || !target_trkr->go_for_terminate()) {
                    _interceptor_state = is_waiting_for_target;
                    break;
                }

                update_aim_and_target_in_flightarea(drone_at_base, target_trkr->last_track_data());
                if (!_n_frames_aim_not_in_range)
                    _interceptor_state = is_lurking;
                else
                    break;

                [[fallthrough]];

        } case is_lurking: {
                if (!target_trkr) {
                    _interceptor_state = is_waiting_for_target;
                    break;
                } if (!target_trkr->tracking() || target_trkr->false_positive() || _trackers->monster_alert() || !target_trkr->go_for_terminate()) {
                    _interceptor_state = is_waiting_for_target;
                    break;
                }

                update_aim_and_target_in_flightarea(drone_at_base, target_trkr->last_track_data());
                if ((!delay_takeoff_for_better_interception(target_trkr)) && !_n_frames_aim_not_in_range)
                    _interceptor_state = is_intercepting;
                else
                    break;
                [[fallthrough]];

        } case is_intercepting: {
                if (!target_trkr) {
                    _interceptor_state = is_waiting_for_target;
                    break;
                }
                if (target_trkr->n_frames_lost() > 0.112 * pparams.fps
                        || _n_frames_aim_not_in_range > 0.34 * pparams.fps
                        || target_trkr->false_positive()
                        || _trackers->monster_alert()) {
                    _interceptor_state = is_waiting_for_target;
                    break;
                }


                if (!target_trkr->n_frames_lost()) {
                    update_hunt_strategy(drone_at_base, target_trkr->last_track_data(), time);
                }
                break;

            }
    }
}

void Interceptor::update_aim_in_flightarea(tti_result tti_res) {
    if (tti_res.valid) {
        _tti = tti_res.time_to_intercept;
        if (_flight_area->inside(tti_res.position_to_intercept, relaxed))
            aim_in_flightarea = true;
    }

    if (aim_in_flightarea) {
        _aim_pos = tti_res.position_to_intercept;
        _n_frames_aim_not_in_range = 0;
    } else
        _n_frames_aim_not_in_range++;
}

void Interceptor::update_aim_and_target_in_flightarea(bool drone_at_base, tracking::TrackData target) {
    TrackData drone = _drone->tracker.last_track_data();

    if (drone_at_base || normf(drone.pos()) < 0.01f) {
        drone.state.pos = _drone->tracker.pad_location();
        drone.state.vel = cv::Point3f(0, 0, 0);
        interception_max_thrust = 2.f * (*_drone->control.max_thrust()); //Try to compensate ground effect
    }

    auto tti_res = tti_optimizer.find_best_interception(drone, target);
    update_aim_in_flightarea(tti_res);

    target_in_flightarea = _flight_area->inside(target.pos(), relaxed);

}

void Interceptor::update_hunt_distance(bool drone_at_base, cv::Point3f drone_pos, cv::Point3f target_pos) {

    hunt_error = normf(target_pos - drone_pos);
    if (hunt_error < _best_hunt_error && !drone_at_base)
        _best_hunt_error = hunt_error;
}

void Interceptor::update_hunt_strategy(bool drone_at_base, tracking::TrackData target, double time) {

    std::chrono::_V2::system_clock::time_point t_start, t_now;
    t_start = std::chrono::high_resolution_clock::now();

    switch (_intercepting_state) {
        case is_approaching: {

                TrackData drone = _drone->tracker.last_track_data();
                if (drone_at_base || normf(drone.pos()) < 0.1f) {
                    drone.state.pos = _drone->tracker.pad_location();
                    drone.state.vel = cv::Point3f(0, 0, 0);
                }

                update_hunt_distance(drone_at_base, drone.pos(), target.pos());
                update_aim_and_target_in_flightarea(drone_at_base, target);

                if (!aim_in_flightarea) {
                    // Insect is currently not interceptable. Try to go directly to the target (and hope is target changing its path)
                    _aim_pos = target.pos() + 0.5 * target.vel(); //+ 0.4 * (target.pos() - drone.pos()) * hunt_error;
                    _control_mode = position_control;
                    return;
                } else {
                    _aim_pos += 0.4f * (_aim_pos - drone.pos()) / normf(_aim_pos - drone.pos());
                    _control_mode = position_control;

                    t_now = std::chrono::high_resolution_clock::now();
                    double time_passed = std::chrono::duration_cast<std::chrono::microseconds>(t_now - t_start).count() * 1e-6;
                    double remaining_time = optimization_time - time_passed;
                    if (remaining_time > 0.002) { // see iip timing statistics in interceptinplanes.cpp run
                        intercept_in_planes_optimizer.max_cpu_time(remaining_time);
                        auto res = intercept_in_planes_optimizer.find_best_interception(drone, target);

                        if (res.valid) {
                            _tti_iip = res.time_to_intercept;
                            _control_mode = acceleration_feedforward;
                            _aim_acc = res.acceleration_to_intercept;
                            _aim_pos = res.position_to_intercept;
                        }
                    }
                }

                if (hunt_error < static_cast<float>(dparams.drone_rotation_delay) * normf(drone.vel())) {
                    _intercepting_state = is_intercept_maneuvering;
                    time_start_intercept_maneuver = time;
                    _aim_pos += 0.4f * (_aim_pos - drone.pos()) / normf(_aim_pos - drone.pos());
                    _control_mode = position_control;
                } else {
                    break;
                }
                [[fallthrough]];
        } case is_intercept_maneuvering: {
                if (norm(time - time_start_intercept_maneuver) > duration_intercept_maneuver)
                    _intercepting_state = is_approaching;
                break;
            }
    }
}



tracking::InsectTracker *Interceptor::update_target_insecttracker() {
    float best_acceleration = INFINITY;
    InsectTracker *best_itrkr = NULL;
    auto all_trackers = _trackers->all_target_trackers();

    TrackData tracking_data = _drone->tracker.last_track_data();
    if (_drone->tracker.drone_on_landing_pad()) {
        //Decision could be made when drone hasn't taken off yet
        tracking_data.pos_valid = true;
        tracking_data.state.pos = _drone->tracker.pad_location();
        tracking_data.state.spos = _drone->tracker.pad_location();
    }
    for (auto trkr : all_trackers) {
        if (trkr->tracking()) {
            auto insect_state = trkr->last_track_data();
            cv::Point3f current_insect_pos = insect_state.pos();
            if (!insect_state.pos_valid) {
                auto prediction = trkr->image_predict_item();
                if (prediction.valid)
                    current_insect_pos = im2world(prediction.pt_unbound, prediction.disparity, _visdat->Qf, _visdat->camera_roll(), _visdat->camera_pitch());
                else
                    current_insect_pos = {0};
            }
            cv::Point3f current_insect_vel = insect_state.vel();

            if (!insect_state.vel_valid)
                current_insect_vel = {0};
            if ((trkr->type() == tt_insect && !pparams.disable_real_hunts) || trkr->type() == tt_replay || trkr->type() == tt_virtualmoth) {
                float req_acceleration = normf(_drone->control.pid_error(tracking_data, current_insect_pos, current_insect_vel, true));
                if (best_acceleration > req_acceleration) {
                    best_acceleration = req_acceleration;
                    best_itrkr = static_cast<InsectTracker *>(trkr);
                }
            }
        }
    }
    _target_insecttracker = best_itrkr;
    return best_itrkr;
}


bool Interceptor::delay_takeoff_for_better_interception(tracking::InsectTracker *target_tracker) {
    tracking::TrackData insect = target_tracker->last_track_data();
    cv::Point3f lurk_distance = interception_center - insect.pos();
    bool interceptability_is_improving = (lurk_distance).dot(target_tracker->last_track_data().vel()) / normf(lurk_distance) / normf(target_tracker->last_track_data().vel()) > 0.5f;

    cv::Point3f prediceted_insect_position = insect.pos() + 0.3f * insect.vel();
    lurk_distance = interception_center - prediceted_insect_position;
    bool interceptability_will_improve = (lurk_distance).dot(target_tracker->last_track_data().vel()) / normf(lurk_distance) / normf(target_tracker->last_track_data().vel()) > 0.5f;

    return interceptability_is_improving && interceptability_will_improve;
}

tracking::TrackData Interceptor::target_last_trackdata() {
    if (target_insecttracker())
        return target_insecttracker()->last_track_data();
    else {
        TrackData d;
        return d;
    }
}
