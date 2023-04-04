#include "interceptor.h"
#include "opencv2/imgproc.hpp"

using namespace tracking;

void Interceptor::init(tracking::TrackerManager *trackers, VisionData *visdat, FlightArea *flight_area, Drone *drone, FlightAreaConfig *flight_area_config) {
    _trackers = trackers;
    _visdat = visdat;
    _flight_area = flight_area;
    _flight_area_config = flight_area_config;
    _drone = drone;
    interception_max_thrust = *drone->control.max_thrust();

    n_frames_target_cleared_timeout = pparams.fps * 1.f;

    // tti_optimizer.init(&interception_max_thrust);
    rapid_route.init(&interception_max_thrust, 0.8f, flight_area_config);
#ifdef OCP_DEV
    // tti_optimizer.init_casadi("../ocp_design/tti/tti_optimizer.so");
    // intercept_in_planes_optimizer.init_casadi("../ocp_design/intercept_in_planes/intercept_in_planes_optimizer.so");
#endif
    // intercept_in_planes_optimizer.init(&interception_max_thrust, _flight_area, relaxed);
    // iip_thread = std::thread(&Interceptor::iip_worker, this);

    initialized = true;
}

void Interceptor::init_flight(std::ofstream *logger) {
    (*logger) << "posX_bestinsect;posY_bestinsect;posZ_bestinsect;velX_bestinsect;velY_bestinsect;velZ_bestinsect;tti;posX_aim;posY_aim;posZ_aim;tti_iip;accX_iip;accY_iip;accZ_iip;";
}

void Interceptor::log(std::ostream *logger) {
    if (_target_insecttracker)
        (*logger) <<
                  _target_insecttracker->last_track_data().pos().x << ";" <<
                  _target_insecttracker->last_track_data().pos().y << ";" <<
                  _target_insecttracker->last_track_data().pos().z << ";" <<
                  _target_insecttracker->last_track_data().vel().x << ";" <<
                  _target_insecttracker->last_track_data().vel().y << ";" <<
                  _target_insecttracker->last_track_data().vel().z << ";";
    else
        (*logger) << - 1 << ";" <<
                  -1 << ";" <<
                  -1 << ";" <<
                  -1 << ";" <<
                  -1 << ";" <<
                  -1 << ";";
    (*logger) << _tti << ";" <<
              _aim_pos.x << ";" <<
              _aim_pos.y << ";" <<
              _aim_pos.z  << ";" <<
              _tti_iip << ";" <<
              _aim_acc.x << ";" <<
              _aim_acc.y << ";" <<
              _aim_acc.z << ";";
}

void Interceptor::close() {
    iip_thread_exit = true;
    // cond_var_iip.notify_one();
    if (initialized) {
        std::cout << "Closing Interceptor" << std::endl;
        // iip_thread.join();

    }
}

intercept_in_planes_result Interceptor::update_iip_thread(float cpu_time) {
    if (iip_thread_finished) {
        time_prev_iip_started = time_iip_started;
        iip_thread_ready = true;
        iip_thread_finished = false;
    }

    if (iip_thread_ready && !iip_thread_exit) {
        time_iip_started = _time;
        intercept_in_planes_optimizer.max_cpu_time(static_cast<double>(cpu_time));
        iip_thread_ready = false;
        cond_var_iip.notify_one();
    }

    intercept_in_planes_result res = intercept_in_planes_result();
    if (static_cast<float>(_time - time_prev_iip_started) < delay_iip_valid && time_prev_iip_started > 0) {
        std::lock_guard lk(_mutex);
        res = iip_res;
    }
    return res;
}

void Interceptor::update(bool drone_at_base, double time[[maybe_unused]]) {
#ifdef PATS_PROFILING
    std::chrono::_V2::system_clock::time_point t_start_interceptor = std::chrono::high_resolution_clock::now();
#endif
    _time = time;
    _tti = -1;
    _tti_iip = -1;
    interception_position_in_flightarea = false;
    stopping_position_in_flightarea = false;
    _aim_acc = cv::Point3f(0, 0, 0);
    _control_mode = position_control;
    auto target_trkr = update_target_insecttracker();

    interception_max_thrust = *_drone->control.max_thrust();
    if (!_drone->nav.drone_hunting()) {
        interception_max_thrust = 2 * (*_drone->control.max_thrust()); // COmpensate ground effect
    }

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
                if (_drone->in_flight())
                    update_hunt_strategy(drone_at_base, target_trkr->last_track_data(), time);
                else
                    _rapid_route_result = update_aim_and_target_in_flightarea(drone_at_base, target_trkr->last_track_data(), 0.5f);
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

                if ((!delay_takeoff_for_better_interception()) && !_n_frames_aim_not_in_range) {
                    _interceptor_state = is_intercepting;
                }
                else
                    break;
                [[fallthrough]];

        } case is_intercepting: {
                if (_intercepting_state == is_intercept_maneuvering && exit_is_intercept_maneuvering(time)) {
                    _intercepting_state = is_approaching;
                    _interceptor_state = is_waiting_for_target;
                    break;
                }
                if (_intercepting_state == is_approaching && !target_trkr) {
                    _interceptor_state = is_waiting_for_target;
                    break;
                } else if (_intercepting_state != is_intercept_maneuvering && (target_trkr->n_frames_lost() > 0.112 * pparams.fps
                           || _n_frames_aim_not_in_range > 0.34 * pparams.fps
                           || target_trkr->false_positive()
                           || _trackers->monster_alert())) {
                    enter_is_intercept_maneuvering(time, _drone->tracker.last_track_data());
                    break;
                }

                if (_intercepting_state == is_approaching)
                    update_hunt_strategy(drone_at_base, target_trkr->last_track_data(), time);
                break;

            }
    }

    if (!_drone->nav.drone_hunting()) { //Correct too high thrust due to ground compensation (see beginning of this method)
        _aim_acc /= 2;
    }
#ifdef PATS_PROFILING
    std::chrono::_V2::system_clock::time_point t_end_interceptor = std::chrono::high_resolution_clock::now();
    std::cout << "timing (update_interceptor): " << (t_end_interceptor - t_start_interceptor).count() * 1e-6 << "ms" << std::endl;
#endif
}

void Interceptor::update_aim_in_flightarea(rapid_route_result rapid_route_res) {
    if (rapid_route_res.valid) {
        _tti = rapid_route_res.time_to_intercept;
        if (_flight_area->inside(rapid_route_res.position_to_intercept, relaxed))
            interception_position_in_flightarea = true;
        else
            interception_position_in_flightarea = false;
        if (_flight_area->inside(rapid_route_res.stopping_position, relaxed))
            stopping_position_in_flightarea = true;
        else
            stopping_position_in_flightarea = false;
    }

    if (interception_position_in_flightarea && stopping_position_in_flightarea) {
        _aim_pos = rapid_route_res.position_to_intercept;
        _n_frames_aim_not_in_range = 0;
    } else
        _n_frames_aim_not_in_range++;
}

rapid_route_result Interceptor::update_aim_and_target_in_flightarea(bool drone_at_base, tracking::TrackData target, float delay) {
    TrackData drone = _drone->tracker.last_track_data();

    if (drone_at_base || normf(drone.pos()) < 0.01f) {
        drone.state.pos = _drone->tracker.pad_location();
        drone.state.vel = cv::Point3f(0, 0, 0);
        interception_max_thrust = 2.f * (*_drone->control.max_thrust()); //Try to compensate ground effect
    }

    // auto tti_res = tti_optimizer.find_best_interception(drone, target);
    _rapid_route_result = rapid_route.find_interception_direct(drone, target, delay, _drone->control.kiv_ctrl.safety);
    update_aim_in_flightarea(_rapid_route_result);
    if (_n_frames_aim_not_in_range) {
        _rapid_route_result = rapid_route.find_interception_via(_drone->tracker.last_track_data(), target, delay, _drone->control.kiv_ctrl.safety);
        _aim_pos = _rapid_route_result.position_to_intercept;
        update_aim_in_flightarea(_rapid_route_result);
    }
    return _rapid_route_result;
}

void Interceptor::update_hunt_distance(bool drone_at_base, cv::Point3f drone_pos, cv::Point3f target_pos, double time) {

    hunt_error = normf(target_pos - drone_pos);
    if (hunt_error < _best_hunt_error && !drone_at_base) {
        _best_hunt_error = hunt_error;
        _time_best_hunt_error = time - _drone->nav.takeoff_time();
        _pos_best_hunt_error = _drone->tracker.last_track_data().pos();
        _vel_best_hunt_error = _drone->tracker.last_track_data().vel();
        _acc_best_hunt_error = _drone->tracker.last_track_data().acc();
    }
}

void Interceptor::update_hunt_strategy(bool drone_at_base, tracking::TrackData target, double time) {

    switch (_intercepting_state) {
        case is_approaching: {

                TrackData drone = _drone->tracker.last_track_data();
                if (drone_at_base || normf(drone.pos()) < 0.1f) {
                    drone.state.pos = _drone->tracker.pad_location();
                    drone.state.vel = cv::Point3f(0, 0, 0);
                }

                update_hunt_distance(drone_at_base, drone.pos(), target.pos(), time);
                update_aim_and_target_in_flightarea(drone_at_base, target, 0.f);

                if (!interception_position_in_flightarea || !stopping_position_in_flightarea) {
                    // Insect is currently not interceptable. Try to go directly to the target (and hope is target changing its path)
                    _aim_pos = target.pos() + _tti * target.vel(); //+ 0.4 * (target.pos() - drone.pos()) * hunt_error;
                    _aim_pos = _flight_area->move_inside(_aim_pos, strict, drone.pos());
                    _control_mode = position_control;
                    return;
                } else {
                    _aim_pos = _flight_area->move_inside(_aim_pos, relaxed, drone.pos());
                    _aim_pos += 0.4f * (_aim_pos - drone.pos()) / normf(_aim_pos - drone.pos());
                    _control_mode = position_control;

                    // auto res = update_iip_thread(delay_iip_valid);

                    // if (res.valid) {
                    //     _tti_iip = res.time_to_intercept;
                    //     _control_mode = acceleration_feedforward;
                    //     _aim_acc = res.acceleration_to_intercept;
                    //     _aim_pos = res.position_to_intercept;
                    // }
                }

                if (hunt_error < static_cast<float>(dparams.drone_rotation_delay) * normf(drone.vel())) {
                    enter_is_intercept_maneuvering(time, drone);
                } else {
                    break;
                }
                [[fallthrough]];
        } case is_intercept_maneuvering: {
                if (exit_is_intercept_maneuvering(time))
                    _intercepting_state = is_approaching;
                break;
            }
    }
}



tracking::InsectTracker *Interceptor::update_target_insecttracker() {
    float best_time_to_intercept = INFINITY;
    bool best_aim_inview = false;
    bool best_stop_inview = false;
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
            rapid_route_result _optim_result = rapid_route.find_interception_direct(tracking_data, insect_state, 0.f, _drone->control.kiv_ctrl.safety);
            if (_optim_result.valid) {
                cv::Point3f aim = _optim_result.position_to_intercept;
                bool aim_inview = _flight_area->inside(aim, bare);
                bool stop_inview = _flight_area->inside(_optim_result.stopping_position, bare);

                if (!insect_state.vel_valid)
                    current_insect_vel = {0};
                if ((trkr->type() == tt_insect && !pparams.disable_real_hunts) || trkr->type() == tt_replay || trkr->type() == tt_virtualmoth) {
                    if (aim_inview > best_aim_inview || (stop_inview > best_stop_inview && aim_inview == best_aim_inview)) {
                        best_time_to_intercept = _optim_result.time_to_intercept;
                        best_aim_inview = aim_inview;
                        best_stop_inview = stop_inview;
                        best_itrkr = static_cast<InsectTracker *>(trkr);
                    } else if (best_time_to_intercept > _optim_result.time_to_intercept && aim_inview == best_aim_inview && stop_inview == best_stop_inview) {
                        best_time_to_intercept = _optim_result.time_to_intercept;
                        best_aim_inview = aim_inview;
                        best_stop_inview = stop_inview;
                        best_itrkr = static_cast<InsectTracker *>(trkr);
                    }
                }
            }
        }
    }
    _target_insecttracker = best_itrkr;
    return best_itrkr;
}


bool Interceptor::delay_takeoff_for_better_interception() {
    return false;
}

tracking::TrackData Interceptor::target_last_trackdata() {
    if (target_insecttracker())
        return target_insecttracker()->last_track_data();
    else {
        TrackData d;
        return d;
    }
}

bool Interceptor::insect_in_pad_area() {
    for (auto drone_trkr : _trackers->dronetrackers()) {
        for (auto insect_trkr : _trackers->insecttrackers()) {
            if (insect_trkr->tracking() && insect_trkr->image_item().valid) {
                cv::Point2f insect_direction_from_pad = insect_trkr->image_item().pt() - drone_trkr->pad_im_location();
                if (normf(insect_direction_from_pad) < drone_trkr->pad_im_size() * 1.5f) {
                    float measured_versus_predicted_angle_diff = acosf((drone_trkr->takeoff_direction_predicted().dot(insect_direction_from_pad)) / (normf(drone_trkr->takeoff_direction_predicted()) * normf(insect_direction_from_pad)));
                    if (measured_versus_predicted_angle_diff < M_PIf32 / 4.f)
                        return true;
                }
            }
        }
    }
    return false;
}