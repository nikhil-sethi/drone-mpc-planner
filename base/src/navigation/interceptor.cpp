#include "interceptor.h"
#include "opencv2/imgproc.hpp"
#include "flightplan.h"

using namespace tracking;

void Interceptor::init(tracking::TrackerManager *trackers, VisionData *visdat, FlightArea *flight_area, Drone *drone, safety_margin_types safety_margin_type) {
    _trackers = trackers;
    _flight_area = flight_area;
    _visdat = visdat;
    _drone = drone;
    interception_max_thrust = *drone->control.max_thrust();
    n_frames_target_cleared_timeout = pparams.fps * 1.f;
    trajectory_optimizer.init(&interception_max_thrust, 0.7f, flight_area, safety_margin_type, drone->control.transmission_delay());
    initialized = true;
}

void Interceptor::init_flight(std::ofstream *logger) {
    (*logger) << "posX_bestinsect;posY_bestinsect;posZ_bestinsect;velX_bestinsect;velY_bestinsect;velZ_bestinsect;tti;posX_aim;posY_aim;posZ_aim;accX;accY;accZ;";
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
              _aim_acc.x << ";" <<
              _aim_acc.y << ";" <<
              _aim_acc.z << ";";
}

void Interceptor::update(double time[[maybe_unused]]) {
#ifdef PATS_PROFILING
    std::chrono::_V2::system_clock::time_point t_start_interceptor = std::chrono::high_resolution_clock::now();
#endif
    _time = time;
    _tti = -1;
    //_aim_pos = _flight_area->move_inside(_drone->tracker.pad_location() + navigation::Waypoint_Thrust_Calibration().xyz, strict);
    _aim_pos = _flight_area->move_inside(cv::Point3f(0, 0, 0), strict);
    _aim_vel = cv::Point3f(0, 0, 0);
    _aim_acc = cv::Point3f(0, 0, 0);
    float _delay = _drone->control.transmission_delay();
    if (!_drone->in_flight() && _drone->control.spinup())
        _delay += _drone->control.remaining_spinup_duration() + _drone->control.takeoff_delay();
    choose_target_and_find_interception(_delay); // returns NULL tracker if no target is found
    interception_max_thrust = *_drone->control.max_thrust();
    switch (_interceptor_state) {
        case  is_init: {
                _interceptor_state = is_waiting_for_target;
                FlightAreaConfig *relaxed_flightareaconfig = _flight_area->flight_area_config(relaxed);
                interception_center = cv::Point3f(0, _drone->tracker.pad_location().y / 2, _drone->tracker.pad_location().z + (relaxed_flightareaconfig->active_back_plane().support.z - _drone->tracker.pad_location().z) / 2);
                [[fallthrough]];
        } case is_waiting_for_target: {
                _n_frames_aim_in_range = 0;
                _n_frames_aim_not_in_range++;
                if (!_target_insecttracker)
                    break;
                else  if (_target_insecttracker->tracking() && !_target_insecttracker->false_positive() && !_trackers->monster_alert() && _target_insecttracker->go_for_terminate()) {
                    _interceptor_state = is_lurking;
                } else
                    break;
                [[fallthrough]];
        } case is_lurking: {
                _control_mode = position_control;
                if (!_target_insecttracker || !_target_insecttracker->tracking() || _target_insecttracker->false_positive() || _trackers->monster_alert() || !_target_insecttracker->go_for_terminate()) {
                    _interceptor_state = is_waiting_for_target;
                    break;
                }
                check_if_aim_in_flightarea();
                if ((!delay_takeoff_for_better_interception()) && !_n_frames_aim_not_in_range) {
                    _interceptor_state = is_intercepting;
                } else
                    break;
                [[fallthrough]];
        } case is_intercepting: {
                if (_hunt_strategy_state == is_intercept_maneuvering && exit_is_intercept_maneuvering(time)) {
                    _hunt_strategy_state = is_approaching;
                    _interceptor_state = is_waiting_for_target;
                    break;
                }
                if (_hunt_strategy_state == is_approaching && !_target_insecttracker) {
                    _interceptor_state = is_waiting_for_target;
                    break;
                }

                if (_target_insecttracker) {
                    if (_hunt_strategy_state == is_approaching) {
                        check_if_aim_in_flightarea();
                        update_hunt_strategy(_target_insecttracker->last_track_data(), time);
                    }
                    if (_drone->nav.drone_hunting() && _drone->tracker.last_track_data().pos_valid && _target_insecttracker->last_track_data().pos_valid)
                        update_hunt_distance(_drone->tracker.last_track_data().pos(), _target_insecttracker->last_track_data().pos(), time);
                }
                break;
            }
    }
#ifdef PATS_PROFILING
    std::chrono::_V2::system_clock::time_point t_end_interceptor = std::chrono::high_resolution_clock::now();
    std::cout << "timing (update_interceptor): " << (t_end_interceptor - t_start_interceptor).count() * 1e-6 << "ms" << std::endl;
#endif
}

void Interceptor::check_if_aim_in_flightarea() {
    if (_optimization_result.via && _optimization_result.interception_position_in_flightarea && _optimization_result.stopping_position_in_flightarea) {
        if (normf(_optimization_result.intermediate_position - _drone->tracker.last_track_data().pos()) > 0.25f)
            _aim_pos = _drone->tracker.last_track_data().pos() + 0.5f * (_optimization_result.intermediate_position - _drone->tracker.last_track_data().pos()) / normf(_optimization_result.intermediate_position - _drone->tracker.last_track_data().pos());
        else
            _aim_pos = _optimization_result.intermediate_position;
        _n_frames_aim_not_in_range = 0;
        _n_frames_aim_in_range++;
    } else if (!_optimization_result.via && _optimization_result.interception_position_in_flightarea && _optimization_result.stopping_position_in_flightarea) {
        _aim_pos = _optimization_result.position_to_intercept;
        _aim_vel =  _drone->tracker.last_track_data().vel() + _optimization_result.acceleration_to_intercept * 1.f / static_cast<float>(pparams.fps);
        _aim_acc = _optimization_result.acceleration_to_intercept + cv::Point3f(0, -GRAVITY, 0);
        _n_frames_aim_not_in_range = 0;
        _n_frames_aim_in_range++;
    } else {
        _n_frames_aim_not_in_range++;
        _n_frames_aim_in_range = 0;
    }
}

void Interceptor::update_hunt_distance(cv::Point3f drone_pos, cv::Point3f target_pos, double time) {
    hunt_error = normf(target_pos - drone_pos);
    if (hunt_error < _best_hunt_error && !_drone->control.at_base()) {
        _best_hunt_error = hunt_error;
        _time_best_hunt_error = time - _drone->nav.takeoff_time();
        _pos_best_hunt_error = _drone->tracker.last_track_data().pos();
        _vel_best_hunt_error = _drone->tracker.last_track_data().vel();
        _acc_best_hunt_error = _drone->tracker.last_track_data().acc();
    }
}

void Interceptor::update_hunt_strategy(tracking::TrackData target, double time) {
    switch (_hunt_strategy_state) {
        case is_approaching: {
                TrackData drone = _drone->tracker.last_track_data();
                if (_drone->control.at_base()) {
                    drone.state.pos = _drone->tracker.pad_location();
                    drone.state.vel = cv::Point3f(0, 0, 0);
                    _control_mode = position_control;
                }
                if (_optimization_result.via && _optimization_result.interception_position_in_flightarea && _optimization_result.valid) {
                    _aim_pos = _flight_area->move_inside(_aim_pos, relaxed, drone.pos());
                    // std::cout << "AIM 1: " << _aim_pos << std::endl;
                    _control_mode = position_control;
                } else if (!_optimization_result.via && _optimization_result.interception_position_in_flightarea && _optimization_result.stopping_position_in_flightarea && _optimization_result.valid) {
                    _control_mode = acceleration_control;
                } else {
                    // Insect is currently not interceptable. Try to go in front of the target (and hope is target changing its path)
                    _aim_pos = target.pos(); // + 3 * _tti * target.vel(); //+ 0.4 * (target.pos() - drone.pos()) * hunt_error; //@rik, tempered this down because I think it causes volatile behavior. However, the line here is not the root cause, that would be the switching behavior in the first place...
                    // std::cout << "AIM 2: " << _aim_pos;
                    _aim_pos = _flight_area->move_inside(_aim_pos, relaxed, drone.pos());
                    // std::cout << " inside --> " << _aim_pos << std::endl;
                    _control_mode = position_control;
                    return;
                }
                if (hunt_error < static_cast<float>(dparams.drone_rotation_delay) * normf(drone.vel()) && !_optimization_result.via && _optimization_result.interception_position_in_flightarea && _optimization_result.stopping_position_in_flightarea && _optimization_result.valid) {
                    enter_is_intercept_maneuvering(time, drone);
                } else {
                    break;
                }
                [[fallthrough]];
        } case is_intercept_maneuvering: {
                if (exit_is_intercept_maneuvering(time))
                    _hunt_strategy_state = is_approaching;
                break;
            }
    }
}

void Interceptor::choose_target_and_find_interception(float delay) {
    float best_time_to_intercept = INFINITY;
    bool best_aim_inview = false;
    bool best_stop_inview = false;
    trajectory_optimization_result best_optim_result;
    InsectTracker *best_itrkr = NULL;
    auto all_trackers = _trackers->all_target_trackers();

    TrackData tracking_data = _drone->tracker.last_track_data();
    if (_drone->tracker.drone_on_landing_pad()) {
        tracking_data.pos_valid = true;
        tracking_data.state.pos = _drone->tracker.pad_location();
        tracking_data.state.vel = cv::Point3f(0, 0, 0);
    }
    for (auto trkr : all_trackers) {
        if (trkr->tracking() && ((trkr->type() == tt_insect && !pparams.disable_real_hunts) || trkr->type() == tt_replay || trkr->type() == tt_virtualmoth)) {
            auto insect_state = trkr->last_track_data();
            if (!insect_state.pos_valid) {
                auto prediction = trkr->image_predict_item();
                if (prediction.valid)
                    insect_state.state.pos = im2world(prediction.pt_unbound, prediction.disparity, _visdat->Qf, _visdat->camera_roll(), _visdat->camera_pitch());
                else
                    insect_state.state.pos = {0};
            }
            if (!insect_state.vel_valid)
                insect_state.state.vel = {0};

            trajectory_optimization_result _optim_result = trajectory_optimizer.find_interception(tracking_data, insect_state, delay, _drone->control.kiv_ctrl.safety);
            if (_optim_result.valid) {
                _optim_result.interception_position_in_flightarea = _flight_area->inside(_optim_result.position_to_intercept, relaxed);
                _optim_result.stopping_position_in_flightarea = _flight_area->inside(_optim_result.stopping_position, relaxed);

                if (_optim_result.interception_position_in_flightarea > best_aim_inview || (_optim_result.stopping_position_in_flightarea > best_stop_inview && _optim_result.interception_position_in_flightarea == best_aim_inview)) {
                    best_time_to_intercept = _optim_result.time_to_intercept + _optim_result.via * _optim_result.time_to_intermediate;
                    best_optim_result = _optim_result;
                    best_itrkr = static_cast<InsectTracker *>(trkr);

                } else if (best_time_to_intercept > _optim_result.time_to_intercept + _optim_result.via * _optim_result.time_to_intermediate && _optim_result.interception_position_in_flightarea == best_aim_inview && _optim_result.stopping_position_in_flightarea == best_stop_inview) {
                    best_time_to_intercept = _optim_result.time_to_intercept + _optim_result.via * _optim_result.time_to_intermediate;
                    best_optim_result = _optim_result;
                    best_itrkr = static_cast<InsectTracker *>(trkr);
                }
            }
        }
    }
    _target_insecttracker = best_itrkr;
    _tti = best_time_to_intercept;
    _optimization_result = best_optim_result;
}

bool Interceptor::delay_takeoff_for_better_interception() {
    if (_optimization_result.valid && !_n_frames_aim_not_in_range && _n_frames_aim_in_range > 2)
        return false;
    else
        return true;
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
                if (normf(insect_direction_from_pad) < drone_trkr->pad_im_size() * 0.75f) {
                    float measured_versus_predicted_angle_diff = acosf((drone_trkr->takeoff_direction_predicted().dot(insect_direction_from_pad)) / (normf(drone_trkr->takeoff_direction_predicted()) * normf(insect_direction_from_pad)));
                    if (measured_versus_predicted_angle_diff < M_PIf32 / 8.f)
                        return true;
                }
            }
        }
    }
    return false;
}

void Interceptor::abort_flight() {
    _interceptor_state = is_init;
}
