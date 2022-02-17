#include "interceptor.h"
#include "opencv2/imgproc.hpp"

using namespace tracking;

void Interceptor::init(tracking::TrackerManager *trackers, VisionData *visdat, FlightArea *flight_area, Drone *drone) {
    _trackers = trackers;
    _visdat = visdat;
    _flight_area = flight_area;
    _drone = drone;

    n_frames_target_cleared_timeout = pparams.fps * 1.f;

    initialized = true;
}

void Interceptor::update(bool drone_at_base, double time[[maybe_unused]]) {
    auto target_trkr = update_target_insecttracker();

    switch (_interceptor_state) {
        case  is_init: {
                _interceptor_state = is_waiting_for_target;
                _aim_pos = _flight_area->move_inside(cv::Point3f(0, 0, 0), strict);
                _tti = -1;
                [[fallthrough]];
        } case is_waiting_for_target: {
                _n_frames_aim_not_in_range++;
                _best_distance = INFINITY;

                if (!target_trkr)
                    break;
                else  if (target_trkr->tracking() && !target_trkr->false_positive() && !_trackers->monster_alert() && target_trkr->go_for_terminate()) {
                    _interceptor_state = is_waiting_in_reach_zone;
                } else
                    break;

                [[fallthrough]];
        } case is_waiting_in_reach_zone: {
                _best_distance = INFINITY;
                if (!target_trkr) {
                    _interceptor_state = is_waiting_for_target;
                    break;
                } if (!target_trkr->tracking() || target_trkr->false_positive() || _trackers->monster_alert() || !target_trkr->go_for_terminate()) {
                    _interceptor_state = is_waiting_for_target;
                    break;
                }

                auto req_aim_pos = update_far_target(drone_at_base);
                update_interceptability(req_aim_pos);
                _tti = -1;
                if (!_n_frames_aim_not_in_range)
                    _interceptor_state = is_move_to_intercept;
                else
                    break;

                [[fallthrough]];
        } case is_move_to_intercept: {
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
                    auto req_aim_pos = update_far_target(drone_at_base);
                    update_interceptability(req_aim_pos);
                }
                if (hunt_distance < 0.4f)
                    _interceptor_state = is_close_chasing;
                else
                    break;

                [[fallthrough]];
        } case is_close_chasing: {
                if (!target_trkr) {
                    _interceptor_state = is_waiting_for_target;
                    break;
                }
                if (target_trkr->n_frames_lost() > 0.15f * pparams.fps
                        || _n_frames_aim_not_in_range > 0.15f * pparams.fps
                        || target_trkr->false_positive()
                        || _trackers->monster_alert()) {
                    _interceptor_state = is_waiting_for_target;
                    break;
                }

                if (!target_trkr->n_frames_lost()) {
                    auto req_aim_pos = update_close_target(drone_at_base);
                    update_interceptability(req_aim_pos);
                    _interceptor_state = is_killing;
                    break;
                }
                break;
        } case is_killing: {
                update_close_target(drone_at_base);

                if (hunt_distance >= 0.45f)
                    _interceptor_state = is_move_to_intercept;


                if (!target_trkr) {
                    _interceptor_state = is_waiting_for_target;
                    break;
                }
                if (target_trkr->n_frames_lost() > 0.15f * pparams.fps
                        || _n_frames_aim_not_in_range > 0.15f * pparams.fps
                        || target_trkr->false_positive()
                        || _trackers->monster_alert()) {
                    _interceptor_state = is_waiting_for_target;
                    break;
                }
                break;
            }
    }
}
cv::Point3f Interceptor::update_far_target(bool drone_at_base) {
    TrackData target = target_last_trackdata();
    cv::Point3f predicted_pos = target.pos();
    cv::Point3f predicted_vel = target.vel();
    // std::cout << "far_target: predicted_pos: " << predicted_pos;
#if ENABLE_MOTH_PREDICTION
    float time_to_intercept = 0.2f;
    predicted_pos += time_to_intercept * predicted_vel;
#endif
    TrackData dtd = _drone->tracker.last_track_data();
    cv::Point3f drone_pos = dtd.pos();

    if (drone_at_base)
        drone_pos = _drone->tracker.pad_location();

    cv::Point3f drone_vel = dtd.vel();
    calc_tti(predicted_pos, predicted_vel, drone_pos, drone_vel, drone_at_base); // only used for viz _tti
    _tti = time_to_intercept; //Overwrite with actual used tti
    auto req_aim_pos = predicted_pos;
    // req_aim_pos.y -= 0.2f;
    // std::cout << "; req_aim_pos: " << req_aim_pos << std::endl;

    hunt_distance = normf(target.pos() - dtd.pos());
    if (hunt_distance < _best_distance && !drone_at_base)
        _best_distance = hunt_distance;

    return req_aim_pos;
}

cv::Point3f Interceptor::update_close_target(bool drone_at_base) {
    TrackData target = target_last_trackdata();
    cv::Point3f predicted_pos = target.pos();
    cv::Point3f predicted_vel = target.vel();
    //std::cout << "close-target: predicted_pos: " << predicted_pos;
#if ENABLE_MOTH_PREDICTION
    float time_to_intercept = 0.1f;
    _tti = time_to_intercept; //Overwrite with actual used tti
    predicted_pos += time_to_intercept * predicted_vel;
#endif
    TrackData dtd = _drone->tracker.last_track_data();
    cv::Point3f drone_pos = dtd.pos();
    auto req_aim_pos = predicted_pos;
#if ENABLE_VELOCITY_COMPENSATION
    req_aim_pos -= 0.2f * dtd.vel(); // The aiming oly works if we can compensate for the current velocity
#endif
    cv::Point3f vector = predicted_pos - drone_pos;
    float norm_vector = norm(vector);
    req_aim_pos += 0.9f * vector / norm_vector;
    predicted_vel.y = 0; // we don't want to follow the vertical speed of the target, ever
    predicted_vel = 0.5f * predicted_vel + vector / norm_vector * 0.8f;


    hunt_distance = normf(target.pos() - dtd.pos());
    if (hunt_distance < _best_distance && !drone_at_base)
        _best_distance = hunt_distance;
    return req_aim_pos;
}

void Interceptor::update_interceptability(cv::Point3f req_aim_pos) {
    target_in_flight_area = _flight_area->inside(target_last_trackdata().pos(), relaxed);
    aim_in_view = _flight_area->inside(req_aim_pos, bare);

    if (aim_in_view) {
        _aim_pos = req_aim_pos;
        _n_frames_aim_not_in_range = 0;
    } else
        _n_frames_aim_not_in_range++;

    if (_interceptor_state == Interceptor::is_move_to_intercept
            || _interceptor_state == Interceptor::is_close_chasing)
        _aim_pos = req_aim_pos;
}

float Interceptor::calc_tti(cv::Point3f target_pos, cv::Point3f target_vel, cv::Point3f drone_pos, cv::Point3f drone_vel, bool drone_taking_off) {
    //basic physics:
    //x = 0.5at² t = sqrt((2x)/a)
    //x = vt t = x/v
    //v = at t = v/a
    //TODO: put in some actually measured values:
    const float drone_vel_max = 5; // [m/s]
    const float drone_acc_max = 40; // [m/s^2]
    const double t_estimated_take_off = 0.29; //[s]
    float ic_dx = normf(target_pos - drone_pos);
    float ic_dv = normf(target_vel - drone_vel);
    float vi = normf(target_vel);
    float vd = normf(drone_vel);
    /*Part 1, match_v*/
    //First the drone needs some time to match speeds. During this time its average speed v_d_avg = 0.5*(vd-vi) + vd
    //Time needed to match speed:
    float t_match_v = ic_dv / drone_acc_max;
    //Average speed and distance traveled during this time
    float v_d_avg = 0.5f * ic_dv + vd;
    float x_match_v = t_match_v * v_d_avg;
    /*Part 2, intercept the remaining distance with max v*/
    float ic_dx_2 = ic_dx - x_match_v;
    float t_ic = 0;

    if (ic_dx_2 > 0) {
        t_ic = ic_dx_2 / (drone_vel_max - vi);
    }

    //time to intercept is time of part 1 and 2 summed
    double tti = t_match_v + t_ic;

    //plus the time needed to takeoff, if not already flying
    if (drone_taking_off) {
        double t_remaining_takeoff = t_estimated_take_off - _drone->tracker.time_since_take_off();

        if (t_remaining_takeoff < 0)
            t_remaining_takeoff = 0;

        tti += t_remaining_takeoff;
    }

    _tti = tti;
    return tti;
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
            if (trkr->type() == tt_insect || trkr->type() == tt_replay || trkr->type() == tt_virtualmoth) {
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
tracking::TrackData Interceptor::target_last_trackdata() {
    if (target_insecttracker())
        return target_insecttracker()->last_track_data();
    else {
        TrackData d;
        return d;
    }
}
