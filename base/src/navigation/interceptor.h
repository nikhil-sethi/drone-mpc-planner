#pragma once
#include "flightarea/flightarea.h"
#include "insecttracker.h"
#include "dronetracker.h"
#include "trackermanager.h"
#include "visiondata.h"
#include "drone.h"

#define ENABLE_UNIFIED_DIRECTION_TRANSITION true
#define ENABLE_MOTH_PREDICTION true
#define ENABLE_VELOCITY_COMPENSATION true

static const char *interceptor_state_names[] = { "is_init",
                                                 "is_await_target",
                                                 "is_await_reach_zone",
                                                 "is_move_to_intercept",
                                                 "is_close_chasing",
                                                 "is_killing"
                                               };
class Drone;

class Interceptor {

private:
    tracking::TrackerManager *_trackers;
    VisionData *_visdat;
    Drone *_drone;
    bool initialized = false;

    cv::Point3f _aim_pos, _aim_vel, _aim_acc;

    FlightArea *_flight_area;
    bool target_in_flight_area = false;

    bool aim_in_view = false;
    uint _n_frames_aim_not_in_range = 0;
    float n_frames_target_cleared_timeout;
    tracking::InsectTracker *_target_insecttracker = NULL;

    float _horizontal_separation, _vertical_separation;
    float total_separation;
    float _best_distance = -1;
    double _tti = -1;
    enum interceptor_states {
        is_init = 0,
        is_waiting_for_target,
        is_waiting_in_reach_zone,
        is_move_to_intercept,
        is_close_chasing,
        is_killing
    };
    interceptor_states _interceptor_state = is_init;

    float calc_tti(cv::Point3f target_pos, cv::Point3f target_vel, cv::Point3f drone_pos, cv::Point3f drone_vel, bool drone_taking_off);
    cv::Point3f update_far_target(bool drone_at_base);
    cv::Point3f update_close_target(bool drone_at_base);
    void update_interceptability(cv::Point3f req_aim_pos);
    tracking::InsectTracker *update_target_insecttracker();

public:
    void init(tracking::TrackerManager *trackers, VisionData *visdat, FlightArea *flight_area, Drone *drone);
    void update(bool drone_at_base, double time);
    tracking::InsectTracker *target_insecttracker() {return _target_insecttracker;}
    int insect_id() {
        if (!_target_insecttracker)
            return -1;
        else
            return _target_insecttracker->insect_trkr_id();
    }

    tracking::TrackData target_last_trackdata();

    bool target_acquired(double time) { return target_detected(time) && target_in_flight_area; }
    bool target_detected(double time) {
        if (!_target_insecttracker)
            return false;
        return !_n_frames_aim_not_in_range
               && _visdat->no_recent_brightness_events(time)
               && !_trackers->monster_alert()
               && !_target_insecttracker->false_positive()
               && _target_insecttracker->properly_tracking();
    }
    bool target_cleared() {return _n_frames_aim_not_in_range > n_frames_target_cleared_timeout;}
    cv::Point3f aim_pos() {return _aim_pos;}
    cv::Point3f aim_vel() {return _aim_vel;}
    cv::Point3f aim_acc() {return _aim_acc;}
    double time_to_intercept() {return _tti;}
    float best_distance() {return _best_distance;}
    void target_is_hunted(int hunt_id) {
        if (_target_insecttracker)
            _target_insecttracker->hunt_id(hunt_id);
    }

    std::string Interceptor_State() {return interceptor_state_names[_interceptor_state];}
};
