#pragma once
#include "flightarea/flightarea.h"
#include "insecttracker.h"
#include "dronetracker.h"
#include "trackermanager.h"
#include "visiondata.h"

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

/*
 * This class calculates the best intersection location (aim), and whether that is even possible, etc
 *
 */
class Interceptor {

private:
    std::ofstream *_logger;
    tracking::TrackerManager *_trackers;
    VisionData *_visdat;
    DroneController *_dctrl;

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

    int v_crcl1 = 250;
    int v_crcl2 = 500;
    int r_crcl1 = 10;
    int r_crcl2 = 30;

    void intercept_spiral();
    float calc_tti(cv::Point3f target_pos, cv::Point3f target_vel, cv::Point3f drone_pos, cv::Point3f drone_vel, bool drone_taking_off);
    void update_flower_of_fire(double time);
    cv::Point3f update_far_target(bool drone_at_base);
    cv::Point3f update_close_target(bool drone_at_base);
    void update_interceptability(cv::Point3f req_aim_pos);
    cv::Point3f get_circle_pos(float timef);
    tracking::InsectTracker *update_target_insecttracker();

public:
    void init(tracking::TrackerManager *trackers, VisionData *visdat, FlightArea *flight_area, ofstream *logger, DroneController *dctrl);
    void update(bool drone_at_base, double time);
    tracking::InsectTracker *target_insecttracker() {return _target_insecttracker;}

    tracking::TrackData target_last_trackdata();

    bool trigger_takeoff() {
        tracking::InsectTracker *best_itrkr = target_insecttracker();
        if (!best_itrkr)
            return false;
        return !_n_frames_aim_not_in_range
               && target_in_flight_area
               && !best_itrkr->false_positive()
               && !_trackers->monster_alert();
    }
    bool aim_in_range() {return !_n_frames_aim_not_in_range;}
    bool target_cleared() {return _n_frames_aim_not_in_range > n_frames_target_cleared_timeout;}
    cv::Point3f aim_pos() {return _aim_pos;}
    cv::Point3f aim_vel() {return _aim_vel;}
    cv::Point3f aim_acc() {return _aim_acc;}
    double time_to_intercept() {return _tti;}
    float best_distance() {return _best_distance;}
    void write_dummy_csv();

    std::string Interceptor_State() {return interceptor_state_names[_interceptor_state];}
};
