#pragma once
#include "cameraview.h"
#include "insecttracker.h"
#include "dronetracker.h"
#include "trackermanager.h"
#include "visiondata.h"

#define ENABLE_UNIFIED_DIRECTION_TRANSITION false
#define ENABLE_MOTH_PREDICTION true
#define ENABLE_VELOCITY_COMPENSATION false

static const char* interceptor_state_names[] = { "is_init",
                                                 "is_await_target",
                                                 "is_await_reach_zone",
                                                 "is_move_to_intercept",
                                                 "is_close_chasing"
                                               };

/*
 * This class calculates the best intersection location (aim), and whether that is even possible, etc
 *
 */
class Interceptor {

private:
    std::ofstream *_logger;
    tracking::TrackerManager * _trackers;
    VisionData *_visdat;

    cv::Point3f _aim_pos,_aim_vel,_aim_acc;

    CameraView* _camview;
    CameraView::hunt_check_result target_in_hunt_volume = CameraView::HuntVolume_Unknown;
    bool aim_in_view = false;
    uint _n_frames_aim_not_in_range = 0;
    float n_frames_target_cleared_timeout;

    float _horizontal_separation, _vertical_separation;
    float total_separation;
    float _best_distance = -1;
    double _tti =-1;

    enum interceptor_states {
        is_init=0,
        is_waiting_for_target,
        is_waiting_in_reach_zone,
        is_move_to_intercept,
        is_close_chasing
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

public:
    void init(tracking::TrackerManager *trackers, VisionData *visdat, CameraView *camview, ofstream *logger);
    void update(bool drone_at_base, double time);

    bool trigger_takeoff(tracking::InsectTracker * best_itrkr) {
        return !_n_frames_aim_not_in_range
               && target_in_hunt_volume == CameraView::HuntVolume_OK
               && !best_itrkr->false_positive();
    }
    bool aim_in_range() {return !_n_frames_aim_not_in_range;}
    bool target_cleared() {return _n_frames_aim_not_in_range > n_frames_target_cleared_timeout; }
    cv::Point3f aim_pos() {return _aim_pos;}
    cv::Point3f aim_vel() {return _aim_vel;}
    cv::Point3f aim_acc() {return _aim_acc;}
    double time_to_intercept() {return _tti;}
    float best_distance() {return _best_distance;}

    std::string Interceptor_State() {return interceptor_state_names[_interceptor_state];}
};
