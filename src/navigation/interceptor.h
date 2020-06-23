#pragma once
#include "cameraview.h"
#include "insecttracker.h"
#include "dronetracker.h"
#include "trackermanager.h"
#include "visiondata.h"



static const char* interceptor_state_names[] = { "is_init",
                                                 "is_await_target",
                                                 "is_await_reach_zone",
                                                 "is_move_to_intercept",
                                                 "is_close_chasing"
                                               };

/*
 * This class calculates the best intersection location, and whether that is even possible, etc
 *
 */
class Interceptor {

private:
    std::ofstream *_logger;
    tracking::TrackerManager * _trackers;
    VisionData *_visdat;
    CameraView* _camview;
    cv::Point3f req_intercept_pos;
    cv::Point3f _intercept_pos,_intercept_vel,_intercept_acc;
    float _horizontal_separation, _vertical_separation;
    CameraView::hunt_check_result hunt_volume_check = CameraView::HuntVolume_Unknown;
    bool view_check = false;

    enum interceptor_states {
        is_init=0,
        is_waiting_for_target,
        is_waiting_in_reach_zone,
        is_move_to_intercept,
        is_close_chasing
    };
    interceptor_states _interceptor_state = is_init;

    uint _count_icpt_postarget_not_in_range = 0;
    double _tti =-1;
    const float minimal_height = 0.2f;

    int v_crcl1 = 250;
    int v_crcl2 = 500;
    int r_crcl1 = 10;
    int r_crcl2 = 30;

    float insect_cleared_timeout;

    void intercept_spiral();
    float calc_tti(cv::Point3f insect_pos, cv::Point3f insect_vel, cv::Point3f drone_pos, cv::Point3f drone_vel, bool drone_taking_off);
    void update_flower_of_fire(double time);
    void update_far_target(bool drone_at_base);
    void update_close_target();
    void update_interceptability();
    cv::Point3f get_circle_pos(float timef);

public:

    void init(tracking::TrackerManager *trackers, VisionData *visdat, CameraView *camview, ofstream *logger);
    void update(bool drone_at_base, double time);
    void reset_insect_cleared() {_count_icpt_postarget_not_in_range = 0;}


    bool insect_in_range_takeoff(tracking::InsectTracker * best_itrkr) { //This is the takeoff trigger in drone-navigation
        return !_count_icpt_postarget_not_in_range
               && hunt_volume_check == CameraView::HuntVolume_OK
               && best_itrkr->properly_tracking()
               && !best_itrkr->false_positive();
    }
    bool insect_in_range() {return !_count_icpt_postarget_not_in_range;}
    bool insect_cleared() {return _count_icpt_postarget_not_in_range > insect_cleared_timeout; }
    cv::Point3f target_position() {return _intercept_pos;}
    cv::Point3f target_speed() {return _intercept_vel;}
    cv::Point3f target_accelleration() {return _intercept_acc;}
    double time_to_intercept() {return _tti;}


    std::string Interceptor_State() {
        if (_interceptor_state == is_move_to_intercept  )
            return hunt_volume_check_names[hunt_volume_check];
        return interceptor_state_names[_interceptor_state];
    }

};
