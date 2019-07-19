#ifndef INTERCEPT_H
#define INTERCEPT_H

#include "defines.h"
#include "insecttracker.h"
#include "dronetracker.h"
#include "itemmanager.h"
#include "visiondata.h"



static const char* interceptor_state_names[] = { "is_init",
                                                    "is_waiting_for_target",
                                                    "is_waiting_in_reach_zone",
                                                    "is_move_to_intercept",
                                                    "is_close_chasing"};

/*
 * This class calculates the best intersection location, and whether that is even possible, etc
 *
 */
class Interceptor{

private:
    ItemManager * _trackers;
    VisionData *_visdat;
    cv::Point3f _intercept_pos,_intercept_vel,_intercept_acc;
    float _horizontal_separation, _vertical_separation;


    enum interceptor_states {
        is_init=0,
        is_waiting_for_target,
        is_waiting_in_reach_zone,
        is_move_to_intercept,
        is_close_chasing
    };
    interceptor_states _interceptor_state = is_init;

    uint _count_insect_not_in_range = 0;
    double _tti =-1;
    const float minimal_height = 0.2f;

    void intercept_spiral();
    float calc_tti(cv::Point3f insect_pos, cv::Point3f insect_vel, cv::Point3f drone_pos, cv::Point3f drone_vel, bool drone_taking_off);
    void update_far_target(bool drone_at_base);
    void update_close_target();
    void update_insect_in_range();

public:

    void init(ItemManager * trackers, VisionData * visdat);
    void update(bool drone_at_base);

    void reset_insect_cleared() {_count_insect_not_in_range = 0;}

    bool insect_in_range() {return !_count_insect_not_in_range;}
    bool insect_cleared() {return _count_insect_not_in_range > VIDEOFPS*0.5f; } // TODO: make a nice variable
    cv::Point3f target_position() {return _intercept_pos;}
    cv::Point3f target_speed() {return _intercept_vel;}
    cv::Point3f target_accelleration() {return _intercept_acc;}
    double time_to_intercept(){return _tti;}


    std::string Interceptor_State() {
        return interceptor_state_names[_interceptor_state];
    }
};

#endif
