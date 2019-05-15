#ifndef INTERCEPT_H
#define INTERCEPT_H

#include "defines.h"
#include "insecttracker.h"
#include "dronetracker.h"
#include "visiondata.h"



/*
 * This class calculates the best intersection location, and whether that is even possible, etc
 *
 */
class Interceptor{

private:
    DroneTracker * _dtrkr;
    InsectTracker * _itrkr;
    VisionData *_visdat;
    bool _insect_in_range;
    cv::Point3f _estimated_interception_location;
    cv::Point3f _prev_estimated_interception_location;
    cv::Point3f _estimated_interception_speed;
    cv::Point3f insectVel;

    const float estimated_take_off_time = 1.f;
    uint _count_insect_not_in_range = 0;

    bool final_approach = false;

    cv::Mat Qfi;

public:

    void init(DroneTracker * dtrkr, InsectTracker * itrkr, VisionData * visdat);
    void update(bool drone_at_base);
    bool get_insect_in_range();
    bool get_insect_cleared();
    void reset_insect_cleared();
    cv::Point3f get_intercept_position();
    cv::Point3f get_prev_intercept_position();
    cv::Point3f get_target_speed();

};

#endif
