#ifndef INTERCEPT_H
#define INTERCEPT_H

#include "defines.h"
#include "insecttracker.h"
#include "dronetracker.h"



/*
 * This class calculates the best intersection location, and whether that is even possible, etc
 *
 */
class Interceptor{

private:
    DroneTracker * _dtrkr;
    InsectTracker * _itrkr;
    bool _insect_in_range;
    cv::Point3f _estimated_interception_location;
    cv::Point3f _estimated_interception_speed;

    const float estimated_take_off_time = 1.f;
    uint _count_insect_not_in_range = 0;

    bool final_approach = false;

public:

    void init(DroneTracker * dtrkr, InsectTracker * itrkr);
    void update(bool drone_at_base);
    bool get_insect_in_range();
    bool get_insect_cleared();
    void reset_insect_cleared();
    cv::Point3f get_intercept_position();

};

#endif
