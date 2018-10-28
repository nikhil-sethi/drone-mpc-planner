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

public:

    void init(DroneTracker * dtrkr, InsectTracker * itrkr);
    void update(bool drone_at_base);
    bool get_insect_in_range();

};

#endif
