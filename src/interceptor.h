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
    cv::Point3f insectAcc;

    const float estimated_take_off_time = 1.f;
    uint _count_insect_not_in_range = 0;

    bool final_approach = false;

    cv::Mat Qfi;

    void update_insect_prediction();
    void intercept_spiral();

public:

    void init(DroneTracker * dtrkr, InsectTracker * itrkr, VisionData * visdat);
    void update(bool drone_at_base);
    bool insect_in_range();
    bool insect_cleared();
    void reset_insect_cleared();
    cv::Point3f intercept_position();
    cv::Point3f prev_intercept_position();
    cv::Point3f target_speed();
    cv::Point3f target_accelleration();


};

#endif
