#pragma once
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "flightplan.h"
#include "tracking.h"
#include "rc.h"

class LandingController {
private:
    // Parameter:
    const float target_descend_vel = -0.5f;
    const float height_trigger_ff = 0.2f;

    // States:

public:
    cv::Point3f setpoint_cc_landing(cv::Point3f landing_location, navigation::Waypoint* current_waypoint, float dt_land);
    bool switch_to_ff_landing(tracking::TrackData drone_track_data, cv::Point3f pos_setpoint, cv::Point3f pad_pos, bool ff_landing);
    int ff_auto_throttle(int ff_auto_throttle_start, float thrust, float landing_target_time, float dt );
};
