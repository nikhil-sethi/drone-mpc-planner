#pragma once
#include <opencv2/core/types.hpp>
#include "flightplan.h"
#include "tracking.h"

class LandingController {
private:
    // Parameter:
    const float target_descend_vel = -0.35f;
    const float height_trigger_ff = 0.1f;
    const float _time_ff_landing = 0.3f;

public:
    cv::Point3f setpoint_cc_landing(cv::Point3f pad_location, navigation::Waypoint *current_waypoint, float dt_land);
    bool switch_to_ff_landing(tracking::TrackData drone_track_data, cv::Point3f pos_setpoint, cv::Point3f pad_pos);
    int ff_auto_throttle(int ff_auto_throttle_start, float dt);

    float time_ff_landing() { return _time_ff_landing;};
};
