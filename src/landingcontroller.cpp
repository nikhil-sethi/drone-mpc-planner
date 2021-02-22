#include "landingcontroller.h"
#include "flightplan.h"
#include <opencv2/core/types.hpp>

cv::Point3f LandingController::setpoint_cc_landing(cv::Point3f landing_location, navigation::Waypoint* current_waypoint, float dt_land) {
    cv::Point3f pad_pos = landing_location;
    cv::Point3f setpoint_pos_world =  pad_pos + current_waypoint->xyz;
    if (setpoint_pos_world.y > -0.5f) // keep some margin to the top of the view, because atm we have an overshoot problem.
        setpoint_pos_world.y = -0.5f;

    float v_descend = target_descend_vel;
    float target_time = -2.f * current_waypoint->xyz.y/v_descend;
    if (dt_land < target_time)
        v_descend = v_descend*(dt_land/target_time);

    cv::Point3f new_pos_setpoint;
    new_pos_setpoint.x = setpoint_pos_world.x;
    new_pos_setpoint.y = setpoint_pos_world.y + dt_land* v_descend;
    new_pos_setpoint.z = setpoint_pos_world.z;

    return new_pos_setpoint;
}

bool LandingController::switch_to_ff_landing(tracking::TrackData drone_track_data, cv::Point3f pos_setpoint, cv::Point3f pad_pos, bool ff_landing) {
    if (pos_setpoint.y < pad_pos.y) {
        if ((!ff_landing)
                && ((drone_track_data.spos().y < pad_pos.y+height_trigger_ff)
                    || (!drone_track_data.spos_valid)))
            return true;
    }
    return false;
}

int LandingController::ff_auto_throttle(int ff_auto_throttle_start, float thrust, float landing_target_time, float dt ) {
    return ff_auto_throttle_start - thrust/38.f * (1.f/landing_target_time) * dt * (ff_auto_throttle_start-RC_BOUND_MIN);
}
