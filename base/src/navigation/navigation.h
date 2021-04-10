#pragma once
#include "common.h"

namespace navigation {
[[maybe_unused]] static const char* navigation_status_names[] = {"ns_init",
                                                                 "ns_locate_drone_init",
                                                                 "ns_locate_drone_led",
                                                                 "ns_wait_locate_drone",
                                                                 "ns_located_drone",
                                                                 "ns_calibrating_pad",
                                                                 "ns_check_telemetry",
                                                                 "ns_check_pad_att",
                                                                 "ns_wait_to_arm",
                                                                 "ns_start_calibrating_motion",
                                                                 "ns_calibrating_motion",
                                                                 "ns_calibrating_motion_done",
                                                                 "ns_wait_for_takeoff",
                                                                 "ns_wait_for_insect",
                                                                 "ns_takeoff",
                                                                 "ns_taking_off",
                                                                 "ns_take_off_completed",
                                                                 "ns_start_the_chase",
                                                                 "ns_chasing_insect_ff",
                                                                 "ns_chasing_insect",
                                                                 "ns_set_waypoint",
                                                                 "ns_wp", // approach, but dist is also printed so need the room
                                                                 "ns_flower_of_fire",
                                                                 "ns_brick_of_fire",
                                                                 "ns_goto_landing",
                                                                 "ns_goto_thrust_calib_waypoint",
                                                                 "ns_goto_yaw",
                                                                 "ns_reset_headless_yaw",
                                                                 "ns_resetting_headless_yaw",
                                                                 "ns_correct_yaw",
                                                                 "ns_correcting_yaw",
                                                                 "ns_calibrate_thrust",
                                                                 "ns_calibrating_thrust",
                                                                 "ns_land",
                                                                 "ns_landing",
                                                                 "ns_landed",
                                                                 "ns_start_shaking",
                                                                 "ns_shaking_drone",
                                                                 "ns_wait_after_shake",
                                                                 "ns_monitoring",
                                                                 "ns_manual",
                                                                 "ns_batlow",
                                                                 "ns_tracker_problem",
                                                                 "ns_drone_lost",
                                                                 "ns_unable_to_locate",
                                                                 "ns_drone_problem"
                                                                };

enum navigation_states {
    ns_init=0,
    ns_locate_drone_init,
    ns_locate_drone_wait_led_on,
    ns_wait_locate_drone,
    ns_located_drone,
    ns_calibrating_pad,
    ns_check_telemetry,
    ns_check_pad_att,
    ns_wait_to_arm,
    ns_start_calibrating_motion,
    ns_calibrating_motion,
    ns_calibrating_motion_done,
    ns_wait_for_takeoff_command,
    ns_wait_for_insect,
    ns_takeoff,
    ns_taking_off,
    ns_take_off_completed,
    ns_start_the_chase,
    ns_chasing_insect_ff,
    ns_chasing_insect,
    ns_set_waypoint,
    ns_approach_waypoint,
    ns_flower_waypoint,
    ns_brick_waypoint,
    ns_goto_landing_waypoint,
    ns_goto_thrust_calib_waypoint,
    ns_goto_yaw_waypoint,
    ns_reset_headless_yaw,
    ns_resetting_headless_yaw,
    ns_correct_yaw,
    ns_correcting_yaw,
    ns_calibrate_thrust,
    ns_calibrating_thrust,
    ns_land,
    ns_landing,
    ns_landed,
    ns_start_shaking,
    ns_shaking_drone,
    ns_wait_after_shake,
    ns_monitoring,
    ns_manual, // also for disarmed
    ns_batlow,
    ns_tracker_problem,
    ns_drone_lost,
    ns_unable_to_locate,
    ns_drone_problem
};

class navigationParameters: public xmls::Serializable
{
public:
    xmls::xFloat time_out_after_landing;

    navigationParameters() {
        // Set the XML class name.
        // This name can differ from the C++ class name
        setClassName("navigationParameters");

        // Set class version
        setVersion("1.1");

        // Register members. Like the class name, member names can differ from their xml depandants
        Register("time_out_after_landing",&time_out_after_landing);
    }
};


enum nav_flight_modes {
    nfm_none,
    nfm_manual,
    nfm_waypoint,
    nfm_hunt
};

}
