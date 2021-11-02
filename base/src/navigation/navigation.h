#pragma once
#include "common.h"

namespace navigation {
[[maybe_unused]] static const char *navigation_status_names[] = {"ns_takeoff",
                                                                 "ns_taking_off",
                                                                 "ns_take_off_completed",
                                                                 "ns_start_the_chase",
                                                                 "ns_chasing_insect_ff",
                                                                 "ns_chasing_insect",
                                                                 "ns_set_waypoint",
                                                                 "ns_wp", // approach, but dist is also printed so need the room
                                                                 "ns_goto_landing",
                                                                 "ns_goto_thrust_calib_waypoint",
                                                                 "ns_goto_yaw",
                                                                 "ns_reset_headless_yaw",
                                                                 "ns_resetting_headless_yaw",
                                                                 "ns_correct_yaw",
                                                                 "ns_bowling_nudge",
                                                                 "ns_correcting_yaw",
                                                                 "ns_calibrate_thrust",
                                                                 "ns_calibrating_thrust",
                                                                 "ns_land",
                                                                 "ns_landing",
                                                                 "ns_landed",
                                                                 "ns_wait_after_landed",
                                                                 "ns_flight_done",
                                                                 "ns_takeoff_failure",
                                                                 "ns_flight_failure",
                                                                 "ns_landing_failure"
                                                                };

enum navigation_states {
    ns_takeoff,
    ns_taking_off,
    ns_take_off_completed,
    ns_start_the_chase,
    ns_chasing_insect_ff,
    ns_chasing_insect,
    ns_set_waypoint,
    ns_approach_waypoint,
    ns_goto_landing_waypoint,
    ns_goto_thrust_calib_waypoint,
    ns_goto_yaw_waypoint,
    ns_reset_headless_yaw,
    ns_resetting_headless_yaw,
    ns_correct_yaw,
    ns_bowling_nudge,
    ns_correcting_yaw,
    ns_calibrate_thrust,
    ns_calibrating_thrust,
    ns_land,
    ns_landing,
    ns_landed,
    ns_wait_after_landed,
    ns_flight_done,
    ns_takeoff_failure,
    ns_flight_failure,
    ns_landing_failure
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
        Register("time_out_after_landing", &time_out_after_landing);
    }
};


enum nav_flight_modes {
    nfm_none,
    nfm_manual,
    nfm_waypoint,
    nfm_hunt
};

}
