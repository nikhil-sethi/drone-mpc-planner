#pragma once
#include "common.h"

namespace navigation {
static const char* navigation_status_names[] = {"ns_init",
                                                "ns_locate_drone_init",
                                                "ns_locate_drone_led",
                                                "ns_wait_locate_drone",
                                                "ns_crippled",
                                                "ns_located_drone",
                                                "ns_wait_for_takeoff",
                                                "ns_wait_for_insect",
                                                "ns_takeoff",
                                                "ns_taking_off",
                                                "ns_take_off_completed",
                                                "ns_start_the_chase",
                                                "ns_chasing_insect_ff",
                                                "ns_chasing_insect",
                                                "ns_set_waypoint",
                                                "ns_approach_wp",
                                                "ns_flower_of_fire",
                                                "ns_brick_of_fire",
                                                "ns_goto_landing",
                                                "ns_initial_reset_yaw",
                                                "ns_reset_yaw",
                                                "ns_land",
                                                "ns_landing",
                                                "ns_landed",
                                                "ns_wait_after_landing",
                                                "ns_manual",
                                                "ns_drone_problem"
                                               };

enum navigation_states {
    ns_init=0,
    ns_locate_drone_init,
    ns_locate_drone_wait_led_on,
    ns_wait_locate_drone,
    ns_crippled,
    ns_located_drone,
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
    ns_initial_reset_yaw,
    ns_wait_reset_yaw,
    ns_land,
    ns_landing,
    ns_landed,
    ns_wait_after_landing,
    ns_manual, // also for disarmed
    ns_drone_problem
};

class navigationParameters: public xmls::Serializable
{
public:
    xmls::xInt distance_threshold_f;
    xmls::xFloat time_out_after_landing;

    navigationParameters() {
        // Set the XML class name.
        // This name can differ from the C++ class name
        setClassName("navigationParameters");

        // Set class version
        setVersion("1.1");

        // Register members. Like the class name, member names can differ from their xml depandants
        Register("distance_threshold_f",&distance_threshold_f);
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
