#pragma once
#include <vector>
#include <map>

namespace logging
{
std::map<std::string, int> read_head_map(std::string heads);

struct LogEntryInsect {
    double elapsed;
    bool replay;
    unsigned long long rs_id;

    float im_x;
    float im_y;
    float disparity;
    float pred_im_x;
    float pred_im_y;
    int n_frames_lost;
    int n_frames_tracking;
    int foundL;

    float pos_x;
    float pos_y;
    float pos_z;

    float spos_x;
    float spos_y;
    float spos_z;

    float svel_x;
    float svel_y;
    float svel_z;

    float sacc_x;
    float sacc_y;
    float sacc_z;
};
struct LogEntryDrone {
    double elapsed;
    unsigned long long rs_id;

    float im_x;
    float im_y;
    float disparity;
    float pred_im_x;
    float pred_im_y;
    int n_frames_lost;
    int n_frames_tracking;
    int foundL;
    int insect_id;
    int nav_flight_mode;
    int charging_state;

    float pos_x;
    float pos_y;
    float pos_z;

    float spos_x;
    float spos_y;
    float spos_z;

    float svel_x;
    float svel_y;
    float svel_z;

    float sacc_x;
    float sacc_y;
    float sacc_z;

    float target_pos_x;
    float target_pos_y;
    float target_pos_z;

    int auto_roll;
    int auto_pitch;
    int auto_yaw;
    int auto_throttle;

    int joy_roll;
    int joy_pitch;
    int joy_yaw;
    int joy_throttle;
    int joy_arm_switch;
    int joy_mode_switch;
    int joy_takeoff_switch;
};
struct LogEntryMain {
    int id;
    unsigned long long rs_id;
    double elapsed;
    int exposure;
    int gain;
    int trackers_state;
    int charging_state;
};

}
