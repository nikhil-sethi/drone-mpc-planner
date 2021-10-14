#pragma once
#include <vector>
#include <map>

namespace logging
{
std::map<std::string, int> read_head_map(std::string heads);

struct LogEntryInsect {
    float time;
    bool replay;
    unsigned long long rs_id;

    float ins_im_x;
    float ins_im_y;
    float ins_disparity;
    float ins_pred_im_x;
    float ins_pred_im_y;
    int ins_n_frames_lost;
    int ins_n_frames_tracking;
    int ins_foundL;

    float ins_pos_x;
    float ins_pos_y;
    float ins_pos_z;

    float ins_spos_x;
    float ins_spos_y;
    float ins_spos_z;

    float ins_svel_x;
    float ins_svel_y;
    float ins_svel_z;

    float ins_sacc_x;
    float ins_sacc_y;
    float ins_sacc_z;
};
struct LogEntryMain {
    int id;
    unsigned long long rs_id;
    double elapsed;
    bool insect_replay_log;
    bool valid;
    int joyThrottle;
    int joyRoll;
    int joyPitch;
    int joyYaw;
    int joyArmSwitch;
    int joyModeSwitch;
    int joyTakeoffSwitch;
    int trkrs_state;
    int nav_state;

    int auto_throttle;
    int auto_roll;
    int auto_pitch;

    float telem_acc_z;
    uint32_t telem_throttle;
    float   telem_throttle_s;
    float   maxthrust;
    float   telem_thrust_rpm;

    float imLx_drone, imLy_drone, disparity_drone;

    std::vector<LogEntryInsect> insects;

};

}
