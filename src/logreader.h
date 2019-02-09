#ifndef LOGREADER_H
#define LOGREADER_H

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <unistd.h>
#include <map>

#include <opencv2/core/core.hpp>


/*
 * This class performs moving average filtering
 *
 */
class LogReader{

public:
    struct Log_Entry {
        int ID;
        int RS_ID;
        bool valid;
        int joyThrottle;
        int joyRoll;
        int joyPitch;
        int joyYaw;
        int joyArmSwitch;
        int joyModeSwitch;

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

        float camera_angle_y;

    };
    void init(std::string file);
    void set_current_frame_number(int frame_number);

    LogReader::Log_Entry current_item;
private:

    Log_Entry createLogEntry(std::string line);
    void setHeadMap(std::string heads);
    std::map<int, Log_Entry> log;
    std::map<std::string, int> headmap;

};

#endif // LOGREADER_H
