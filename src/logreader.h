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
    struct Log_Entry_Insect {
        float time;
        bool replay;
        unsigned long long RS_ID;

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
    struct Log_Entry_Main {
        int ID;
        unsigned long long RS_ID;
        bool insect_replay_log;
        bool valid;
        int joyThrottle;
        int joyRoll;
        int joyPitch;
        int joyYaw;
        int joyArmSwitch;
        int joyModeSwitch;
        int joyTakeoffSwitch;

        int auto_throttle;
        int auto_roll;
        int auto_pitch;

        std::vector<Log_Entry_Insect> insects;

    };

    void init(std::string path);
    void read_insect_replay_log(std::string path);
    void current_frame_number(unsigned long long RS_id);
    bool set_next_frame_number();
    double first_takeoff_time() {return _takeoff_time;}

    LogReader::Log_Entry_Main current_entry;
    LogReader::Log_Entry_Insect current_replay_insect_entry;
private:

    void read_multi_insect_logs(std::string path);
    Log_Entry_Main create_main_log_entry(std::string line,std::map<std::string, int> headmap);
    Log_Entry_Insect create_insect_log_entry(std::string line,std::map<std::string, int> headmap);
    std::map<std::string, int> read_head_map(std::string heads);
    std::tuple<std::map<int, Log_Entry_Main>,std::map<std::string, int>> read_main_log(std::string file);
    std::tuple<std::map<int, Log_Entry_Insect>,std::map<std::string, int>> read_insect_log(std::string file);

    std::map<int, Log_Entry_Main> log_main;
    std::map<std::string, int> headmap_main;
    std::vector<std::tuple<std::map<int, LogReader::Log_Entry_Insect>,std::map<std::string, int>>> log_insects;

    double _takeoff_time = INFINITY;

};

#endif // LOGREADER_H
