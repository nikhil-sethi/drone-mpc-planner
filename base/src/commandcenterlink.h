#pragma once
#include "pats.h"

class CommandCenterLink {
public :
    void init(bool log_replay_mode, Patser *patser, VisionData *visdat);
    void close();
    void reset_commandcenter_status_file(std::string status_msg, bool never_overwrite);
    int n_replay_moth() {return _n_replay_moth;}
    void update(cv::Mat frame, double time) {
        _time_since_start = time;
        if (new_frame_request) {
            _frame = frame.clone();
            new_frame_request = 0;
        }
    };

private:
    std::string pats_flags_folder = "/home/pats/pats/flags/";
    std::string demo_waypoint_fn = pats_flags_folder + "pats_demo.xml";
    std::string demo_insect_fn = pats_flags_folder + "insect_demo";
    std::string virtual_insect_fn = pats_flags_folder + "virtual_insect";
    std::string calib_fn = pats_flags_folder + "calib_now";
    std::string beep_fn = pats_flags_folder + "beep_now";
    std::string shake_fn = pats_flags_folder + "shake_now";
    std::string blink_fn = pats_flags_folder + "blink_now";
    std::string benchmark_fn = pats_flags_folder + "benchmark.csv";
    std::thread thread;
    bool initialized = false;
    bool _never_overwrite = false;

    int new_frame_request = 1;
    cv::Mat _frame;
    double _time_since_start = 0;

    VisionData *_visdat;
    Patser *_patser;
    bool _log_replay_mode = false;
    int reset_cnt = 0;
    int _n_replay_moth = 0;

    bool exit_thread = false;

    void write_commandcenter_status_image();
    void write_commandcenter_status_file();
    void check_commandcenter_triggers();
    void background_worker();

};
