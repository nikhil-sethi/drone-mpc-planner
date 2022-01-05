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
    std::string demo_waypoint_fn = "/home/pats/pats/flags/pats_demo.xml";
    std::string demo_insect_fn = "/home/pats/pats/flags/insect_demo";
    std::string virtual_insect_fn = "/home/pats/pats/flags/virtual_insect";
    std::string calib_fn = "/home/pats/pats/flags/calib_now";
    std::string beep_fn = "/home/pats/pats/flags/beep_now";
    std::string shake_fn = "/home/pats/pats/flags/shake_now";
    std::string blink_fn = "/home/pats/pats/flags/blink_now";
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
