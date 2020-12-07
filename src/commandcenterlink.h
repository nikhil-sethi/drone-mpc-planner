#pragma once
#include "dronenavigation.h"

class CommandCenterLink {
public :
    void init(bool log_replay_mode, navigation::DroneNavigation * dnav,DroneController * dctrl,Rc * rc,tracking::TrackerManager * trackers);
    void close();
    void reset_commandcenter_status_file(std::string status_msg, bool never_overwrite);
    void trigger_demo_flight_from_log(std::string replay_dir, int tracker_mode);
    int n_replay_moth() {return _n_replay_moth;}
    void update(cv::Mat frame, double time) {
        _time_since_start = time;
        if (new_frame_request) {
            _frame = frame.clone();
            new_frame_request =0;
        }
    };

private:
    std::string demo_waypoint_fn = "/home/pats/pats_demo.xml";
    std::string demo_insect_fn = "/home/pats/insect_demo";
    std::string calib_fn = "/home/pats/calib_now";
    std::string beep_fn = "/home/pats/beep_now";
    std::string shake_fn = "/home/pats/shake_now";
    std::thread thread;
    bool initialized = false;
    bool _never_overwrite = false;

    int new_frame_request = 1;
    cv::Mat _frame;
    double _time_since_start = 0;

    navigation::DroneNavigation * _dnav;
    DroneController * _dctrl;
    Rc * _rc;
    tracking::TrackerManager * _trackers;
    bool _log_replay_mode = false;
    int reset_cnt = 0;
    int _n_replay_moth = 0;

    bool exit_thread = false;

    void write_commandcenter_status_image();
    void write_commandcenter_status_file();
    void check_commandcenter_triggers();
    void background_worker();

};
