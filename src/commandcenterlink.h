#pragma once
#include "dronenavigation.h"

class CommandCenterLink {
public :
    void init(bool log_replay_mode, navigation::DroneNavigation * dnav,DroneController * dctrl,MultiModule * rc,Cam * cam,tracking::TrackerManager * trackers);
    void close();
    void reset_commandcenter_status_file(std::string status_msg);
    void trigger_demo_flight_from_log(std::string replay_dir, int tracker_mode);

private:
    std::string demo_waypoint_fn = "/home/pats/pats_demo.xml";
    std::string demo_insect_fn = "/home/pats/insect_demo";
    std::string calib_fn = "/home/pats/calib_now";
    std::string beep_fn = "/home/pats/beep_now";
    std::thread thread;
    bool initialized = false;

    navigation::DroneNavigation * _dnav;
    DroneController * _dctrl;
    MultiModule * _rc;
    Cam * _cam;
    tracking::TrackerManager * _trackers;
    bool _log_replay_mode = false;
    int reset_cnt = 0;

    bool exit_thread = false;

    void write_commandcenter_status_image();
    void write_commandcenter_status_file();
    void check_commandcenter_triggers();
    void background_worker();

};
