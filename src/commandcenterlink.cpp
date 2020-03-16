#include "common.h"
#include "commandcenterlink.h"

#include <mutex>
#include <thread>
#include <unistd.h> //usleep
#include<iostream>

void CommandCenterLink::init(bool log_replay_mode,navigation::DroneNavigation * dnav,DroneController * dctrl,MultiModule * rc,Cam * cam,tracking::TrackerManager * trackers) {
    remove(demo_insect_fn.c_str());
    remove(demo_waypoint_fn.c_str());
    remove(calib_fn.c_str());
    remove(beep_fn.c_str());

    _dnav = dnav;
    _dctrl = dctrl;
    _rc = rc;
    _cam = cam;
    _trackers = trackers;
    _log_replay_mode = log_replay_mode;

    thread = std::thread(&CommandCenterLink::background_worker,this);
    initialized = true;
}

void CommandCenterLink::close() {
    exit_thread = true;
    if (initialized) {
        std::cout << "Closing CommandCenterLink" << std::endl;
        thread.join();
    }
    reset_commandcenter_status_file("Closed");
}

void CommandCenterLink::background_worker() {
    int fps = static_cast<int>(1.f/pparams.fps * 1e6f);
    while (!exit_thread) {
        if (!_log_replay_mode)
            check_commandcenter_triggers();
        if (!reset_cnt)
            write_commandcenter_status_file();
        else
            reset_cnt--;

        usleep(fps);
    }
}

void CommandCenterLink::trigger_demo_flight_from_log(std::string replay_dir) {
    if (pparams.joystick == rc_none && _dctrl->joy_takeoff_switch()) {
        _dnav->demo_flight(replay_dir + "/pats_demo.xml");
    }
}

//if put a demo flightplan.xml in demo_fn, this will trigger an automatic demo flight
void CommandCenterLink::check_commandcenter_triggers() {
    static int demo_div_cnt = 0;
    demo_div_cnt = (demo_div_cnt + 1) % pparams.fps; // only check once per second, to save cpu
    if (!demo_div_cnt) {
        if (pparams.op_mode == op_modes::op_mode_deployed) {
            if (file_exist(demo_waypoint_fn)) {
                std::cout << "Waypoint demo!" << std::endl;
                _dnav->demo_flight(demo_waypoint_fn);
                _dctrl->joy_takeoff_switch_file_trigger(true);
                rename(demo_waypoint_fn.c_str(),"./logging/pats_demo.xml");
            }
        }
        if (file_exist(demo_insect_fn)) {
            std::cout << "Replay insect demo!" << std::endl;
            _trackers->init_replay_moth(demo_insect_fn);
            remove(demo_insect_fn.c_str());
        }
        if (file_exist(calib_fn)) {
            std::cout << "Calibrating drone!" << std::endl;
            _rc->calibrate_acc();
            remove(calib_fn.c_str());
        }
        if (file_exist(beep_fn)) {
            std::cout << "Beeping drone!" << std::endl;
            _rc->beep();
            remove(beep_fn.c_str());
        }
    } else {
        _dctrl->joy_takeoff_switch_file_trigger(false);
    }
}

void CommandCenterLink::write_commandcenter_status_file() {

    static std::string nav_status = "";
    bool status_update_needed = nav_status.compare(_dnav->navigation_status()) != 0;
    nav_status = _dnav->navigation_status();

    static double prev_imwrite_time = -pparams.live_image_frq;
    if (_cam->frame_time() - prev_imwrite_time > pparams.live_image_frq && pparams.live_image_frq >= 0) {
        prev_imwrite_time = _cam->frame_time();
        write_commandcenter_status_image();
        status_update_needed = true;
    }
    if (status_update_needed) {
        std::ofstream status_file;
        status_file.open("../../../../pats_status.txt",std::ofstream::out);
        auto time_now = chrono::system_clock::to_time_t(chrono::system_clock::now());
        status_file << std::put_time(std::localtime(&time_now), "%Y/%m/%d %T") << '\n';
        status_file << "Runtime: " << to_string_with_precision(_cam->frame_time(),1) << "s" << '\n';
        status_file << nav_status << std::endl;
    }
}

void CommandCenterLink::reset_commandcenter_status_file(std::string status_msg) {
    reset_cnt = pparams.fps*3;
    std::ofstream status_file;
    status_file.open("../../../../pats_status.txt",std::ofstream::out);
    auto time_now = chrono::system_clock::to_time_t(chrono::system_clock::now());
    status_file << std::put_time(std::localtime(&time_now), "%Y/%m/%d %T") << '\n';
    status_file << "Runtime: " << 0 << "s" << '\n';
    status_file << status_msg << std::endl;
}

void CommandCenterLink::write_commandcenter_status_image() {
    cv::Mat out = _cam->frameL.clone();
    cvtColor(out,out,CV_GRAY2BGR);
    putText(out,"State: " + _dnav->navigation_status() + " " + _trackers->mode_str() + " " + _dctrl->flight_mode() +
            " "  + _trackers->dronetracker()->drone_tracking_state(),cv::Point(5,14),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));
    putText(out,"Time:       " + to_string_with_precision(_cam->frame_time(),2),cv::Point(5,28),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));

    cv::imwrite("../../../../pats_monitor_tmp.jpg", out);
}