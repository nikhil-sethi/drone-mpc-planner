#include "common.h"
#include "commandcenterlink.h"
#include "trackermanager.h"

#include <thread>
#include <unistd.h> //usleep
#include<iostream>

void CommandCenterLink::init(bool log_replay_mode, navigation::DroneNavigation *dnav, DroneController *dctrl, Rc *rc, tracking::TrackerManager *trackers, VisionData *visdat) {
    remove(demo_insect_fn.c_str());
    remove(demo_waypoint_fn.c_str());
    remove(calib_fn.c_str());
    remove(beep_fn.c_str());
    remove(shake_fn.c_str());

    _dnav = dnav;
    _dctrl = dctrl;
    _rc = rc;
    _trackers = trackers;
    _log_replay_mode = log_replay_mode;
    _visdat = visdat;

    if (!_log_replay_mode) {
        thread = std::thread(&CommandCenterLink::background_worker, this);
        initialized = true;
    }
}

void CommandCenterLink::close() {
    exit_thread = true;
    if (initialized) {
        std::cout << "Closing CommandCenterLink" << std::endl;
        thread.join();
    }
    reset_commandcenter_status_file("Closed", false);
}

void CommandCenterLink::background_worker() {
    int fps = static_cast<int>(1.f / pparams.fps * 1e6f);
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

void CommandCenterLink::trigger_demo_flight_from_log(std::string replay_dir, int tracker_mode) {
    static tracking::TrackerManager::detection_mode prev_tracker_mode = tracking::TrackerManager::detection_mode::mode_idle;

    if (pparams.joystick == rc_none && static_cast<tracking::TrackerManager::detection_mode>(tracker_mode) == tracking::TrackerManager::detection_mode::mode_drone_only && prev_tracker_mode != tracking::TrackerManager::detection_mode::mode_drone_only) {
        if (file_exist(replay_dir + "/pats_demo.xml"))
            _dnav->demo_flight(replay_dir + "/pats_demo.xml");
        else
            _dnav->demo_flight(replay_dir + "/flightplan.xml");
    }
    prev_tracker_mode = static_cast<tracking::TrackerManager::detection_mode>(tracker_mode);
}

void CommandCenterLink::check_commandcenter_triggers() {
    static int demo_div_cnt = 0;
    demo_div_cnt = (demo_div_cnt + 1) % pparams.fps; // only check once per second, to save cpu
    if (!demo_div_cnt) {
        if (pparams.op_mode == op_modes::op_mode_waypoint) {
            if (file_exist(demo_waypoint_fn)) {
                std::cout << "Waypoint demo!" << std::endl;
                _dnav->demo_flight(demo_waypoint_fn);
                _dctrl->joy_takeoff_switch_file_trigger(true);
                rename(demo_waypoint_fn.c_str(), "./logging/pats_demo.xml");
            }
        }
        if (file_exist(demo_insect_fn)) {
            _n_replay_moth++;
            std::cout << "Replay insect!" << std::endl;
            _trackers->init_replay_moth(demo_insect_fn);
            remove(demo_insect_fn.c_str());
        }
        if (file_exist(virt_insect_fn)) {
            _n_replay_moth++;
            std::cout << "Virtual insect!" << std::endl;
            _trackers->init_virtual_moth();
            remove(virt_insect_fn.c_str());
        }
        if (file_exist(shake_fn)) {
            std::cout << "Shaking drone!" << std::endl;
            if (!_dnav->drone_flying())
                _dnav->shake_drone();
            remove(shake_fn.c_str());
        }
        if (file_exist(calib_fn)) {
            std::cout << "Calibrating drone!" << std::endl;
            if (!_dnav->drone_flying())
                _rc->calibrate_acc();
            remove(calib_fn.c_str());
        }
        if (file_exist(beep_fn)) {
            std::cout << "Beeping drone!" << std::endl;
            if (!_dnav->drone_flying())
                _dnav->set_drone_problem();
            remove(beep_fn.c_str());
        }
        if (file_exist(blink_fn)) {
            std::cout << "Redetecting drone location!" << std::endl;
            if (!_dnav->drone_flying())
                _dnav->redetect_drone_location();
            remove(blink_fn.c_str());
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
    if (_time_since_start - prev_imwrite_time > pparams.live_image_frq && pparams.live_image_frq > 0) {
        prev_imwrite_time = _time_since_start;
        write_commandcenter_status_image();
        status_update_needed = true;
    }
    if (status_update_needed) {
        std::ofstream status_file;
        status_file.open("../../../../pats/status/status.txt", std::ofstream::out);
        auto time_now = chrono::system_clock::to_time_t(chrono::system_clock::now());
        status_file << std::put_time(std::localtime(&time_now), "%Y/%m/%d %T") << '\n';
        status_file << "Runtime: " << to_string_with_precision(_time_since_start, 1) << "s" << '\n';
        status_file << nav_status << std::endl;
        status_file << "cell v: " << to_string_with_precision(_rc->telemetry.batt_cell_v, 2) << std::endl;
        status_file << "arming: " << _rc->telemetry.arming_state << std::endl;
        status_file << "rssi: " << static_cast<int>(_rc->telemetry.rssi) << std::endl;
        status_file << "drone att: " << to_string_with_precision(_rc->telemetry.roll, 1) << ", " << to_string_with_precision(_rc->telemetry.pitch, 1) << std::endl;
        status_file << "cam angles: " << to_string_with_precision(_visdat->camera_roll, 1) << ", " << to_string_with_precision(_visdat->camera_pitch, 1) << std::endl;
        status_file.close();
    }
}

void CommandCenterLink::reset_commandcenter_status_file(std::string status_msg, bool never_overwrite) {
    if (_never_overwrite)
        return;
    _never_overwrite = never_overwrite;

    reset_cnt = pparams.fps * 3;
    std::ofstream status_file;
    status_file.open("../../../../pats/status/status.txt", std::ofstream::out);
    auto time_now = chrono::system_clock::to_time_t(chrono::system_clock::now());
    status_file << std::put_time(std::localtime(&time_now), "%Y/%m/%d %T") << '\n';
    status_file << "Runtime: " << 0 << "s" << '\n';
    status_file << status_msg << std::endl;
    status_file.close();
}

void CommandCenterLink::write_commandcenter_status_image() {
    if (_frame.cols > 0) {
        if (!new_frame_request) {
            if (_frame.cols) {
                cv::Mat out_rgb;
                cvtColor(_frame, out_rgb, cv::COLOR_GRAY2BGR);
                putText(out_rgb, "State: " + _dnav->navigation_status() + " " + _trackers->mode_str() + " " + _dctrl->flight_mode() +
                        " "  + _trackers->dronetracker()->drone_tracking_state(), cv::Point(5, 14), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
                putText(out_rgb, "Time:       " + to_string_with_precision(_time_since_start, 2), cv::Point(5, 28), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
                if (pparams.op_mode != op_mode_monitoring)
                    cv::circle(out_rgb, _trackers->dronetracker()->pad_im_location(), _trackers->dronetracker()->pad_im_size() / 2, cv::Scalar(0, 255, 0));

                cv::imwrite("../../../../pats/status/monitor_tmp.jpg", out_rgb);
            }
            new_frame_request = 1;
        }
    } else
        new_frame_request = 1;
}
