#include "common.h"
#include "commandcenterlink.h"
#include "trackermanager.h"
#include "benchmarkreader.h"

void CommandCenterLink::init(bool log_replay_mode, Patser *patser, VisionData *visdat) {
    _log_replay_mode = log_replay_mode;
    _visdat = visdat;
    _patser = patser;

    if (!_log_replay_mode) {
        remove(demo_insect_fn.c_str());
        remove(demo_waypoint_fn.c_str());
        remove(calib_fn.c_str());
        remove(beep_fn.c_str());
        remove(shake_fn.c_str());
        remove(benchmark_fn.c_str());
        thread = std::thread(&CommandCenterLink::background_worker, this);
        std::cout << "Commandcenter link initialized" << std::endl;
        initialized = true;
    }
}

void CommandCenterLink::close() {
    exit_thread = true;
    if (initialized) {
        std::cout << "Closing CommandCenterLink" << std::endl;
        thread.join();
        reset_commandcenter_status_file("Closed", false);
    }
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

void CommandCenterLink::check_commandcenter_triggers() {
    static int demo_div_cnt = 0;
    demo_div_cnt = (demo_div_cnt + 1) % pparams.fps; // only check once per second, to save cpu
    if (!demo_div_cnt) {
        if (pparams.op_mode == op_modes::op_mode_x) {
            if (file_exist(demo_waypoint_fn)) {
                std::cout << "Waypoint demo!" << std::endl;
                rename(demo_waypoint_fn.c_str(), (data_output_dir + "pats_demo_trigger.xml").c_str());
                _patser->drone.waypoint_flight(data_output_dir + "pats_demo_trigger.xml");
            }
            if (file_exist(shake_fn)) {
                std::cout << "Shaking_drone!" << std::endl;
                if (!_patser->drone.in_flight())
                    _patser->drone.shake_drone();
                remove(shake_fn.c_str());
            }
            if (file_exist(calib_fn)) {
                std::cout << "Calibrating_drone!" << std::endl;
                if (!_patser->drone.in_flight())
                    _patser->rc()->calibrate_acc();
                remove(calib_fn.c_str());
            }
            if (file_exist(beep_fn)) {
                std::cout << "Beeping_drone!" << std::endl;
                if (!_patser->drone.in_flight())
                    _patser->drone.beep_drone();
                remove(beep_fn.c_str());
            }
            if (file_exist(blink_fn)) {
                std::cout << "Redetecting_drone location!" << std::endl;
                if (!_patser->drone.in_flight())
                    _patser->drone.redetect_drone_location();
                remove(blink_fn.c_str());
            }
            if (file_exist(benchmark_fn)) {
                if (_patser->drone.benchmark_mode) {
                    std::cout << "Benchmark already running!" << std::endl;
                    remove(benchmark_fn.c_str());
                } else {
                    rename(benchmark_fn.c_str(), (pats_flags_folder + "pats_benchmark_trigger.csv").c_str());
                    remove(benchmark_fn.c_str());
                }
            }
            if (file_exist(pats_flags_folder + "pats_benchmark_trigger.csv")) {
                BenchmarkReader benchmark_reader;
                if (!_patser->drone.benchmark_mode) {
                    std::cout << "Parsing benchmark" << std::endl;
                    _patser->drone.benchmark_hash = benchmark_reader.ParseBenchmarkCSV((pats_flags_folder + "pats_benchmark_trigger.csv").c_str());
                    _patser->drone.benchmark_len = benchmark_entries.size();
                    _patser->drone.benchmark_mode = true;

                    if (file_exist(pats_flags_folder + "benchmark_entry.txt")) {
                        std::cout << "Resuming benchmark" << std::endl;
                        std::ifstream file((pats_flags_folder + "benchmark_entry.txt").c_str());
                        std::string str;
                        int _idx = 0;
                        while (std::getline(file, str))
                        {
                            if (_idx == 0)
                                _patser->drone.benchmark_entry_id = std::stoi(str);
                            else if (_idx == 1)
                                _patser->drone.benchmark_time = str;
                            _idx++;
                        }
                        file.close();
                    } else {
                        std::cout << "Starting benchmark" << std::endl;
                        time_t _time_now = chrono::system_clock::to_time_t(chrono::system_clock::now());
                        std::ostringstream oss;
                        oss << std::put_time(std::localtime(&_time_now), "%Y%m%d_%H%M%S");
                        auto str = oss.str();
                        _patser->drone.benchmark_time = str;
                        _patser->drone.benchmark_entry_id = 0;
                        benchmark_reader.WriteBenchmarkEntry(_patser->drone.benchmark_entry_id, _patser->drone.benchmark_time, pats_flags_folder);
                    }
                }

                if (_patser->drone.benchmark_entry_id < _patser->drone.benchmark_len && _patser->drone.benchmark_mode) {
                    static int _ready_cnt = 0;
                    _ready_cnt = (_ready_cnt + 1) % (30); // wait 30 seconds before initializing the next moth, to give quadcopter time to start the chase
                    if (_patser->drone.drone_ready_and_waiting() && !_ready_cnt && !_patser->trackers.monster_alert()) {
                        BenchmarkEntry _current_entry = benchmark_entries[_patser->drone.benchmark_entry_id];
                        _patser->drone.benchmark_entry = _current_entry;

                        if (_current_entry.type == "replay") {
                            _patser->trackers.init_replay_moth(_current_entry.id);
                            _n_replay_moth++;
                        } else if (_current_entry.type == "virtual") {
                            tracking::VirtualMothTracker::moth_behavior_type _moth_behavior = {tracking::VirtualMothTracker::trigger_type(_current_entry.evasion_trigger), tracking::VirtualMothTracker::evasion_type(_current_entry.evasion_type)};
                            _patser->trackers.init_virtual_moth(&(_patser->drone.control), _moth_behavior, {_current_entry.pos_x, _current_entry.pos_y, _current_entry.pos_z}, {_current_entry.vel_x, _current_entry.vel_y, _current_entry.vel_z});
                            _n_replay_moth++;
                        } else {
                            std::cout << "Unknown benchmark type: " << benchmark_entries[0].type << std::endl;
                        }
                        _patser->drone.benchmark_entry_id++;
                        benchmark_reader.WriteBenchmarkEntry(_patser->drone.benchmark_entry_id, _patser->drone.benchmark_time, pats_flags_folder);
                    }
                } else {
                    remove((pats_flags_folder + "pats_benchmark_trigger.csv").c_str());
                    remove((pats_flags_folder + "benchmark_entry.txt").c_str());
                    _patser->drone.benchmark_mode = false;
                }
            } else if (!file_exist(pats_flags_folder + "benchmark_entry.txt")) {
                _patser->drone.benchmark_mode = false;
            }
        }
        if (file_exist(demo_insect_fn)) {
            _n_replay_moth++;
            std::cout << "Replay insect demo!" << std::endl;
            _patser->trackers.init_replay_moth(demo_insect_fn);
            remove(demo_insect_fn.c_str());
        }
        if (file_exist(virtual_insect_fn)) {
            _n_replay_moth++;
            std::cout << "Virtual insect!" << std::endl;
            _patser->trackers.init_virtual_moth(&(_patser->drone.control));
            remove(virtual_insect_fn.c_str());
        }
    }
}

void CommandCenterLink::write_commandcenter_status_file() {
    static std::string nav_status = "";
    bool status_update_needed = nav_status.compare(_patser->drone.nav.navigation_status()) != 0;
    nav_status = _patser->drone.nav.navigation_status();

    static double prev_imwrite_time = -pparams.live_image_frq;
    if (_time_since_start - prev_imwrite_time > pparams.live_image_frq && pparams.live_image_frq > 0) {
        prev_imwrite_time = _time_since_start;
        write_commandcenter_status_image();
        status_update_needed = true;
    }
    if (status_update_needed) {
        ofstream status_file(pats_folder + "status/status.txt", std::ofstream::out);
        auto time_now = chrono::system_clock::to_time_t(chrono::system_clock::now());
        status_file << std::put_time(std::localtime(&time_now), "%Y/%m/%d %T") << '\n';
        status_file << "Runtime: " << to_string_with_precision(_time_since_start, 1) << "s" << '\n';
        status_file << nav_status << std::endl;
        status_file << "cell v: " << to_string_with_precision(_patser->rc()->telemetry.batt_cell_v, 2) << std::endl;
        status_file << "arming: " << _patser->rc()->telemetry.arming_state << std::endl;
        status_file << "rssi: " << static_cast<int>(_patser->rc()->telemetry.rssi) << std::endl;
        status_file << "drone att: " << to_string_with_precision(_patser->rc()->telemetry.roll, 1) << ", " << to_string_with_precision(_patser->rc()->telemetry.pitch, 1) << std::endl;
        status_file << "cam angles: " << to_string_with_precision(_visdat->camera_roll(), 1) << ", " << to_string_with_precision(_visdat->camera_pitch(), 1) << std::endl;
        status_file.close();
    }
}

void CommandCenterLink::reset_commandcenter_status_file(std::string status_msg, bool never_overwrite) {
    if (initialized) {
        if (_never_overwrite)
            return;
        _never_overwrite = never_overwrite;

        reset_cnt = pparams.fps * 3;
        ofstream status_file(pats_folder + "status/status.txt", std::ofstream::out);
        auto time_now = chrono::system_clock::to_time_t(chrono::system_clock::now());
        status_file << std::put_time(std::localtime(&time_now), "%Y/%m/%d %T") << '\n';
        status_file << "Runtime: " << 0 << "s" << '\n';
        status_file << status_msg << std::endl;
        status_file.close();
    }
}

void CommandCenterLink::write_commandcenter_status_image() {
    if (initialized) {
        if (_frame.cols > 0) {
            if (!new_frame_request) {
                if (_frame.cols) {
                    cv::Mat out_rgb;
                    cvtColor(_frame, out_rgb, cv::COLOR_GRAY2BGR);
                    auto time_now = chrono::system_clock::to_time_t(chrono::system_clock::now());
                    std::stringstream date_ss;
                    date_ss << "Time: " << std::put_time(std::localtime(&time_now), "%Y/%m/%d %T");
                    putText(out_rgb, date_ss.str(), cv::Point(5, 17), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
                    putText(out_rgb, "State: " +  _patser->state_str() + " " + _patser->trackers.mode_str() + " " + _patser->drone.control.flight_mode_str() + " "  + _patser->trackers.drone_tracking_state(), cv::Point(5, 37), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
                    putText(out_rgb, "Runtime: " + to_string_with_precision(_time_since_start, 2), cv::Point(5, 57), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);

                    if (pparams.op_mode == op_mode_x) {
                        cv::circle(out_rgb, _patser->drone.tracker.pad_im_location(), _patser->drone.tracker.pad_im_size() / 2, cv::Scalar(0, 255, 0));
                        std::string drone_str = "Drone: " + _patser->baseboard()->charging_state_str() ;
                        putText(out_rgb, drone_str, cv::Point(5, 77), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
                    }
                    cv::imwrite(pats_folder + "status/live.jpg", out_rgb);
                }
                new_frame_request = 1;
            }
        } else
            new_frame_request = 1;
    }
}