#include <mutex>
#include <thread>
#include <fstream>      // std::ifstream
#include <sstream>
#include <iostream>
#include <iomanip>
#include <csignal>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h> //usleep
#include <ctime>
#include <sys/stat.h>
#include <condition_variable>
#include<iostream>
#include <chrono>
#include <experimental/filesystem>

#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "pats.h"
#include "common.h"
#include "flightarea/flightarea.h"
#include "smoother.h"
#include "multimodule.h"
#include "replayrc.h"
#include "stopwatch.h"
#include "drone.h"
#include "gstream.h"
#include "vizs.h"
#include "insecttracker.h"
#include "trackermanager.h"
#include "logreader.h"
#include "visiondata.h"
#include "commandcenterlink.h"
#include "filecam.h"
#include "generatorcam.h"
#include "realsense.h"
#include "airsimcam.h"
#include "airsimcontroller.h"
#include "interceptor.h"
#include "baseboardlink.h"
#include "daemonlink.h"
#if ROSVIS
#include "rosvisualizerdatacollector.h"
#endif


using namespace cv;
using namespace std;

/***********Variables****************/
bool exit_now = false;
bool exit_watchdog_thread = false;
bool watchdog = true;
volatile std::sig_atomic_t term_sig_fired;
int received_img_count;
int encoded_img_count = 0;
int n_fps_warnings = 0;
filtering::Smoother fps_smoothed;
GStream video_render, video_raw;
time_t start_datetime;
float light_level = 0;
auto wdt_timeout = 10s;

enum enable_window_modes {
    enable_window_disabled,
    enable_window_start_only,
    enable_window_end_only,
    enable_window_not_passes_midnight,
    enable_window_passes_midnight
};

xmls::PatsParameters pparams;
xmls::DroneParameters dparams;
stopwatch_c stopWatch;
std::string data_output_dir;
std::time_t enable_window_start_time = 0;
std::time_t enable_window_end_time = 0;
enable_window_modes enable_window_mode = enable_window_disabled;
std::time_t start_time = 0;
std::time_t periodic_stop_time = 0;
bool draw_plots = false;
bool realsense_reset = false;
bool log_replay_mode = false;
bool flight_replay_mode = false;
bool insect_replay_mode = false;
bool generator_mode = false;
bool airsim_mode = false;
bool airsim_wp_mode = false;
bool render_mode = false;
bool watchdog_skip_video_delay_override = false;
uint16_t rc_id = 0;
std::string replay_dir, replay_video_fn;
std::string airsim_map;
std::string logger_fn; //contains filename of current log # for insect logging (same as video #)
std::string pats_xml_fn = "", drone_xml_fn = "";

std::ofstream logger;
std::ofstream logger_video_ids;
std::unique_ptr<RC> rc;

Visualizer visualizer;
#if ROSVIS
RosVisualizerDataCollector rosvis;
#endif
logging::LogReader logreader;
CommandCenterLink cmdcenter;
BaseboardLink baseboard_link;
DaemonLink daemon_link;
Patser patser;
std::unique_ptr<Cam> cam;
VisionData visdat;


/****Threadpool*******/
#define NUM_OF_THREADS 1
struct Processer {
    int id;
    std::thread *thread;
    std::mutex m1, m2;
    std::condition_variable data_processed, new_data;
    bool data_is_new = false;
    bool data_is_processed = true;
    StereoPair *frame;
};
Processer tp[NUM_OF_THREADS];
std::condition_variable cv_watchdog;
std::mutex cv_m_watchdog;
std::thread thread_watchdog;
StereoPair *prev_frame;

/*******Private prototypes*********/
void process_frame(StereoPair *frame);
void process_video();
int main(int argc, char **argv);
bool handle_key(double time);
void save_results_log();
void print_warnings();
void print_terminal(StereoPair *frame);
void encode_video(StereoPair *frame);
void update_enable_window();
void check_exit_conditions(double time, bool escape_key_pressed);
void close(bool sig_kill);
void watchdog_worker(void);
void communicate_state(executor_states s);

/************ code ***********/
void process_video() {
    stopWatch.Start();
    start_datetime = chrono::system_clock::to_time_t(chrono::system_clock::now());
    //main while loop:
    while (!exit_now) {
        auto frame = cam->update();
        std::unique_lock<std::mutex> lk(tp[0].m2, std::defer_lock);
        tp[0].data_processed.wait(lk, []() {return tp[0].data_is_processed; });
        tp[0].data_is_processed = false;

        bool escape_key_pressed = false;
        if (pparams.has_screen || render_mode) {
            static int speed_div;
            if (!(speed_div++ % 4) || (((log_replay_mode || generator_mode) && (!cam->turbo || render_mode)) || cam->frame_by_frame)) {
#if ROSVIS
                rosvis.update();
#endif
                if (pparams.has_screen) {
                    visualizer.paint();
                    escape_key_pressed = handle_key(frame->time);
                }
                if (render_mode)
                    visualizer.render();

            }
        }

        tp[0].m1.lock();
        tp[0].frame = frame;
        tp[0].data_is_new = true;
        tp[0].new_data.notify_one();
        tp[0].m1.unlock();

        light_level = visdat.light_level();
        check_exit_conditions(frame->time, escape_key_pressed);

        encode_video(frame);
        print_terminal(frame);
        received_img_count++;

        while (cam->frame_by_frame) {
            unsigned char k = cv::waitKey(0);
            if (k == 'f')
                break;
            else if (k == ' ') {
                cam->frame_by_frame = false;
                break;
            }
        }
    } // main while loop
    std::cout << "Exiting main loop" << std::endl;
}

#ifdef PATS_PROFILING
std::chrono::_V2::system_clock::time_point time_last_process_frame_called;
float process_frame_call_dt = -1;
#endif

void process_frame(StereoPair *frame) {
    std::chrono::_V2::system_clock::time_point t_start_process_frame = std::chrono::high_resolution_clock::now();
#ifdef PATS_PROFILING
    if ((t_start_process_frame - time_last_process_frame_called).count() > 0 && time_last_process_frame_called != std::chrono::_V2::system_clock::time_point())
        process_frame_call_dt = (t_start_process_frame - time_last_process_frame_called).count() * 1e-6; //Don't cout here because it messes up cout in different thread.
    time_last_process_frame_called = t_start_process_frame;
#endif

    if (log_replay_mode && pparams.op_mode != op_mode_c) {
        if (logreader.current_frame_number(frame->rs_id)) {
            exit_now = true;
            return;
        }
        static_cast<ReplayRC *>(rc.get())->telemetry_from_log(frame->time);
        static_cast<FileCam *>(cam.get())->inject_log(logreader.current_entry);
        patser.trackers.process_replay_moth(frame->rs_id);
        if (logreader.log_drone()->current_entry.rs_id == frame->rs_id) {
            baseboard_link.inject_log(logreader.log_drone()->current_entry.charging_state);
            if (frame->rs_id > logreader.log_drone()->start_rs_id())
                patser.drone.inject_log(logreader.log_drone()->current_entry, frame->rs_id);
        } else {
            baseboard_link.inject_log();
        }
    }

    visdat.update(frame);
    prev_frame->processing = false;
    prev_frame = frame;

    auto time_now = chrono::system_clock::to_time_t(chrono::system_clock::now());
    logger << received_img_count << ";"
           << frame->rs_id << ";"
           << std::put_time(std::localtime(&time_now), "%Y/%m/%d %T") << ";"
           << frame->time << ";"
           << light_level << ";"
           << cam->measured_exposure() << ";" << cam->measured_gain() << ";"
           << baseboard_link.charging_state_str() << ";" << static_cast<uint16_t>(baseboard_link.charging_state()) << ";";

    if (pparams.op_mode == op_mode_x) {
        double max_opti_time = 1000. / 2 / pparams.fps - ((std::chrono::high_resolution_clock::now() - t_start_process_frame).count()) * 1e-6;
        patser.interceptor.max_optimization_time(max_opti_time * 1e-3);
    }

#ifdef PATS_PROFILING
    std::cout << "timing (available_opt_time): " << max_opti_time << "ms" << std::endl;
#endif
    patser.update(frame->time);

    if (pparams.drone != drone_none && dparams.tx != tx_none)
        rc->send_commands(frame->time);
    baseboard_link.time(frame->time);
    daemon_link.time(frame->time);

    if (pparams.has_screen || render_mode) {
        visualizer.add_plot_sample();
        visualizer.update_tracker_data(visdat.frameL, patser.drone.nav.setpoint().pos(), frame->time, draw_plots);
        if (flight_replay_mode)
            visualizer.draw_drone_from_log(logreader.log_drone()->current_entry);
        if (pparams.video_render && !exit_now) {
            if (log_replay_mode)
                video_render.block(); // only use this for rendering
            video_render.write(visualizer.trackframe);

        }
    }

    if (!log_replay_mode)
        cmdcenter.update(frame->left, frame->time);

    watchdog = true;
    logger << '\n';
#ifdef PATS_PROFILING
    if (process_frame_call_dt > 0)
        std::cout << "timing (process_frame_call_dt): " << process_frame_call_dt << "ms" << std::endl;
    std::chrono::_V2::system_clock::time_point t_end_process_frame = std::chrono::high_resolution_clock::now();
    std::cout << "timing (process_frame): " << (t_end_process_frame - t_start_process_frame).count() * 1e-6 << "ms" << std::endl;
#endif
}


void communicate_state(executor_states s) {
    if (!log_replay_mode) {
        daemon_link.queue_executor_state(s);
        baseboard_link.executor_state(s, patser.drone.issues(), light_level);
    }
}

void init_insect_log(int n) {
    patser.trackers.init_replay_moth(n);
}

bool handle_key(double time [[maybe_unused]]) {
    if (exit_now)
        return true;
    if (!visualizer.tracking_viz_initialized())
        return false;

    unsigned char key = cv::waitKey(1);
    key = key & 0xff;
    if (key == 27)   //esc
        return true;

    switch (key) {
        case 'b':
            rc->bind(true);
            break;
        case 's':
            patser.drone.shake_drone();
            break;
        case 'B':
            rc->beep();
            break;
        case 'c':
            rc->calibrate_acc();
            break;
        case 'l':
            patser.drone.redetect_drone_location();
            break;
        case 'p':
            if (log_replay_mode || generator_mode)
                draw_plots = true;
            break;
        case '[':
            if (log_replay_mode || generator_mode)
                patser.trackers.enable_trkr_viz();
            break;
        case ']':
            if (log_replay_mode || generator_mode)
                patser.trackers.enable_blob_viz();
            break;
        case '\\':
            if (log_replay_mode || generator_mode)
                patser.trackers.enable_draw_stereo_viz();
            break;
        case ';':
            if (log_replay_mode || generator_mode)
                visualizer.enable_draw_noise_viz();
            break;
        case '\'':
            if (log_replay_mode || generator_mode)
                visualizer.enable_draw_exposure_viz();
            break;
        case 'o':
            patser.drone.control.LED(true);
            patser.drone.nav.nav_flight_mode(navigation::nfm_manual);
            break;
        case '1':
            init_insect_log(56);
            break;
        case '!':
            if (patser.drone.drone_ready_and_waiting()) {
                std::string fp = "../xml/flightplans/thrust-calib.xml";
                std::cout << "Flightplan trigger:" << fp << std::endl;
                patser.drone.waypoint_flight(fp);
            }
            break;
        case '2':
            init_insect_log(66);
            break;
        case '@':
            if (patser.drone.drone_ready_and_waiting()) {
                std::string fp = "../xml/flightplans/simple-demo-darkroom.xml";
                std::cout << "Flightplan trigger:" << fp << std::endl;
                patser.drone.waypoint_flight(fp);
            }
            break;
        case '3':
            init_insect_log(58);
            break;
        case '#':
            if (patser.drone.drone_ready_and_waiting()) {
                std::string fp = "../xml/flightplans/simple-demo-koppert.xml";
                std::cout << "Flightplan trigger:" << fp << std::endl;
                patser.drone.waypoint_flight(fp);
            }
            break;
        case '$':
            if (patser.drone.drone_ready_and_waiting()) {
                std::string fp = "../xml/flightplans/bejo.xml";
                std::cout << "Flightplan trigger:" << fp << std::endl;
                patser.drone.waypoint_flight(fp);
            }
            break;
        case '4':
            init_insect_log(54);
            break;
        case '5':
            init_insect_log(63);
            break;
        case '6':
            init_insect_log(64);
            break;
        case '7':
            init_insect_log(20);
            break;
        case '8':
            init_insect_log(61);
            break;
        case '9':
            init_insect_log(15);
            break;
        case '0':
            init_insect_log(62);
            break;
        case 'v':
            patser.trackers.init_virtual_moth(&patser.drone.control);
            break;
        case 82: // arrow up
            patser.drone.nav.manual_trigger_next_wp();
            break;
        case 84: // arrow down
            patser.drone.nav.manual_trigger_prev_wp();
            break;
        case ' ':
        case 'f':
            if (log_replay_mode || generator_mode || airsim_mode)
                cam->frame_by_frame = true;
            break;
        case 't':
            cam->turbo = !cam->turbo;
            break;
        case '.':
            static_cast<FileCam *>(cam.get())->skip(5);
            break;
        case ',':
            static_cast<FileCam *>(cam.get())->back_one_sec();
            break;
        case 'a':
            rc->arm(bf_armed);
            patser.drone.nav.nav_flight_mode(navigation::nfm_waypoint);
            break;
        case 'h':
            rc->arm(bf_armed);
            patser.drone.nav.nav_flight_mode(navigation::nfm_hunt);
            break;
        case 'd':
            rc->arm(bf_disarmed);
            patser.drone.nav.nav_flight_mode(navigation::nfm_manual);
            break;
    } // end switch key

    return false;
}

void encode_video(StereoPair *frame) {
    if (pparams.video_raw && ! cam->frame_lagging()) {
        // auto tmpf = frame->left.clone();
        // putText(tmpf, to_string(frame->rs_id), cv::Point(3, 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));

        int frame_written = video_raw.write(frame->left, frame->right);
        logger_video_ids << encoded_img_count << ";" << received_img_count << ";" << frame->rs_id << ";" << frame->time << '\n';
        if (!frame_written)
            encoded_img_count++;
        else {
            std::cout << "Video frame write PROBLEM: " << frame_written << std::endl;
        }
    }
}

void check_exit_conditions(double time, bool escape_key_pressed) {

    if (render_mode && flight_replay_mode && patser.drone.nav.drone_resetting_yaw()) {
        std::cout << "Render mode: yaw reset detected. Stopping." << std::endl;
        communicate_state(es_user_restart);
        exit_now = true;
    }
    if (escape_key_pressed) {
        communicate_state(es_user_restart);
        exit_now = true;
    }

    if (!(received_img_count % pparams.fps)) {
        if (term_sig_fired == 2) {
            std::cout << "\nCaught ctrl-c: " << term_sig_fired << std::endl;
            communicate_state(es_user_restart);
            exit_now = true;
        } else if (term_sig_fired == 15) {
            std::cout << "\nCaught TERM signal: " << term_sig_fired << std::endl;
            communicate_state(es_user_restart);
            exit_now = true;
        } else if (term_sig_fired) {
            std::cout << "\nCaught unknown signal: " << term_sig_fired << std::endl;
            communicate_state(es_user_restart);
            exit_now = true;
        }
        update_enable_window();
        if (patser.drone.program_restart_allowed()) {
            if (!log_replay_mode && !generator_mode && !airsim_mode) {
                if ((fabs(time - received_img_count / pparams.fps)) > 60) {
                    std::cout << "Error: more then 60s of frameloss detected. Assuming there's some camera problem. Cowardly exiting." << std::endl;
                    communicate_state(es_realsense_fps_problem);
                    exit_now = true;
                }
                else if ((fps_smoothed.latest() < pparams.fps / 6 * 5 && fps_smoothed.ready() && time < 5)) {
                    std::cout << "Error: Detected FPS warning during start up, assuming there's some camera problem. Cowardly exiting." << std::endl;
                    communicate_state(es_realsense_fps_problem);
                    exit_now = true;
                }
            }
            if (rc->init_package_failure()) {
                exit_now = true;
                communicate_state(es_rc_problem);
                cmdcenter.reset_commandcenter_status_file("MultiModule init package error", true);
            } else if (rc->bf_version_error()) {
                exit_now = true;
                communicate_state(es_drone_version_mismatch);
                cmdcenter.reset_commandcenter_status_file("Betaflight version error", true);
            } else if (rc->bf_uid_error()) {
                if (rc->bf_uid_str() == "hacr") {
                    std::cout << "Detected a drone config that fits, reloading!" << std::endl;
                    pparams.drone = drone_hammer;
                    pparams.serialize(pats_xml_fn);
                } else if (rc->bf_uid_str() == "ancr") {
                    std::cout << "Detected a drone config that fits, reloading!" << std::endl;
                    pparams.drone = drone_anvil_crazybee;
                    pparams.serialize(pats_xml_fn);
                } else if (rc->bf_uid_str() == "ansu") {
                    std::cout << "Detected a drone config that fits, reloading!" << std::endl;
                    pparams.drone = drone_anvil_superbee;
                    pparams.serialize(pats_xml_fn);
                } else if (rc->bf_uid_str() == "andi") {
                    std::cout << "Detected a drone config that fits, reloading!" << std::endl;
                    pparams.drone = drone_anvil_diamond;
                    pparams.serialize(pats_xml_fn);
                } else if (rc->bf_uid_str() == "qutt") {
                    std::cout << "Detected a drone config that fits, reloading!" << std::endl;
                    pparams.drone = drone_qutt;
                    pparams.serialize(pats_xml_fn);
                } else if (rc->bf_uid_str() == "quf4") {
                    std::cout << "Detected a drone config that fits, reloading!" << std::endl;
                    pparams.drone = drone_quf4;
                    pparams.serialize(pats_xml_fn);
                } else if (rc->bf_uid_str() == "quto") {
                    std::cout << "Detected a drone config that fits, reloading!" << std::endl;
                    pparams.drone = drone_quto;
                    pparams.serialize(pats_xml_fn);
                }
                if (!patser.drone.in_flight()) {
                    exit_now = true;
                    communicate_state(es_drone_config_restart);
                    cmdcenter.reset_commandcenter_status_file("Wrong drone config error", true);
                }
            } else if (baseboard_link.exit_now()) {
                exit_now = true;
                communicate_state(es_baseboard_problem);
                cmdcenter.reset_commandcenter_status_file("BaseboardLink problem", true);
                std::cout << " BaseboardLink problem" << std::endl;
            } else if (daemon_link.exit_now()) {
                exit_now = true;
                std::cout << " Daemon link problem" << std::endl;
                cmdcenter.reset_commandcenter_status_file("Daemon link problem", true);
            }

            auto time_now = chrono::system_clock::to_time_t(chrono::system_clock::now());
            auto seconds_to_periodic_stop_time = std::difftime(periodic_stop_time, time_now);
            if (!log_replay_mode && (pparams.periodic_restart_minutes > 0 && seconds_to_periodic_stop_time <= 0)) {
                std::cout << "Initiating periodic restart" << std::endl;
                communicate_state(es_periodic_restart);
                exit_now = true;
            } else if (light_level > pparams.light_level_threshold * 1.05f && pparams.light_level_threshold > 0 && time > 10 && !baseboard_link.contact_problem()) {
                std::cout << "Initiating restart because light level (" << light_level << ") is higher than threshold (" << pparams.light_level_threshold * 1.05f << ")" << std::endl;
                communicate_state(es_light_level_restart);
                exit_now = true;
            } else if (enable_window_mode && !log_replay_mode && (std::difftime(time_now, enable_window_start_time) < 0 || std::difftime(time_now, enable_window_end_time) > 0) && time > 10 && !baseboard_link.contact_problem()) {
                communicate_state(es_enable_window_restart);
                std::cout << "Initiating restart because not in enable window" << std::endl;
                exit_now = true;
            }
            if (cam->frame_loss_cnt() > 500) {
                std::cout << "Error: Too many frames lost, assuming there's some camera problem. Cowardly exiting." << std::endl;
                communicate_state(es_realsense_frame_loss_problem);
                exit_now = true;
            }
        }
    }
}

void print_terminal(StereoPair *frame) {
    //keep track of time and fps
    float t_pc = stopWatch.Read() / 1000.f;
    float t_cam = static_cast<float>(frame->time);
    float t;
    if (log_replay_mode || generator_mode)
        t = t_pc;
    else
        t = t_cam;
    static float prev_time = -1.f / pparams.fps;
    float current_fps = 1.f / (t - prev_time);
    float fps = fps_smoothed.addSample(current_fps);
    if (fps != fps || isinf(fps))
        fps_smoothed.reset();
    prev_time = t;

    std::cout <<
              //   "\r\e[K" <<
              received_img_count <<
              ", R:" << frame->rs_id <<
              ", T: " << to_string_with_precision(frame->time, 2) <<
              " @ " << to_string_with_precision(fps, 1) <<
              " " << patser.state_str();


    if (pparams.op_mode == op_mode_c) {
        std::cout << ", light: " << to_string_with_precision(light_level, 2);
        if (patser.trackers.detections_count())
            std::cout <<
                      ", detections: " << patser.trackers.detections_count() <<
                      ", insects: " << patser.trackers.insects_count();
    } else {
        std:: cout <<
                   ", " << patser.drone.drone_state_str() <<
                   ", light: " << to_string_with_precision(light_level, 2) <<
                   " " << rc->telemetry.batt_cell_v <<
                   "v, rssi: " << static_cast<int>(rc->telemetry.rssi) <<
                   ", att: [" << rc->telemetry.roll << "," << rc->telemetry.pitch << "]" <<
                   ", arm: " << static_cast<int>(rc->telemetry.arming_state) <<
                   ", t_base: " << static_cast<int>(baseboard_link.uptime());
    }
    if (patser.trackers.monster_alert())
        std:: cout << ", monsters: " << patser.trackers.fp_monsters_count();
    std:: cout << std::endl;
    //   std::flush;

    if (rc->telemetry.arming_state && !patser.drone.drone_ready_and_waiting() && rc->arm_command())
        std::cout << rc->arming_state_str() << std::endl;
}

//This is where frames get processed after it was received from the cam in the main thread
void pool_worker(int id) {
    std::unique_lock<std::mutex> lk(tp[id].m1, std::defer_lock);
    while (!exit_now) {
        tp[id].new_data.wait(lk, []() {return tp[0].data_is_new;});
        tp[id].data_is_new = false;
        if (exit_now)
            break;
        process_frame(tp->frame);
        tp[id].m2.lock();
        tp[id].data_is_processed = true;
        tp[id].data_processed.notify_one();
        tp[id].m2.unlock();
    }
    std::cout << "Exit pool thread " << id << std::endl;
    tp[id].m1.unlock();
}
bool threads_initialised = false;
void init_thread_pool() {
    for (uint i = 0; i < NUM_OF_THREADS; i++) {
        tp[i].thread = new thread(&pool_worker, i);
    }
    threads_initialised = true;
}
void close_thread_pool() {
    if (threads_initialised) {
        std::cout << "Stopping threads in pool" << std::endl;
        usleep(1000);
        for (uint i = 0; i < NUM_OF_THREADS; i++) {
            tp[i].data_is_processed = false;
            usleep(100);
            tp[i].new_data.notify_all();
            tp[i].data_is_processed = true;
            usleep(100);
            tp[i].data_processed.notify_all();
            tp[i].m1.unlock();
            tp[i].m2.unlock();
        }
        if (render_mode)
            video_render.manual_unblock();
        for (uint i = 0; i < NUM_OF_THREADS; i++) {
            tp[i].thread->join();
            delete tp[i].thread;
        }

        std::cout << "Threads in pool closed" << std::endl;
        threads_initialised = false;
    }
}

void kill_sig_handler(int s) {
    term_sig_fired = s;
}
void init_terminal_signals() {
    //init ctrl - c catch
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = kill_sig_handler;
    sigIntHandler.sa_flags = SA_NODEFER;
    sigemptyset(&sigIntHandler.sa_mask);
    sigaddset(&sigIntHandler.sa_mask, SIGINT);
    sigaction(SIGINT, &sigIntHandler, 0);

    //init killall catch
    struct sigaction sigTermHandler;
    sigTermHandler.sa_handler = kill_sig_handler;
    sigemptyset(&sigTermHandler.sa_mask);
    sigaddset(&sigTermHandler.sa_mask, SIGTERM);
    sigTermHandler.sa_flags = SA_NODEFER;
    sigaction(SIGTERM, &sigTermHandler, 0);
}

void init_loggers() {
    if (log_replay_mode) {
        data_output_dir = data_output_dir + "replay/";
        if (path_exist(data_output_dir)) {
            std::string rmcmd = "rm -r " + data_output_dir;
            auto res [[maybe_unused]] = std::system(rmcmd.c_str());
        }
        mkdir(data_output_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }

    if (!log_replay_mode) {
        pparams.serialize(data_output_dir + "pats.xml");
        dparams.serialize(data_output_dir + "drone.xml");
    } else if (log_replay_mode && pparams.op_mode == op_mode_x) {
        if (flight_replay_mode) {
            std::experimental::filesystem::path drone_log_fn = std::experimental::filesystem::path(replay_video_fn).filename().replace_extension(".csv");
            drone_log_fn = replay_dir + "/log_" + drone_log_fn.string();
            logreader.init(replay_dir, drone_log_fn.string());
        } else
            logreader.init(replay_dir);
        patser.trackers.init_replay_moth(logreader.replay_moths());

    }

    logger.open(data_output_dir + "log.csv", std::ofstream::out);
    logger << "id;rs_id;time;elapsed;light_level;exposure;gain;charging_state_str;charging_state;";
    logger_fn = data_output_dir + "log" + to_string(0) + ".csv"; // only used with pparams.video_cuts
    if (pparams.video_raw) {
        logger_video_ids.open(data_output_dir + "frames.csv", std::ofstream::out);
        logger_video_ids << "encoded_img_count" << ";" << "imgcount" << ";" << "rs_id" << ";" << "time" << '\n';
    }

    if (rc->connected())
        rc->init_logger();
    baseboard_link.init_logger();
}

void init_video_recorders() {
    /*****init the video writer*****/
    if (pparams.video_render)
        if (video_render.init(pparams.video_render, data_output_dir + "videoRender.mkv", visualizer.viz_frame_size().width, visualizer.viz_frame_size().height, pparams.fps, "192.168.1.255", 5000, true)) {throw std::runtime_error("could not open results video");}
    if (pparams.video_raw)
        if (video_raw.init(pparams.video_raw, data_output_dir + "videoRawLR.mkv", IMG_W, IMG_H * 2, pparams.fps, "192.168.1.255", 5000, false)) {throw std::runtime_error("could not open LR video");}
}

void process_arg(int argc, char **argv) {
    if (argc > 1) {
        for (int i = 1; i < argc; i++) {
            string s = argv[i];
            bool arg_recognized = false;
            if (s.compare("--rs-reset") == 0) {
                realsense_reset = true;
                arg_recognized = true;
            } else if (s.compare("--pats-xml") == 0) {
                arg_recognized = true;
                i++;
                pats_xml_fn = argv[i];
            } else if (s.compare("--drone-xml") == 0) {
                arg_recognized = true;
                i++;
                drone_xml_fn = argv[i];
            } else if (s.compare("--drone-id") == 0) {
                arg_recognized = true;
                i++;
                rc_id = stoi(argv[i]);
            } else if (s.compare("--generator") == 0) {
                arg_recognized = true;
                i++;
                generator_mode = true;
            } else if (s.compare("--log") == 0) {
                arg_recognized = true;
                i++;
                log_replay_mode = true;
                replay_dir = argv[i];
                pats_xml_fn = replay_dir + "/pats.xml";
                drone_xml_fn = replay_dir + "/drone.xml";
            } else if (s.compare("--flight") == 0) {
                //usage example: ./executor --log logging --flight logging/flight1.mkv or ./executor --log logging --flight flight1.mkv
                arg_recognized = true;
                i++;
                if (is_number(argv[i]))
                    replay_video_fn = "flight" + string(argv[i]) + ".mkv";
                else
                    replay_video_fn = argv[i];
                flight_replay_mode = true;
            } else if (s.compare("--insect") == 0) {
                //monitor_render accepts a (concatinated) videorawLR.mkv of monitored insects, but also needs an original logging folder for camera calibraton files.
                //usage example: ./executor --log logging --insect ~/Bla/vogel.mkv or ./executor --log logging --insect insect1.mkv
                arg_recognized = true;
                i++;
                insect_replay_mode = true;
                if (is_number(argv[i]))
                    replay_video_fn = "insect" + string(argv[i]) + ".mkv";
                else
                    replay_video_fn = argv[i];
            } else if (s.compare("--render") == 0) {
                arg_recognized = true;
                render_mode = true;
            } else if (s.compare("--airsim") == 0) {
                arg_recognized = true;
                i++;
                if (HAS_AIRSIM) {
                    airsim_mode = true;
                    airsim_map = (argv[i]) ? argv[i] : "";
                } else {
                    std::cout << "Error: using airsim mode but AirSim is not compiled. Add -DWITH_AIRSIM=TRUE flag when cmaking." << std::endl;
                    exit(1);
                }
            } else if (s.compare("--airsim-wp") == 0) {
                arg_recognized = true;
                i++;
                if (HAS_AIRSIM) {
                    airsim_mode = true;
                    airsim_wp_mode = true;
                    airsim_map = (argv[i]) ? argv[i] : "";
                } else {
                    std::cout << "Error: using airsim mode but AirSim is not compiled. Add -DWITH_AIRSIM=TRUE flag when cmaking." << std::endl;
                    exit(1);
                }
            }
            if (!arg_recognized) {
                std::cout << "Error argument nog recognized: " << argv[i] << std::endl;
                exit(1);
            }
        }
    }

    if (pats_xml_fn.size() == 0) {
        if (file_exist("../../../../pats/xml/pats.xml"))
            pats_xml_fn = "../../../../pats/xml/pats.xml"; // ~/pats/xml/pats.xml
        else
            pats_xml_fn = "../xml/pats.xml"; // default on git repo
    }
}

void check_hardware() {
    communicate_state(es_hardware_check);
    if (pparams.op_mode == op_mode_x && !log_replay_mode && !generator_mode) {
        // Ensure that joystick was found and that we can use it
        if (!patser.drone.control.joystick_ready() && pparams.joystick != rc_none && ! airsim_wp_mode) {
            throw std::runtime_error("no joystick connected.");
        }

        if (dparams.tx == tx_none)
            throw std::runtime_error("Error: tx set to none, but not in monitoring mode");

        // init rc
        if (airsim_mode) {
            rc = std::unique_ptr<RC>(new AirSimController());
        } else if (dparams.tx != tx_none) {
            rc = std::unique_ptr<RC>(new MultiModule());
        }
        if (!rc->connect()) {
            std::ofstream(data_output_dir + "no_multimodule_flag").close();
            std::string rc_type = (airsim_mode) ? "AirSim" : "MultiModule";
            throw std::runtime_error("cannot connect to " + rc_type);
        }
    } else if (log_replay_mode) {
        rc = std::unique_ptr<RC>(new ReplayRC(replay_dir));
    } else {
        rc = std::unique_ptr<RC>(new MultiModule());
    }
    //init cam and check version
    std::cout << "Connecting camera..." << std::endl;
    if (log_replay_mode && !render_mode && replay_video_fn == "") {
        if (file_exist(replay_dir + '/' + FileCam::default_playback_filename()))
            cam = std::unique_ptr<Cam>(new FileCam(replay_dir, &logreader));
        else
            throw std::runtime_error("Could not find a video in the replay folder!");
    } else if (render_mode || replay_video_fn != "") {
        if (file_exist(replay_video_fn))
            cam = std::unique_ptr<Cam>(new FileCam(replay_dir, replay_video_fn));
        else if (file_exist(replay_dir + '/' + replay_video_fn))
            cam = std::unique_ptr<Cam>(new FileCam(replay_dir, replay_dir + '/' + replay_video_fn));
        else
            throw std::runtime_error("Could not find a video file!");
    } else if (generator_mode) {
        cam = std::unique_ptr<Cam>(new GeneratorCam());
        static_cast<GeneratorCam *>(cam.get())->rc(rc.get());
    } else if (airsim_mode) {
        cam = std::unique_ptr<Cam>(new AirSimCam(airsim_map));
    } else {
        cam = std::unique_ptr<Cam>(new Realsense());
        static_cast<Realsense *>(cam.get())->connect_and_check("", 0);
    }
}

void init() {
    wdt_timeout = 5s;
    watchdog = true;
    communicate_state(es_init);
    init_terminal_signals();
    init_loggers();
    init_video_recorders();

    communicate_state(es_realsense_init);
    cam->init();
    if (flight_replay_mode)
        static_cast<FileCam *>(cam.get())->start_cnt(logreader.log_drone()->video_start_rs_id());
    else if (log_replay_mode)
        static_cast<FileCam *>(cam.get())->start_cnt(logreader.start_rs_id());
    if (render_mode)
        cam->turbo = true;
    prev_frame = cam->current();
    communicate_state(es_init_vision);
    visdat.init(cam.get());

    communicate_state(es_init);
    rc->init(rc_id);
    patser.init(&logger, rc_id, rc.get(), replay_dir, cam.get(), &visdat, &baseboard_link);
    if (flight_replay_mode) {
        patser.init_flight_replay(replay_dir, logreader.log_drone()->flight_id());
        visdat.init_replay(replay_dir, logreader.log_drone()->flight_id());
    } else if (insect_replay_mode) {
        patser.init_insect_replay();
        visdat.init_replay(replay_dir);
    } else if (log_replay_mode) {
        patser.drone.init_full_log_replay(replay_dir);
    }

    cmdcenter.init(log_replay_mode || generator_mode || airsim_mode, &patser, &visdat);

    if (pparams.has_screen || render_mode) {
        visualizer.init(&visdat, &patser, log_replay_mode);
        if (generator_mode) {
            visualizer.set_generator_cam(static_cast<GeneratorCam *>(cam.get()));
        }
#if ROSVIS
        rosvis.init(&patser);
#endif
    }

    init_thread_pool();

    start_time = chrono::system_clock::to_time_t(chrono::system_clock::now());
    auto start_minute = (*std::localtime(std::addressof(start_time))).tm_min;
    int minutes_to_periodic_restart = roundf((start_minute + pparams.periodic_restart_minutes - 5.f) / pparams.periodic_restart_minutes) * pparams.periodic_restart_minutes + 5 - start_minute;
    periodic_stop_time = chrono::system_clock::to_time_t(chrono::system_clock::now() + std::chrono::minutes(minutes_to_periodic_restart));
    std::cout << "Periodic restart scheduled at: " << std::put_time(std::localtime(&periodic_stop_time), "%T") << std::endl;
    if (render_mode)
        wdt_timeout = 15s;
    else
        wdt_timeout = 1s;
    watchdog = true;

    fps_smoothed.init(100);

    logger << '\n'; // this concludes the header log line
    std::cout << "Main init successfull" << std::endl;

    communicate_state(es_init_vision); // patser still has to do some initializing, like motion calibration
}

void close_before_running() {
    exit_now = true;
    std::this_thread::sleep_for(1s);
    baseboard_link.close_link();
    daemon_link.close_link();
    exit_watchdog_thread = true;
    std::cout << "Wait for watchdog thread..." << std::endl;
    cv_watchdog.notify_one();
    thread_watchdog.join();
}

void close(bool sig_kill) {
    std::cout << "Closing" << std::endl;
    wdt_timeout = 5s;
    watchdog = true;

    if (cam)
        cam->stop(); //cam needs to be closed after dnav, because of the camview class!

    communicate_state(es_closing);
    cmdcenter.reset_commandcenter_status_file("Closing", false);

    if (pparams.has_screen)
        cv::destroyAllWindows();

    if (received_img_count > 0)
        save_results_log();

    /*****Close everything down*****/
    if (!sig_kill)
        close_thread_pool();

    if (!log_replay_mode)
        rc->close();
    if (pparams.has_screen || render_mode)
        visualizer.close();
    visdat.close();
    baseboard_link.close_link();
    patser.close();
    if (cam)
        cam->close(); //cam needs to be closed after dnav, because of the camview class!

    std::cout << "Closing logs" << std::endl;
    logger << std::flush;
    logger.close();
    if (pparams.video_raw) {
        logger_video_ids << std::flush;
        logger_video_ids.close();
        video_raw.close();
    }
    if (pparams.video_render)
        video_render.close();

    cmdcenter.close();
    daemon_link.close_link();

    print_warnings();
    std::cout << "Releasing cam..." << std::endl;
    if (cam)
        cam.release();
    std::cout << "Releasing rc..." << std::endl;
    if (rc)
        rc.release();
    exit_watchdog_thread = true;
    std::cout << "Watchdog thread join..." << std::endl;
    if (!pparams.has_screen) {
        cv_watchdog.notify_one();
        thread_watchdog.join();
    }
    std::cout << "Closed" << std::endl;
}

void save_results_log() {
    auto nav = &patser.drone.nav;
    auto end_datetime = chrono::system_clock::to_time_t(chrono::system_clock::now());
    std::ofstream results_log;
    results_log.open(data_output_dir + "results.txt", std::ofstream::out);
    results_log << "op_mode:" << pparams.op_mode << '\n';
    results_log << "n_detections:" << patser.trackers.detections_count() << '\n';
    results_log << "n_monsters:" << patser.trackers.fp_monsters_count() << '\n';
    results_log << "n_static_fps:" << patser.trackers.fp_statics_count() << '\n';
    results_log << "n_short_fps:" << patser.trackers.fp_shorts_count() << '\n';
    results_log << "n_insects:" << patser.trackers.insects_count() << '\n';
    if (pparams.op_mode == op_mode_x) {
        results_log << "drone_has_been_ready:" << patser.drone.has_been_ready() << '\n';
        results_log << "n_takeoffs:" << patser.drone.n_take_offs() << '\n';
        results_log << "n_landings:" << patser.drone.n_landings() << '\n';
        results_log << "n_hunts:" << patser.drone.n_hunt_flights() << '\n';
        results_log << "n_replay_hunts:" << cmdcenter.n_replay_moth() << '\n';
        results_log << "n_wp_flights:" << patser.drone.n_wp_flights() << '\n';
        results_log << "n_drone_detects:" << patser.drone.n_drone_detects() << '\n';
        results_log << "drone_problem:" << nav->drone_problem() << '\n';
    }
    results_log << "run_time:" << visdat.current_time() << '\n';
    results_log << "start_datetime:" << std::put_time(std::localtime(&start_datetime), "%Y/%m/%d %T") << '\n';
    results_log << "end_datetime:" << std::put_time(std::localtime(&end_datetime), "%Y/%m/%d %T") << '\n';
    results_log.close();
}

void print_warnings() {
    if (pparams.video_raw && !log_replay_mode) {
        std::cout << "Video frames written: " << encoded_img_count - 1 << std::endl;
        if (encoded_img_count != received_img_count)
            std::cout << "WARNING VIDEO FRAMES MISSING: " << received_img_count - encoded_img_count << std::endl;
    }
    if (cam) {
        if (n_fps_warnings)
            std::cout << "WARNING FPS PROBLEMS: : " << n_fps_warnings << std::endl;
        if (cam->frame_loss_cnt())
            std::cout << "WARNING FRAME LOSSES: : " << cam->frame_loss_cnt() << std::endl;
    }
}

void save_periodic_images(cv::Mat frame_bgr, cv::Mat frameL, cv::Mat frameR) {
    static int last_save_bgr_hour = -1;
    auto time_now = chrono::system_clock::to_time_t(chrono::system_clock::now());
    auto tm_now = std::localtime(&time_now);
    if ((tm_now->tm_hour == 13 && last_save_bgr_hour != 13) ||
            (tm_now->tm_hour == 11 && last_save_bgr_hour != 11) ||
            (tm_now->tm_hour == 15 && last_save_bgr_hour != 15)) {
        std::stringstream date_ss;
        date_ss << std::put_time(tm_now, "%Y%m%d_%H%M%S");
        cv::imwrite("rgb_" + date_ss.str() + ".png", frame_bgr);
        cv::imwrite("stereoL_" + date_ss.str() + ".png", frameL);
        cv::imwrite("stereoR_" + date_ss.str() + ".png", frameR);
        last_save_bgr_hour = tm_now->tm_hour;
    }
}




void write_live_status_image(cv::Mat frame, string state_str) {
    if (frame.cols) {
        cv::Mat out_rgb;
        cvtColor(frame, out_rgb, cv::COLOR_GRAY2BGR);
        auto time_now = chrono::system_clock::to_time_t(chrono::system_clock::now());
        std::stringstream date_ss;
        date_ss << "Time: " << std::put_time(std::localtime(&time_now), "%Y/%m/%d %T");
        putText(out_rgb, date_ss.str(), cv::Point(5, 17), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        putText(out_rgb, state_str, cv::Point(5, 34), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255));


        if (pparams.op_mode == op_mode_x)
            cv::circle(out_rgb, patser.drone.tracker.pad_im_location(), patser.drone.tracker.pad_im_size() / 2, cv::Scalar(0, 255, 0));
        cv::imwrite("../../../../pats/status/live.jpg", out_rgb);
    }
}


void wait_for_start_conditions() {
    bool enable_window_ok = false;
    bool light_conditions_ok = false;
    bool cam_angles_ok = false;

    std::ofstream wait_logger;
    wait_logger.open(data_output_dir + "wait_for_start.csv", std::ofstream::out);
    wait_logger << "Datetime;Light level;Exposure;Gain;Brightness;Light_level_ok;Cam_angle_ok;Enable_window_ok" << std::endl;

    wdt_timeout = 30s;

    while (true) {
        auto [roll, pitch, light_level_, expo, gain, frameL, frameR, frame_bgr, avg_brightness] = static_cast<Realsense *>(cam.get())->measure_camera_conditions();
        watchdog = true;
        auto time_now = chrono::system_clock::to_time_t(chrono::system_clock::now());
        update_enable_window();
        save_periodic_images(frame_bgr, frameL, frameR);

        light_conditions_ok = pparams.light_level_threshold <= 0 || light_level_ < pparams.light_level_threshold;
        cam_angles_ok = (pparams.max_cam_angle <= 0 || std::abs(roll) < pparams.max_cam_angle) && (pparams.max_cam_angle <= 0 || std::abs(pitch - 35) < pparams.max_cam_angle);
        enable_window_ok = (!enable_window_mode || !(std::difftime(time_now, enable_window_start_time) < 0 || std::difftime(time_now, enable_window_end_time) > 0));

        auto t = chrono::system_clock::to_time_t(chrono::system_clock::now());
        auto datetime = std::put_time(std::localtime(&t), "%Y/%m/%d %T");
        wait_logger << datetime << ";" << light_level_ << ";" << expo << ";" << gain << ";" << avg_brightness << ";" << enable_window_ok << ";" << cam_angles_ok << ";" << enable_window_ok << std::endl;

        if (!enable_window_ok || !light_conditions_ok || !cam_angles_ok) {
            std::cout << std::put_time(std::localtime(&time_now), "%Y/%m/%d %T") << ", waiting for\t";
            std::stringstream buf;
            if (!enable_window_ok)
                buf << "enable window [" << std::chrono::duration_cast<std::chrono::minutes>(std::chrono::seconds(enable_window_start_time - time_now)).count() << " minutes]\t";
            if (!cam_angles_ok)
                buf << "cam angle [" << roll << ", " << pitch - 35 << ">" << pparams.max_cam_angle << "]\t";
            if (!light_conditions_ok)
                buf << "Light level [" << to_string_with_precision(light_level_, 2) << ">" << pparams.light_level_threshold << "]";
            std::cout << buf.str() << std::endl;
            if (!cam_angles_ok)
                communicate_state(es_wait_for_cam_angle);
            else if (!enable_window_ok)
                communicate_state(es_wait_for_enable_window);
            else if (!light_conditions_ok)
                communicate_state(es_wait_for_light_level);

            write_live_status_image(frameL, buf.str());
            std::this_thread::sleep_for(10s);
        } else {
            break;
        }

        if (baseboard_link.exit_now() || daemon_link.exit_now()) {
            throw std::runtime_error("Internal communication error");
            break;
        }
        if (baseboard_link.contact_problem() && pparams.op_mode == op_mode_x && !baseboard_link.disabled()) {
            std::cout << "Exiting wait prematurely because of a contact problem of some sort..." << std::endl;
            break;
        }
    }
    wait_logger.close();
}

void watchdog_worker(void) {
    std::cout << "Watchdog thread started" << std::endl;
    std::unique_lock<std::mutex> lk(cv_m_watchdog);

    while (!exit_watchdog_thread) {
        cv_watchdog.wait_until(lk, std::chrono::system_clock::now() + wdt_timeout);
        if (watchdog_skip_video_delay_override && !exit_watchdog_thread) {
            for (int i = 0; i < 20; i++) {
                if (exit_watchdog_thread)
                    break;
                usleep(1e6);
            }
            watchdog_skip_video_delay_override = false;
        }
        if (!watchdog && !exit_watchdog_thread) {
            std::cout << "Watchdog alert! Closing as much as possible." << std::endl;
            communicate_state(es_watchdog_restart);
            cam.get()->stop();

            if (received_img_count > 0)
                save_results_log();

            patser.drone.close();
            patser.trackers.close();
            visdat.close();

            logger << std::flush;
            logger.close();
            if (pparams.video_raw) {
                logger_video_ids << std::flush;
                logger_video_ids.close();
                video_raw.close();
                std::cout << "Closed video raw..." << std::endl;
            }
            if (pparams.video_render)
                video_render.close();

            print_warnings();

            std::cout << "Watchdog is now killing the process. Nice knowing you." << std::endl;

            pid_t pid = getpid();
            std::cout << "pid: " << pid << std::endl;
            exit_now = true;
            exit_watchdog_thread = true;
            usleep(5e5);
            string kill_cmd = "kill -9 " + std::to_string(pid);
            auto res [[maybe_unused]] = std::system(kill_cmd.c_str());
        }
        watchdog = false;
    }
}

void update_enable_window() {
    auto t_now = chrono::system_clock::to_time_t(chrono::system_clock::now());
    auto tm_now = *std::localtime(&t_now);
    if (std::difftime(t_now, enable_window_end_time) > 0) {
        if (enable_window_mode == enable_window_passes_midnight) {
            auto tm_start = * std::localtime(&enable_window_start_time);
            auto tm_end = * std::localtime(&enable_window_end_time);
            tm_end.tm_mday = tm_now.tm_mday + 1;
            tm_start.tm_mday = tm_now.tm_mday;
            enable_window_start_time = mktime(&tm_start);
            enable_window_end_time = mktime(&tm_end);
        } else if (enable_window_mode == enable_window_not_passes_midnight) {
            auto tm_start = * std::localtime(&enable_window_start_time);
            auto tm_end = * std::localtime(&enable_window_end_time);
            tm_start.tm_mday = tm_now.tm_mday + 1;
            tm_end.tm_mday = tm_now.tm_mday + 1;
            enable_window_start_time = mktime(&tm_start);
            enable_window_end_time = mktime(&tm_end);
        } else if (enable_window_mode == enable_window_end_only) {
            auto tm_start = * std::localtime(&enable_window_start_time);
            auto tm_end = * std::localtime(&enable_window_end_time);
            tm_start.tm_mday = tm_now.tm_mday - 999;
            tm_end.tm_mday = tm_now.tm_mday + 1;
            enable_window_start_time = mktime(&tm_start);
            enable_window_end_time = mktime(&tm_end);
        }
    }  else if (enable_window_mode == enable_window_start_only) {
        auto tm_start = * std::localtime(&enable_window_start_time);
        auto tm_end = * std::localtime(&enable_window_end_time);
        tm_end.tm_mday = tm_now.tm_mday + 999;
        tm_start.tm_mday = tm_now.tm_mday;
        enable_window_start_time = mktime(&tm_start);
        enable_window_end_time = mktime(&tm_end);
    }
}

int main(int argc, char **argv)
{

    try {
        data_output_dir = "./logging/";
        mkdir(data_output_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

        process_arg(argc, argv);
        pparams.deserialize(pats_xml_fn);
        if (drone_xml_fn == "") // should be empty still, but can have the --drone-xml arg override
            drone_xml_fn = "../xml/" + string(drone_types_str[pparams.drone]) + ".xml";
        dparams.deserialize(drone_xml_fn);

        if (log_replay_mode && !render_mode)
            pparams.has_screen = true;
        if (log_replay_mode)
            pparams.video_raw = video_disabled;
        if (render_mode)
            pparams.video_render = video_file;
        if (insect_replay_mode)
            pparams.op_mode = op_mode_c;

        daemon_link.init(log_replay_mode || generator_mode || airsim_mode);
        baseboard_link.init(log_replay_mode || generator_mode || airsim_mode);
        communicate_state(es_starting);
        if (realsense_reset) {
            communicate_state(es_realsense_reset);
            try {
                Realsense rs;
                rs.reset();
            }  catch (NoRealsenseConnected const &err) {
                std::cout << "Realsense reset failed because not connected..." << std::endl;
            }
            exit_now = true;
            daemon_link.close_link();
            baseboard_link.close_link();
            std::this_thread::sleep_for(1s);
            return 0;
        }

        if (pparams.enable_start.compare("disabled") != 0) {
            auto time_now = chrono::system_clock::to_time_t(chrono::system_clock::now());
            struct std::tm *tm = std::localtime(&time_now);
            std::istringstream ss(pparams.enable_start);
            ss >> std::get_time(tm, "%H:%M:%S");
            enable_window_start_time = mktime(tm);
            enable_window_mode = enable_window_start_only;
        } else {
            enable_window_start_time = chrono::system_clock::to_time_t(chrono::system_clock::now() - + std::chrono::seconds(1));
        }
        if (pparams.enable_end.compare("disabled") != 0) {
            auto time_now = chrono::system_clock::to_time_t(chrono::system_clock::now());
            struct std::tm *tm = std::localtime(&time_now);
            std::istringstream ss(pparams.enable_end);
            ss >> std::get_time(tm, "%H:%M:%S");
            enable_window_end_time = mktime(tm);
            if (enable_window_mode == enable_window_start_only) {
                if (enable_window_start_time < enable_window_end_time)
                    enable_window_mode = enable_window_not_passes_midnight;
                else
                    enable_window_mode = enable_window_passes_midnight;
            } else
                enable_window_mode = enable_window_end_only;
        } else {
            enable_window_end_time = chrono::system_clock::to_time_t(chrono::system_clock::now() + std::chrono::hours(999));
        }

        if (!pparams.has_screen)
            thread_watchdog = std::thread(&watchdog_worker);
        check_hardware();
        if (!generator_mode && !log_replay_mode && !airsim_mode) {
            wait_for_start_conditions();
        } else if (generator_mode || airsim_wp_mode)
            pparams.joystick = rc_none;
    } catch (rs2::error const &err) {
        std::cout << "Realsense error: " << err.what() << std::endl;
        communicate_state(es_realsense_error);
        cmdcenter.reset_commandcenter_status_file("Resetting realsense", false);
        try {
            static_cast<Realsense *>(cam.get())->reset();
        } catch (std::runtime_error const &err2) {
            std::cout << "Error: " << err2.what() << std::endl;
            cmdcenter.reset_commandcenter_status_file(err2.what(), true);
        }
        close_before_running();
        return 1;
    } catch (std::runtime_error const &err) {
        std::cout << "Error: " << err.what() << std::endl;
        communicate_state(es_runtime_error);
        cmdcenter.reset_commandcenter_status_file(err.what(), true);
        close_before_running();
        return 1;
    } catch (NoRealsenseConnected const &err) {
        communicate_state(es_realsense_not_found);
        std::cout << "Error: " << err.msg << std::endl;
        close_before_running();
        return 1;
    } catch (cv::Exception const &err) {
        communicate_state(es_runtime_error);
        cmdcenter.reset_commandcenter_status_file(err.msg, true);
        std::cout << "Error: " << err.msg << std::endl;
        close_before_running();
        return 1;
    }

    try {
        init();
        process_video();
    } catch (ReplayVideoEnded const &err) {
        std::cout << "Video ended" << std::endl;
        exit_now = true;
    } catch (std::runtime_error const &e) {
        exit_now = true;
        close(false);
        communicate_state(es_runtime_error);
        cmdcenter.reset_commandcenter_status_file(e.what(), true);
        std::cout << "Error: " << e.what() << std::endl;
        return 1;
    }

    close(false);
    return 0;
}
