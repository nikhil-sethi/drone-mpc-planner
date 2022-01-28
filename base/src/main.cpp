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
#include "3dviz/visualizer3d.h"
#include "commandcenterlink.h"
#include "filecam.h"
#include "generatorcam.h"
#include "realsense.h"
#include "airsimcam.h"
#include "airsimcontroller.h"
#include "interceptor.h"
#include "baseboardlink.h"
#include "daemonlink.h"


using namespace cv;
using namespace std;

/***********Variables****************/
bool exit_now = false;
bool watchdog = true;
volatile std::sig_atomic_t term_sig_fired;
int imgcount; // to measure fps
int encoded_img_count = 0;
int n_fps_warnings = 0;
GStream video_render, video_raw;
time_t start_datetime;

xmls::PatsParameters pparams;
xmls::DroneParameters dparams;
stopwatch_c stopWatch;
std::string data_output_dir;
std::time_t plukker_time;
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
uint8_t rc_id = 0;
std::string replay_dir, replay_video_fn;
std::string airsim_map;
std::string logger_fn; //contains filename of current log # for insect logging (same as video #)
std::string pats_xml_fn = "", drone_xml_fn = "";

std::ofstream logger;
std::ofstream logger_video_ids;
std::unique_ptr<RC> rc;

Visualizer visualizer;
Visualizer3D visualizer_3d;
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
std::thread thread_watchdog;
StereoPair *prev_frame;

/*******Private prototypes*********/
void process_frame(StereoPair *frame);
void process_video();
int main(int argc, char **argv);
bool handle_key(double time);
void save_results_log();
void print_warnings();
void close(bool sig_kill);
void watchdog_worker(void);
void communicate_state(executor_states s);

/************ code ***********/
void process_video() {

    filtering::Smoother fps_smoothed;
    fps_smoothed.init(100);
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
                visualizer_3d.run();
                if (pparams.has_screen) {
                    visualizer.paint();
                    escape_key_pressed = handle_key(frame->time);
                }
                if (render_mode)
                    visualizer.render();

            }
        }
        if (render_mode && flight_replay_mode &&  patser.drone.nav.drone_resetting_yaw()) {
            std::cout << "Render mode: yaw reset detected. Stopping." << std::endl;
            escape_key_pressed = true;
        }

        tp[0].m1.lock();
        tp[0].frame = frame;
        tp[0].data_is_new = true;
        tp[0].new_data.notify_one();
        tp[0].m1.unlock();

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
        } else if (escape_key_pressed) {
            communicate_state(es_user_restart);
            exit_now = true;
        }

        if (rc->init_package_failure()) {
            exit_now = true;
            communicate_state(es_rc_problem);
            cmdcenter.reset_commandcenter_status_file("MultiModule init package error", true);
        } else if (rc->bf_version_error() and !patser.drone.drone_flying()) {
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
            if (!patser.drone.drone_flying()) {
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

        if (pparams.video_raw) {
            // auto tmpf = frame->left.clone();
            // putText(tmpf, to_string(frame->rs_id), cv::Point(3, 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));

            int frame_written = video_raw.write(frame->left, frame->right);
            logger_video_ids << encoded_img_count << ";" << imgcount << ";" << frame->rs_id << ";" << frame->time << '\n';
            if (!frame_written)
                encoded_img_count++;
            else {
                std::cout << "Video frame write PROBLEM: " << frame_written << std::endl;
            }
        }

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
        if (fps < pparams.fps / 6 * 5 && fps_smoothed.ready() && !log_replay_mode && !generator_mode && !airsim_mode) {
            n_fps_warnings++;
            std::cout << "FPS WARNING: " << n_fps_warnings << std::endl;
            if (t_cam < 2) {
                std::cout << "Error: Detected FPS warning during start up, assuming there's some camera problem. Cowardly exiting." << std::endl;
                communicate_state(es_realsense_fps_problem);
                exit_now = true;
            }
        }
        if (cam->frame_loss_cnt() > 500) {
            std::cout << "Error: Too many frames lost, assuming there's some camera problem. Cowardly exiting." << std::endl;
            communicate_state(es_realsense_frame_loss_problem);
            exit_now = true;
        }

        std::cout <<
                  //   "\r\e[K" <<
                  imgcount <<
                  ", R:" << frame->rs_id <<
                  ", T: " << to_string_with_precision(frame->time, 2)  <<
                  " @ " << to_string_with_precision(fps, 1) <<
                  " " << patser.state_str() <<
                  ", exp: " << cam->measured_exposure() <<
                  ", gain: " << cam->measured_gain() <<
                  ", bright: " << to_string_with_precision(visdat.average_brightness(), 1);

        if (pparams.op_mode == op_mode_c) {
            if (patser.trackers.detections_count())
                std::cout  <<
                           ", detections: " << patser.trackers.detections_count() <<
                           ", insects: " << patser.trackers.insects_count();
        } else {
            std:: cout <<
                       ", " << patser.drone.drone_state_str() <<
                       " " << rc->telemetry.batt_cell_v <<
                       "v, arm: " << static_cast<int>(rc->telemetry.arming_state) <<
                       ", thr: " << rc->throttle <<
                       ", att: [" << rc->telemetry.roll << "," << rc->telemetry.pitch << "]" <<
                       ", rssi: " << static_cast<int>(rc->telemetry.rssi) <<
                       ", t_base: " << static_cast<int>(baseboard_link.uptime());
        }
        if (patser.trackers.monster_alert())
            std:: cout << ", monsters: " << patser.trackers.fp_monsters_count();
        std:: cout << std::endl;
        //   std::flush;

        if (rc->telemetry.arming_state && !patser.drone.drone_ready_and_waiting() && rc->arm_command())
            std::cout << rc->arming_state_str() << std::endl;

        imgcount++;
        prev_time = t;

        if (fps != fps || isinf(fps))
            fps_smoothed.reset();

        if (patser.drone.program_restart_allowed()) {

            auto time_now = chrono::system_clock::to_time_t(chrono::system_clock::now());

            if (!log_replay_mode  && ((imgcount > pparams.close_after_n_images && pparams.close_after_n_images > 0))) {
                std::cout << "Initiating periodic restart" << std::endl;
                communicate_state(es_periodic_restart);
                exit_now = true;
            } else if ((cam->measured_exposure() <= pparams.exposure_threshold && pparams.exposure_threshold > 0 && frame->time > 3)) {
                std::cout << "Initiating restart because exposure (" << cam->measured_exposure() << ") is lower than threshold (" << pparams.exposure_threshold << ")" << std::endl;
                communicate_state(es_brightness_restart);
                exit_now = true;
            } else if ((cam->measured_gain() < pparams.gain_threshold && pparams.exposure_threshold > 0 && frame->time > 3)) {
                std::cout << "Initiating restart because gain (" << cam->measured_gain() << ") is lower than threshold (" << pparams.gain_threshold << ")" << std::endl;
                communicate_state(es_brightness_restart);
                exit_now = true;
            } else if (visdat.average_brightness() > pparams.brightness_threshold + 10 && pparams.exposure_threshold > 0 && frame->time > 3) {
                std::cout << "Initiating restart because avg brightness (" << visdat.average_brightness() << ") is higher than threshold (" << pparams.brightness_threshold + 10 << ")" << std::endl;
                communicate_state(es_brightness_restart);
                exit_now = true;
            } else if (plukker_time > 0 && !log_replay_mode && std::difftime(time_now, plukker_time) > 0 && std::difftime(time_now, plukker_time) < 60 * 60 * 4) {
                communicate_state(es_plukker_restart);
                std::cout << "Initiating restart because plukker humans are in the way" << std::endl;
                exit_now = true;
            }
        }
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

void process_frame(StereoPair *frame) {
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
    logger << imgcount << ";"
           << frame->rs_id << ";"
           << std::put_time(std::localtime(&time_now), "%Y/%m/%d %T") << ";"
           << frame->time << ";"
           << cam->measured_exposure() << ";" << cam->measured_gain() << ";"
           << baseboard_link.charging_state_str() << ";" << static_cast<uint16_t>(baseboard_link.charging_state()) << ";";
    patser.update(frame->time);
    if (pparams.drone != drone_none && dparams.tx != tx_none)
        rc->send_commands(frame->time);
    baseboard_link.time(frame->time);
    daemon_link.time(frame->time);

    if (pparams.has_screen || render_mode) {
        visualizer.add_plot_sample();
        visualizer.update_tracker_data(visdat.frameL, patser.drone.nav.setpoint().pos(), frame->time, draw_plots);
        if (pparams.video_render && !exit_now) {
            if (log_replay_mode)
                video_render.block(); // only use this for rendering
            video_render.write(visualizer.trackframe);
        }
    }
    if (!log_replay_mode)
        cmdcenter.update(frame->left, frame->time);

    watchdog = true;
    logger  << '\n';
}


void communicate_state(executor_states s) {
    daemon_link.executor_state(s);
    baseboard_link.executor_state(s);
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
            patser.trackers.init_virtual_moth(tracking::VirtualMothTracker::diving, &patser.drone.control);
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
    if (log_replay_mode)
        data_output_dir = data_output_dir + "replay/";
    if (path_exist(data_output_dir)) {
        std::string rmcmd = "rm -r " + data_output_dir;
        auto res [[maybe_unused]] = std::system(rmcmd.c_str());
    }

    mkdir(data_output_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    if (!log_replay_mode) {
        pparams.serialize(data_output_dir + "pats.xml");
        dparams.serialize(data_output_dir + "drone.xml");
    } else if (log_replay_mode && pparams.op_mode == op_mode_x) {
        if (flight_replay_mode) {
            std::experimental::filesystem::path  drone_log_fn = std::experimental::filesystem::path(replay_video_fn).filename().replace_extension(".csv");
            drone_log_fn = replay_dir + "/log_" + drone_log_fn.string();
            logreader.init(replay_dir, drone_log_fn.string());
        } else
            logreader.init(replay_dir);
        patser.trackers.init_replay_moth(logreader.replay_moths());

    }

    logger.open(data_output_dir  + "log.csv", std::ofstream::out);
    logger << "id;rs_id;time;elapsed;exposure;gain;charging_state_str;charging_state;";
    logger_fn = data_output_dir  + "log" + to_string(0) + ".csv"; // only used with pparams.video_cuts
    if (pparams.video_raw) {
        logger_video_ids.open(data_output_dir  + "frames.csv", std::ofstream::out);
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
                replay_video_fn = argv[i];
                flight_replay_mode = true;
            } else if (s.compare("--insect") == 0) {
                //monitor_render accepts a (concatinated) videorawLR.mkv of monitored insects, but also needs an original logging folder for camera calibraton files.
                //usage example: ./executor --log logging --insect ~/Bla/vogel.mkv or ./executor --log logging --insect insect1.mkv
                arg_recognized = true;
                i++;
                insect_replay_mode = true;
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
    patser.init(&logger, rc_id, rc.get(), replay_dir, cam.get(), &visdat,  &baseboard_link);
    if (flight_replay_mode) {
        patser.init_flight_replay(replay_dir, logreader.log_drone()->flight_id());
        visdat.init_flight_replay(replay_dir, logreader.log_drone()->flight_id());
    } else if (insect_replay_mode) {
        patser.init_insect_replay();
        visdat.init_insect_replay(replay_dir);
    } else if (log_replay_mode) {
        patser.drone.init_full_log_replay(replay_dir);
    }

    cmdcenter.init(log_replay_mode || generator_mode || airsim_mode, &patser, &visdat);

    if (pparams.has_screen || render_mode) {
        visualizer.init(&visdat, &patser, log_replay_mode);
        if (generator_mode) {
            visualizer.set_generator_cam(static_cast<GeneratorCam *>(cam.get()));
        }
        visualizer_3d.init(&patser);
    }

    init_thread_pool();
    if (!pparams.has_screen)
        thread_watchdog = std::thread(&watchdog_worker);

    logger << '\n'; // this concludes the header log line
    std::cout << "Main init successfull" << std::endl;

    communicate_state(es_init_vision); // patser still has to do some initializing, like motion calibration
}

void close(bool sig_kill) {
    std::cout << "Closing" << std::endl;

    if (cam)
        cam->stop(); //cam needs to be closed after dnav, because of the camview class!

    communicate_state(es_closing);
    cmdcenter.reset_commandcenter_status_file("Closing", false);

    if (pparams.has_screen)
        cv::destroyAllWindows();

    if (imgcount > 0)
        save_results_log();

    /*****Close everything down*****/
    if (!sig_kill)
        close_thread_pool();

    if (!log_replay_mode)
        rc->close();
    if (pparams.has_screen || render_mode)
        visualizer.close();
    visdat.close();
    baseboard_link.close();
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
    daemon_link.close();

    print_warnings();
    if (cam)
        cam.release();
    if (rc)
        rc.release();
    if (!pparams.has_screen)
        thread_watchdog.join();
    std::cout << "Closed" << std::endl;
}

void save_results_log() {
    auto nav = &patser.drone.nav;
    auto end_datetime = chrono::system_clock::to_time_t(chrono::system_clock::now());
    std::ofstream results_log;
    results_log.open(data_output_dir  + "results.txt", std::ofstream::out);
    results_log << "op_mode:" << pparams.op_mode << '\n';
    results_log << "n_detections:" << patser.trackers.detections_count() << '\n';
    results_log << "n_monsters:" << patser.trackers.fp_monsters_count() << '\n';
    results_log << "n_static_fps:" << patser.trackers.fp_statics_count() << '\n';
    results_log << "n_short_fps:" << patser.trackers.fp_shorts_count() << '\n';
    results_log << "n_insects:" << patser.trackers.insects_count() << '\n';
    results_log << "n_takeoffs:" << patser.drone.n_take_offs() << '\n';
    results_log << "n_landings:" << patser.drone.n_landings() << '\n';
    results_log << "n_hunts:" << patser.drone.n_hunt_flights() << '\n';
    results_log << "n_replay_hunts:" << cmdcenter.n_replay_moth() << '\n';
    results_log << "n_wp_flights:" << patser.drone.n_wp_flights() << '\n';
    results_log << "n_drone_detects:" << patser.drone.n_drone_detects() << '\n';
    results_log << "drone_problem:" << nav->drone_problem() << '\n';
    results_log << "run_time:" << visdat.current_time() << '\n';
    results_log << "start_datetime:" << std::put_time(std::localtime(&start_datetime), "%Y/%m/%d %T") << '\n';
    results_log << "end_datetime:" <<  std::put_time(std::localtime(&end_datetime), "%Y/%m/%d %T") << '\n';
    results_log.close();
}

void print_warnings() {
    if (pparams.video_raw && !log_replay_mode) {
        std::cout << "Video frames written: " << encoded_img_count - 1 << std::endl;
        if (encoded_img_count != imgcount)
            std::cout << "WARNING VIDEO FRAMES MISSING: " << imgcount - encoded_img_count << std::endl;
    }
    if (n_fps_warnings)
        std::cout << "WARNING FPS PROBLEMS: : " << n_fps_warnings << std::endl;
    if (cam->frame_loss_cnt())
        std::cout << "WARNING FRAME LOSSES: : " << cam->frame_loss_cnt() << std::endl;

}

void wait_for_cam_angle() {
    int enable_delay = 0;
    if (pparams.max_cam_roll > 0 && !log_replay_mode) {
        std::cout << "Checking cam angle..." << std::endl;

        while (true) {
            auto [roll, pitch, frame_time, frameL] = static_cast<Realsense *>(cam.get())->measure_angle();
            static double roll_start_time = frame_time;
            double elapsed_time = (frame_time - roll_start_time) / 1000.;

            auto t = chrono::system_clock::to_time_t(chrono::system_clock::now());
            std::cout << std::put_time(std::localtime(&t), "%Y/%m/%d %T") << ". Camera roll: " << to_string_with_precision(roll, 2) << "°- max: " << pparams.max_cam_roll << "°. Pitch: " << to_string_with_precision(pitch, 2) << "°" << std::endl;

            static double prev_imwrite_time = -pparams.live_image_frq;
            if (elapsed_time - prev_imwrite_time > pparams.live_image_frq && pparams.live_image_frq >= 0) {
                prev_imwrite_time = elapsed_time;
                cv::imwrite("../../../../pats/status/monitor_tmp.jpg", frameL);
            }
            cmdcenter.reset_commandcenter_status_file("Roll: " + to_string_with_precision(roll, 2), false);

            if (fabs(roll)  < pparams.max_cam_roll && !enable_delay)
                break;
            else if (fabs(roll)  > pparams.max_cam_roll) {
                if (!enable_delay)
                    communicate_state(es_wait_for_angle);
                enable_delay = 60;
            } else
                enable_delay--;


            usleep(30000); // measure every third second
        }
    }
}

void wait_for_plukker() {
    if (plukker_time > 0) {
        std::cout << "Checking for plukker time..." << std::endl;
        while (true) {
            auto time_now = chrono::system_clock::to_time_t(chrono::system_clock::now());
            if (std::difftime(time_now, plukker_time) > 0 && std::difftime(time_now, plukker_time) < 60 * 60 * 4) {
                communicate_state(es_wait_for_plukker);
                cmdcenter.reset_commandcenter_status_file("Waiting. Plukkers: ", false);
                std::cout << std::put_time(std::localtime(&time_now), "%Y/%m/%d %T") << " Waiting for plukkers" << std::endl;
                usleep(1e7);
            } else {
                break;
            }
        }
    }
}

void wait_for_dark() {
    if (pparams.exposure_threshold > 0 && !log_replay_mode) {
        std::cout << "Checking if dark..." << std::endl;
        int last_save_bgr_hour = -1;
        while (true) {
            auto [expo, gain, frameL, frameR, frame_bgr, avg_brightness] = static_cast<Realsense *>(cam.get())->measure_auto_exposure();
            auto t = chrono::system_clock::to_time_t(chrono::system_clock::now());
            std::cout << std::put_time(std::localtime(&t), "%Y/%m/%d %T") << " Measured exposure: " << expo << ">" << pparams.exposure_threshold << ", gain: " << gain << "<" << pparams.gain_threshold << ", brightness: " <<  to_string_with_precision(avg_brightness, 0) << "<" << pparams.brightness_threshold << std::endl;
            if (expo >= pparams.exposure_threshold && gain >= pparams.gain_threshold && avg_brightness < pparams.brightness_threshold) {
                break;
            }
            cv::imwrite("../../../../pats/status/monitor_tmp.jpg", frameL);
            communicate_state(es_wait_for_darkness);
            cmdcenter.reset_commandcenter_status_file("Waiting. Exposure: " + std::to_string(static_cast<int>(expo)) + ", gain: " + std::to_string(static_cast<int>(gain))  + ", brightness: " + std::to_string(static_cast<int>(visdat.average_brightness())), false);

            if ((std::localtime(&t)->tm_hour == 13 && last_save_bgr_hour != 13)  ||
                    (std::localtime(&t)->tm_hour == 11 && last_save_bgr_hour != 11) ||
                    (std::localtime(&t)->tm_hour == 15 && last_save_bgr_hour != 15)) {
                std::stringstream date_ss;
                date_ss << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S");
                cv::imwrite("rgb_" + date_ss.str() + ".png", frame_bgr);
                cv::imwrite("stereoL_" + date_ss.str() + ".png", frameL);
                cv::imwrite("stereoR_" + date_ss.str() + ".png", frameR);
                last_save_bgr_hour = std::localtime(&t)->tm_hour;
            }
            usleep(1e7); // measure every 10 seconds
        }
    }
}

void watchdog_worker(void) {
    std::cout << "Watchdog thread started" << std::endl;
    usleep(10000000); //wait until camera is running for sure
    while (!exit_now) {
        usleep(pparams.wdt_timeout_us);
        if (watchdog_skip_video_delay_override) {
            for (int i = 0; i < 20; i++) {
                if (exit_now)
                    break;
                usleep(1e6);
            }
            watchdog_skip_video_delay_override = false;
        }
        if (!watchdog && !exit_now) {
            std::cout << "Watchdog alert! Closing as much as possible." << std::endl;
            communicate_state(es_watchdog_restart);

            if (imgcount > 0)
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
            }
            if (pparams.video_render)
                video_render.close();

            print_warnings();

            std::cout << "Watchdog is now killing the process. Nice knowing you." << std::endl;

            pid_t pid = getpid();
            std::cout << "pid: " << pid << std::endl;
            exit_now = true;
            usleep(5e5);
            string kill_cmd = "kill -9 " + std::to_string(pid);
            auto res [[maybe_unused]] = std::system(kill_cmd.c_str());
        }
        watchdog = false;
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

        daemon_link.init(log_replay_mode || generator_mode || airsim_mode);
        baseboard_link.init(log_replay_mode || generator_mode || airsim_mode);
        cmdcenter.reset_commandcenter_status_file("Starting", false);
        communicate_state(es_starting);
        if (realsense_reset) {
            communicate_state(es_realsense_reset);
            cmdcenter.reset_commandcenter_status_file("Reseting realsense", false);
            Realsense rs;
            rs.reset();
            return 0;
        }

        auto time_now = chrono::system_clock::to_time_t(chrono::system_clock::now());
        struct std::tm *tm_start = std::localtime(&time_now);;
        if (pparams.plukker_start.compare("disabled") != 0) {
            std::istringstream ss(pparams.plukker_start);
            ss >> std::get_time(tm_start, "%H:%M:%S");
            plukker_time = mktime(tm_start);
        } else
            plukker_time = -1;

        check_hardware();
        if (!generator_mode && !log_replay_mode && !airsim_mode) {
            wait_for_plukker();
            wait_for_cam_angle();
            wait_for_dark();
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
        return 1;
    } catch (std::runtime_error const &err) {
        std::cout << "Error: " << err.what() << std::endl;
        communicate_state(es_runtime_error);
        cmdcenter.reset_commandcenter_status_file(err.what(), true);
        return 1;
    } catch (NoRealsenseConnected const &err) {
        communicate_state(es_realsense_not_found);
        std::cout << "Error: " << err.msg << std::endl;
        return 1;
    } catch (cv::Exception const &err) {
        communicate_state(es_runtime_error);
        cmdcenter.reset_commandcenter_status_file(err.msg, true);
        std::cout << "Error: " << err.msg << std::endl;
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
        std::cout << "Error: " << e.what() << std::endl;
        communicate_state(es_runtime_error);
        cmdcenter.reset_commandcenter_status_file(e.what(), true);
        return 1;
    }

    close(false);
    return 0;
}
