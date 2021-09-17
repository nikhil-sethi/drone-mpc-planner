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

#include "common.h"
#include "flightarea/flightarea.h"
#include "smoother.h"
#include "multimodule.h"
#include "replayrc.h"
#include "stopwatch.h"
#include "dronetracker.h"
#include "dronecontroller.h"
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
#include "baseboard.h"

using namespace cv;
using namespace std;

/***********Variables****************/
bool exit_now = false;
bool watchdog = true;
volatile std::sig_atomic_t term_sig_fired;
int imgcount; // to measure fps
int raw_video_frame_counter = 0;
int n_fps_warnings = 0;
GStream output_video_results, output_video_LR;
time_t start_datetime;

xmls::PatsParameters pparams;
xmls::DroneParameters dparams;
stopwatch_c stopWatch;
std::string data_output_dir;
bool draw_plots = false;
bool realsense_reset = false;
bool log_replay_mode = false;
bool generator_mode = false;
bool airsim_mode = false;
bool airsim_wp_mode = false;
bool render_monitor_video_mode = false;
bool render_hunt_mode = false;
bool skipped_to_hunt = false;
bool watchdog_skip_video_delay_override = false;
uint8_t drone_id;
std::string replay_dir;
std::string airsim_map;
std::string logger_fn; //contains filename of current log # for insect logging (same as video #)
std::string pats_xml_fn = "../xml/pats.xml", drone_xml_fn, monitor_video_fn;

std::ofstream logger;
std::ofstream logger_video_ids;
std::unique_ptr<Rc> rc;
FlightArea flight_area;
DroneController dctrl;
navigation::DroneNavigation dnav;
tracking::TrackerManager trackers;
Visualizer visualizer;
Visualizer3D visualizer_3d;
logging::LogReader logreader;
CommandCenterLink cmdcenter;
Baseboard baseboard;
std::unique_ptr<Cam> cam;
VisionData visdat;
Interceptor iceptor;

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
void skip_to_hunt(double, double);
int main(int argc, char **argv);
bool handle_key(double time);
void save_results_log();
void print_warnings();
void close(bool sig_kill);
void watchdog_worker(void);

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
        if (pparams.has_screen || render_hunt_mode || render_monitor_video_mode) {
            static int speed_div;
            if (!(speed_div++ % 4) || (((log_replay_mode || generator_mode) && !cam->turbo) || cam->frame_by_frame)) {
                visualizer_3d.run();
                if (pparams.has_screen) {
                    visualizer.paint();
                    escape_key_pressed = handle_key(frame->time);
                }
                if (render_hunt_mode || render_monitor_video_mode)
                    visualizer.render();

            }
        }
        if (log_replay_mode && !skipped_to_hunt) {
            skip_to_hunt(1.5, frame->time);
            if (skipped_to_hunt && !render_monitor_video_mode && !render_hunt_mode)
                cam->turbo = false;
        }

        if (render_hunt_mode) {
            if (dnav.drone_ready_and_waiting() && !skipped_to_hunt)
                skip_to_hunt(1.5, frame->time);
            if (dnav.drone_resetting_yaw()) {
                std::cout << "Render mode: yaw reset detected. Stopping." << std::endl;
                escape_key_pressed = true;
            }
        }
        if (airsim_wp_mode && dnav.drone_ready_and_waiting() && rc->arm_switch == bf_armed)
            dnav.demo_flight(pparams.flightplan);

        tp[0].m1.lock();
        tp[0].frame = frame;
        tp[0].data_is_new = true;
        tp[0].new_data.notify_one();
        tp[0].m1.unlock();

        if (term_sig_fired == 2) {
            std::cout << "\nCaught ctrl-c: " << term_sig_fired << std::endl;
            exit_now = true;
        } else if (term_sig_fired == 15) {
            std::cout << "\nCaught TERM signal: " << term_sig_fired << std::endl;
            exit_now = true;
        } else if (term_sig_fired) {
            std::cout << "\nCaught unknown signal: " << term_sig_fired << std::endl;
            exit_now = true;
        } else if (escape_key_pressed)
            exit_now = true;

        if (rc->init_package_failure()) {
            exit_now = true;
            cmdcenter.reset_commandcenter_status_file("MultiModule init package error", true);
        }
        if (rc->bf_version_error() and !dnav.drone_flying()) {
            exit_now = true;
            cmdcenter.reset_commandcenter_status_file("Betaflight version error", true);
        }
        if (rc->bf_uid_error()) {
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
            if (!dnav.drone_flying()) {
                exit_now = true;
                cmdcenter.reset_commandcenter_status_file("Wrong drone config error", true);
            }
        }

        if (pparams.video_raw && pparams.video_raw != video_bag) {
            // cv::Mat id_fr = cam->frameL.clone();
            // putText(id_fr,std::to_string(frame->rs_id),cv::Point(0, 13),cv::FONT_HERSHEY_SIMPLEX,0.5,Scalar(255));
            int frame_written = output_video_LR.write(frame->left, frame->right);
            logger_video_ids << raw_video_frame_counter << ";" << imgcount << ";" << frame->rs_id << ";" << frame->time << '\n';
            if (!frame_written)
                raw_video_frame_counter++;
            else {
                std::cout << "Video frame write PROBLEM: " << frame_written << std::endl;
            }
        }

        if (render_hunt_mode) {
            if (!dnav.n_drone_readys() && frame->time - 3 > logreader.first_drone_ready_time()) {
                std::cout << "\n\nError: Log results differ from replay results, could not find drone in time.\n" << replay_dir << "\n\n" << std::endl;
                exit_now = true;
            }
            if (!dnav.drone_resetting_yaw() && frame->time - 3 > logreader.first_yaw_reset_time()) {
                std::cout << "\n\nError: Log results differ from replay results, should have yaw reset already.\n" << replay_dir << "\n\n" << std::endl;
                exit_now = true;
            }

            static uint render_drone_problem_cnt = 0;
            if (dnav.drone_problem() || logreader.current_entry.nav_state == navigation::ns_drone_problem || logreader.current_entry.nav_state == navigation::ns_drone_lost || logreader.current_entry.nav_state == navigation::ns_batlow || logreader.current_entry.nav_state == navigation::ns_tracker_problem) {
                render_drone_problem_cnt++;
                if (render_drone_problem_cnt > pparams.fps * 2) {
                    std::cout << "\nProblem: " << dnav.navigation_status() << ". Stopping render\n" << replay_dir << "\n\n" << std::endl;
                    exit_now = true;
                }
            }
        }

        //keep track of time and fps
        float t_pc = stopWatch.Read() / 1000.f;
        float t_cam = static_cast<float>(frame->time);
        float t;
        if (log_replay_mode || generator_mode || render_hunt_mode || render_monitor_video_mode)
            t = t_pc;
        else
            t = t_cam;
        static float prev_time = -1.f / pparams.fps;
        float current_fps = 1.f / (t - prev_time);
        float fps = fps_smoothed.addSample(current_fps);
        if (fps < pparams.fps / 6 * 5 && fps_smoothed.ready() && !log_replay_mode && !generator_mode && !airsim_mode && !render_hunt_mode && !render_monitor_video_mode) {
            n_fps_warnings++;
            std::cout << "FPS WARNING: " << n_fps_warnings << std::endl;
            if (t_cam < 2) {
                std::cout << "Error: Detected FPS warning during start up, assuming there's some camera problem. Cowardly exiting." << std::endl;
                set_fps_warning_flag();
                exit_now = true;
            }
        }
        if (cam->frame_loss_cnt() > 500) {
            std::cout << "Error: Too many frames lost, assuming there's some camera problem. Cowardly exiting." << std::endl;
            set_frame_loss_warning_flag();
            exit_now = true;
        }

        std::cout <<
                  //   "\r\e[K" <<
                  dnav.navigation_status() <<
                  "; " << imgcount <<
                  ", " << frame->rs_id <<
                  ", T: " << to_string_with_precision(frame->time, 2)  <<
                  " @ " << to_string_with_precision(fps, 1) <<
                  ", exp: " << cam->measured_exposure() <<
                  ", gain: " << cam->measured_gain() <<
                  ", bright: " << to_string_with_precision(visdat.average_brightness(), 1);

        if (pparams.op_mode == op_mode_monitoring) {
            if (trackers.detections_count())
                std::cout  <<
                           ", detections: " << trackers.detections_count() <<
                           ", insects: " << trackers.insects_count();
        } else {
            std:: cout <<
                       ", " << rc->telemetry.batt_cell_v <<
                       "v, arm: " << static_cast<int>(rc->telemetry.arming_state) <<
                       ", thr: " << rc->throttle <<
                       ", att: [" << rc->telemetry.roll << "," << rc->telemetry.pitch << "]" <<
                       ", rssi: " << static_cast<int>(rc->telemetry.rssi) <<
                       ", t_base: " << static_cast<int>(baseboard.uptime());
        }
        if (trackers.monster_alert())
            std:: cout << ", monsters: " << trackers.fp_monsters_count();
        std:: cout << std::endl;
        //   std::flush;

        if (rc->telemetry.arming_state && !dnav.drone_ready_and_waiting())
            std::cout << rc->arming_state_str() << std::endl;

        imgcount++;
        prev_time = t;

        if (fps != fps || isinf(fps))
            fps_smoothed.reset();

        if (dctrl.in_flight_duration(frame->time) < 1.f / pparams.fps || dnav.drone_problem(1)) {
            if (!log_replay_mode  && ((imgcount > pparams.close_after_n_images && pparams.close_after_n_images > 0))) {
                std::cout << "Initiating periodic restart" << std::endl;
                exit_now = true;
            } else if ((cam->measured_exposure() <= pparams.exposure_threshold && pparams.exposure_threshold > 0 && frame->time > 3)) {
                std::cout << "Initiating restart because exposure (" << cam->measured_exposure() << ") is lower than threshold (" << pparams.exposure_threshold << ")" << std::endl;
                exit_now = true;
            } else if ((cam->measured_gain() < pparams.gain_threshold && pparams.exposure_threshold > 0 && frame->time > 3)) {
                std::cout << "Initiating restart because gain (" << cam->measured_gain() << ") is lower than threshold (" << pparams.gain_threshold << ")" << std::endl;
                exit_now = true;
            } else if (visdat.average_brightness() > pparams.brightness_threshold + 10 && pparams.exposure_threshold > 0 && frame->time > 3) {
                std::cout << "Initiating restart because avg brightness (" << visdat.average_brightness() << ") is higher than threshold (" << pparams.brightness_threshold + 10 << ")" << std::endl;
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
    if (log_replay_mode && pparams.op_mode != op_mode_monitoring) {
        if (logreader.current_frame_number(frame->rs_id)) {
            exit_now = true;
            return;
        }
        static_cast<ReplayRc *>(rc.get())->telemetry_from_log(frame->time);

        trackers.process_replay_moth(frame->rs_id);
        cmdcenter.trigger_demo_flight_from_log(replay_dir, logreader.current_entry.trkrs_state);

    } else if (generator_mode) {
        if (dnav.drone_ready_and_waiting()) {
            dnav.demo_flight(pparams.flightplan);
            dctrl.joy_takeoff_switch_file_trigger(true);
        }
    }

#ifdef PROFILING
    auto profile_t0 = std::chrono::high_resolution_clock::now();
#endif
    visdat.update(frame);
    prev_frame->processing = false;
    prev_frame = frame;
#ifdef PROFILING
    auto profile_t1_visdat = std::chrono::high_resolution_clock::now();
#endif

    auto time_now = chrono::system_clock::to_time_t(chrono::system_clock::now());
    logger << imgcount << ";"
           << frame->rs_id << ";"
           << std::put_time(std::localtime(&time_now), "%Y/%m/%d %T") << ";"
           << frame->time << ";"
           << cam->measured_exposure() << ";";

    trackers.update(frame->time, dctrl.active());
#ifdef PROFILING
    auto profile_t2_trkrs = std::chrono::high_resolution_clock::now();
#endif

    if (log_replay_mode && pparams.op_mode != op_mode_monitoring) {
        dctrl.insert_log(logreader.current_entry.joyRoll, logreader.current_entry.joyPitch, logreader.current_entry.joyYaw, logreader.current_entry.joyThrottle, logreader.current_entry.joyArmSwitch, logreader.current_entry.joyModeSwitch, logreader.current_entry.joyTakeoffSwitch, logreader.current_entry.auto_roll, logreader.current_entry.auto_pitch, logreader.current_entry.auto_throttle, logreader.current_entry.telem_acc_z);
    }
    if (pparams.op_mode == op_mode_hunt)
        iceptor.update(dctrl.at_base(), frame->time);
    else
        iceptor.write_dummy_csv();
    dnav.update(frame->time);
#ifdef PROFILING
    auto profile_t3_nav = std::chrono::high_resolution_clock::now();
#endif

    if (pparams.op_mode != op_mode_monitoring)
        dctrl.control(trackers.dronetracker()->last_track_data(), dnav.setpoint(), iceptor.target_last_trackdata(), frame->time);
#ifdef PROFILING
    auto profile_t4_ctrl = std::chrono::high_resolution_clock::now();
#endif

#ifdef PROFILING
    auto profile_t5_prdct = std::chrono::high_resolution_clock::now();
#endif

    if (pparams.has_screen || render_hunt_mode || render_monitor_video_mode) {
        visualizer.add_plot_sample();
        visualizer.update_tracker_data(visdat.frameL, dnav.setpoint().pos(), frame->time, draw_plots);
        if (pparams.video_result && !exit_now) {
            if ((render_hunt_mode && skipped_to_hunt) || !render_hunt_mode) {
                if (log_replay_mode || render_monitor_video_mode || render_hunt_mode)
                    output_video_results.block(); // only use this for rendering
                output_video_results.write(visualizer.trackframe);
            }
        }
    }
    if (!render_hunt_mode && !render_monitor_video_mode && !log_replay_mode)
        cmdcenter.update(frame->left, frame->time);

#ifdef PROFILING
    auto dur1_visdat = std::chrono::duration_cast<std::chrono::microseconds>(profile_t1_visdat - profile_t0).count();
    auto dur2 = std::chrono::duration_cast<std::chrono::microseconds>(profile_t2_trkrs - profile_t1_visdat).count();
    auto dur3 = std::chrono::duration_cast<std::chrono::microseconds>(profile_t3_nav - profile_t2_trkrs).count();
    auto dur4 = std::chrono::duration_cast<std::chrono::microseconds>(profile_t4_ctrl - profile_t3_nav).count();
    auto dur5 = std::chrono::duration_cast<std::chrono::microseconds>(profile_t5_prdct - profile_t4_ctrl).count();
    auto profile_t6_frame = std::chrono::high_resolution_clock::now();
    auto dur_tot = std::chrono::duration_cast<std::chrono::microseconds>(profile_t6_frame - profile_t0).count();

    logger << dur1_visdat << ";"
           << dur2 << ";"
           << dur3 << ";"
           << dur4 << ";"
           << dur5 << ";"
           << dur_tot << ";";
#endif
    watchdog = true;
    logger  << '\n';
}

void skip_to_hunt(double pre_delay, double time) {
    double dt = logreader.first_takeoff_time() - time - pre_delay;
    if (dt > 0 && !isinf(dt) && dnav.drone_ready_and_waiting()) {
        watchdog_skip_video_delay_override = true;
        static_cast<FileCam *>(cam.get())->skip(dt);
        skipped_to_hunt = true;
        std::cout << "Skipping to " << time + dt << std::endl;
    }
}

void init_insect_log(int n) {
    trackers.init_replay_moth(n);
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
            dnav.shake_drone();
            break;
        case 'B':
            rc->beep();
            break;
        case 'c':
            rc->calibrate_acc();
            break;
        case 'l':
            dnav.redetect_drone_location();
            break;
        case 'p':
            if (log_replay_mode || generator_mode || render_monitor_video_mode)
                draw_plots = true;
            break;
        case '[':
            if (log_replay_mode || generator_mode || render_monitor_video_mode)
                trackers.enable_trkr_viz();
            break;
        case ']':
            if (log_replay_mode || generator_mode || render_monitor_video_mode)
                trackers.enable_blob_viz();
            break;
        case '\\':
            if (log_replay_mode || generator_mode || render_monitor_video_mode)
                trackers.enable_draw_stereo_viz();
            break;
        case ';':
            if (log_replay_mode || generator_mode || render_monitor_video_mode)
                visualizer.enable_draw_noise_viz();
            break;
        case '\'':
            if (log_replay_mode || generator_mode || render_monitor_video_mode)
                visualizer.enable_draw_exposure_viz();
            break;
        case 'o':
            dctrl.LED(true);
            dnav.nav_flight_mode(navigation::nfm_manual);
            break;
        case '1':
            if (dnav.nav_flight_mode() == navigation::nfm_waypoint) {
                if (dnav.drone_ready_and_waiting()) {
                    std::string fp = "../xml/flightplans/thrust-calib.xml";
                    std::cout << "Flightplan trigger:" << fp << std::endl;
                    dnav.demo_flight(fp);
                    dctrl.joy_takeoff_switch_file_trigger(true);
                    experimental::filesystem::copy_file(fp, "./logging/pats_demo.xml");
                }
            } else
                init_insect_log(56);
            break;
        case '2':
            if (dnav.nav_flight_mode() == navigation::nfm_waypoint) {
                if (dnav.drone_ready_and_waiting()) {
                    std::string fp = "../xml/flightplans/simple-demo-darkroom.xml";
                    std::cout << "Flightplan trigger:" << fp << std::endl;
                    dnav.demo_flight(fp);
                    dctrl.joy_takeoff_switch_file_trigger(true);
                    experimental::filesystem::copy_file(fp, "./logging/pats_demo.xml");
                }
            } else
                init_insect_log(66);
            break;
        case '3':
            if (dnav.nav_flight_mode() == navigation::nfm_waypoint) {
                if (dnav.drone_ready_and_waiting()) {
                    std::string fp = "../xml/flightplans/simple-demo-koppert.xml";
                    std::cout << "Flightplan trigger:" << fp << std::endl;
                    dnav.demo_flight(fp);
                    dctrl.joy_takeoff_switch_file_trigger(true);
                    experimental::filesystem::copy_file(fp, "./logging/pats_demo.xml");
                }
            } else
                init_insect_log(58);
            break;
        case '4':
            if (dnav.nav_flight_mode() == navigation::nfm_waypoint) {
                if (dnav.drone_ready_and_waiting()) {
                    std::string fp = "../xml/flightplans/bejo.xml";
                    std::cout << "Flightplan trigger:" << fp << std::endl;
                    dnav.demo_flight(fp);
                    dctrl.joy_takeoff_switch_file_trigger(true);
                    experimental::filesystem::copy_file(fp, "./logging/pats_demo.xml");
                }
            } else
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
            trackers.init_virtual_moth(tracking::VirtualMothTracker::diving, &dctrl);
            break;
        case 82: // arrow up
            dnav.manual_trigger_next_wp();
            break;
        case 84: // arrow down
            dnav.manual_trigger_prev_wp();
            break;
        case ' ':
        case 'f':
            if (log_replay_mode || generator_mode || render_hunt_mode || render_monitor_video_mode || airsim_mode)
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
            dnav.nav_flight_mode(navigation::nfm_waypoint);
            break;
        case 'h':
            rc->arm(bf_armed);
            dnav.nav_flight_mode(navigation::nfm_hunt);
            break;
        case 'd':
            rc->arm(bf_disarmed);
            dnav.nav_flight_mode(navigation::nfm_manual);
            break;
    } // end switch key

    return false;
}

//This is where frames get processed after it was received from the cam in the main thread
void pool_worker(int id) {
    std::unique_lock<std::mutex> lk(tp[id].m1, std::defer_lock);
    while (!exit_now) {
        tp[id].new_data.wait(lk, []() {return tp[0].data_is_new;});
        tp[0].data_is_new = false;
        if (exit_now)
            break;
        process_frame(tp->frame);
        tp[id].m2.lock();
        tp[id].data_is_processed = true;
        tp[id].data_processed.notify_one();
        tp[id].m2.unlock();
    }
    std::cout << "Exit pool thread " << id << std::endl;
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
        if (render_hunt_mode || render_monitor_video_mode)
            output_video_results.manual_unblock();
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
    if (log_replay_mode || render_monitor_video_mode)
        data_output_dir = data_output_dir + "replay/";
    if (path_exist(data_output_dir)) {
        std::string rmcmd = "rm -r " + data_output_dir;
        auto res [[maybe_unused]] = std::system(rmcmd.c_str());
    }

    mkdir(data_output_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    if (!log_replay_mode) {
        pparams.serialize("./logging/pats.xml");
        dparams.serialize("./logging/drone.xml");
    }

    logger.open(data_output_dir  + "log.csv", std::ofstream::out);
    logger << "ID;RS_ID;time;elapsed;Exposure;";
    logger_fn = data_output_dir  + "log" + to_string(0) + ".csv"; // only used with pparams.video_cuts
    logger_video_ids.open(data_output_dir  + "frames.csv", std::ofstream::out);

    if (rc->connected())
        rc->init_logger();
}

void init_video_recorders() {
    /*****init the video writer*****/
    if (pparams.video_result)
        if (output_video_results.init(pparams.video_result, data_output_dir + "videoResult.mkv", visualizer.viz_frame_size().width, visualizer.viz_frame_size().height, pparams.fps, "192.168.1.255", 5000, true)) {throw MyExit("could not open results video");}
    if (pparams.video_raw && pparams.video_raw != video_bag)
        if (output_video_LR.init(pparams.video_raw, data_output_dir + "videoRawLR.mkv", IMG_W, IMG_H * 2, pparams.fps, "192.168.1.255", 5000, false)) {throw MyExit("could not open LR video");}
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
                drone_id = stoi(argv[i]);
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
            } else if (s.compare("--render") == 0) {
                arg_recognized = true;
                render_hunt_mode = true;
            } else if (s.compare("--monitor-render") == 0) {
                //monitor_render accepts a (concatinated) videorawLR.mkv of monitored insects, but also needs an original logging folder for camera calibraton files.
                //usage example: ./pats --log logging --monitor_render ~/Bla/vogel.mkv
                arg_recognized = true;
                i++;
                render_monitor_video_mode = true;
                log_replay_mode = false;
                monitor_video_fn = argv[i];
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

    if (pats_xml_fn.find("pats_deploy.xml") != string::npos) {
        if (file_exist("../../../../pats/pats.xml"))
            pats_xml_fn = "../../../../pats/pats.xml";
        else
            pats_xml_fn = "../xml/pats_deploy.xml";
    }

}

void check_hardware() {
    if (pparams.op_mode != op_mode_monitoring && !log_replay_mode && !render_monitor_video_mode && !render_hunt_mode && !generator_mode) {
        // Ensure that joystick was found and that we can use it
        if (!dctrl.joystick_ready() && pparams.joystick != rc_none && ! airsim_wp_mode) {
            throw MyExit("no joystick connected.");
        }

        // init rc
        if (airsim_mode) {
            rc = std::unique_ptr<Rc>(new AirSimController());
        } else if (dparams.tx != tx_none) {
            rc = std::unique_ptr<Rc>(new MultiModule());
        }
        if (!rc->connect() && pparams.op_mode != op_mode_monitoring) {
            std::string rc_type = (airsim_mode) ? "AirSim" : "MultiModule";
            throw MyExit("cannot connect to " + rc_type);
        }
    } else if (log_replay_mode) {
        rc = std::unique_ptr<Rc>(new ReplayRc(replay_dir));
    } else {
        rc = std::unique_ptr<Rc>(new MultiModule());
    }
    //init cam and check version
    std::cout << "Connecting camera..." << std::endl;
    if (log_replay_mode && !render_monitor_video_mode) {
        if (file_exist(replay_dir + '/' + FileCam::playback_filename()))
            cam = std::unique_ptr<Cam>(new FileCam(replay_dir, &logreader));
        else
            throw MyExit("Could not find a video in the replay folder!");
    } else if (render_monitor_video_mode) {
        if (file_exist(monitor_video_fn))
            cam = std::unique_ptr<Cam>(new FileCam(replay_dir, monitor_video_fn));
        else
            throw MyExit("Could not find a video file!");
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
    init_terminal_signals();

    if (log_replay_mode && pparams.op_mode != op_mode_monitoring) {
        logreader.init(replay_dir);

        if (isinf(logreader.first_drone_ready_time()) && render_hunt_mode)
            throw MyExit("According to the log the drone was never ready: " + replay_dir);
        if (isinf(logreader.first_takeoff_time()) && render_hunt_mode)
            throw MyExit("According to the log the drone never took off: " + replay_dir);

        trackers.init_replay_moth(logreader.replay_moths());
    }
    init_loggers();
    init_video_recorders();

    rc->init(drone_id);
    cam->init();
    flight_area.init(replay_dir, cam.get());
    prev_frame = cam->current();
    visdat.init(cam.get()); // do after cam update to populate frames
    trackers.init(&logger, replay_dir, &visdat, &iceptor);
    iceptor.init(&trackers, &visdat, &flight_area, &logger, &dctrl);
    baseboard.init(log_replay_mode);
    dnav.init(&logger, &trackers, &dctrl, &visdat, &flight_area, replay_dir, &iceptor, &baseboard);
    if (log_replay_mode) {
        if (logreader.blink_at_startup())
            dnav.replay_detect_drone_location();
    }
    if (pparams.op_mode != op_mode_monitoring)
        dctrl.init(&logger, replay_dir, generator_mode, airsim_mode, rc.get(), trackers.dronetracker(), &flight_area, cam->measured_exposure());

    if (!render_hunt_mode && !render_monitor_video_mode)
        cmdcenter.init(log_replay_mode, &dnav, &dctrl, rc.get(), &trackers, &visdat);

    if (render_monitor_video_mode)
        dnav.render_now_override();

    if (pparams.has_screen || render_hunt_mode || render_monitor_video_mode) {
        visualizer.init(&visdat, &trackers, &dctrl, &dnav, rc.get(), log_replay_mode, &iceptor);
        if (generator_mode) {
            visualizer.set_generator_cam(static_cast<GeneratorCam *>(cam.get()));
        }
        visualizer_3d.init(&trackers, &flight_area, &dctrl, &dnav);
        if (log_replay_mode)
            visualizer.first_take_off_time = logreader.first_takeoff_time();
    }

    init_thread_pool();
    if (!pparams.has_screen)
        thread_watchdog = std::thread(&watchdog_worker);



#ifdef PROFILING
    logger << "t_visdat;t_trkrs;t_nav;t_ctrl;t_prdct;t_frame;"; // trail of the logging heads, needs to happen last
#endif
    logger << '\n'; // this concludes the header log line
    std::cout << "Main init successfull" << std::endl;
}

void close(bool sig_kill) {
    std::cout << "Closing" << std::endl;

    if (cam)
        cam->stop(); //cam needs to be closed after dnav, because of the camview class!

    if (!render_hunt_mode && !render_monitor_video_mode)
        cmdcenter.reset_commandcenter_status_file("Closing", false);

    if (pparams.has_screen)
        cv::destroyAllWindows();

    if (imgcount > 0)
        save_results_log();

    /*****Close everything down*****/
    if (!sig_kill)
        close_thread_pool();
    dctrl.close();
    dnav.close();
    trackers.close();
    if (!log_replay_mode)
        rc->close();
    if (pparams.has_screen || render_hunt_mode || render_monitor_video_mode)
        visualizer.close();
    visdat.close();
    baseboard.close();
    if (cam)
        cam->close(); //cam needs to be closed after dnav, because of the camview class!

    std::cout << "Closing logs" << std::endl;
    logger_video_ids << std::flush;
    logger << std::flush;
    logger_video_ids.close();
    logger.close();

    if (pparams.video_result)
        output_video_results.close();
    if (pparams.video_raw && pparams.video_raw != video_bag)
        output_video_LR.close();

    if (!render_hunt_mode && !render_monitor_video_mode)
        cmdcenter.close();

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
    auto end_datetime = chrono::system_clock::to_time_t(chrono::system_clock::now());
    std::ofstream results_log;
    results_log.open(data_output_dir  + "results.txt", std::ofstream::out);
    results_log << "op_mode:" << pparams.op_mode << '\n';
    results_log << "n_detections:" << trackers.detections_count() << '\n';
    results_log << "n_monsters:" << trackers.fp_monsters_count() << '\n';
    results_log << "n_static_fps:" << trackers.fp_statics_count() << '\n';
    results_log << "n_short_fps:" << trackers.fp_shorts_count() << '\n';
    results_log << "n_insects:" << trackers.insects_count() << '\n';
    results_log << "n_takeoffs:" << dnav.n_take_offs() << '\n';
    results_log << "n_landings:" << dnav.n_landings() << '\n';
    results_log << "n_hunts:" << dnav.n_hunt_flights() << '\n';
    results_log << "n_replay_hunts:" << cmdcenter.n_replay_moth() << '\n';
    results_log << "n_wp_flights:" << dnav.n_wp_flights() << '\n';
    results_log << "best_interception_distance:" << iceptor.best_distance() << '\n';
    results_log << "n_drone_detects:" << dnav.n_drone_detects() << '\n';
    results_log << "drone_problem:" << dnav.drone_problem() << '\n';
    results_log << "Flight_time:" << dnav.flight_time() << '\n';
    results_log << "Run_time:" << visdat.current_time() << '\n';
    results_log << "Start_datetime:" << std::put_time(std::localtime(&start_datetime), "%Y/%m/%d %T") << '\n';
    results_log << "End_datetime:" <<  std::put_time(std::localtime(&end_datetime), "%Y/%m/%d %T") << '\n';
    results_log.close();
}

void print_warnings() {
    if (pparams.video_raw && !log_replay_mode) {
        std::cout << "Video frames written: " << raw_video_frame_counter - 1 << std::endl;
        if (raw_video_frame_counter != imgcount)
            std::cout << "WARNING VIDEO FRAMES MISSING: " << imgcount - raw_video_frame_counter << std::endl;
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
            static double prev_wdt_flag_time = 0;
            if (elapsed_time - prev_wdt_flag_time > 10) {
                prev_wdt_flag_time = elapsed_time;
                set_external_wdt_flag();
            }

            cmdcenter.reset_commandcenter_status_file("Roll: " + to_string_with_precision(roll, 2), false);

            if (fabs(roll)  < pparams.max_cam_roll && !enable_delay)
                break;
            else if (fabs(roll)  > pparams.max_cam_roll) {
                if (!enable_delay)
                    std::ofstream output("logging/cam_roll_problem_flag"); //set a file flag that is periodically being checked by an external python script:
                enable_delay = 60;
            } else
                enable_delay--;


            usleep(30000); // measure every third second
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
            std::cout << std::put_time(std::localtime(&t), "%Y/%m/%d %T") << " Measured exposure: " << expo << ", gain: " << gain << ", brightness: " << avg_brightness << std::endl;
            if (expo > pparams.exposure_threshold && gain >= pparams.gain_threshold && avg_brightness < pparams.brightness_threshold) {
                break;
            }
            cv::imwrite("../../../../pats/status/monitor_tmp.jpg", frameL);
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
            set_external_wdt_flag();
            usleep(10000000); // measure every 10 seconds
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

            if (imgcount > 0)
                save_results_log();

            dctrl.close();
            dnav.close();
            trackers.close();
            visdat.close();

            logger_video_ids << std::flush;
            logger << std::flush;
            logger_video_ids.close();
            logger.close();

            if (pparams.video_result)
                output_video_results.close();
            if (pparams.video_raw && pparams.video_raw != video_bag)
                output_video_LR.close();

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
        set_external_wdt_flag();
    }
}

int main(int argc, char **argv)
{



    // baseboard.init(false);


    // while (true)
    // {
    //     std::cout << "up: " << baseboard.uptime() << std::endl;
    //     usleep(1e6);
    // }



    try {
        data_output_dir = "./logging/";
        mkdir(data_output_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

        process_arg(argc, argv);
        if (!log_replay_mode && !generator_mode && !render_monitor_video_mode && !render_hunt_mode && !airsim_mode)
            cmdcenter.reset_commandcenter_status_file("Starting", false);

        if (realsense_reset) {
            cmdcenter.reset_commandcenter_status_file("Reseting realsense", false);
            Realsense rs;
            rs.reset();
            return 0;
        }

        pparams.deserialize(pats_xml_fn);
        if (drone_xml_fn == "") // should be empty still, but can have the --drone-xml arg override
            drone_xml_fn = "../xml/" + string(drone_types_str[pparams.drone]) + ".xml";
        dparams.deserialize(drone_xml_fn);

        if (log_replay_mode)
            pparams.video_raw = video_disabled;
        if (log_replay_mode && !render_hunt_mode && !render_monitor_video_mode)
            pparams.has_screen = true; // override log so that vizs always are on when replaying because most of the logs are deployed system now (without a screen)
        if (render_hunt_mode)
            pparams.video_result = video_mkv;
        if (render_monitor_video_mode) {
            pparams.video_result = video_mkv;
            pparams.op_mode = op_mode_monitoring;
            pparams.video_raw = video_disabled;
            pparams.close_after_n_images = -1;
        }

        check_hardware();
        if (!log_replay_mode && !generator_mode && !render_monitor_video_mode && !render_hunt_mode && !airsim_mode) {
            wait_for_cam_angle();
            wait_for_dark();
        } else {
            pparams.navigation_tuning = false;
            pparams.control_tuning = false;
            pparams.vision_tuning = false;
            pparams.drone_tracking_tuning = false;
            pparams.insect_tracking_tuning = false;
            pparams.cam_tuning = false;
            if (generator_mode || render_monitor_video_mode || render_hunt_mode || airsim_wp_mode)
                pparams.joystick = rc_none;
        }

    } catch (MyExit const &err) {
        std::cout << "Error: " << err.msg << std::endl;
        cmdcenter.reset_commandcenter_status_file(err.msg, true);
        return 1;
    } catch (rs2::error const &err) {
        std::cout << "Realsense error: " << err.what() << std::endl;
        cmdcenter.reset_commandcenter_status_file("Resetting realsense", false);
        try {
            static_cast<Realsense *>(cam.get())->reset();
        } catch (MyExit const &err2) {
            std::cout << "Error: " << err2.msg << std::endl;
            cmdcenter.reset_commandcenter_status_file(err2.msg, true);
        }
        return 1;
    } catch (NoRealsenseConnected const &err) {
        set_no_realsense_flag();
        std::cout << "Error: " << err.msg << std::endl;
        return 1;
    } catch (cv::Exception const &err) {
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
    } catch (MyExit const &e) {
        exit_now = true;
        close(false);
        std::cout << "Error: " << e.msg << std::endl;
        cmdcenter.reset_commandcenter_status_file(e.msg, true);
        return 1;
    }

    close(false);
    return 0;
}
