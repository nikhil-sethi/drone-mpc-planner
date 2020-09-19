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

#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "common.h"
#include "cameraview.h"
#include "smoother.h"
#include "multimodule.h"
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
#ifdef HASGUI
#include "gui/mainwindow.h"
#endif

using namespace cv;
using namespace std;

/***********Variables****************/
bool exit_now = false;
volatile std::sig_atomic_t term_sig_fired;
int imgcount; // to measure fps
int raw_video_frame_counter = 0;
int n_fps_warnings = 0;
int insect_cnt = 0;
GStream output_video_results,output_video_LR,output_video_cuts;

xmls::PatsParameters pparams;
xmls::DroneParameters dparams;
stopwatch_c stopWatch;
std::string data_output_dir;
bool draw_plots = false;
bool log_replay_mode = false;
bool generator_mode = false;
uint8_t drone_id;
std::string replay_dir;
std::string logger_fn; //contains filename of current log # for insect logging (same as video #)
std::string pats_xml_fn,drone_xml_fn;

std::ofstream logger;
std::ofstream logger_insect;
std::ofstream logger_video_ids;
MultiModule rc;
DroneController dctrl;
navigation::DroneNavigation dnav;
tracking::TrackerManager trackers;
Visualizer visualizer;
Visualizer3D visualizer_3d;
logging::LogReader logreader;
CommandCenterLink cmdcenter;
std::unique_ptr<Cam> cam;
VisionData visdat;

/****Threadpool*******/
#define NUM_OF_THREADS 1
struct Stereo_Frame_Data {
    cv::Mat frameL,frameR;
    uint imgcount;
    unsigned long long RS_id;
    double time;
};
struct Processer {
    int id;
    std::thread * thread;
    std::mutex m1,m2;
    std::condition_variable data_processed,new_data;
    bool data_is_new = false;
    bool data_is_processed = true;
    Stereo_Frame_Data data;
};
Processer tp[NUM_OF_THREADS];

#ifdef HASGUI
MainWindow gui;
#endif

/*******Private prototypes*********/
void process_frame(Stereo_Frame_Data data_drone);
void process_video();
int main( int argc, char **argv);
bool handle_key(double time);
void save_results_log();
void print_warnings();
void close(bool sig_kill);

/************ code ***********/
void process_video() {

    filtering::Smoother fps_smoothed;
    fps_smoothed.init(100);
    stopWatch.Start();

    //main while loop:
    while (!exit_now) // ESC
    {
        cam->update();

        Stereo_Frame_Data data;
        data.frameL = cam->frameL.clone(); // these frames must be cloned because, at least for the generator, the memory is overwritten when a new frame is received
        data.frameR = cam->frameR.clone(); // it may be different for the realsense cam, but difficult to check
        data.RS_id = cam->frame_number();
        data.time = cam->frame_time();
        data.imgcount = imgcount;

        std::unique_lock<std::mutex> lk(tp[0].m2,std::defer_lock);
        tp[0].data_processed.wait(lk, []() {return tp[0].data_is_processed; });
        tp[0].data_is_processed= false;


        bool escape_key_pressed = false;
        if (pparams.has_screen) {
            static int speed_div;
            if (!(speed_div++ % 4) || (((log_replay_mode || generator_mode ) && !cam->turbo) || cam->frame_by_frame)) {
                visualizer_3d.run();
                visualizer.paint();
                escape_key_pressed = handle_key(data.time);
            }
        }
        tp[0].m1.lock();
        tp[0].data = data;
        tp[0].data_is_new = true;
        tp[0].new_data.notify_one();
        tp[0].m1.unlock();

        if (term_sig_fired==2) {
            std::cout <<"\nCaught ctrl-c: " << term_sig_fired << std::endl;
            exit_now = true;
        } else if (term_sig_fired==15) {
            std::cout <<"\nCaught TERM signal: " << term_sig_fired << std::endl;
            exit_now = true;
        } else if (term_sig_fired) {
            std::cout <<"\nCaught unknown signal: " << term_sig_fired << std::endl;
            exit_now = true;
        } else if (escape_key_pressed)
            exit_now = true;

        if (rc.init_package_failure()) {
            exit_now = true;
            cmdcenter.reset_commandcenter_status_file("MultiModule init package error",true);
        }
        if (rc.bf_version_error()) {
            exit_now = true;
            cmdcenter.reset_commandcenter_status_file("Betaflight version error",true);
        }

        static bool recording = false;
        double dtr = data.time - trackers.insecttracker_best()->last_sighting_time;
        if (dtr > 1 && recording) {
            recording = false;
            auto time_insect_now = chrono::system_clock::to_time_t(chrono::system_clock::now());
            logger_insect << "New detection ended at: " << std::put_time(std::localtime(&time_insect_now), "%Y/%m/%d %T") << " Duration: " << dtr << " End cam frame number: " <<  cam->frame_number() << '\n';
        }
        if ((trackers.insecttracker_best()->tracking() || dtr < 1) && data.time > 5) {
            recording = true;
        }

        if (pparams.video_cuts) {
            static bool was_recording = false;

            if (recording != was_recording) {
                if (recording)
                    insect_cnt++;
                else if (!recording && insect_cnt>0) {
                    output_video_cuts.close();
                    logger.close();
                    std::string cvfn = "insect" + to_string(insect_cnt) + ".mkv";
                    std::cout << "Opening new video: " << cvfn << std::endl;
                    if (output_video_cuts.init(pparams.video_cuts,data_output_dir + cvfn,IMG_W,IMG_H*2,pparams.fps, "192.168.1.255",5000,false)) {std::cout << "WARNING: could not open cut video " << cvfn << std::endl;}
                    logger_fn = data_output_dir  + "log" + to_string(insect_cnt) + ".csv";
                    logger.open(logger_fn,std::ofstream::out);
                }
                was_recording = recording;
            }

            if (recording) {
                static int cut_video_frame_counter = 0;
                int frame_written = output_video_cuts.write(data.frameL,data.frameR);
                if (!frame_written)
                    cut_video_frame_counter++;
                std::cout << "Recording! Frames written: " << cut_video_frame_counter << std::endl;
            } else {
                logger.close();
                logger.open(logger_fn,std::ofstream::out);
            }
        }

        if (pparams.video_raw && pparams.video_raw != video_bag && !log_replay_mode && pparams.op_mode != op_mode_waypoint) {
            int frame_written = 0;
            // cv::Mat id_fr = cam->frameL.clone();
            // putText(id_fr,std::to_string(data.RS_id),cv::Point(0, 13),cv::FONT_HERSHEY_SIMPLEX,0.5,Scalar(255));
            frame_written = output_video_LR.write(data.frameL,data.frameR);
            logger_video_ids << raw_video_frame_counter << ";" << data.imgcount << ";" << data.RS_id<< ";" << data.time << '\n';
            if (!frame_written)
                raw_video_frame_counter++;
            else {
                std::cout << "Video frame write PROBLEM: " << frame_written << std::endl;
            }
        }

        //keep track of time and fps

        float t_pc = stopWatch.Read() / 1000.f;
        float t_cam = static_cast<float>(cam->frame_time());
        float t;
        if ( log_replay_mode || generator_mode)
            t = t_pc;
        else
            t = t_cam;
        static float prev_time = -1.f/pparams.fps;
        float current_fps = 1.f / (t - prev_time);
        float fps = fps_smoothed.addSample(current_fps);
        if (fps < pparams.fps / 6 * 5 && fps_smoothed.ready() && !log_replay_mode && !generator_mode) {
            std::cout << "FPS WARNING!" << std::endl;
            n_fps_warnings++;
        }

        static double time =0;
        float dt __attribute__((unused)) = static_cast<float>(cam->frame_time() - time);
        time = cam->frame_time();
        std::cout <<
                  //   "\r\e[K" <<
                  dnav.navigation_status() <<
                  "; frame: " << imgcount <<
                  ", " << cam->frame_number() <<
                  ". FPS: " << to_string_with_precision(fps,1) <<
                  ". Time: " << to_string_with_precision(time,2)  <<
                  ", dt " << to_string_with_precision(dt,3) <<
                  ", " << rc.telemetry.batt_cell_v <<
                  "v, arm: " << static_cast<int>(rc.telemetry.arming_state) <<
                  ", rssi: " << static_cast<int>(rc.telemetry.rssi) <<
                  ", exposure: " << cam->measured_exposure() <<
                  ", gain: " << cam->measured_gain() <<
                  ", brightness: " << visdat.average_brightness() <<
                  std::endl;
        //   std::flush;


        if (rc.telemetry.arming_state && rc.arm_switch == bf_armed ) {
            if ( rc.telemetry.arming_state & ARMING_DISABLED_NO_GYRO)
                std::cout << "ARMING_DISABLED_NO_GYRO | ";
            if ( rc.telemetry.arming_state & ARMING_DISABLED_FAILSAFE)
                std::cout << "ARMING_DISABLED_FAILSAFE | ";
            if ( rc.telemetry.arming_state & ARMING_DISABLED_RX_FAILSAFE)
                std::cout << "ARMING_DISABLED_RX_FAILSAFE | ";
            if ( rc.telemetry.arming_state & ARMING_DISABLED_BAD_RX_RECOVERY)
                std::cout << "ARMING_DISABLED_BAD_RX_RECOVERY | ";
            if ( rc.telemetry.arming_state & ARMING_DISABLED_BOXFAILSAFE)
                std::cout << "ARMING_DISABLED_BOXFAILSAFE | ";
            if ( rc.telemetry.arming_state & ARMING_DISABLED_RUNAWAY_TAKEOFF)
                std::cout << "ARMING_DISABLED_RUNAWAY_TAKEOFF | ";
            if ( rc.telemetry.arming_state & ARMING_DISABLED_CRASH_DETECTED)
                std::cout << "ARMING_DISABLED_CRASH_DETECTED | ";
            if ( rc.telemetry.arming_state & ARMING_DISABLED_THROTTLE)
                std::cout << "ARMING_DISABLED_THROTTLE | ";
            if ( rc.telemetry.arming_state & ARMING_DISABLED_ANGLE)
                std::cout << "ARMING_DISABLED_ANGLE | ";
            if ( rc.telemetry.arming_state & ARMING_DISABLED_BOOT_GRACE_TIME)
                std::cout << "ARMING_DISABLED_BOOT_GRACE_TIME | ";
            if ( rc.telemetry.arming_state & ARMING_DISABLED_NOPREARM)
                std::cout << "ARMING_DISABLED_NOPREARM | ";
            if ( rc.telemetry.arming_state & ARMING_DISABLED_LOAD)
                std::cout << "ARMING_DISABLED_LOAD | ";
            if ( rc.telemetry.arming_state & ARMING_DISABLED_CALIBRATING)
                std::cout << "ARMING_DISABLED_CALIBRATING | ";
            if ( rc.telemetry.arming_state & ARMING_DISABLED_CLI)
                std::cout << "ARMING_DISABLED_CLI | ";
            if ( rc.telemetry.arming_state & ARMING_DISABLED_CMS_MENU)
                std::cout << "ARMING_DISABLED_CMS_MENU | ";
            if ( rc.telemetry.arming_state & ARMING_DISABLED_BST)
                std::cout << "ARMING_DISABLED_BST | ";
            if ( rc.telemetry.arming_state & ARMING_DISABLED_MSP)
                std::cout << "ARMING_DISABLED_MSP | ";
            if ( rc.telemetry.arming_state & ARMING_DISABLED_PARALYZE)
                std::cout << "ARMING_DISABLED_PARALYZE | ";
            if ( rc.telemetry.arming_state & ARMING_DISABLED_GPS)
                std::cout << "ARMING_DISABLED_GPS | ";
            if ( rc.telemetry.arming_state & ARMING_DISABLED_RESC)
                std::cout << "ARMING_DISABLED_RESC | ";
            if ( rc.telemetry.arming_state & ARMING_DISABLED_RPMFILTER)
                std::cout << "ARMING_DISABLED_RPMFILTER | ";
            if ( rc.telemetry.arming_state & ARMING_DISABLED_REBOOT_REQUIRED)
                std::cout << "ARMING_DISABLED_REBOOT_REQUIRED | ";
            if ( rc.telemetry.arming_state & ARMING_DISABLED_DSHOT_BITBANG)
                std::cout << "ARMING_DISABLED_DSHOT_BITBANG | ";
            if ( rc.telemetry.arming_state & ARMING_DISABLED_ACC_CALIBRATION)
                std::cout << "ARMING_DISABLED_ACC_CALIBRATION | ";
            if ( rc.telemetry.arming_state & ARMING_DISABLED_ARM_SWITCH)
                std::cout << "ARMING_DISABLED_ARM_SWITCH ";
            std::cout << std::endl;
        }

        imgcount++;
        prev_time = t;

        if (fps != fps || isinf(fps))
            fps_smoothed.reset();

        static uint restart_delay = 0;
        if (dnav.time_for_restart())
            restart_delay++;
        else
            restart_delay = 0;

        if (dctrl.in_flight_duration(time) < 0.1f || dnav.drone_problem()) {
            if (!log_replay_mode  && ((imgcount > pparams.close_after_n_images && pparams.close_after_n_images>0))) {
                std::cout << "Initiating periodic restart" << std::endl;
                exit_now = true;
            } else if ((cam->measured_exposure() <= pparams.darkness_threshold && pparams.darkness_threshold>0)) {
                std::cout << "Initiating restart because exposure (" << cam->measured_exposure() << ") is lower than darkness_threshold (" << pparams.darkness_threshold << ")" << std::endl;
                exit_now = true;
            } else if (visdat.average_brightness() > pparams.max_brightness && pparams.darkness_threshold>0) {
                std::cout << "Initiating restart because avg brightness (" << visdat.average_brightness() << ") is higher than max_brightness (" << pparams.max_brightness << ")" << std::endl;
                exit_now = true;
            } else if(restart_delay > 5) {
                std::cout << "Flight termintated" << std::endl;
                if (dctrl.flight_aborted())
                    std::cout << "Control problem: " << dctrl.flight_mode() << std::endl;
                else
                    std::cout << "Nav status: " << dnav.navigation_status() << std::endl;
                exit_now = true;
            }
        }

    } // main while loop
    std::cout << "Exiting main loop" << std::endl;
}

void process_frame(Stereo_Frame_Data data) {

    if (log_replay_mode) {
        if (logreader.current_frame_number(data.RS_id)) {
            exit_now = true;
            return;
        }

        trackers.process_replay_moth(data.RS_id);
        cmdcenter.trigger_demo_flight_from_log(replay_dir,logreader.current_entry.trkrs_state);
    } else if (generator_mode) {
        if (dnav.drone_is_ready_and_waiting()) {
            dnav.demo_flight(pparams.flightplan);
            dctrl.joy_takeoff_switch_file_trigger(true);
        }
    }

#ifdef PROFILING
    auto profile_t0 = std::chrono::high_resolution_clock::now();
#endif
    visdat.update(data.frameL,data.frameR,data.time,data.RS_id);
#ifdef PROFILING
    auto profile_t1_visdat = std::chrono::high_resolution_clock::now();
#endif

    auto time_now = chrono::system_clock::to_time_t(chrono::system_clock::now());
    logger << data.imgcount << ";"
           << data.RS_id << ";"
           << std::put_time(std::localtime(&time_now), "%Y/%m/%d %T") << ";"
           << data.time << ";"
           << cam->measured_exposure() << ";";

    trackers.update(data.time,dctrl.drone_is_active());
#ifdef PROFILING
    auto profile_t2_trkrs = std::chrono::high_resolution_clock::now();
#endif

    if (log_replay_mode) {
        dctrl.insert_log(logreader.current_entry.joyRoll, logreader.current_entry.joyPitch, logreader.current_entry.joyYaw, logreader.current_entry.joyThrottle,logreader.current_entry.joyArmSwitch,logreader.current_entry.joyModeSwitch,logreader.current_entry.joyTakeoffSwitch,logreader.current_entry.auto_roll,logreader.current_entry.auto_pitch,logreader.current_entry.auto_throttle, logreader.current_entry.telem_acc_z, logreader.current_entry.telem_throttle, logreader.current_entry.telem_throttle_s, logreader.current_entry.maxthrust, logreader.current_entry.telem_thrust_rpm);
    }
    dnav.update(data.time);
#ifdef PROFILING
    auto profile_t3_nav = std::chrono::high_resolution_clock::now();
#endif

    dctrl.control(trackers.dronetracker()->Last_track_data(),dnav.setpoint(),trackers.insecttracker_best()->Last_track_data(),data.time);
#ifdef PROFILING
    auto profile_t4_ctrl = std::chrono::high_resolution_clock::now();
#endif

#ifdef PROFILING
    auto profile_t5_prdct = std::chrono::high_resolution_clock::now();
#endif

    trackers.dronetracker()->manual_flight_mode(dnav.drone_is_manual()); // TODO: hacky

    if (pparams.has_screen) {
        visualizer.add_plot_sample();
        visualizer.update_tracker_data(visdat.frameL,dnav.setpoint().pos(),data.time, draw_plots, trackers.insecttracker_best());
        if (pparams.video_result) {
            if (log_replay_mode)
                output_video_results.block(); // only use this for rendering
            output_video_results.write(visualizer.trackframe);
        }
    }
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
    logger  << '\n';
}

void init_insect_log(int n) {
    trackers.init_replay_moth(n);
}

bool handle_key(double time [[maybe_unused]]) {
    if (exit_now)
        return true;

    unsigned char key = cv::waitKey(1);
    key = key & 0xff;
    if (key == 27)   //esc
        return true;

    switch(key) {
    case 'b':
        rc.bind(true);
        break;
    case 's':
        dnav.shake_drone();
        break;
    case 'B':
        rc.beep();
        break;
    case 'c':
        rc.calibrate_acc();
        break;
    case 'l':
        dnav.redetect_drone_location();
        break;
    case 'p':
        if(log_replay_mode || generator_mode)
            draw_plots = true;
        break;
    case 'o':
        dctrl.LED(true);
        dnav.nav_flight_mode(navigation::nfm_manual);
        break;
    case '1':
        init_insect_log(56);
        break;
    case '2':
        init_insect_log(53);
        break;
    case '3':
        init_insect_log(58);
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
        trackers.init_virtualmoth_moth(tracking::diving, &dctrl);
        break;
    case 82: // arrow up
        dnav.manual_trigger_next_wp();
        break;
    case 84: // arrow down
        dnav.manual_trigger_prev_wp();
        break;
    case ' ':
    case 'f':
        if(log_replay_mode || generator_mode)
            cam->frame_by_frame = true;
        break;
    case 't':
        cam->turbo = !cam->turbo;
        break;
    case '>': {
        double dt = logreader.first_takeoff_time() - time - 5;
        if (dt>0 && trackers.mode() != tracking::TrackerManager::mode_locate_drone)
            cam->skip(dt);
        break;
    }
    case '.':
        cam->skip_one_sec();
        break;
    case ',':
        cam->back_one_sec();
        break;
    case 'a':
        rc.arm(bf_armed);
        dnav.nav_flight_mode(navigation::nfm_waypoint);
        break;
    case 'h':
        rc.arm(bf_armed);
        dnav.nav_flight_mode(navigation::nfm_hunt);
        break;
    case 'd':
        rc.arm(bf_disarmed);
        dnav.nav_flight_mode(navigation::nfm_manual);
        break;
    } // end switch key
    return false;
}

//This is where frames get processed after it was received from the cam in the main thread
void pool_worker(int id ) {
    std::unique_lock<std::mutex> lk(tp[id].m1,std::defer_lock);
    while(!exit_now) {
        tp[id].new_data.wait(lk,[]() {return tp[0].data_is_new;});
        tp[0].data_is_new = false;
        if (exit_now)
            break;
        process_frame(tp->data);
        tp[id].m2.lock();
        tp[id].data_is_processed = true;
        tp[id].data_processed.notify_one();
        tp[id].m2.unlock();
    }
}
bool threads_initialised = false;
void init_thread_pool() {
    for (uint i = 0; i < NUM_OF_THREADS; i++) {
        tp[i].thread = new thread(&pool_worker,i);
    }
    threads_initialised=true;
}
void close_thread_pool() {
    if (threads_initialised) {
        std::cout <<"Stopping threads in pool"<< std::endl;
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
        for (uint i = 0; i < NUM_OF_THREADS; i++) {
            tp[i].thread->join();
            delete tp[i].thread;
        }

        std::cout <<"Threads in pool closed"<< std::endl;
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

    if (replay_dir == "") {
        pparams.serialize("./logging/pats.xml");
        dparams.serialize("./logging/drone.xml");
    }

    logger.open(data_output_dir  + "log.csv",std::ofstream::out);
    cout << "data_output_dir: " << data_output_dir << endl;

    logger << "ID;RS_ID;time;elapsed;Exposure;";
    logger_fn = data_output_dir  + "log" + to_string(0) + ".csv"; // only used with pparams.video_cuts

    logger_insect.open(data_output_dir  + "insect.log",std::ofstream::out);
    logger_video_ids.open(data_output_dir  + "frames.csv",std::ofstream::out);
}

void init_video_recorders() {
    /*****init the video writer*****/
    if (pparams.video_result)
        if (output_video_results.init(pparams.video_result, data_output_dir + "videoResult.mkv",visualizer.viz_frame_size().width,visualizer.viz_frame_size().height,pparams.fps,"192.168.1.255",5000,true)) {throw my_exit("could not open results video");}
    if (pparams.video_raw && pparams.video_raw != video_bag && !log_replay_mode && pparams.op_mode != op_mode_waypoint)
        if (output_video_LR.init(pparams.video_raw,data_output_dir + "videoRawLR.mkv",IMG_W,IMG_H*2,pparams.fps, "192.168.1.255",5000,false)) {throw my_exit("could not open LR video");}
    if (pparams.video_cuts)
        if (output_video_cuts.init(pparams.video_cuts,data_output_dir + "insect" + to_string(0) + ".mkv",IMG_W,IMG_H*2,pparams.fps, "192.168.1.255",5000,false)) {std::cout << "WARNING: could not open cut video " << data_output_dir + "insect" + to_string(0) + ".mkv" << std::endl;}
}

std::tuple<bool,bool,bool, std::string,uint8_t,std::string,std::string> process_arg(int argc, char **argv) {

    int default_drone_id = 1;
    int arg_drone_id = default_drone_id;
    bool replay_mode = false, gen_mode = false;
    std::string arg_log_folder="",arg_pats_xml_fn="../../xml/pats.xml",arg_drone_xml_fn="";
    if (argc > 1) {
        for (int i = 1; i < argc; i++) {
            string s = argv[i];
            bool arg_recognized = false;
            if (s.compare("--rs_reset") == 0) {
                return make_tuple(true,false,false,"",0,"","");
            } else if (s.compare("--pats-xml") == 0) {
                arg_recognized = true;
                i++;
                arg_pats_xml_fn = argv[i];
            } else if (s.compare("--drone-xml") == 0) {
                arg_recognized = true;
                i++;
                arg_drone_xml_fn = argv[i];
            } else if (s.compare("--drone-id") == 0) {
                arg_recognized = true;
                i++;
                arg_drone_id = stoi(argv[i]);
            } else if (s.compare("--generator") == 0) {
                arg_recognized = true;
                i++;
                gen_mode = true;
            } else if (s.compare("--log") == 0) {
                arg_recognized = true;
                i++;
                replay_mode=true;
                arg_log_folder=argv[i];
                arg_pats_xml_fn = arg_log_folder + "/pats.xml";
                arg_drone_xml_fn = arg_log_folder + "/drone.xml";
            }
            if (!arg_recognized) {
                std::cout << "Error argument nog recognized: " << argv[i] <<std::endl;
                exit(1);
            }
        }
    }
    return make_tuple(false,replay_mode,gen_mode,arg_log_folder,arg_drone_id,arg_pats_xml_fn,arg_drone_xml_fn);
}

void check_hardware() {
    if (! log_replay_mode) {
        //multimodule rc
        if (! generator_mode && dparams.tx != tx_none )
            rc.init(drone_id);
        if (!rc.connected() && pparams.op_mode != op_mode_monitoring)
            throw my_exit("cannot connect the MultiModule");

        // Ensure that joystick was found and that we can use it
        if (!dctrl.joystick_ready() && !generator_mode && pparams.joystick != rc_none) {
            throw my_exit("no joystick connected.");
        }
    }
    //init cam and check version
    if (log_replay_mode) {
        if (file_exist(replay_dir+'/' + Realsense::playback_filename()))
            cam = std::unique_ptr<Cam>(new Realsense(replay_dir));
        else if (file_exist(replay_dir+'/' + FileCam::playback_filename()))
            cam = std::unique_ptr<Cam>(new FileCam(replay_dir,&logreader));
        else
            throw my_exit("Could not find a video in the replay folder!");
    } else if (generator_mode) {
        cam = std::unique_ptr<Cam>(new GeneratorCam());
        static_cast<GeneratorCam *>(cam.get())->rc(&rc);
    } else {
        cam = std::unique_ptr<Cam>(new Realsense());
        static_cast<Realsense *>(cam.get())->connect_and_check();
    }
}

void init() {
    init_terminal_signals();

    if (log_replay_mode) {
        logreader.init(replay_dir);
        trackers.init_replay_moth(logreader.replay_moths());
    }
    init_loggers();

#ifdef HASGUI
    gui.init(argc,argv);
#endif
    cam->init();
    visdat.init(cam->Qf, cam->frameL,cam->frameR,cam->camera_angle(),cam->measured_gain(),cam->measured_exposure(),cam->depth_background_mm); // do after cam update to populate frames
    trackers.init(&logger, &visdat, &(cam->camera_volume));
    dnav.init(&logger,&trackers,&dctrl,&visdat, &(cam->camera_volume),replay_dir);
    dctrl.init(&logger,log_replay_mode,generator_mode,&rc,trackers.dronetracker(), &(cam->camera_volume),cam->measured_exposure());

    if (pparams.has_screen) {
        visualizer.init(&visdat,&trackers,&dctrl,&dnav,&rc,log_replay_mode);
        if (generator_mode) {
            visualizer.set_generator_cam(static_cast<GeneratorCam *>(cam.get()));
        }
        visualizer_3d.init(&trackers, &(cam->camera_volume), &dctrl, &dnav);
        if (log_replay_mode)
            visualizer.first_take_off_time = logreader.first_takeoff_time();
    }

    init_video_recorders();

    init_thread_pool();

    cmdcenter.init(log_replay_mode,&dnav,&dctrl,&rc,cam.get(),&trackers);

#ifdef PROFILING
    logger << "t_visdat;t_trkrs;t_nav;t_ctrl;t_prdct;t_frame;"; // trail of the logging heads, needs to happen last
#endif
    logger << '\n'; // this concludes the header log line
    std::cout << "Main init successfull" << std::endl;
}

void close(bool sig_kill) {
    std::cout <<"Closing"<< std::endl;
    cmdcenter.reset_commandcenter_status_file("Closing",false);

    if (pparams.has_screen)
        cv::destroyAllWindows();
#ifdef HASGUI
    gui.close();
#endif

    save_results_log();

    /*****Close everything down*****/
    if (cam)
        cam->close(); // attempt to save the video first

    dctrl.close();
    dnav.close();
    trackers.close();
    if (!log_replay_mode)
        rc.close();
    if (pparams.has_screen)
        visualizer.close();
    visdat.close();

    if (pparams.video_result)
        output_video_results.close();
    if (pparams.video_raw && pparams.video_raw != video_bag && !log_replay_mode && pparams.op_mode != op_mode_waypoint)
        output_video_LR.close();
    if (pparams.video_cuts)
        output_video_cuts.close();

    logger_insect << std::flush;
    logger_video_ids << std::flush;
    logger << std::flush;
    logger_video_ids.close();
    logger_insect.close();
    logger.close();
    if (!sig_kill)
        close_thread_pool();
    cmdcenter.close();

    print_warnings();
    if(cam)
        cam.release();
    std::cout <<"Closed"<< std::endl;
}

void save_results_log() {
    std::ofstream results_log;
    results_log.open(data_output_dir  + "results.txt",std::ofstream::out);
    results_log << "op_mode:" << pparams.op_mode << '\n';
    results_log << "n drone detects:" << dnav.n_drone_detects() << '\n';
    results_log << "n_insects:" << insect_cnt << '\n';
    results_log << "n_takeoffs:" << dnav.n_take_offs() << '\n';
    results_log << "n_landings:" << dnav.n_landings() << '\n';
    results_log << "n_hunts:" << dnav.n_hunt_flights() << '\n';
    results_log << "n_replay_hunts:" << cmdcenter.n_replay_moth() << '\n';
    results_log << "n_wp_flights:" << dnav.n_wp_flights() << '\n';
    results_log << "best_interception_distance:" << dnav.interceptor().best_distance() << '\n';
    results_log << "n_drone_detects:" << dnav.n_drone_detects() << '\n';
    results_log << "drone problem:" << dnav.drone_problem() << '\n';
    results_log << "Flight_time:" << dnav.flight_time() << '\n';
    results_log << "Run_time:" << cam->frame_time() << '\n';
    results_log.close();
}

void print_warnings() {
    if (pparams.video_raw && !log_replay_mode) {
        std::cout <<"Video frames written: " << raw_video_frame_counter-1 << std::endl;
        if (raw_video_frame_counter != imgcount)
            std::cout <<"WARNING VIDEO FRAMES MISSING: : " << raw_video_frame_counter - imgcount << std::endl;
    }
    if (n_fps_warnings)
        std::cout <<"WARNING FPS PROBLEMS: : " << n_fps_warnings << std::endl;
    if (cam->frame_loss_cnt())
        std::cout <<"WARNING FRAME LOSSES: : " << cam->frame_loss_cnt() << std::endl;

}

void wait_for_cam_angle() {
    int enable_delay = 0;
    if (pparams.max_cam_roll > 0 && !log_replay_mode) {
        std::cout << "Checking cam angle." << std::endl;

        while(true) {
            auto [roll,pitch,frame_time,frameL] = static_cast<Realsense *>(cam.get())->measure_angle();
            static double roll_start_time = frame_time;
            double elapsed_time = (frame_time- roll_start_time) / 1000.;

            auto t = chrono::system_clock::to_time_t(chrono::system_clock::now());
            std::cout << std::put_time(std::localtime(&t), "%Y/%m/%d %T") << ". Camera roll: " << to_string_with_precision(roll,2) << "°- max: " << pparams.max_cam_roll << "°. Pitch: " << to_string_with_precision(pitch,2) << "°" << std::endl;

            static double prev_imwrite_time = -pparams.live_image_frq;
            if (elapsed_time - prev_imwrite_time > pparams.live_image_frq && pparams.live_image_frq >= 0) {
                prev_imwrite_time = elapsed_time;
                cv::imwrite("../../../../pats_monitor_tmp.jpg", frameL);
            }
            cmdcenter.reset_commandcenter_status_file("Roll: " + to_string_with_precision(roll,2),false);

            if (fabs(roll)  < pparams.max_cam_roll && !enable_delay)
                break;
            else if (fabs(roll)  > pparams.max_cam_roll)
                enable_delay = 60;
            else
                enable_delay--;


            usleep(30000); // measure every third second
        }
    }
}

void wait_for_dark() {
    if (pparams.darkness_threshold > 0 && !log_replay_mode) {
        std::cout << "Checking if dark." << std::endl;
        while(true) {
            auto [expo,gain,frameL,avg_brightness] = static_cast<Realsense *>(cam.get())->measure_auto_exposure();
            auto t = chrono::system_clock::to_time_t(chrono::system_clock::now());
            std::cout << std::put_time(std::localtime(&t), "%Y/%m/%d %T") << " Measured exposure: " << expo << ", gain: " << gain << ", avg_brightness: " << avg_brightness << std::endl;
            if (expo >pparams.darkness_threshold && gain >= 16 && avg_brightness < pparams.max_brightness) { // minimum RS gain is 16, so at the moment this condition does nothing
                break;
            }
            cv::imwrite("../../../../pats_monitor_tmp.jpg", frameL);
            cmdcenter.reset_commandcenter_status_file("Waiting. Exposure: " + std::to_string(static_cast<int>(expo)) + ", brightness: " + std::to_string(static_cast<int>(visdat.average_brightness())),false);
            usleep(10000000); // measure every 1 minute
        }
    }
}

int main( int argc, char **argv )
{
    try {
        data_output_dir = "./logging/";
        mkdir(data_output_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

        cmdcenter.reset_commandcenter_status_file("Starting",false);
        bool realsense_reset;
        std::tie(realsense_reset,log_replay_mode,generator_mode,replay_dir,drone_id,pats_xml_fn,drone_xml_fn) = process_arg(argc,argv);

        if (realsense_reset) {
            cmdcenter.reset_commandcenter_status_file("Reseting realsense",false);
            Realsense rs;
            rs.reset();
            return 0;
        }

        pparams.deserialize(pats_xml_fn);
        if (replay_dir == "" && drone_xml_fn == "")
            drone_xml_fn = "../../xml/" + string(drone_types_str[pparams.drone]) + ".xml";
        else
            pparams.has_screen = true; // override log so that vizs always are on when replaying because most of the logs are deployed system now (without a screen)
        dparams.deserialize(drone_xml_fn);

        check_hardware();
        if (!log_replay_mode && !generator_mode) {
            wait_for_cam_angle();
            wait_for_dark();
        } else {
            pparams.navigation_tuning = false;
            pparams.control_tuning = false;
            pparams.vision_tuning = false;
            pparams.drone_tracking_tuning = false;
            pparams.insect_tracking_tuning = false;
            pparams.cam_tuning = false;
            if (generator_mode) {
                pparams.joystick = rc_none;
            }

        }

    } catch(my_exit const &err) {
        std::cout << "Error: " << err.msg << std::endl;
        cmdcenter.reset_commandcenter_status_file(err.msg,true);
        return 1;
    } catch(rs2::error const &err) {
        cmdcenter.reset_commandcenter_status_file("Resetting realsense",false);
        try {
            static_cast<Realsense *>(cam.get())->reset();
        } catch(my_exit const &err2) {
            std::cout << "Error: " << err2.msg << std::endl;
            cmdcenter.reset_commandcenter_status_file(err2.msg,true);
        }
        return 1;
    } catch (cv::Exception const &err) {
        cmdcenter.reset_commandcenter_status_file(err.msg,true);
        std::cout << "Error: " << err.msg << std::endl;
        return 1;
    }

    try {
        init();
        process_video();
    } catch(bag_video_ended const &err) {
        std::cout << "Video ended" << std::endl;
        exit_now = true;
    } catch(my_exit const &e) {
        exit_now = true;
        close(false);
        std::cout << "Error: " << e.msg << std::endl;
        cmdcenter.reset_commandcenter_status_file(e.msg,true);
        return 1;
    }

    close(false);
    return 0;
}
