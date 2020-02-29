#include <mutex>
#include <thread>
#include <fstream>      // std::ifstream
#include <sstream>
#include <iostream>
#include <iomanip>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h> //usleep
#include <ctime>
#include <sys/stat.h>
#include <condition_variable>
#include<iostream>

#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "common.h"
#include "cameraview.h"
#include "defines.h"
#include "smoother.h"
#include "multimodule.h"
#include "stopwatch.h"
#include "dronetracker.h"
#include "dronecontroller.h"
#include "dronepredictor.h"
#include "gstream.h"
#include "vizs.h"
#include "insecttracker.h"
#include "trackermanager.h"
#include "logreader.h"
#include "visiondata.h"
#include "3dviz/visualizer3d.h"
#if CAMMODE == CAMMODE_FROMVIDEOFILE
#include "filecam.h"
#elif CAMMODE == CAMMODE_AIRSIM
#include "airsim.h"
#elif CAMMODE == CAMMODE_REALSENSE
#include "cam.h"
#elif CAMMODE == CAMMODE_GENERATOR
#include "generatorcam.h"
#endif
#ifdef HASGUI
#include "gui/mainwindow.h"
#endif

using namespace cv;
using namespace std;

/***********Variables****************/
unsigned char key = 0;
int imgcount,detectcount; // to measure fps
GStream output_video_results,output_video_LR,output_video_cuts;

int main_argc;
char **main_argv;
xmls::PatsParameters pparams;
xmls::DroneParameters dparams;
stopwatch_c stopWatch;
std::string data_output_dir;
bool draw_plots = false;
bool log_replay_mode = false;
uint8_t drone_id;
std::string replay_dir;
std::string logger_fn; //contains filename of current log # for insect logging (same as video #)

std::ofstream logger;
std::ofstream logger_insect;
MultiModule rc;
DroneController dctrl;
DronePredictor dprdct;
navigation::DroneNavigation dnav;
tracking::TrackerManager trackers;
Visualizer visualizer;
Visualizer3D visualizer_3d;
logging::LogReader logreader;
#if CAMMODE == CAMMODE_FROMVIDEOFILE
FileCam cam;
#define Cam FileCam //wow that is pretty hacky :)
#elif CAMMODE == CAMMODE_AIRSIM
Airsim cam;
#define Cam Airsim //wow that is pretty hacky :)
#elif CAMMODE == CAMMODE_REALSENSE
Cam cam;
#elif CAMMODE == CAMMODE_GENERATOR
GeneratorCam cam;
#endif
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
void handle_key(double time);
void close(bool sig_kill);
void write_status_image();
void write_status_file();

/************ code ***********/
void process_video() {

    filtering::Smoother fps_smoothed;
    fps_smoothed.init(100);
    stopWatch.Start();

    //main while loop:
    while (key != 27) // ESC
    {
        cam.update();

        Stereo_Frame_Data data;
        data.frameL = cam.frameL;
        data.frameR = cam.frameR;
        data.RS_id = cam.frame_number();
        data.time = cam.frame_time();
        data.imgcount = imgcount;

        std::unique_lock<std::mutex> lk(tp[0].m2,std::defer_lock);
        tp[0].data_processed.wait(lk, []() {return tp[0].data_is_processed; });
        tp[0].data_is_processed= false;
        if (pparams.has_screen) {
            static int speed_div;
            if (!(speed_div++ % 4) || ((log_replay_mode && !cam.turbo) || cam.frame_by_frame)) {
                visualizer_3d.run();
                visualizer.paint();
                handle_key(data.time);
            }
        }
        tp[0].m1.lock();
        tp[0].data = data;
        tp[0].data_is_new = true;
        tp[0].new_data.notify_one();
        tp[0].m1.unlock();

        static bool recording = false;
        double dtr = data.time - trackers.insecttracker_best()->last_sighting_time;
        if (dtr > 1 && recording) {
            recording = false;
            auto time_insect_now = chrono::system_clock::to_time_t(chrono::system_clock::now());
            logger_insect << "New detection ended at: " << std::put_time(std::localtime(&time_insect_now), "%Y/%m/%d %T") << " Duration: " << dtr << " End cam frame number: " <<  cam.frame_number() << std::endl;
            detectcount++;
        }
        if ((trackers.insecttracker_best()->tracking() || dtr < 1) && data.time > 5) {
            recording = true;
        }

        if (pparams.video_cuts) {
            static bool was_recording = false;
            static int insect_cnt = 0;
            if (cam.frame_number() / 2 && cam.frame_number() % 2) {
                if (recording != was_recording) {
                    if (recording)
                        insect_cnt++;
                    else if (!recording && insect_cnt>0) {
                        output_video_cuts.close();
                        logger.close();
                        std::string cvfn = "insect" + to_string(insect_cnt) + ".mp4";
                        std::cout << "Opening new video: " << cvfn << std::endl;
                        if (output_video_cuts.init(main_argc,main_argv,pparams.video_cuts,data_output_dir + cvfn,IMG_W*2,IMG_H,pparams.fps/2, "192.168.1.255",5000,true,false)) {std::cout << "WARNING: could not open cut video " << cvfn << std::endl;}
                        logger_fn = data_output_dir  + "log" + to_string(insect_cnt) + ".csv";
                        logger.open(logger_fn,std::ofstream::out);
                    }
                    was_recording = recording;
                }

                if (recording) {
                    static int cut_video_frame_counter = 0;

                    cv::Mat frame(cam.frameL.rows,cam.frameL.cols+trackers.diff_viz.cols,CV_8UC3);
                    cvtColor(cam.frameL,frame(cv::Rect(0,0,cam.frameL.cols, cam.frameL.rows)),CV_GRAY2BGR);
                    trackers.diff_viz.copyTo(frame(cv::Rect(cam.frameL.cols,0,trackers.diff_viz.cols, trackers.diff_viz.rows)));
                    cv::putText(frame,std::to_string(cam.frame_number()) + ": " + to_string_with_precision( cam.frame_time(),2),cv::Point(trackers.diff_viz.cols, 13),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,0,255));
                    int frame_written = output_video_cuts.write(frame);
                    if (!frame_written)
                        cut_video_frame_counter++;
                    std::cout << "Recording! Frames written: " << cut_video_frame_counter << std::endl;
                }
            }
        }
        if (!recording && pparams.video_cuts && (cam.frame_number() / 2 && cam.frame_number() % 2)) {
            logger.close();
            logger.open(logger_fn,std::ofstream::out);
        }
        if (pparams.video_raw && pparams.video_raw != video_bag) {
            int frame_written = 0;
            if (recording) {
                cv::Mat frameL  =cam.frameL.clone();
                cv::putText(frameL,std::to_string(cam.frame_number()),cv::Point(0, 13),cv::FONT_HERSHEY_SIMPLEX,0.5,255);
                frame_written = output_video_LR.write(frameL,cam.frameR);
                static int video_frame_counter = 0;
                if (!frame_written)
                    video_frame_counter++;
                else {
                    std::cout << "PROBLEM: " << frame_written << std::endl;
                }
                std::cout << "Recording! Frames written: " << video_frame_counter << std::endl;
            }
        }

        //keep track of time and fps
        float t = stopWatch.Read() / 1000.f;
        static float prev_time = -1.f/pparams.fps;
        float current_fps = 1.f / (t - prev_time);
        float fps = fps_smoothed.addSample(current_fps);
        if (fps < pparams.fps / 6 * 5 && !log_replay_mode)
            std::cout << "FPS WARNING!" << std::endl;

        static double time =0;
        float dt __attribute__((unused)) = static_cast<float>(cam.frame_time() - time);
        time = cam.frame_time();
        std::cout << dnav.navigation_status() <<  "; frame: " <<imgcount << ", " << cam.frame_number() << ". FPS: " << to_string_with_precision(fps,1) << ". Time: " << to_string_with_precision(time,2)  << ", dt " << to_string_with_precision(dt,3) << std::endl;
        imgcount++;
        prev_time = t;

        if (fps != fps || isinf(fps))
            fps_smoothed.reset();

        static uint restart_delay = 0;
        if (dnav.time_for_restart())
            restart_delay++;
        else
            restart_delay = 0;

        if (!log_replay_mode && ((imgcount > 360000 && pparams.insect_logging_mode)
                                 || (cam.measured_exposure() <= pparams.darkness_threshold && pparams.darkness_threshold>0))) {
            std::cout << "Initiating periodic restart" << std::endl;
            key =27;
        } else if(restart_delay > 1.5f*dnav.time_out_after_landing*pparams.fps) {
            std::cout << "Flight termintated" << std::endl;
            if (dctrl.flight_aborted())
                std::cout << "Control problem: " << dctrl.flight_mode() << std::endl;
            else
                std::cout << "Nav status: " << dnav.navigation_status() << std::endl;
            key =27;
        }


    } // main while loop
}

void write_status_file() {

    static std::string nav_status = "";
    bool status_update_needed = nav_status.compare(dnav.navigation_status()) != 0;
    nav_status = dnav.navigation_status();

    static double prev_imwrite_time = -pparams.live_image_frq;
    if (cam.frame_time() - prev_imwrite_time > pparams.live_image_frq && pparams.live_image_frq >= 0) {
        prev_imwrite_time = cam.frame_time();

        write_status_image();

        status_update_needed = true;

    }
    if (status_update_needed) {
        char hostname[20];
        gethostname(hostname,20);
        std::ofstream status_file;
        status_file.open("../../../../pats_status.txt",std::ofstream::out);
        status_file << hostname << std::endl;
        auto time_now = chrono::system_clock::to_time_t(chrono::system_clock::now());
        status_file << std::put_time(std::localtime(&time_now), "%Y/%m/%d %T") << std::endl;
        status_file << "Runtime: " << cam.frame_time() << "s" << std::endl;
        status_file << nav_status << std::endl;
    }

}

void reset_status_file(std::string status_msg) {
    char hostname[20];
    gethostname(hostname,20);
    std::ofstream status_file;
    status_file.open("../../../../pats_status.txt",std::ofstream::out);
    status_file << hostname << std::endl;
    auto time_now = chrono::system_clock::to_time_t(chrono::system_clock::now());
    status_file << std::put_time(std::localtime(&time_now), "%Y/%m/%d %T") << std::endl;
    status_file << "Runtime: " << 0 << "s" << std::endl;
    status_file << status_msg << std::endl;
}

void write_status_image() {
    cv::Mat out = cam.frameL.clone();
    cvtColor(out,out,CV_GRAY2BGR);
    putText(out,"State: " + dnav.navigation_status() + " " + trackers.mode_str() + " " + dctrl.flight_mode() +
            " "  + trackers.dronetracker()->drone_tracking_state(),cv::Point(5,14),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));
    putText(out,"Time:       " + to_string_with_precision(cam.frame_time(),2),cv::Point(5,28),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));
    putText(out,"Detections: " + std::to_string(detectcount),cv::Point(5,42),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));

    cv::imwrite("../../../../pats_monitor_tmp.jpg", out);
}

void process_frame(Stereo_Frame_Data data) {

    if (log_replay_mode) {
        logreader.current_frame_number(data.RS_id);
        trackers.process_replay_moth(data.RS_id);
    }

    visdat.update(data.frameL,data.frameR,data.time,data.RS_id);

    auto time_now = chrono::system_clock::to_time_t(chrono::system_clock::now());
    logger << data.imgcount << ";"
           << data.RS_id << ";"
           << std::put_time(std::localtime(&time_now), "%Y/%m/%d %T") << ";"
           << data.time << ";"
           << cam.measured_exposure() << ";";

    trackers.update(data.time,dctrl.drone_is_active());
    if (log_replay_mode) {
        dctrl.insert_log(logreader.current_entry.joyRoll, logreader.current_entry.joyPitch, logreader.current_entry.joyYaw, logreader.current_entry.joyThrottle,logreader.current_entry.joyArmSwitch,logreader.current_entry.joyModeSwitch,logreader.current_entry.joyTakeoffSwitch,logreader.current_entry.auto_roll,logreader.current_entry.auto_pitch,logreader.current_entry.auto_throttle);
    }
    dnav.update(data.time);
    dctrl.control(trackers.dronetracker()->Last_track_data(),dnav.setpoint(),trackers.insecttracker_best()->Last_track_data(),data.time);
    dprdct.update(dctrl.drone_is_active(),data.time);

    trackers.dronetracker()->_manual_flight_mode =dnav.drone_is_manual(); // TODO: hacky

    logger << std::endl;

    if (pparams.has_screen) {
        visualizer.add_plot_sample();
        visualizer.update_tracker_data(visdat.frameL,dnav.setpoint().pos(),data.time, draw_plots, trackers.insecttracker_best());
        if (pparams.video_result) {
            if (log_replay_mode)
                output_video_results.block(); // only use this for rendering
            output_video_results.write(visualizer.trackframe);
        }
    }

    write_status_file();
}

void init_insect_log(int n) {
    trackers.init_replay_moth(n);
}

void handle_key(double time [[maybe_unused]]) {
    if (key == 27) { // set by an external ctrl-c
        return;
    }
    key = cv::waitKey(1);
    key = key & 0xff;
    if (key == 27) {  //esc
        return; // don't clear key, just exit
    }

    switch(key) {
    case 'b':
        rc.bind(true);
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
        if(log_replay_mode)
            draw_plots = true;
        break;
    case 'o':
        dctrl.blink_by_binding(false);
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
    case 82: // arrow up
        dnav.manual_trigger_next_wp();
        break;
    case 84: // arrow down
        dnav.manual_trigger_prev_wp();
        break;
#if CAMMODE == CAMMODE_REALSENSE
    case ' ':
    case 'f':
        if(log_replay_mode)
            cam.frame_by_frame = true;
        break;
    case 't':
        cam.turbo = !cam.turbo;
        break;
    case '>': {
        double dt = logreader.first_takeoff_time() - time - 1;
        if (dt>0 && trackers.mode() != tracking::TrackerManager::mode_locate_drone)
            cam.skip(dt);
        break;
    }
    case '.':
        cam.skip_one_sec();
        break;
    case ',':
        cam.back_one_sec();
        break;
#endif
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
    key=0;
}

//This is where frames get processed after it was received from the cam in the main thread
void pool_worker(int id ) {
    std::unique_lock<std::mutex> lk(tp[id].m1,std::defer_lock);
    while(key!=27) {
        tp[id].new_data.wait(lk,[]() {return tp[0].data_is_new;});
        tp[0].data_is_new = false;
        if (key == 27)
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
        }
        std::cout <<"Threads in pool closed"<< std::endl;
        threads_initialised = false;
    }
}

void kill_sig_handler(int s) {
    std::cout << "Caught ctrl-c:" << s << std::endl;
    key=27;
    close(true);
    exit(0);
}
void init_terminal_signals() {
    //init ctrl - c catch
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = kill_sig_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    struct sigaction sigTermHandler;
    sigTermHandler.sa_handler = kill_sig_handler;
    sigemptyset(&sigTermHandler.sa_mask);
    sigTermHandler.sa_flags = 0;
    sigaction(SIGTERM, &sigTermHandler, NULL);
}

void init_loggers() {
    data_output_dir = "./logging/";
    mkdir(data_output_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (log_replay_mode)
        data_output_dir = data_output_dir + "replay/";
    mkdir(data_output_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    logger.open(data_output_dir  + "log.csv",std::ofstream::out);
    cout << "data_output_dir: " << data_output_dir << endl;

    logger << "ID;RS_ID;time;elapsed;Exposure;";
    logger_fn = data_output_dir  + "log" + to_string(0) + ".csv"; // only used with pparams.video_cuts

    logger_insect.open(data_output_dir  + "insect.log",std::ofstream::out);
}

void init_video_recorders() {
    /*****init the video writer*****/
    if (pparams.video_result)
        if (output_video_results.init(main_argc,main_argv,pparams.video_result, data_output_dir + "videoResult.mp4",visualizer.viz_frame_size().width,visualizer.viz_frame_size().height,pparams.fps,"192.168.1.255",5000,true,true)) {throw my_exit("could not open results video");}
    if (pparams.video_raw && pparams.video_raw != video_bag)
        if (output_video_LR.init(main_argc,main_argv,pparams.video_raw,data_output_dir + "videoRawLR.mp4",IMG_W*2,IMG_H,pparams.fps, "192.168.1.255",5000,false,false)) {throw my_exit("could not open LR video");}
    if (pparams.video_cuts)
        if (output_video_cuts.init(main_argc,main_argv,pparams.video_cuts,data_output_dir + "insect" + to_string(0) + ".mp4",IMG_W*2,IMG_H,pparams.fps/2, "192.168.1.255",5000,true,false)) {std::cout << "WARNING: could not open cut video " << data_output_dir + "insect" + to_string(0) + ".mp4" << std::endl;}
}

std::tuple<bool,bool, std::string,uint8_t> process_arg(int argc, char **argv) {
    main_argc = argc;
    main_argv = argv;

    std::string empty_str = "";
    int default_drone_id = 1;

    if (argc == 2) {
        string s = argv[1];
        if (s.compare("rs_reset") == 0) {
            return make_tuple(false,true,empty_str,default_drone_id);
        } else {
            for(uint i=0; i<s.length(); i++) {
                if(!isdigit(s[i]))
                    return make_tuple(true,false,s,default_drone_id);
            }
            return make_tuple(false,false,empty_str,std::stoi(s));
        }
    } else {
        return make_tuple(false,false,empty_str,default_drone_id);
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

#if CAMMODE == CAMMODE_REALSENSE
    if (!pparams.insect_logging_mode && !log_replay_mode)
        rc.init(drone_id, log_replay_mode);
#endif

    /*****Start capturing images*****/
    if (log_replay_mode)
        cam.init(replay_dir);
    else
        cam.init();

    visdat.init(cam.Qf, cam.frameL,cam.frameR,cam.camera_angle(),cam.measured_gain(),cam.depth_background_mm); // do after cam update to populate frames
    trackers.init(&logger, &visdat, &(cam.camera_volume));
    dnav.init(&logger,&trackers,&dctrl,&visdat, &(cam.camera_volume),replay_dir);
    dctrl.init(&logger,log_replay_mode,&rc,trackers.dronetracker(), &(cam.camera_volume));
    dprdct.init(&visdat,trackers.dronetracker(),trackers.insecttracker_best(),&dctrl);

    // Ensure that joystick was found and that we can use it
    if (!dctrl.joystick_ready() && !log_replay_mode && pparams.joystick != rc_none) {
        throw my_exit("no joystick connected.");
    }

    logger << std::endl; // this concludes the header log line
    if (pparams.has_screen) {
        visualizer.init(&visdat,&trackers,&dctrl,&dnav,&rc,log_replay_mode,&dprdct);
        visualizer_3d.init(&trackers, &(cam.camera_volume), &dctrl, &dnav);
        if (log_replay_mode)
            visualizer.first_take_off_time = logreader.first_takeoff_time();
    }

    init_video_recorders();

    init_thread_pool();

    std::cout << "Main init successfull" << std::endl;
}

void close(bool sig_kill) {
    std::cout <<"Closing"<< std::endl;

    cam.stop_watchdog();

    if (pparams.has_screen)
        cv::destroyAllWindows();
#ifdef HASGUI
    gui.close();
#endif

    /*****Close everything down*****/
    cam.close(); // attempt to save the video first

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
    if (pparams.video_raw && pparams.video_raw != video_bag)
        output_video_LR.close();
    if (pparams.video_cuts)
        output_video_cuts.close();

    logger.close();
    if (!sig_kill) // seems to get stuck. TODO: streamline
        close_thread_pool();

    std::cout <<"Closed"<< std::endl;
}


void wait_for_cam_angle() {

    int enable_delay = 0;

    if (pparams.darkness_threshold > 0 && !log_replay_mode) {
        std::cout << "Checking cam angle." << std::endl;
        while(true) {
            auto [roll,pitch,frame_time,frameL] = cam.measure_angle();
            static double roll_start_time = frame_time;
            double elapsed_time = (frame_time- roll_start_time) / 1000.;

            auto t = chrono::system_clock::to_time_t(chrono::system_clock::now());
            std::cout << std::put_time(std::localtime(&t), "%Y/%m/%d %T") << ". Camera roll: " << to_string_with_precision(roll,2) << "°- max: " << pparams.max_cam_roll << "°. Pitch: " << to_string_with_precision(pitch,2) << "°" << std::endl;
            if (fabs(roll)  < pparams.max_cam_roll && !enable_delay)
                break;

            if (fabs(roll)  > pparams.max_cam_roll)
                enable_delay = 60;
            else
                enable_delay--;

            static double prev_imwrite_time = -pparams.live_image_frq;
            if (elapsed_time - prev_imwrite_time > pparams.live_image_frq && pparams.live_image_frq >= 0) {
                prev_imwrite_time = elapsed_time;
                cv::imwrite("../../../../pats_monitor_tmp.jpg", frameL);
            }
            reset_status_file("Roll: " + to_string_with_precision(roll,2));
            usleep(30000); // measure every third second
        }
    }
}

void wait_for_dark() {
    if (pparams.darkness_threshold > 0 && !log_replay_mode) {
        std::cout << "Checking if dark." << std::endl;
        while(true) {
            auto [expo,frameL] = cam.measure_auto_exposure();
            auto t = chrono::system_clock::to_time_t(chrono::system_clock::now());
            std::cout << std::put_time(std::localtime(&t), "%Y/%m/%d %T") << " Measured exposure: " << expo << std::endl;
            if (expo >pparams.darkness_threshold) {
                break;
            }
            cv::imwrite("../../../../pats_monitor_tmp.jpg", frameL);
            reset_status_file("Waiting. Exposure: " + std::to_string(static_cast<int>(expo)));
            usleep(10000000); // measure every 1 minute
        }
    }
}

int main( int argc, char **argv )
{
    reset_status_file("Starting");
    bool realsense_reset;
    try {
        std::tie(log_replay_mode, realsense_reset,replay_dir,drone_id) = process_arg(argc,argv);
    } catch (Exception err) {
        std::cout << "Error processing command line arguments:" << err.msg << std::endl;
        return 1;
    }

    if (realsense_reset) {
        reset_status_file("Reseting realsense");
        try {
            cam.reset();
        } catch(my_exit const &e) {
            std::cout << "Error: " << e.msg << std::endl;
            reset_status_file(e.msg);
            return 1;
        }
        return 0;
    }

    try {
        if (replay_dir == "") {
            pparams.deserialize("../../xml/pats.xml");
            dparams.deserialize("../../xml/" + string(drone_types_str[pparams.drone]) + ".xml");
            mkdir("./logging", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
            pparams.serialize("./logging/pats.xml");
            dparams.serialize("./logging/drone.xml");
        } else {
            pparams.deserialize(replay_dir + "/pats.xml");
            dparams.deserialize(replay_dir + "/drone.xml");
        }
    } catch(my_exit const &e) {
        std::cout << "Error reading xml settings: " << e.msg << std::endl;
        reset_status_file(e.msg);
        return 1;
    }

    try {
        wait_for_cam_angle();
        wait_for_dark();
        init();
        process_video();
    } catch(bag_video_ended) {
        std::cout << "Video ended" << std::endl;
        key = 27; //secret signal to close everything (it's the esc key)
    } catch(my_exit const &e) {
        key = 27;
        close(false);
        std::cout << "Error: " << e.msg << std::endl;
        reset_status_file(e.msg);
        return 1;
    }

    close(false);
    return 0;
}
