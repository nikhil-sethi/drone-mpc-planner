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

#include "common.h"
#include "defines.h"
#include "smoother.h"

#include "multimodule.h"

#include <thread>
#include <mutex>
#include <condition_variable>

#include "stopwatch.h"
#include "dronetracker.h"
#include "dronecontroller.h"
#include "dronepredictor.h"
#include "gstream.h"
#include "vizs.h"
#include "insecttracker.h"
#include "itemmanager.h"
#include "logreader.h"
#if CAMMODE == CAMMODE_FROMVIDEOFILE
#include "filecam.h"
#elif CAMMODE == CAMMODE_AIRSIM
#include "airsim.h"
#elif CAMMODE == CAMMODE_REALSENSE
#include "cam.h"
#elif CAMMODE == CAMMODE_GENERATOR
#include "generatorcam.h"
#endif
#include "visiondata.h"

#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#ifdef HASGUI
#include "gui/mainwindow.h"
#endif

using namespace cv;
using namespace std;

/***********Variables****************/
unsigned char key = 0;
int imgcount,detectcount; // to measure fps
GStream output_video_results,output_video_LR;

stopwatch_c stopWatch_break;
stopwatch_c stopWatch;
std::string file;
std::string data_output_dir;
std::string calib_folder;

int mouseX, mouseY;
int mouseLDown;

std::ofstream logger;
std::ofstream logger_insect;
//Arduino rc;
MultiModule rc;
DroneController dctrl;
DronePredictor dprdct;
DroneNavigation dnav;
ItemManager trackers;
Visualizer visualizer;
LogReader logreader;
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
enum log_mode{
    log_mode_none,
    log_mode_full,
    log_mode_insect_only
};
log_mode fromfile = log_mode_none;

/****Threadpool*******/
#define NUM_OF_THREADS 1
struct Stereo_Frame_Data{
    cv::Mat frameL,frameR;
    uint imgcount;
    unsigned long long number;
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
void process_frame(Stereo_Frame_Data data);
void process_video();
int main( int argc, char **argv);
void handle_key();
void close();

void manual_drone_locater(cv::Mat frame);
void CallBackFunc(int event, int x, int y, int flags, void* userdata);

/************ code ***********/
void process_video() {

    Smoother fps_smoothed;
    fps_smoothed.init(100);
    stopWatch.Start();

    //main while loop:
    while (key != 27) // ESC
    {
        cam.update();

        Stereo_Frame_Data data;
        data.frameL = cam.frameL;
        data.frameR = cam.frameR;
        data.number = cam.frame_number();
        data.time = cam.frame_time();
        data.imgcount = imgcount;

        std::unique_lock<std::mutex> lk(tp[0].m2,std::defer_lock);
        tp[0].data_processed.wait(lk, [](){return tp[0].data_is_processed; });
        tp[0].data_is_processed= false;
#ifdef HASSCREEN
        static int speed_div;
        if (!(speed_div++ % 4) || fromfile!=log_mode_none){
            visualizer.paint();
            handle_key();
        }
#endif
        tp[0].m1.lock();
        tp[0].data = data;
        tp[0].data_is_new = true;
        tp[0].new_data.notify_one();
        tp[0].m1.unlock();

        static bool new_recording = false;
        double dtr = data.time - trackers.insecttracker()->last_sighting_time;
        if (dtr > 1 && new_recording) {
            new_recording = false;
            auto time_insect_now = chrono::system_clock::to_time_t(chrono::system_clock::now());
            logger_insect << "New detection ended at: " << std::put_time(std::localtime(&time_insect_now), "%Y/%m/%d %T") << " Duration: " << dtr << " End cam frame number: " <<  cam.frame_number() << std::endl;
        }

#if VIDEORAWLR && VIDEORAWLR != VIDEOMODE_BAG
        int frameWritten = 0;
        if ((trackers.insecttracker()->tracking() || dtr < 1) && data.time > 5) {
            cv::Mat frameL  =cam.frameL.clone();
            cv::putText(frameL,std::to_string(cam.frame_number()),cv::Point(0, 13),cv::FONT_HERSHEY_SIMPLEX,0.5,255);
            frameWritten = output_video_LR.write(frameL,cam.frameR);
            static int video_frame_counter = 0;
            if (!frameWritten)
                video_frame_counter++;
            else {
                std::cout << "PROBLEM: " << frameWritten << std::endl;
            }
            new_recording = true;
            std::cout << "Recording! Frames written: " << video_frame_counter << std::endl;
        }
#endif

        //keep track of time and fps
        float t = stopWatch.Read() / 1000.f;
        static float prev_time = -1.f/VIDEOFPS;
        float current_fps = 1.f / (t - prev_time);
        float fps = fps_smoothed.addSample(current_fps);
        if (fps < 50 && fromfile!=log_mode_none) {
            //            std::cout << "FPS WARNING!" << std::endl;
            static float limit_fps_warning_sound = t;
            if (t - limit_fps_warning_sound > 3.f ) {
                limit_fps_warning_sound = t;
            }
        }

        static double time =0;
        float dt __attribute__((unused)) = static_cast<float>(cam.frame_time() - time);
        time = cam.frame_time();
        std::cout << "Frame: " <<imgcount << ", " << cam.frame_number() << ". FPS: " << to_string_with_precision(imgcount / time,1) << ". Time: " << to_string_with_precision(time,2)  << ", dt " << to_string_with_precision(dt,3) << " FPS stopwatch smooth: " << to_string_with_precision(fps,1) << " current: " << to_string_with_precision(current_fps,1) << std::endl;
        imgcount++;
        prev_time = t;

        if (fps != fps || isinf(fps))
            fps_smoothed.reset();

#ifdef INSECT_LOGGING_MODE
        if (imgcount > 360000)
            key =27;
#endif

    } // main while loop
}

void process_frame(Stereo_Frame_Data data) {

    if (fromfile==log_mode_full){
        logreader.current_frame_number(data.number);
        if (logreader.current_item.insect_log) {
            trackers.mode(ItemManager::mode_hunt_replay_moth);
        }
    } else if (fromfile==log_mode_insect_only) {
        if (logreader.set_next_frame_number()){
            trackers.mode(ItemManager::mode_hunt);
            fromfile = log_mode_none;
        } else {
            trackers.mode(ItemManager::mode_hunt_replay_moth);
        }
    }

    visdat.update(data.frameL,data.frameR,data.time,data.number);

    auto timenow = chrono::system_clock::to_time_t(chrono::system_clock::now());
    logger << data.imgcount << ";"
           << data.number << ";"
           << std::put_time(std::localtime(&timenow), "%Y/%m/%d %T") << ";"
           << (fromfile==log_mode_insect_only) << ";";


    trackers.update(data.time,logreader.current_item,dctrl.drone_is_active());
    if (fromfile==log_mode_full) {
        dctrl.insert_log(logreader.current_item.joyRoll, logreader.current_item.joyPitch, logreader.current_item.joyYaw, logreader.current_item.joyThrottle,logreader.current_item.joyArmSwitch,logreader.current_item.joyModeSwitch,logreader.current_item.joyTakeoffSwitch,logreader.current_item.auto_roll,logreader.current_item.auto_pitch,logreader.current_item.auto_throttle);
    }
    dnav.update(data.time);
    dctrl.control(trackers.dronetracker()->Last_track_data(),dnav.setpoint_pos_world,dnav.setpoint_vel_world,dnav.setpoint_acc_world);
    dprdct.update(dctrl.drone_is_active(),data.time);

    trackers.dronetracker()-> _manual_flight_mode =dnav.drone_is_manual(); // TODO: hacky

    logger << std::endl;

#ifdef HASSCREEN
    visualizer.addPlotSample();
    visualizer.update_tracker_data(visdat.frameL,dnav.setpoint,data.time);
#if VIDEORESULTS
    output_video_results.block(); // only use this for rendering
    output_video_results.write(visualizer.trackframe);
#endif
#endif

}

void init_insect_log(int n){
    logreader.init("../insect_logs/" + std::to_string(n) + ".log",true);
    fromfile = log_mode_insect_only;
}

void handle_key() {
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
    case 'l':
        dnav.redetect_drone_location();
        break;
    case 'o':
        dctrl.blink_by_binding(false);
        dctrl.LED(true);
        dnav.nav_flight_mode(DroneNavigation::nfm_manual);
        break;
    case '1':
        init_insect_log(49);
        break;
    case '2':
        init_insect_log(53);
        break;
    case '3':
        init_insect_log(46);
        break;
    case '4':
        init_insect_log(1);
        break;
    case '5':
        init_insect_log(36);
        break;
    case '6':
        init_insect_log(26);
        break;
    case '7':
        init_insect_log(20);
        break;
    case '8':
        init_insect_log(38);
        break;
    case '9':
        init_insect_log(15);
        break;
    case '0':
        init_insect_log(18);
        break;

#if CAMMODE == CAMMODE_REALSENSE
    case ' ':
    case 'f':
        cam.frame_by_frame = true;
        break;
    case 't':
        cam.turbo = !cam.turbo;
        break;
    case '.':
        cam.skip_one_sec();
        break;
    case ',':
        cam.back_one_sec();
        break;
#endif
    case 'a':
        rc.arm(true);
        dnav.nav_flight_mode(DroneNavigation::nfm_waypoint);
        break;
    case 'h':
        rc.arm(true);
        dnav.nav_flight_mode(DroneNavigation::nfm_hunt);
        break;
    case 'd':
        rc.arm(false);
        dnav.nav_flight_mode(DroneNavigation::nfm_manual);
        break;
    } // end switch key
    key=0;
}

//This is where frames get processed after it was received from the cam in the main thread
void pool_worker(int id ){
    std::unique_lock<std::mutex> lk(tp[id].m1,std::defer_lock);
    while(key!=27) {
        tp[id].new_data.wait(lk,[](){return tp[0].data_is_new;});
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
void close_thread_pool(){
    if (threads_initialised){
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

void kill_sig_handler(int s){
    std::cout << "Caught ctrl-c:" << s << std::endl;
    key=27;
}
void init_sig(){
    //init ctrl - c catch
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = kill_sig_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
}

void init_loggers(int argc, char **argv) {
    std::string data_in_dir = "";
    if (argc ==2 ) {
        string fn = string(argv[1]);
        logreader.init(fn + "/test.log",false);
        fromfile = log_mode_full;
        data_in_dir = fn;
    }
    data_output_dir = "./logging/";
    mkdir("./logging/", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    cout << "data_output_dir: " << data_output_dir << endl;
    if (fromfile==log_mode_full)
        logger.open(data_output_dir  + "test_fromfile.log",std::ofstream::out);
    else {
        logger.open(data_output_dir  + "test.log",std::ofstream::out);
    }

    logger << "ID;RS_ID;time;insect_log;";

    logger_insect.open(data_output_dir  + "insect.log",std::ofstream::out);
}

void init_video_recorders(int argc __attribute__((unused)), char **argv __attribute__((unused))) {
    /*****init the video writer*****/
#if VIDEORESULTS
    if (output_video_results.init(argc,argv,VIDEORESULTS, data_output_dir + "videoResult.mp4",visualizer.viz_frame_size().width,visualizer.viz_frame_size().height,VIDEOFPS,"192.168.1.255",5000,true)) {throw my_exit("could not open results video");}
#endif
#if VIDEORAWLR && VIDEORAWLR != VIDEOMODE_BAG
    if (output_video_LR.init(argc,argv,VIDEORAWLR,data_output_dir + "videoRawLR.avi",IMG_W*2,IMG_H,VIDEOFPS, "192.168.1.255",5000,false)) {throw my_exit("could not open LR video");}
#endif
}

void init(int argc, char **argv) {

    init_loggers(argc,argv);

#ifdef HASGUI
    gui.init(argc,argv);
#endif

#ifndef INSECT_LOGGING_MODE
#if CAMMODE == CAMMODE_REALSENSE
    if (fromfile!=log_mode_full)
        rc.init(fromfile);
#endif
#endif

    /*****Start capturing images*****/
    if (fromfile == log_mode_full)
        cam.init(argc,argv);
    else
        cam.init();

    visdat.init(fromfile==log_mode_full,cam.Qf, cam.frameL,cam.frameR,cam.camera_angle(),cam.measured_gain(),cam.depth_background_mm); // do after cam update to populate frames
    trackers.init(&logger, &visdat);
    dnav.init(&logger,&trackers,&dctrl,&visdat);
    dctrl.init(&logger,fromfile==log_mode_full,&rc,trackers.dronetracker());
    dprdct.init(&visdat,trackers.dronetracker(),trackers.insecttracker(),&dctrl);

#ifdef MANUAL_DRONE_LOCATE
    if (!fromfile){
        manual_drone_locater(cam.frameL);
        dtrkr.set_drone_location(cv::Point(mouseX,mouseY));
    }
#endif

    // Ensure that joystick was found and that we can use it
    if (!dctrl.joystick_ready() && fromfile!=log_mode_full && JOYSTICK_TYPE != RC_NONE) {
        throw my_exit("no joystick connected.");
    }

    logger << std::endl; // this concludes the header log line
#ifdef HASSCREEN
    visualizer.init(&visdat,&trackers,&dctrl,&dnav,&rc,fromfile==log_mode_full,&dprdct);
    if (fromfile==log_mode_full)
        visualizer.first_take_off_time = logreader.first_takeoff_time();
#endif

    init_video_recorders(argc,argv);

    init_thread_pool();

    std::cout << "Main init successfull" << std::endl;
}

void close() {
    std::cout <<"Closing"<< std::endl;

#ifdef HASSCREEN
    cv::destroyAllWindows();
#endif
#ifdef HASGUI
    gui.close();
#endif

    /*****Close everything down*****/
    dctrl.close();
    dnav.close();
    trackers.close();
    if (fromfile!=log_mode_full)
        rc.close();
#ifdef HASSCREEN
    visualizer.close();
#endif
    visdat.close();

#if VIDEORESULTS
    output_video_results.close();
#endif
#if VIDEORAWLR && VIDEORAWLR != VIDEOMODE_BAG
    output_video_LR.close();
#endif

    std::terminate(); // TMP because of deadlock bug RS
    cam.close();
    close_thread_pool();

    std::cout <<"Closed"<< std::endl;
}

void wait_for_dark() {
    std::cout << "Insect logging mode, check if dark." << std::endl;
    while(true) {
        float expo = cam.measure_auto_exposure();
        auto t = chrono::system_clock::to_time_t(chrono::system_clock::now());
        std::cout << std::put_time(std::localtime(&t), "%Y/%m/%d %T") << " Measured exposure: " << expo << std::endl;
        if (expo >10000) {
            break;
        }
        usleep(60000000); // measure every 1 minute
    }
}

int main( int argc, char **argv )
{
    //manually reset the rs camera from the command line with the rs_reset command
    if (argc == 2)
        if (string(argv[1]).compare("rs_reset") == 0){
            try{
                cam.reset();
            } catch(my_exit const &e) {
                std::cout << "Error: " << e.msg << std::endl;
                return 1;
            }
            return 0;
        }

#ifdef INSECT_LOGGING_MODE
    wait_for_dark();
#endif

    try {
        init(argc,argv);
        process_video();
    } catch(bag_video_ended) {
        std::cout << "Video ended" << std::endl;
        key = 27; //secret signal to close everything (it's the esc key)
    } catch(my_exit const &e) {
        key = 27;
        close();
        std::cout << "Error: " << e.msg << std::endl;
        return 1;
    }

    close();
    return 0;
}

void manual_drone_locater(cv::Mat frame){
    cv::namedWindow("Manual_drone_locator", CV_WINDOW_NORMAL);
    cv::setMouseCallback("Manual_drone_locator", CallBackFunc, NULL);

    while(1) {
        cv::Mat f2;
        cvtColor(frame,f2,CV_GRAY2BGR);
        cv::circle(f2,cv::Point(mouseX,mouseY),3,cv::Scalar(0,255,0),2);
        cv::imshow("Manual_drone_locator", f2);
        unsigned char k = cv::waitKey(20);
        if (k==' ')
            break;

    }
    cv::destroyWindow("Manual_drone_locator");
}

void CallBackFunc(int event, int x, int y , int flags __attribute__((unused)), void* userdata __attribute__((unused)))
{
    if  ( event == cv::EVENT_LBUTTONDOWN ) {
        mouseLDown=10;
        mouseX = x;
        mouseY = y;
    }
}
