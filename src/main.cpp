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
#include "trackermanager.h"
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
GStream output_video_results,output_video_LR,output_video_cuts;

int main_argc;
char **main_argv;
xmls::PatsParameters pparams;
xmls::DroneParameters dparams;

stopwatch_c stopWatch_break;
stopwatch_c stopWatch;
std::string file;
std::string data_output_dir;
std::string calib_folder;

bool draw_plots = false;

std::string logger_fn; //contains filename of current log # for insect logging (same as video #)

int mouseX, mouseY;
int mouseLDown;

std::ofstream logger;
std::ofstream logger_insect;
//Arduino rc;
MultiModule rc;
DroneController dctrl;
DronePredictor dprdct;
DroneNavigation dnav;
TrackerManager trackers;
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
void process_frame(Stereo_Frame_Data data_drone);
void process_video();
int main( int argc, char **argv);
void handle_key(double time);
void close(bool sigkill);

void write_occasional_image();
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
        if (pparams.has_screen) {
            static int speed_div;
            if (!(speed_div++ % 4) || fromfile!=log_mode_none){
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
        double dtr = data.time - trackers.insecttracker()->last_sighting_time;
        if (dtr > 1 && recording) {
            recording = false;
            auto time_insect_now = chrono::system_clock::to_time_t(chrono::system_clock::now());
            logger_insect << "New detection ended at: " << std::put_time(std::localtime(&time_insect_now), "%Y/%m/%d %T") << " Duration: " << dtr << " End cam frame number: " <<  cam.frame_number() << std::endl;
            detectcount++;
        }
        if ((trackers.insecttracker()->tracking() || dtr < 1) && data.time > 5) {
            recording = true;
        }

        if (pparams.video_cuts) {
            static bool was_recording = false;
            static int insect_cnt = 0;
            if (cam.frame_number() / 2 && cam.frame_number() % 2) {
                if (recording != was_recording) {
                    if (recording)
                        insect_cnt++;
                    else if (!recording && insect_cnt>0){
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

                if (recording){
                    static int cut_video_frame_counter = 0;

                    cv::Mat frame(cam.frameL.rows,cam.frameL.cols+trackers.diff_viz.cols,CV_8UC3);
                    cvtColor(cam.frameL,frame(cv::Rect(0,0,cam.frameL.cols, cam.frameL.rows)),CV_GRAY2BGR);
                    trackers.diff_viz.copyTo(frame(cv::Rect(cam.frameL.cols,0,trackers.diff_viz.cols, trackers.diff_viz.rows)));
                    cv::putText(frame,std::to_string(cam.frame_number()) + ": " + to_string_with_precision( cam.frame_time(),2),cv::Point(trackers.diff_viz.cols, 13),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,0,255));
                    int frameWritten = output_video_cuts.write(frame);
                    if (!frameWritten)
                        cut_video_frame_counter++;
                    std::cout << "Recording! Frames written: " << cut_video_frame_counter << std::endl;
                }
            }
        }
        if (!recording && pparams.video_cuts){
            logger.close();
            logger.open(logger_fn,std::ofstream::out);
        }
        if (pparams.video_raw && pparams.video_raw != video_bag) {
            int frameWritten = 0;
            if (recording) {
                cv::Mat frameL  =cam.frameL.clone();
                cv::putText(frameL,std::to_string(cam.frame_number()),cv::Point(0, 13),cv::FONT_HERSHEY_SIMPLEX,0.5,255);
                frameWritten = output_video_LR.write(frameL,cam.frameR);
                static int video_frame_counter = 0;
                if (!frameWritten)
                    video_frame_counter++;
                else {
                    std::cout << "PROBLEM: " << frameWritten << std::endl;
                }
                std::cout << "Recording! Frames written: " << video_frame_counter << std::endl;
            }
        }

        //keep track of time and fps
        float t = stopWatch.Read() / 1000.f;
        static float prev_time = -1.f/pparams.fps;
        float current_fps = 1.f / (t - prev_time);
        float fps = fps_smoothed.addSample(current_fps);
        if (fps < pparams.fps / 6 * 5 && fromfile==log_mode_none)
            std::cout << "FPS WARNING!" << std::endl;

        static double time =0;
        float dt __attribute__((unused)) = static_cast<float>(cam.frame_time() - time);
        time = cam.frame_time();
        std::cout << "Frame: " <<imgcount << ", " << cam.frame_number() << ". FPS: " << to_string_with_precision(fps,1) << ". Time: " << to_string_with_precision(time,2)  << ", dt " << to_string_with_precision(dt,3) << std::endl;
        imgcount++;
        prev_time = t;

        if (fps != fps || isinf(fps))
            fps_smoothed.reset();

        if (imgcount > 360000 && pparams.insect_logging_mode)
            key =27;

    } // main while loop
}


void write_occasional_image(Stereo_Frame_Data data) {
    static double prev_imwrite_time = -20;

    if (data.time - prev_imwrite_time > 20) {
        cv::Mat out = data.frameL.clone();
        cvtColor(out,out,CV_GRAY2BGR);
        putText(out,"State: " + dnav.navigation_status() + " " + trackers.mode_str() + " " + dctrl.flight_mode() +
                         " "  + trackers.dronetracker()->drone_tracking_state(),cv::Point(5,14),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));
        putText(out,"Time:       " + to_string_with_precision(data.time,2),cv::Point(5,28),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));
        putText(out,"Detections: " + std::to_string(detectcount),cv::Point(5,42),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));

        cv::imwrite("monitor.png", out);
        prev_imwrite_time = data.time;
    }

}

void process_frame(Stereo_Frame_Data data) {

    if (fromfile==log_mode_full){
        logreader.current_frame_number(data.number);
        if (logreader.current_item.insect_log) {
            trackers.mode(TrackerManager::mode_hunt_replay_moth);
        }
    } else if (fromfile==log_mode_insect_only) {
        if (logreader.set_next_frame_number()){
            trackers.mode(TrackerManager::mode_hunt);
            fromfile = log_mode_none;
        } else {
            trackers.mode(TrackerManager::mode_hunt_replay_moth);
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
    dctrl.control(trackers.dronetracker()->Last_track_data(),trackers.insecttracker()->Last_track_data(), dnav.setpoint_pos_world,dnav.setpoint_vel_world,dnav.setpoint_acc_world,data.time);
    dprdct.update(dctrl.drone_is_active(),data.time);

    trackers.dronetracker()->_manual_flight_mode =dnav.drone_is_manual(); // TODO: hacky

    logger << std::endl;

    if (pparams.has_screen) {
        visualizer.addPlotSample();
        visualizer.update_tracker_data(visdat.frameL,dnav.setpoint,data.time, draw_plots);
        if (pparams.video_result) {
            if (fromfile)
                output_video_results.block(); // only use this for rendering
            output_video_results.write(visualizer.trackframe);
        }
    }
    if (!dctrl.drone_is_active()) {
        write_occasional_image(data);
    }

}

void init_insect_log(int n){
    logreader.init("../insect_logs/" + std::to_string(n) + ".csv",true);
    fromfile = log_mode_insect_only;
}

void handle_key(double time) {
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
    case 'p':
        if(fromfile) {
            draw_plots = true;
        }
        break;
    case 'o':
        dctrl.blink_by_binding(false);
        dctrl.LED(true);
        dnav.nav_flight_mode(DroneNavigation::nfm_manual);
        break;
    case '1':
        init_insect_log(55);
        break;
    case '2':
        init_insect_log(53);
        break;
    case '3':
        init_insect_log(46);
        break;
    case '4':
        init_insect_log(54);
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
        if(fromfile)
            cam.frame_by_frame = true;
        break;
    case 't':
        cam.turbo = !cam.turbo;
        break;
    case '>': {
        double dt = logreader.first_takeoff_time() - time - 1;
        if (dt>0)
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
    close(true);
    exit(0);
}
void init_sig(){
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

int init_loggers() {
    std::string data_in_dir = "";
    int drone_id = 1;
    if (main_argc ==2 ) {
        string fn = string(main_argv[1]);
        drone_id = get_drone_id (fn);
        if(drone_id<0){
            logreader.init(fn + "/log.csv",false);
            fromfile = log_mode_full;
            data_in_dir = fn;
        }
    }
    data_output_dir = "./logging/";
    mkdir("./logging/", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    cout << "data_output_dir: " << data_output_dir << endl;
    if (fromfile==log_mode_full)
        logger.open(data_output_dir  + "log_regenerated.csv",std::ofstream::out);
    else {
        logger.open(data_output_dir  + "log.csv",std::ofstream::out);
    }

    logger << "ID;RS_ID;time;insect_log;";
    logger_fn = data_output_dir  + "log" + to_string(0) + ".csv"; // only used with pparams.video_cuts

    logger_insect.open(data_output_dir  + "insect.log",std::ofstream::out);

    return drone_id;
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

void init(int argc, char **argv) {

    main_argc = argc;
    main_argv = argv;

    init_sig();
    int drone_id = init_loggers();

#ifdef HASGUI
    gui.init(argc,argv);
#endif

#if CAMMODE == CAMMODE_REALSENSE
    if (!pparams.insect_logging_mode && fromfile!=log_mode_full)
        rc.init(drone_id, fromfile);
#endif

    /*****Start capturing images*****/
    if (fromfile == log_mode_full)
        cam.init(argc,argv);
    else
        cam.init();

    visdat.init(fromfile==log_mode_full,cam.Qf, cam.frameL,cam.frameR,cam.camera_angle(),cam.measured_gain(),cam.depth_background_mm); // do after cam update to populate frames
    trackers.init(&logger, &visdat);
    dnav.init(&logger,&trackers,&dctrl,&visdat, &(cam.camera_volume));
    dctrl.init(&logger,fromfile==log_mode_full,&rc,trackers.dronetracker(), &(cam.camera_volume));
    dprdct.init(&visdat,trackers.dronetracker(),trackers.insecttracker(),&dctrl);

    // Ensure that joystick was found and that we can use it
    if (!dctrl.joystick_ready() && fromfile!=log_mode_full && pparams.joystick != rc_none) {
        throw my_exit("no joystick connected.");
    }

    logger << std::endl; // this concludes the header log line
    if (pparams.has_screen) {
        visualizer.init(&visdat,&trackers,&dctrl,&dnav,&rc,fromfile==log_mode_full,&dprdct);
        if (fromfile==log_mode_full)
            visualizer.first_take_off_time = logreader.first_takeoff_time();
    }

    init_video_recorders();

    init_thread_pool();

    std::cout << "Main init successfull" << std::endl;
}

void close(bool sigkill) {
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
    if (fromfile!=log_mode_full)
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
    if (!sigkill) // seems to get stuck. TODO: streamline
        close_thread_pool();

    std::cout <<"Closed"<< std::endl;
}

void wait_for_dark() {
    std::cout << "Insect logging mode, check if dark." << std::endl;
    while(true) {
        float expo = cam.measure_auto_exposure();
        auto t = chrono::system_clock::to_time_t(chrono::system_clock::now());
        std::cout << std::put_time(std::localtime(&t), "%Y/%m/%d %T") << " Measured exposure: " << expo << std::endl;
        if (expo >pparams.darkness_threshold) {
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

    try {
        pparams.deserialize();
        dparams.deserialize("../../xml/" + string(drone_type_str[pparams.drone]) + ".xml");
    } catch(my_exit const &e) {
        std::cout << "Error: " << e.msg << std::endl;
        return 1;
    }

    if (pparams.insect_logging_mode)
        wait_for_dark();

    try {
        init(argc,argv);
        process_video();
    } catch(bag_video_ended) {
        std::cout << "Video ended" << std::endl;
        key = 27; //secret signal to close everything (it's the esc key)
    } catch(my_exit const &e) {
        key = 27;
        close(false);
        std::cout << "Error: " << e.msg << std::endl;
        return 1;
    }

    close(false);
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
