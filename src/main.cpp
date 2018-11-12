#include <mutex>
#include <thread>
#include <fstream>      // std::ifstream
#include <sstream>
#include <iostream>
#include <iomanip>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <ctime>
#include <sys/stat.h>

#include "common.h"
#include "defines.h"
#include "smoother.h"

#include <thread>
#include <mutex>
#include <condition_variable>

#include "stopwatch.h"
#include "dronetracker.h"
#include "dronecontroller.h"
#include "gstream.h"
#include "vizs.h"
#include "insecttracker.h"
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
#if CV_MAJOR_VERSION==2
#include <opencv2/contrib/contrib.hpp>
#endif

using namespace cv;
using namespace std;

/***********Enums****************/


/***********Variables****************/
unsigned char key = 0;
int imgcount,detectcount; // to measure fps
GStream output_video_results,output_video_LR;
cv::VideoWriter output_video_disp;

stopwatch_c stopWatch_break;
stopwatch_c stopWatch;
std::string file;
std::string data_output_dir;
std::string calib_folder;

std::ofstream logger;
Arduino arduino;
DroneTracker dtrkr;
DroneController dctrl;
DroneNavigation dnav;
InsectTracker itrkr;
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
bool fromfile = false;

/****Threadpool*******/
#define NUM_OF_THREADS 1
struct Stereo_Frame_Data{
    cv::Mat frameL,frameR;
    uint imgcount;
    uint number;
    float time;
};
struct Processer {
    int id;
    std::thread * thread;
    std::mutex m1,m2;
    std::condition_variable data_processed,new_data;
    bool data_is_processed = true;
    bool data_is_send = false;
    Stereo_Frame_Data data;
};
Processer tp[NUM_OF_THREADS];

/*******Private prototypes*********/
void process_frame(Stereo_Frame_Data data);
void process_video();
int main( int argc, char **argv);
void handleKey();

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
        tp[0].data = data;
        tp[0].data_is_send = true;
        tp[0].data_is_processed= false;
        tp[0].new_data.notify_one();

        int frameWritten = 0;
#if VIDEORAWLR
        frameWritten = output_video_LR.write(cam.frameL,cam.frameR);
#endif
        if (frameWritten == 0) {
#if VIDEODISPARITY
            output_video_disp.write(cam.get_disp_frame());
#endif
#if VIDEORESULTS
            output_video_results.write(visualizer.trackframe);
#endif
        }

        //keep track of time and fps
        float t = stopWatch.Read() / 1000.f;
        static float prev_time = -1.f/VIDEOFPS;
        float fps = fps_smoothed.addSample( 1.f / (t - prev_time));
        if (fps < 50 && !fromfile) {
            std::cout << "FPS WARNING!" << std::endl;
            static float limit_fps_warning_sound = t;
            if (t - limit_fps_warning_sound > 3.f ) {
                alert("canberra-gtk-play -f /usr/share/sounds/ubuntu/stereo/phone-incoming-call.ogg &");
                limit_fps_warning_sound = t;
            }
        }

        static float time =0;
        float dt = cam.frame_time() - time;
        time = cam.frame_time();

        std::cout << "Frame: " <<imgcount << ", " << cam.frame_number() << ". FPS: " << to_string_with_precision(imgcount / time,1) << ". Time: " << to_string_with_precision(time,2)  << ", dt " << to_string_with_precision(dt,3) << " FPS now: " << to_string_with_precision(fps,1) << std::endl;
        imgcount++;
        prev_time = t;

#ifdef HASSCREEN
        static int speed_div;
        if (!(speed_div++ % 4) || fromfile){
            visualizer.paint();
            handleKey();
        }
#endif

    } // main while loop
}

void process_frame(Stereo_Frame_Data data) {

    visdat.update(data.frameL,data.frameR,data.time,data.number);
    logger << data.imgcount << ";" << cam.frame_number() << ";" ;
    //WARNING: changing the order of the functions with logging must be matched with the init functions!
    dtrkr.track(data.time,itrkr.predicted_pathL,dctrl.getDroneIsActive());
    itrkr.track(data.time,dtrkr.predicted_pathL,dctrl.getDroneIsActive());
    //        std::cout << "Found drone location:      [" << dtrkr.find_result.best_image_locationL.pt.x << "," << dtrkr.find_result.best_image_locationL.pt.y << "]" << std::endl;
    dnav.update();
    dctrl.control(dtrkr.get_last_track_data(),dnav.setpoint_world,dnav.setspeed_world);
    logger << std::endl;

#ifdef HASSCREEN

    if (fromfile) {
        int rs_id = cam.frame_number();
        LogReader::Log_Entry tmp  = logreader.getItem(rs_id);
        if (tmp.RS_ID == rs_id) {
            dctrl.joyRoll = tmp.joyRoll;
            dctrl.joyPitch = tmp.joyPitch;
            dctrl.joyYaw = tmp.joyYaw;
            dctrl.joyThrottle = tmp.joyThrottle;
            dctrl.joySwitch = tmp.joySwitch;
        }
    }

    visualizer.addPlotSample();
    visualizer.update_tracker_data(visdat.frameL,dnav.setpoint,cam.frame_time(),&dtrkr,&itrkr);

#endif
}

void handleKey() {
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
        arduino.bind();
        break;
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
    } // end switch key
    key=0;
}

void pool_worker(int id __attribute__((unused))){
    std::unique_lock<std::mutex> lk(tp[id].m1,std::defer_lock);
    while(true) {
        tp[id].new_data.wait(lk,[](){return tp[0].data_is_send;});
        if (key == 27)
            break;
        process_frame(tp->data);
        tp[0].data_is_send = false;
        tp[id].data_is_processed = true;
        tp[id].data_processed.notify_one();
    }
}
void init_thread_pool() {
    for (uint i = 0; i < NUM_OF_THREADS; i++) {
        tp[i].thread = new thread(&pool_worker,i);
    }
}
void close_thread_pool(){
    std::cout <<"Stopping threads in pool"<< std::endl;
    for (uint i = 0; i < NUM_OF_THREADS; i++) {
        tp[i].data_is_send = true;
        tp[i].new_data.notify_all();
        tp[i].data_is_processed = true;
        tp[i].data_processed.notify_all();
    }
    for (uint i = 0; i < NUM_OF_THREADS; i++) {
        tp[i].thread->join();
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

int init(int argc, char **argv) {
    if (argc ==2 ) {
        fromfile = true;
        logreader.init(string(argv[1]) + ".log");
    }
    data_output_dir = "./logging/";
    mkdir("./logging/", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    cout << "data_output_dir: " << data_output_dir << endl;
    if (fromfile)
        logger.open(data_output_dir  + "test_fromfile.log",std::ofstream::out);
    else
        logger.open(data_output_dir  + "test.log",std::ofstream::out);

    logger << "ID;RS_ID;";

#if CAMMODE == CAMMODE_REALSENSE
    if (!fromfile)
        arduino.init(fromfile);
#endif

    /*****Start capturing images*****/
    cam.init(argc,argv);
    cam.update(); // wait for first frames

    visdat.init(cam.Qf, cam.frameL,cam.frameR); // do after cam update to populate frames
    visdat.update(cam.frameL,cam.frameR,cam.frame_time(),cam.frame_number()); //TODO: necessary? If so, streamline

    //WARNING: changing the order of the inits with logging must be match with the process_video functions!
    dtrkr.init(&logger,&visdat);
    itrkr.init(&logger,&visdat);
    dnav.init(&logger,&dtrkr,&dctrl,&itrkr);
    dctrl.init(&logger,fromfile,&arduino);

#if CAMMODE == CAMMODE_REALSENSE
    // Ensure that joystick was found and that we can use it
    if (!dctrl.joystick_ready() && !fromfile) {
        std::cout << "joystick failed." << std::endl;
        //        exit(1);
    }
#endif

    logger << std::endl;
#ifdef HASSCREEN
    visualizer.init(&dctrl,&dtrkr,&itrkr,&dnav,fromfile);
#endif

    /*****init the video writer*****/
#if VIDEORESULTS
    if (output_video_results.init(argc,argv,VIDEORESULTS, data_output_dir + "videoResult.avi",IMG_W,IMG_H,VIDEOFPS,"192.168.1.255",5000,true)) {return 1;}
#endif
#if VIDEORAWLR
    if (output_video_LR.init(argc,argv,VIDEORAWLR,data_output_dir + "test_videoRawLR.avi",IMG_W*2,IMG_H,VIDEOFPS, "192.168.1.255",5000,false)) {return 1;}
#endif

#if VIDEODISPARITY
    cv::Mat fd = cam.get_disp_frame();
    std::cout << "Opening video file for disparity at " << fd.cols << "x" << fd.rows	 << " pixels with " << cam.getFPS() << "fps " << std::endl;
    cv::Size sizeDisp(fd.cols,fd.rows);
    output_video_disp.open(data_output_dir + "videoDisp.avi",CV_FOURCC('F','M','P','4'),cam.getFPS(),sizeDisp,false);

    if (!output_video_disp.isOpened())
    {
        std::cerr << "Disparity result video could not be opened!" << std::endl;
        return 1;
    }
#endif

    init_thread_pool();

    std::cout << "Main init successfull" << std::endl;

    return 0;
}

void close() {
    std::cout <<"Closing"<< std::endl;

#ifdef HASSCREEN
    cv::destroyAllWindows();
#endif

    /*****Close everything down*****/
    dtrkr.close();
    dctrl.close();
    dnav.close();
    itrkr.close();
#if CAMMODE == CAMMODE_REALSENSE
    if (!fromfile)
        arduino.close();
#endif
#ifdef HASSCREEN
    visualizer.close();
#endif
    visdat.close();
    cam.close();

#if VIDEORESULTS
    output_video_results.close();
#endif
#if VIDEORAWLR
    output_video_LR.close();
#endif

    close_thread_pool();
    std::cout <<"Closed"<< std::endl;
}

int main( int argc, char **argv )
{
    if (!init(argc,argv)) {
        process_video();
    }
    close();
    return 0;
}
