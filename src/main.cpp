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

#include "common.h"
#include "defines.h"
#include "smoother.h"

#include "stopwatch.h"
#include "dronetracker.h"
#include "dronecontroller.h"
#include "gstream.h"
#include "vizs.h"
#include "insect.h"
#include "logreader.h"

#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

using namespace cv;
using namespace std;

/***********Enums****************/


/***********Variables****************/
unsigned char key = 0;
int imgcount,detectcount; // to measure fps
cv::Mat resFrame;
GStream outputVideoColor,outputVideoRawLR;
cv::VideoWriter outputVideoDisp;
stopwatch_c stopWatch;
std::string file;
std::string data_output_dir;
std::string calib_folder;

int breakpause =-1;


// Declare RealSense pipeline, encapsulating the actual device and sensors
rs2::pipeline cam;
rs2::device pd;

Size imgsize(IMG_W, IMG_H);
cv::Mat Qf;
#define IR_ID_LEFT 1 //as seen from the camera itself
#define IR_ID_RIGHT 2

std::ofstream logger;
DroneTracker dtrkr;
DroneController dctrl;
Insect insect;
Visualizer visualizer;
LogReader logreader;
bool fromfile = false;


//tmp for capturing moth data
#define FRAME_BUF_SIZE 20
int frame_buffer_write_id = 0;
int frame_buffer_read_id = 2*FRAME_BUF_SIZE;
int frame_write_id_during_event = 0;
cv::Mat frameBL[FRAME_BUF_SIZE];
cv::Mat frameBR[FRAME_BUF_SIZE];

/*******Private prototypes*********/
void process_video();
int main( int argc, char **argv);
void handleKey();

/************ code ***********/
void process_video() {
    //    auto start = std::chrono::system_clock::now();
    //    std::time_t time = std::chrono::system_clock::to_time_t(start);
    //    std::cout << "Starting at " << std::ctime(&time) << std::endl; // something weird is going on with this line, it seems to crash the debugger if it is in another function...?
    //    std::cout << "Running..." << std::endl;
    stopWatch.Start();

    cv::Mat frameR,frameL ;
    rs2::frameset frame;
    frame = cam.wait_for_frames(); // init it with something

    //main while loop:
    while (key != 27) // ESC
    {

        static int breakpause_prev =-1;
        if (breakpause == 0 && breakpause_prev!=0) {
            ((rs2::playback)pd).pause();
            //dtrkr.breakpause = true;
        } else if (breakpause != 0 && breakpause_prev==0) {
            ((rs2::playback)pd).resume();
            dtrkr.breakpause = false;
        }
        breakpause_prev = breakpause;

        if (breakpause != 0) {
            frame = cam.wait_for_frames();
            frameL = Mat(imgsize, CV_8UC1, (void*)frame.get_infrared_frame(IR_ID_LEFT).get_data(), Mat::AUTO_STEP).clone();
            frameR = Mat(imgsize, CV_8UC1, (void*)frame.get_infrared_frame(IR_ID_RIGHT).get_data(), Mat::AUTO_STEP).clone();


            if (breakpause > 0)
                breakpause--;
        }

        logger << imgcount << ";" << frame.get_frame_number() << ";" ;
        if (!INSECT_DATA_LOGGING_MODE) {
            if (dtrkr.track(frameL,frameR, Qf)) {
                breakpause = 0;
            }
            dctrl.control(&(dtrkr.data));
        }
        insect.track(frameL,frameR, Qf);


#ifdef HASSCREEN
        if (fromfile) {
            int rs_id = frame.get_frame_number();
            LogReader::Log_Entry tmp  = logreader.getItem(rs_id);
            if (tmp.RS_ID == rs_id) {
                dctrl.joyRoll = tmp.joyRoll;
                dctrl.joyPitch = tmp.joyPitch;
                dctrl.joyYaw = tmp.joyYaw;
                dctrl.joyThrottle = tmp.joyThrottle;
                dctrl.joySwitch = tmp.joySwitch;
            }
        }
        visualizer.plot();

        resFrame = dtrkr.resFrame;

        cv::imshow("Results", resFrame);
#endif

        frameBL[frame_buffer_write_id] = frameL.clone();
        frameBR[frame_buffer_write_id] = frameR.clone();

        frame_buffer_write_id = (frame_buffer_write_id + 1) % FRAME_BUF_SIZE;
        if (insect.data.valid) { //
            frame_buffer_read_id = 0;
            frame_write_id_during_event = (frame_buffer_write_id + FRAME_BUF_SIZE - 1 ) % FRAME_BUF_SIZE;
        }

        if (frame_buffer_read_id<2*FRAME_BUF_SIZE) {
            detectcount++;
            frame_buffer_read_id++;
            int id = (frame_write_id_during_event + frame_buffer_read_id)  % FRAME_BUF_SIZE;
            int frameWritten = 0;
#if VIDEORAWLR
            frameWritten = outputVideoRawLR.write(frameBL[id],frameBR[id]);
#endif
            if (frameWritten == 0) {
#if VIDEODISPARITY
                outputVideoDisp.write(cam.get_disp_frame());
#endif
#if VIDEORESULTS
                outputVideoColor.write(resFrame);
#endif
            }
        }
        imgcount++;
        float time = ((float)stopWatch.Read())/1000.0;
        dtrkr.data.background_calibrated= (time > 15);
        std::cout << "Frame: " <<imgcount << " (" << detectcount << ", " << frame.get_frame_number() << "). FPS: " << imgcount / time << ". Time: " << time << std::endl;
        handleKey();
        if (imgcount > 60000)
            break;
        logger << std::endl;
    } // main while loop

#ifdef HASSCREEN
    cv::destroyAllWindows();
#endif
}

void handleKey() {

    //#ifdef HASSCREEN
    key = cv::waitKey(1);
    key = key & 0xff;
    if (key == 27) {  //esc
        //        cam.stopcam();
        return; // don't clear key, just exit
    }
    //#endif

    switch(key) {
    case 114: // [r]: reset stopwatch
        imgcount=0;
        detectcount=0;
        stopWatch.Restart();
        break;
    case ' ': // [r]: reset stopwatch
        //dtrkr.breakpause = true;
        if (breakpause >-1) {
            breakpause =-1;
        } else {
            breakpause = 0;
        }
        break;
    case 'n': // next frame
        //dtrkr.breakpause = true;
        breakpause = 1;
        break;

    } // end switch key
#ifndef HASSCREEN
    if (key!=0) {
        //std::cout << "Terminal: "  << msg << std::endl;
    }
#endif
    key=0;
}

void my_handler(int s){
    std::cout << "Caught ctrl-c:" << s << std::endl;
}

int init(int argc, char **argv) {


    if (argc ==2 ) {
        fromfile = true;
    }
    data_output_dir = "./";

    logger.open(data_output_dir  + "log.txt",std::ofstream::out);
    logger << "ID;RS_ID;";
    if (!INSECT_DATA_LOGGING_MODE) {
        dtrkr.init(&logger);
        dctrl.init(&logger,fromfile);
    }
    insect.init(&logger);
    logger << std::endl;

    std::cout << "Frame buf size: " << FRAME_BUF_SIZE << std::endl;

    /*****Start capturing images*****/
    std::cout << "Initializing cam\n";
    // Declare config
    rs2::config cfg;
    cfg.disable_all_streams();
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, IMG_W, IMG_H, RS2_FORMAT_Y8, VIDEOFPS);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, IMG_W, IMG_H, RS2_FORMAT_Y8, VIDEOFPS);

    if (argc ==2 ) {
        cfg.enable_device_from_file(string(argv[1]) + ".bag");
        fromfile = true;
        logreader.init(string(argv[1]) + ".txt");
    } else {
        cfg.enable_record_to_file("test");
    }

    rs2::pipeline_profile selection = cam.start(cfg);
    std::cout << "Started cam\n";

    if (fromfile ) {
        pd = selection.get_device();
        ((rs2::playback)pd).set_real_time(false);
    } else {
        rs2::device selected_device = selection.get_device();
        auto depth_sensor = selected_device.first<rs2::depth_sensor>();

        if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
        {
            depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);
        }
        //        if (depth_sensor.supports(RS2_OPTION_LASER_POWER)) {
        //            auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
        //            depth_sensor.set_option(RS2_OPTION_LASER_POWER, (range.max - range.min)/2 + range.min);
        //        }
        if (depth_sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
            depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE,.0);
        }

        if (depth_sensor.supports(RS2_OPTION_EXPOSURE)) {
            auto range = depth_sensor.get_option_range(RS2_OPTION_EXPOSURE);
            depth_sensor.set_option(RS2_OPTION_EXPOSURE, 1000); //TODO: automate this setting.
        }
        //weird with D435 this is totally unneccesary...? probably ROI related
        if (depth_sensor.supports(RS2_OPTION_GAIN)) {
            auto range = depth_sensor.get_option_range(RS2_OPTION_GAIN);
            depth_sensor.set_option(RS2_OPTION_GAIN, (range.max - range.min)/2 + range.min);
            //depth_sensor.set_option(RS2_OPTION_GAIN, range.min); // increasing this causes noise
        }
        //        cam.stop();
        //        selection = cam.start(cfg);

        std::cout << "Set cam config\n";
    }

    // Obtain focal length and principal point (from intrinsics)
    auto depth_stream = selection.get_stream(RS2_STREAM_INFRARED, 1).as<rs2::video_stream_profile>();
    auto i = depth_stream.get_intrinsics();
    float focal_length = i.fx; // same as fy
    float cx = i.ppx; // same for both cameras
    float cy = i.ppy;

    // Obtain baseline (from extrinsics)
    auto ir1_stream = selection.get_stream(RS2_STREAM_INFRARED, 1);
    auto ir2_stream = selection.get_stream(RS2_STREAM_INFRARED, 2);
    rs2_extrinsics e = ir2_stream.get_extrinsics_to(ir1_stream);
    float baseline = e.translation[0];

    //init Qf: https://stackoverflow.com/questions/27374970/q-matrix-for-the-reprojectimageto3d-function-in-opencv
    Qf = (Mat_<double>(4, 4) << 1.0, 0.0, 0.0, -cx, 0.0, 1.0, 0.0, -cy, 0.0, 0.0, 0.0, focal_length, 0.0, 0.0, 1/baseline, 0.0);

    /*****init the (G)UI*****/
#ifdef HASSCREEN
    cv::namedWindow("Results", CV_WINDOW_AUTOSIZE);
#endif

#if defined(HASSCREEN) || VIDEORESULTS
    resFrame = cv::Mat::zeros(IMG_H, IMG_W,CV_8UC3);
#endif

    /*****init the video writer*****/
#if VIDEORESULTS
    if (outputVideoColor.init(argc,argv,VIDEORESULTS, data_output_dir + "videoResult.avi",IMG_W,IMG_H,VIDEOFPS,"192.168.1.10",5004,true)) {return 1;}
#endif
#if VIDEORAWLR
    if (outputVideoRawLR.init(argc,argv,VIDEORAWLR,data_output_dir + "videoRawLR.avi",IMG_W*2,IMG_H,VIDEOFPS, "127.0.0.1",5000,false)) {return 1;}
#endif

#if VIDEODISPARITY
    cv::Mat fd = cam.get_disp_frame();
    std::cout << "Opening video file for disparity at " << fd.cols << "x" << fd.rows	 << " pixels with " << cam.getFPS() << "fps " << std::endl;
    cv::Size sizeDisp(fd.cols,fd.rows);
    outputVideoDisp.open(data_output_dir + "videoDisp.avi",CV_FOURCC('F','M','P','4'),cam.getFPS(),sizeDisp,false);

    if (!outputVideoDisp.isOpened())
    {
        std::cerr << "Disparity result video could not be opened!" << std::endl;
        return 1;
    }
#endif

    visualizer.init(&dctrl,&dtrkr);

    //init ctrl - c catch
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    //init pre-capture buffer
    for (int i = 0; i<FRAME_BUF_SIZE;i++){
        rs2::frameset tmpdata = cam.wait_for_frames(); // Wait for next set of frames from the camera
        cv::Mat image = cv::Mat(imgsize, CV_8UC1, (void*)tmpdata.get_data(), cv::Mat::AUTO_STEP);
        frameBL[i] = Mat(imgsize, CV_8UC1, (void*)tmpdata.get_infrared_frame(IR_ID_LEFT).get_data(), Mat::AUTO_STEP).clone();
        frameBR[i] = Mat(imgsize, CV_8UC1, (void*)tmpdata.get_infrared_frame(IR_ID_RIGHT).get_data(), Mat::AUTO_STEP).clone();
    }

    std::cout << "Main init successfull" << std::endl;

    return 0;
}

void close() {
    std::cout <<"Closing"<< std::endl;
    /*****Close everything down*****/
    if (!INSECT_DATA_LOGGING_MODE) {
        dtrkr.close();
        dctrl.close();
    }
    insect.close();

#if VIDEORESULTS   
    outputVideoColor.close();
#endif
#if VIDEORAWLR
    outputVideoRawLR.close();
#endif

    std::cout <<"Closed"<< std::endl;
}

int main( int argc, char **argv )
{
    if (init(argc,argv)) {return 1;}
    process_video();
    close();
    return 0;
}
