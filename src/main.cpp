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
GStream outputVideoResults,outputVideoRawLR;
cv::VideoWriter outputVideoDisp;
stopwatch_c stopWatch;
std::string file;
std::string data_output_dir;
std::string calib_folder;

// Declare RealSense pipeline, encapsulating the actual device and sensors
rs2::pipeline cam;
const int img_w = 848;
const int img_h = 480;
int fps = 90;
Size imgsize(img_w, img_h);
cv::Mat Qf;
#define IR_ID_LEFT 1 //as seen from the camera itself
#define IR_ID_RIGHT 2

std::ofstream logger;
DroneTracker dtrkr;
DroneController dctrl;
Insect insect;
Visualizer visualizer;

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
    auto start = std::chrono::system_clock::now();
    std::time_t time = std::chrono::system_clock::to_time_t(start);
    std::cout << "Starting at " << std::ctime(&time) << std::endl; // something weird is going on with this line, it seems to crash the debugger if it is in another function...?
    std::cout << "Running..." << std::endl;
    stopWatch.Start();

    cv::Mat image,frameR,frameL ;
    //main while loop:
    while (key != 27) // ESC
    {

        rs2::frameset frame = cam.wait_for_frames();
        image = Mat(imgsize, CV_8UC1, (void*)frame.get_data(), Mat::AUTO_STEP);
        frameL = Mat(imgsize, CV_8UC1, (void*)frame.get_infrared_frame(IR_ID_LEFT).get_data(), Mat::AUTO_STEP);
        frameR = Mat(imgsize, CV_8UC1, (void*)frame.get_infrared_frame(IR_ID_RIGHT).get_data(), Mat::AUTO_STEP);


        logger << imgcount << ";";
        insect.track(frameL,frameR, Qf);
        //dtrkr.track(stereo.frameLrect,stereo.frameRrect, stereo.Qf);
        //dctrl.control(dtrkr.data);
        //putText(cam.frameR,std::to_string(imgcount),cv::Point(100,100),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(125,125,255));

#ifdef HASSCREEN
        //        visualizer.plot();
        //        resFrame = cam.get_disp_frame();
        //        cv::resize(resFrame,resFrame,cv::Size(480,480));
        //        cv::applyColorMap(resFrame, resFrame, cv::COLORMAP_JET);

        //        cv::Mat frameL = stereo.frameLrect;
        //        cv::Mat frameR = stereo.frameRrect;
        //        cv::Mat frame(frameL.rows,frameL.cols+frameR.cols,CV_8UC3);
        //        frameL.copyTo(frame(cv::Rect(0,0,frameL.cols, frameL.rows)));
        //        frameR.copyTo(frame(cv::Rect(frameL.cols,0,frameR.cols, frameR.rows)));
        //        resFrame = frame;

        //resFrame = insect.resFrame;

        cv::imshow("Results", frameL);
#endif

        frameBL[frame_buffer_write_id] = frameL.clone();
        frameBR[frame_buffer_write_id] = frameR.clone();

        frame_buffer_write_id = (frame_buffer_write_id + 1) % FRAME_BUF_SIZE;
        if (insect.data.valid) {
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
                outputVideoResults.write(resFrame);
#endif
            }
        }
        imgcount++;
        float time = ((float)stopWatch.Read())/1000.0;
        std::cout << "Frame: " <<imgcount << " (" << detectcount << "). FPS: " << imgcount / time << ". Time: " << time << std::endl;
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
    case ' ':
//        pausecam=!pausecam;
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

    data_output_dir = "./";

    logger.open(data_output_dir  + "log.txt",std::ofstream::out);
    logger << "ID;";
    dtrkr.init(&logger);
    dctrl.init(&logger); // for led driver
    insect.init(&logger);

    std::cout << "Frame buf size: " << FRAME_BUF_SIZE << std::endl;

    /*****Start capturing images*****/
    std::cout << "Initializing cam\n";
    // Declare config
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, img_w, img_h, RS2_FORMAT_Y8, fps);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, img_w, img_h, RS2_FORMAT_Y8, fps);
    cam.start(cfg);
    std::cout << "Started cam\n";

    /*****init the (G)UI*****/
#ifdef HASSCREEN
    cv::namedWindow("Results", CV_WINDOW_AUTOSIZE);
    //cv::namedWindow("Results", CV_WINDOW_NORMAL);
    //cv::resizeWindow("Results", 1280, 720); //makes it slower

    //cv::namedWindow("Results", CV_WINDOW_FULLSCREEN); // faster, but it is still windowed

    //real fullscreen opencv hack:
    //cv::namedWindow("Results", CV_WINDOW_NORMAL);
    //cv::setWindowProperty("Results", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
#endif

#if defined(HASSCREEN) || VIDEORESULTS
    resFrame = cv::Mat::zeros(480, 640,CV_8UC3);
#endif

    /*****init the video writer*****/
#if VIDEORESULTS
    if (outputVideoResults.init(argc,argv,VIDEORESULTS, data_output_dir + "videoResult.avi",864,864,"192.168.1.10",5004)) {return 1;}
#endif
#if VIDEORAWLR
    if (outputVideoRawLR.init(argc,argv,VIDEORAWLR,data_output_dir + "videoRawLR.avi",img_w*2,img_h,"192.168.1.10",5004)) {return 1;}
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
        frameBL[i] = Mat(imgsize, CV_8UC1, (void*)tmpdata.get_infrared_frame(IR_ID_LEFT).get_data(), Mat::AUTO_STEP);
        frameBR[i] = Mat(imgsize, CV_8UC1, (void*)tmpdata.get_infrared_frame(IR_ID_RIGHT).get_data(), Mat::AUTO_STEP);
    }

    //init Qf:
    Qf = cv::Mat::zeros(cv::Size(3,3),CV_32F);
    //TODO: init Qf with proper values

    std::cout << "Main init successfull" << std::endl;

    return 0;
}

void close() {
    std::cout <<"Closing"<< std::endl;
    /*****Close everything down*****/
    dtrkr.close();
//    cam.close();
    dctrl.close();
    insect.close();

#if VIDEORESULTS   
    outputVideoResults.close(); 
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
