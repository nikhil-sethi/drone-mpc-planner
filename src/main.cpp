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

#include "common.h"

#include "defines.h"
#include "smoother.h"
#ifdef _PC
#include "kalamosfilecam.h"
#else
#include "kalamoscam.h"
#endif
#include "stopwatch.h"
#include "stereoalg.h"
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

using namespace cv;

/*
Benchmark:
Een camera uitlezen en yuv->rgb: 30 fps
Beide camera's uitlezen en yuv->rgb: 16.8 fps
	+ HASSCREEN (imshow frameL): 11.2 fps
	+ rectify: 12.4 fps
	+ rectify + drone tracker: 4.8 fps
	+ rectify + drone tracker + controller: 4.8 fps
	+ HASSCREEN + rectify + drone tracker: 3.3 fps
*/

/***********Enums****************/


/***********Variables****************/
unsigned char key = 0;
std::string msg;
int imgcount,detectcount; // to measure fps
cv::Mat resFrame;
GStream outputVideoResults,outputVideoRawLR;
cv::VideoWriter outputVideoDisp;
stopwatch_c stopWatch;
std::string file;
std::string data_output_dir;
std::string calib_folder;

int mouseX, mouseY;
int mouseLDown;
int mouseMDown;
int mouseRDown;
bool pausecam = false;

std::ofstream logger;

Visualizer visualizer;
#ifdef _PC
KalamosFileCam cam;

#else
KalamosCam cam;
#endif
stereoAlg stereo;
DroneTracker dtrkr;
DroneController dctrl;
Insect insect;

#define FRAME_BUF_SIZE 20
int frame_buffer_write_id = 0;
int frame_buffer_read_id = 2*FRAME_BUF_SIZE;
int frame_write_id_during_event = 0;
cv::Mat frameBL[FRAME_BUF_SIZE];
cv::Mat frameBR[FRAME_BUF_SIZE];


/*******Private prototypes*********/
void process_video();
int main( int argc, char **argv);
void CallBackFunc(int event, int x, int y, int flags, void* userdata);
void handleKey();

/************ code ***********/
void process_video() {
    std::cout << "Running...\n";
    stopWatch.Start();

    //init the buffer:
    cam.waitForImage();
    for (int i = 0; i<FRAME_BUF_SIZE;i++){
        frameBL[i] = cam.frameL.clone();
        frameBR[i] = cam.frameR.clone();
    }

    //main while loop:
    while (key != 27 && cam.getCamRunning()) // ESC
    {
        if (!pausecam) {
            cam.waitForImage();
        }

        logger << imgcount << ";";
        stereo.rectify(cam.frameL, cam.frameR);
        insect.track(stereo.frameLrect,stereo.frameRrect, stereo.Qf);
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

        resFrame = insect.resFrame;

        cv::imshow("Results", resFrame);
#endif

        frameBL[frame_buffer_write_id] = cam.frameL.clone();
        frameBR[frame_buffer_write_id] = cam.frameR.clone();

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

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    mouseX = x;
    mouseY = y;

    if  ( event == cv::EVENT_LBUTTONDOWN ) {
        mouseLDown=1;
    } else if  ( event == cv::EVENT_LBUTTONUP ) {
        mouseLDown=0;
    } else if  ( event == cv::EVENT_RBUTTONDOWN ) {
        mouseRDown=1;
    } else if  ( event == cv::EVENT_RBUTTONUP ) {
        mouseRDown=0;
    } else if  ( event == cv::EVENT_MBUTTONDOWN ) {
        mouseMDown=1;
    } else if  ( event == cv::EVENT_MBUTTONUP ) {
        mouseMDown=0;
    } else if ( event == cv::EVENT_MOUSEMOVE )  {

    }
}

void handleKey() {

//#ifdef HASSCREEN
    key = cv::waitKey(1);
    key = key & 0xff;
    if (key == 27) {  //esc
        cam.stopcam();
        return; // don't clear key, just exit
    }
//#endif

    switch(key) {
    case 114: // [r]: reset stopwatch
        imgcount=0;
        detectcount=0;
        stopWatch.Restart();
        msg="fps Reset";
        break;
    case ' ':
        pausecam=!pausecam;
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
         cam.stopcam();
}

int init(int argc, char **argv) {

    if (argc > 3) {
        data_output_dir = std::string(argv[1]) + "/";
        std::cout << "Data output dir: " << data_output_dir << std::endl;
        std::cout << "Calib folder: " << std::string(argv[2]) << std::endl;
        calib_folder = std::string(argv[2]) + "/";
    } else {
        std::cout << "Missing arguments. Format [data_output_dir, calibration dir,|data_input_dir|]" << std::endl;
        return 1;
    }

    logger.open(data_output_dir  + "log.txt",std::ofstream::out);
    logger << "ID;";
//   dtrkr.init(&logger);
    dctrl.init(&logger); // for led driver
    insect.init(&logger);

     std::cout << "Frame buf size: " << FRAME_BUF_SIZE << std::endl;

#ifdef _PC
    std::string data_in_dir = std::string(argv[3])+ "/";
     std::cout << "Loading images from: " << data_in_dir << std::endl;
    if (cam.init(data_in_dir)) {
        return 1;
    }
#else
    cam.init();
#endif

    /*****Start capturing images*****/
    std::cout << "Start cam\n";
    cam.start();
    std::cout << "Started cam\n";
    /***init the stereo vision (groundtruth) algorithm ****/
    std::cout << "Initialising manual stereo algorithm\n";
    if (stereo.init(calib_folder))
        return 1;

    /*****init the (G)UI*****/
#ifdef HASSCREEN
    cv::namedWindow("Results", CV_WINDOW_AUTOSIZE);
    //cv::namedWindow("Results", CV_WINDOW_NORMAL);
    //cv::resizeWindow("Results", 1280, 720); //makes it slower

    //cv::namedWindow("Results", CV_WINDOW_FULLSCREEN); // faster, but it is still windowed

    //real fullscreen opencv hack:
    //cv::namedWindow("Results", CV_WINDOW_NORMAL);
    //cv::setWindowProperty("Results", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

    cv::setMouseCallback("Results", CallBackFunc, NULL);

#endif

#if defined(HASSCREEN) || VIDEORESULTS
    resFrame = cv::Mat::zeros(480, 640,CV_8UC3);    
#endif

    /*****init the video writer*****/
#if VIDEORESULTS   
    if (outputVideoResults.init(argc,argv,VIDEORESULTS, data_output_dir + "videoResult.avi",864,864,"192.168.1.10",5004)) {return 1;}
#endif
#if VIDEORAWLR
    if (outputVideoRawLR.init(argc,argv,VIDEORAWLR,data_output_dir + "videoRawLR.avi",cam.getImWidth()*2,cam.getImHeight(),"192.168.1.10",5004)) {return 1;}
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

#ifdef _PC
    visualizer.init(&cam,&dctrl,&dtrkr);
#else
    visualizer.init(&dctrl,&dtrkr);
#endif

    msg="";

    //init ctrl - c catch
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    std::cout << "Main init successfull" << std::endl;

    auto start = std::chrono::system_clock::now();
    std::time_t time = std::chrono::system_clock::to_time_t(start);
    std::cout << "Starting at " << std::ctime(&time) << std::endl;
    return 0;
}

void close() {
    std::cout <<"Closing"<< std::endl;
    /*****Close everything down*****/
//    dtrkr.close();
    cam.close();
//    dctrl.close();

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
