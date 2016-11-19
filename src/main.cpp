#include <mutex>
#include <thread>
#include <fstream>      // std::ifstream
#include <sstream>
#include <iostream>
#include <iomanip>

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

#include "opencv2/features2d/features2d.hpp"

using namespace cv;

/***********Enums****************/


/***********Variables****************/
unsigned char key = 0;
std::string msg;
int imgcount; // to measure fps
cv::Mat resFrame;
cv::VideoWriter outputVideoResults;
cv::VideoWriter outputVideoRawL,outputVideoRawR;
cv::VideoWriter outputVideoDisp;
stopwatch_c stopWatch;
std::string file;
std::string imageOutputDir;

int mouseX, mouseY;
int mouseLDown;
int mouseMDown;
int mouseRDown;
bool pausecam = false;

#ifdef _PC
KalamosFileCam cam;
#else
KalamosCam cam;
#endif
stereoAlg stereo;
DroneTracker dtrk;

/*******Private prototypes*********/
void process_video();
int main( int argc, char **argv);
void CallBackFunc(int event, int x, int y, int flags, void* userdata);
void handleKey();

/************ code ***********/

void process_video() {
    std::cout << "Running...\n";
    stopWatch.Start();


    //main while loop:
    while (key != 27 && cam.getCamRunning()) // ESC
    {


        if (!pausecam) {
            cam.waitForImage();
        }
        stereo.rectify(cam.frameL, cam.frameR);

        dtrk.track(stereo.frameLrect,stereo.frameRrect);
        resFrame = dtrk.resFrame;
#if defined(HASSCREEN) || defined(VIDEORESULTS)		
#ifdef HASSCREEN
        cv::imshow("Results", resFrame);
#endif
#ifdef VIDEORESULTS
        outputVideoResults.write(resFrame);
#endif
#endif
#ifdef VIDEORAW
        outputVideoRawL.write(cam.frameL);
        outputVideoRawR.write(cam.frameR);

#endif
#ifdef VIDEODISPARITY
        outputVideoDisp.write(cam.get_disp_frame());
#endif

        handleKey();

        imgcount++;
        float time = ((float)stopWatch.Read())/1000.0;
        //std::cout << "LOG " << time << " #" << imgcount << " --> " << ((float)imgcount) / time << " fps" << std::endl;

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

#ifdef HASSCREEN
    key = cv::waitKey(1);
    key = key & 0xff;
    if (key == 27) {  //esc
        cam.stopcam();
        return; // don't clear key, just exit
    }
#endif

    switch(key) {
    case 114: // [r]: reset stopwatch
        imgcount=0;
        stopWatch.Restart();
        msg="fps Reset";
        break;
    case ' ':
        pausecam=!pausecam;
        break;

    } // end switch key
#ifndef HASSCREEN
    if (key!=0) {
        std::cout << "Terminal: "  << msg << std::endl;
    }
#endif
    key=0;
}

int init(int argc, char **argv) {
#ifdef _PC
    if (argc != 3) {
        std::cout << "Wrong arguments. Specify the location to load images..." << std::endl;
        return 1;
    }
    std::cout << "Loading images from: " << std::string(argv[1]) << std::endl;
    if (cam.init(std::string(argv[1]))) {
        return 1;
    }

    std::string calib_folder = std::string(argv[2]);
#else
    cam.init();
    std::string calib_folder = "/factory/";
#endif


    /*****Start capturing images*****/
    std::cout << "Start cam\n";
    cam.start();
    
    /***init the stereo vision (groundtruth) algorithm ****/
    std::cout << "Initialising manual stereo algorithm\n";
    stereo.init(calib_folder);

    /*****init the (G)UI*****/
#ifdef HASSCREEN
    cv::namedWindow("Results", CV_WINDOW_AUTOSIZE);
    //cv::namedWindow("Results", CV_WINDOW_NORMAL);
    //cv::resizeWindow("Results", 1280, 720); //makes it slower

    //cv::namedWindow("Results", CV_WINDOW_FULLSCREEN); // fater, but it is still windowed

    //real fullscreen opencv hack:
    //cv::namedWindow("Results", CV_WINDOW_NORMAL);
    //cv::setWindowProperty("Results", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

    cv::setMouseCallback("Results", CallBackFunc, NULL);

#endif

#if defined(HASSCREEN) || defined(VIDEORESULTS)
    resFrame = cv::Mat::zeros(480, 640,CV_8UC3);
#endif


    /*****init the video writer*****/
#ifdef VIDEORESULTS    
    std::cout << "Opening video file for processed results at " << resFrame.cols << "x" << resFrame.rows << " pixels with " << cam.getFPS() << "fps " << std::endl;
    cv::Size sizeRes(resFrame.cols,resFrame.rows);
    outputVideoResults.open("videoResults.avi",CV_FOURCC('H','J','P','G'),cam.getFPS(),sizeRes,true);

    if (!outputVideoResults.isOpened())
    {
        std::cerr << "Output result video could not be opened!" << std::endl;
        return 1;
    }
#endif

#ifdef VIDEORAW
    std::cout << "Opening video file for raw video input at " << cam.getImWidth() << "x" << cam.getImHeight() << " pixels with " << cam.getFPS() << "fps " << std::endl;
    cv::Size sizeRaw(cam.getImWidth(),cam.getImHeight());
    outputVideoRawL.open("videoRawL.avi",CV_FOURCC('F','M','P','4'),cam.getFPS(),sizeRaw,true);

    if (!outputVideoRawL.isOpened())
    {
        std::cerr << "Raw result video could not be opened!" << std::endl;
        return 1;
    }

    outputVideoRawR.open("videoRawR.avi",CV_FOURCC('F','M','P','4'),cam.getFPS(),sizeRaw,true);

    if (!outputVideoRawR.isOpened())
    {
        std::cerr << "Raw result video could not be opened!" << std::endl;
        return 1;
    }
#endif
#ifdef VIDEODISPARITY
    cv::Mat fd = cam.get_disp_frame();
    std::cout << "Opening video file for disparity at " << fd.cols << "x" << fd.rows	 << " pixels with " << cam.getFPS() << "fps " << std::endl;
    cv::Size sizeDisp(fd.cols,fd.rows);
    outputVideoDisp.open("videoDisp.avi",CV_FOURCC('F','M','P','4'),cam.getFPS(),sizeDisp,false);

    if (!outputVideoDisp.isOpened())
    {
        std::cerr << "Disparity result video could not be opened!" << std::endl;
        return 1;
    }
#endif


    msg="";
    return 0;
}

void close() {

    /*****Close everything down*****/
    cam.close();

}

int main( int argc, char **argv )
{
    if (init(argc,argv)) {return 1;}

    process_video();
    close();

    return 0;
}


