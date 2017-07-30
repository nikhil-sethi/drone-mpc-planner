#include <mutex>
#include <thread>
#include <fstream>      // std::ifstream
#include <sstream>
#include <iostream>
#include <iomanip>

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

#include "opencv2/features2d/features2d.hpp"

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
int imgcount; // to measure fps
cv::Mat resFrame;
GStream outputVideoResults,outputVideoRawLR;
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
DroneController dctrl;

/*******Private prototypes*********/
void process_video();
int main( int argc, char **argv);
void CallBackFunc(int event, int x, int y, int flags, void* userdata);
void handleKey();

/************ code ***********/
void plot(cv::Mat data1,cv::Mat data2, std::string name);
cv::Mat roll_joystick(1,1,CV_32FC1);
cv::Mat roll_calculated(1,1,CV_32FC1);
cv::Mat throttle_joystick(1,1,CV_32FC1);
cv::Mat throttle_calculated(1,1,CV_32FC1);



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

        dtrk.track(stereo.frameLrect,stereo.frameRrect, stereo.Qf);
        dctrl.control(dtrk.data);
        resFrame = dtrk.resFrame;

#ifdef _PC
        std::cout << "Log: " << cam.getCurrentThrust()  << ", " << cam.getCurrentRoll() << ", " << cam.getCurrentPitch() << ", " << cam.getCurrentYaw() << std::endl;
#endif

#ifdef HASSCREEN
        cv::imshow("Results", resFrame);

        roll_joystick.push_back(cam.getCurrentRoll());
        roll_calculated.push_back((float)dctrl.commandedRoll);
        plot(roll_joystick,roll_calculated, "Roll");

        throttle_joystick.push_back(cam.getCurrentThrust());
        throttle_calculated.push_back((float)dctrl.commandedThrottle);
        plot(throttle_joystick,throttle_calculated, "Throttle");
#endif
#if VIDEORESULTS
        outputVideoResults.write(resFrame);
#endif
#if VIDEORAWLR
        outputVideoRawLR.write(cam.frameL,cam.frameR);
#endif
#if VIDEODISPARITY
        outputVideoDisp.write(cam.get_disp_frame());
#endif

        handleKey();

        imgcount++;
        float time = ((float)stopWatch.Read())/1000.0;
        std::cout << "Frame: " <<imgcount << ". FPS: " << imgcount / time << std::endl;

    } // main while loop

#ifdef HASSCREEN
    cv::destroyAllWindows();
#endif
}


void plot(cv::Mat data1,cv::Mat data2, const std::string name) {

    int line_width = 1;

    const int fsizex = 500;
    const int fsizey = 300;
    double min,max;

    cv::Mat tmp;
    tmp.push_back(data1);
    tmp.push_back(data2);
    cv::minMaxIdx(tmp,&min,&max,NULL,NULL);

    min-=1;
    max+=1;

    const float scaleX = (float)((fsizex))/(data1.rows);
    const float scaleY = (fsizey)/(max-min);

    cv::Mat frame = cv::Mat::zeros(fsizey+4*line_width, fsizex+4*line_width, CV_8UC3);

    int prev_y1 =0;
    int prev_y2 =0;
    for (int j = 0; j < data1.rows-1; j++)  {
        int y1 = data1.at<float>(j,1) - min;
        int y2 = data2.at<float>(j,1) - min;
        cv::line(frame, cv::Point(j*scaleX + 2*line_width , fsizey- prev_y1*scaleY +line_width*2) , cv::Point((j+1)*scaleX + 2*line_width, fsizey -  y1*scaleY +2*line_width), cv::Scalar(0,255,0), line_width, CV_AA, 0);
        cv::line(frame, cv::Point(j*scaleX + 2*line_width , fsizey- prev_y2*scaleY +line_width*2) , cv::Point((j+1)*scaleX + 2*line_width, fsizey -  y2*scaleY +2*line_width), cv::Scalar(255,0,0), line_width, CV_AA, 0);
        prev_y1 = y1;
        prev_y2 = y2;
    }

    imshow(name,frame);


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
        //std::cout << "Terminal: "  << msg << std::endl;
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
    std::cout << "Started cam\n";
    /***init the stereo vision (groundtruth) algorithm ****/
    std::cout << "Initialising manual stereo algorithm\n";
    stereo.init(calib_folder);

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

#if defined(HASSCREEN) || defined(VIDEORESULTS)
    resFrame = cv::Mat::zeros(480, 640,CV_8UC3);
#endif


    /*****init the video writer*****/

#if VIDEORESULTS   
    if (outputVideoResults.init(argc,argv,VIDEORESULTS, "videoResult.avi",864,864,"192.168.1.10",5004)) {return 1;}
#endif
#if VIDEORAWLR
    if (outputVideoRawLR.init(argc,argv,VIDEORAWLR,"videoRawLR.avi",2560,960,"192.168.1.10",5004)) {return 1;}
#endif

#if VIDEODISPARITY
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

    dtrk.init();
    dctrl.init();

    msg="";
    return 0;
}

void close() {

    /*****Close everything down*****/
    dtrk.close();
    cam.close();
    dctrl.close();

#if VIDEORESULTS   
    outputVideoResults.close(); 
#endif
#if VIDEORAWLR
    outputVideoRawLR.close();
#endif

}




int main( int argc, char **argv )
{

    roll_joystick.pop_back();
    roll_calculated.pop_back();
    throttle_joystick.pop_back();
    throttle_calculated.pop_back();
    if (init(argc,argv)) {return 1;}
    process_video();
    close();
    return 0;
}


