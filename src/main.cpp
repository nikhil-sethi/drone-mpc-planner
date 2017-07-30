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
#include "vizs.h"

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

std::ofstream logger;

#ifdef _PC
KalamosFileCam cam;
Visualizer visualizer;
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
void process_video() {
    std::cout << "Running...\n";
    stopWatch.Start();

    //main while loop:
    while (key != 27 && cam.getCamRunning()) // ESC
    {
        if (!pausecam) {
            cam.waitForImage();
        }
#ifdef _PC
        stereo.rectify(cam.frameL, cam.frameR);
        dtrk.track(stereo.frameLrect,stereo.frameRrect, stereo.Qf);
        resFrame = dtrk.resFrame;
#endif
        dctrl.control(dtrk.data);


#ifdef _PC
        logger << "TRPY: " << cam.getCurrentThrust()  << ", " << cam.getCurrentRoll() << ", " << cam.getCurrentPitch() << ", " << cam.getCurrentYaw() << std::endl;
        std::cout << "TRPY: " << cam.getCurrentThrust()  << ", " << cam.getCurrentRoll() << ", " << cam.getCurrentPitch() << ", " << cam.getCurrentYaw() << std::endl;
        visualizer.plot();
#endif

#ifdef HASSCREEN
        cv::imshow("Results", resFrame);
#endif
		int frameWritten = 0;


#if VIDEORAWLR
//while (!outputVideoRawLR.getWanted()) {usleep(10000);}
        frameWritten = outputVideoRawLR.write(cam.frameL,cam.frameR);
#endif
		if (frameWritten == 0) {
#if VIDEODISPARITY
    	    outputVideoDisp.write(cam.get_disp_frame());
#endif
#if VIDEORESULTS
	        outputVideoResults.write(resFrame);
#endif
	        handleKey();

	        imgcount++;
	        float time = ((float)stopWatch.Read())/1000.0;
            logger << "Frame: " <<imgcount << ". FPS: " << imgcount / time << std::endl;
            std::cout << "Frame: " <<imgcount << ". FPS: " << imgcount / time << std::endl;
		}
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
        //std::cout << "Terminal: "  << msg << std::endl;
    }
#endif
    key=0;
}

int init(int argc, char **argv) {

   logger.open("log.txt",std::ofstream::out);

#ifdef _PC
    if (argc != 3) {
        std::cout << "Wrong arguments. Specify the locations for: (1) loading images, (2) the calib folder..." << std::endl;
        return 1;
    }
    std::cout << "Loading images from: " << std::string(argv[1]) << std::endl;
    if (cam.init(std::string(argv[1]))) {
        return 1;
    }
    std::cout << "Calib folder: " << std::string(argv[2]) << std::endl;
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

#if defined(HASSCREEN) || VIDEORESULTS
    resFrame = cv::Mat::zeros(480, 640,CV_8UC3);    
#endif


    /*****init the video writer*****/

#if VIDEORESULTS   
    if (outputVideoResults.init(argc,argv,VIDEORESULTS, "videoResult.avi",864,864,"192.168.1.10",5004)) {return 1;}
#endif
#if VIDEORAWLR
    if (outputVideoRawLR.init(argc,argv,VIDEORAWLR,"videoRawLR.avi",cam.getImWidth()*2,cam.getImHeight(),"192.168.1.10",5004)) {return 1;}
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
    dctrl.init(&logger);
#ifdef _PC
    visualizer.init(&cam,&dctrl);
#endif

    msg="";

    std::cout << "Main init successfull" << std::endl;
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
    if (init(argc,argv)) {return 1;}
    process_video();
    close();
    return 0;
}


