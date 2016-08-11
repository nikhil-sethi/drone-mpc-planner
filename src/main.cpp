#include <mutex>
#include <thread>
#include <fstream>      // std::ifstream
#include <sstream>
#include <iostream>
#include <iomanip>

#include "defines.h"
#include "smoother.h"
#include "kalamoscam.h"
#include "stopwatch.h"




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

#ifdef USE_TERMINAL_INPUT
std::thread thread_TerminalInput;
#endif

KalamosCam cam;

/*******Private prototypes*********/
void process_video();
#ifdef USE_TERMINAL_INPUT
void TerminalInputThread();
#endif
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


	cam.waitForImage();
   

	resFrame = cam.frameL;

        if (mouseLDown>1) {mouseLDown--;}
        if (mouseRDown>1) {mouseRDown--;}
        if (mouseMDown>1) {mouseMDown--;}




#if defined(HASSCREEN) || defined(VIDEORESULTS)		
#ifdef HASSCREEN

        cv::imshow("Results", cam.frameL);
#endif
#ifdef VIDEORESULTS
        outputVideoResults.write(resFrame);
#endif
#endif
#ifdef VIDEORAW
	outputVideoRawL.write(cam.frameL);
	outputVideoRawR.write(cam.frameR);
/*
	std::stringstream sL;
	sL << "/home/kalamos/kevin/nosquito/kalamos/build/tmp/" << imgcount << "L" << ".bmp";
	imwrite(sL.str(),cam.frameL);

	std::stringstream sR;
	sR << "/home/kalamos/kevin/nosquito/kalamos/build/tmp/" << imgcount << "R" << ".bmp";
	imwrite(sR.str(),cam.frameR);
*/

#endif
#ifdef VIDEODISPARITY
        outputVideoDisp.write(cam.get_disp_frame());	
#endif

        handleKey();

        imgcount++;
        float time = ((float)stopWatch.Read())/1000.0;
        std::cout << "LOG " << time << " #" << imgcount << " --> " << ((float)imgcount) / time << " fps" << std::endl;

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
        mouseLDown=10;
    } else if  ( event == cv::EVENT_LBUTTONUP ) {
        mouseLDown=0;
    } else if  ( event == cv::EVENT_RBUTTONDOWN ) {
        mouseRDown=10;
    } else if  ( event == cv::EVENT_RBUTTONUP ) {
        mouseRDown=0;
    } else if  ( event == cv::EVENT_MBUTTONDOWN ) {
        mouseMDown=10;
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

    } // end switch key
#ifndef HASSCREEN
    if (key!=0) {
        std::cout << "Terminal: "  << msg << std::endl;
    }
#endif
    key=0;
}

void TerminalInputThread() {
#ifdef USE_TERMINAL_INPUT
    std::cout << "Terminal input enabled! (This disrupts running in background)" << std::endl;
    while(cam.getCamRunning()) {
        std::cin >> key;
        if (key==120 || key==113) {
            key=27; // translate x to esc
            cam.stopcam();
            std::cout << "Exiting\n";
        }
    }
#endif
}

int init(int argc, char **argv) {

    cam.init();

    /*****Start capturing images*****/
    std::cout << "Start cam\n";
    cam.start();
    
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
#ifdef USE_TERMINAL_INPUT
    thread_TerminalInput = std::thread(TerminalInputThread);
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
#ifdef USE_TERMINAL_INPUT
    thread_TerminalInput.detach();	//cin is blocking
#endif

}

int main( int argc, char **argv )
{
    if (init(argc,argv)) {return 1;}

    process_video();
    close();

    return 0;
}


