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

#include "opencv2/features2d/features2d.hpp"
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

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

#ifdef USE_TERMINAL_INPUT
std::thread thread_TerminalInput;
#endif

KalamosCam cam;


    //thresh params
    int iLowH1r = 0;
    int iHighH1r = 6;
    int iLowS1r = 0;
    int iHighS1r = 255;
    int iLowV1r = 188;
    int iHighV1r = 255;
    int iOpen1r =1;
    int iClose1r =1;

    //thresh params
    int iLowH1b = 92;
    int iHighH1b = 117;
    int iLowS1b = 0;
    int iHighS1b = 255;
    int iLowV1b = 165;
    int iHighV1b = 255;
    int iOpen1b =1;
    int iClose1b =1;

//blob params

    // Change thresholds
    int minThreshold = 10;
    int maxThreshold = 91;

    // Filter by Area.
    int filterByArea = 1;
    int minArea = 2;
    int maxArea = 21;

    // Filter by Circularity
    int filterByCircularity = 0;
    int minCircularity = 10;
    int maxCircularity = 100;

    // Filter by Convexity
    int filterByConvexity = 0;
    int minConvexity = 87;
    int maxConvexity = 100;

    // Filter by Inertia
    int filterByInertia = 0;
    int minInertiaRatio = 1;
    int maxInertiaRatio = 100;


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


    namedWindow("Thresh Moeder1", WINDOW_NORMAL); //create a window called "Control"
    createTrackbar("LowH1", "Thresh Moeder1", &iLowH1b, 255);
    createTrackbar("HighH1", "Thresh Moeder1", &iHighH1b, 255);
    createTrackbar("LowS1", "Thresh Moeder1", &iLowS1b, 255);
    createTrackbar("HighS1", "Thresh Moeder1", &iHighS1b, 255);
    createTrackbar("LowV1", "Thresh Moeder1", &iLowV1b, 255);
    createTrackbar("HighV1", "Thresh Moeder1", &iHighV1b, 255);
    createTrackbar("Opening1", "Thresh Moeder1", &iOpen1b, 30);
    createTrackbar("Closing1", "Thresh Moeder1", &iClose1b, 30);


 namedWindow("Blob Moeder", cv::WINDOW_NORMAL); //create a window called "Control"

    createTrackbar("minThreshold", "Blob Moeder", &minThreshold, 255);
    createTrackbar("maxThreshold", "Blob Moeder", &maxThreshold, 255);

    createTrackbar("filterByArea", "Blob Moeder", &filterByArea, 1);
    createTrackbar("minArea", "Blob Moeder", &minArea, 10000);
    createTrackbar("maxArea", "Blob Moeder", &maxArea, 10000);


    createTrackbar("filterByCircularity", "Blob Moeder", &filterByCircularity, 1);
    createTrackbar("minCircularity", "Blob Moeder", &minCircularity, 100);
    createTrackbar("maxCircularity", "Blob Moeder", &maxCircularity, 100);

    createTrackbar("filterByConvexity", "Blob Moeder", &filterByConvexity, 1);
    createTrackbar("minConvexity", "Blob Moeder", &minConvexity, 100);
    createTrackbar("maxConvexity", "Blob Moeder", &maxConvexity, 100);


    createTrackbar("filterByInertia", "Blob Moeder", &filterByInertia, 1);
    createTrackbar("minInertiaRatio", "Blob Moeder", &minInertiaRatio, 100);
    createTrackbar("maxInertiaRatio", "Blob Moeder", &maxInertiaRatio, 100);
    



    //    // Draw detected blobs as red circles.
    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
  //    Mat im_with_keypoints;
 //    drawKeypoints( imgGray, keypoints, imgThresholded, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    //imgThresholded.copyTo(imBGR);









	
	cv::namedWindow("ori", CV_WINDOW_NORMAL);
    cv::resizeWindow("ori", 1280, 720); //makes it slower
	cv::namedWindow("Results", CV_WINDOW_NORMAL);
    cv::resizeWindow("Results", 1280, 720); //makes it slower
    while (key != 27 && cam.getCamRunning()) // ESC
    {


	cam.waitForImage();


	resFrame = cv::Mat(cam.frameL.rows/4, cam.frameL.cols/4,CV_8UC3);
	cv::resize(cam.frameL,resFrame,resFrame.size(),0,0);

	
    Mat imgHSV;
    cvtColor(resFrame, imgHSV, COLOR_BGR2HSV);

    Mat imgThresholdedr,imgThresholdedb,imgThresholded;
    inRange(imgHSV, Scalar(iLowH1r, iLowS1r, iLowV1r), Scalar(iHighH1r, iHighS1r, iHighV1r), imgThresholdedr); //Threshold the image
    inRange(imgHSV, Scalar(iLowH1b, iLowS1b, iLowV1b), Scalar(iHighH1b, iHighS1b, iHighV1b), imgThresholdedb); //Threshold the image

	imgThresholded = imgThresholdedb;// + imgThresholdedr;

    // Setup SimpleBlobDetector parameters.
	SimpleBlobDetector::Params params;

  // Change thresholds
   	params.minThreshold = minThreshold+1;
	params.maxThreshold = maxThreshold+1;

     // Filter by Area.
	params.filterByArea = filterByArea;
	params.minArea = minArea+1;
	params.maxArea = maxArea+1;

     // Filter by Circularity
	params.filterByCircularity = filterByCircularity;
	params.minCircularity = ((float)minCircularity)/100.0f;
	params.maxCircularity = ((float)maxCircularity)/100.0f;

    //    // Filter by Convexity
	params.filterByConvexity = filterByConvexity;
	params.minConvexity = ((float)minConvexity)/100.0f;
	params.maxConvexity = ((float)maxConvexity)/100.0f;

    //    // Filter by Inertia
	params.filterByInertia = filterByInertia;
	params.minInertiaRatio = ((float)minInertiaRatio)/100.0f;
	params.maxInertiaRatio = ((float)maxInertiaRatio)/100.0f;

    params.minRepeatability = 0;
    params.minDistBetweenBlobs=0;
    params.filterByColor = 0;
    params.thresholdStep=1;

    //    // Set up detector with params
    SimpleBlobDetector detector(params);

    //    // Detect blobs.
    std::vector<KeyPoint> keypointsr;
    detector.detect( imgThresholdedr, keypointsr);

    Mat im_with_keypoints;
    drawKeypoints( resFrame, keypointsr, resFrame, Scalar(0,255,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
	cv::imshow("ori" , resFrame);

std::cout << " Keypoints: " << keypointsr.size() << std::endl;

        if (mouseLDown>1) {mouseLDown--;}
        if (mouseRDown>1) {mouseRDown--;}
        if (mouseMDown>1) {mouseMDown--;}




#if defined(HASSCREEN) || defined(VIDEORESULTS)		
#ifdef HASSCREEN

        cv::imshow("Results", imgThresholded);
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
    //cv::namedWindow("Results", CV_WINDOW_AUTOSIZE);
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


