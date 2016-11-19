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
#ifdef _PC
KalamosFileCam cam;
#else
KalamosCam cam;
#endif
stereoAlg stereo;

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
int main( int argc, char **argv);
void CallBackFunc(int event, int x, int y, int flags, void* userdata);
void handleKey();

/************ code ***********/

void process_video() {
    std::cout << "Running...\n";
    stopWatch.Start();

    //#define TUNING
#ifdef TUNING

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

#endif

    // Setup SimpleBlobDetector parameters.
    SimpleBlobDetector::Params params;
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

    // Filter by Convexity
    params.filterByConvexity = filterByConvexity;
    params.minConvexity = ((float)minConvexity)/100.0f;
    params.maxConvexity = ((float)maxConvexity)/100.0f;

    // Filter by Inertia
    params.filterByInertia = filterByInertia;
    params.minInertiaRatio = ((float)minInertiaRatio)/100.0f;
    params.maxInertiaRatio = ((float)maxInertiaRatio)/100.0f;

    params.minRepeatability = 0;
    params.minDistBetweenBlobs=0;
    params.filterByColor = 0;
    params.thresholdStep=1;

    //main while loop:
    while (key != 27 && cam.getCamRunning()) // ESC
    {

#ifdef TUNING
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

        // Filter by Convexity
        params.filterByConvexity = filterByConvexity;
        params.minConvexity = ((float)minConvexity)/100.0f;
        params.maxConvexity = ((float)maxConvexity)/100.0f;

        // Filter by Inertia
        params.filterByInertia = filterByInertia;
        params.minInertiaRatio = ((float)minInertiaRatio)/100.0f;
        params.maxInertiaRatio = ((float)maxInertiaRatio)/100.0f;

        params.minRepeatability = 0;
        params.minDistBetweenBlobs=0;
        params.filterByColor = 0;
        params.thresholdStep=1;
#endif

        cam.waitForImage();
        //stereo.rectify(cam.frameL, cam.frameR);

        cv::Mat resFrameL,resFrameR;
        resFrameL = cv::Mat(cam.frameL.rows/4, cam.frameL.cols/4,CV_8UC3);
        resFrameR = cv::Mat(cam.frameR.rows/4, cam.frameR.cols/4,CV_8UC3);
        cv::resize(cam.frameL,resFrameL,resFrameL.size(),0,0);
        cv::resize(cam.frameR,resFrameR,resFrameR.size(),0,0);

        Mat imgHSVL,imgHSVR;
        cvtColor(resFrameL, imgHSVL, COLOR_BGR2HSV);
        cvtColor(resFrameR, imgHSVR, COLOR_BGR2HSV);

        Mat imgTRedL,imgTBlueL,imgTRedR,imgTBlueR;
        inRange(imgHSVL, Scalar(iLowH1r, iLowS1r, iLowV1r), Scalar(iHighH1r, iHighS1r, iHighV1r), imgTRedL); //Threshold the image
        inRange(imgHSVL, Scalar(iLowH1b, iLowS1b, iLowV1b), Scalar(iHighH1b, iHighS1b, iHighV1b), imgTBlueL);
        inRange(imgHSVR, Scalar(iLowH1r, iLowS1r, iLowV1r), Scalar(iHighH1r, iHighS1r, iHighV1r), imgTRedR);
        inRange(imgHSVR, Scalar(iLowH1b, iLowS1b, iLowV1b), Scalar(iHighH1b, iHighS1b, iHighV1b), imgTBlueR);

        // Set up detector with params
        SimpleBlobDetector detector(params);

        // Detect blobs.
        std::vector<KeyPoint> keypRedL,keypBlueL, keypRedR,keypBlueR;
        detector.detect( imgTRedL, keypRedL);
        detector.detect( imgTBlueL, keypBlueL);
        detector.detect( imgTRedR, keypRedR);
        detector.detect( imgTBlueR, keypBlueR);

        Mat im_with_keypoints;
        drawKeypoints( resFrameL, keypRedL, resFrame, Scalar(255,255,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        drawKeypoints( resFrameL, keypBlueL, resFrame, Scalar(0,255,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        drawKeypoints( resFrameR, keypRedR, resFrame, Scalar(255,255,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        drawKeypoints( resFrameR, keypBlueR, resFrame, Scalar(0,255,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );


        //tmp
        stereo.combineImage(resFrameL,resFrameR);
        resFrame = stereo.frameC;

        std::cout << "Red: " << keypRedL.size() << ", " << keypRedR.size() << ", blue: " << keypBlueL.size() << ", " << keypBlueR.size()  << std::endl;
        //calculate blob disparity
        if (keypRedL.size() > 0 && keypRedR.size() > 0)
            std::cout << "KeyRed: " << keypRedL[0].pt.x - keypRedR[0].pt.x << std::endl;
        if (keypBlueL.size() > 0 && keypBlueR.size() > 0)
            std::cout << "KeyBlue: " << keypBlueL[0].pt.x - keypBlueR[0].pt.x << std::endl;

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


