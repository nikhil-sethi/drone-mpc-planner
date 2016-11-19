#include <iostream>
#include "dronetracker.h"
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "common.h"

using namespace cv;
using namespace std;

const string settingsFile = "../settings.dat";
bool DroneTracker::init(void) {


    if (checkFileExist(settingsFile)) {
        std::ifstream is(settingsFile, std::ios::binary);
        cereal::BinaryInputArchive archive( is );
        archive(settings);
    }

    #define TUNING
#ifdef TUNING

    namedWindow("Thresh Moeder1", WINDOW_NORMAL); //create a window called "Control"
    createTrackbar("LowH1", "Thresh Moeder1", &settings.iLowH1b, 255);
    createTrackbar("HighH1", "Thresh Moeder1", &settings.iHighH1b, 255);
    createTrackbar("LowS1", "Thresh Moeder1", &settings.iLowS1b, 255);
    createTrackbar("HighS1", "Thresh Moeder1", &settings.iHighS1b, 255);
    createTrackbar("LowV1", "Thresh Moeder1", &settings.iLowV1b, 255);
    createTrackbar("HighV1", "Thresh Moeder1", &settings.iHighV1b, 255);
    createTrackbar("Opening1", "Thresh Moeder1", &settings.iOpen1b, 30);
    createTrackbar("Closing1", "Thresh Moeder1", &settings.iClose1b, 30);


    namedWindow("Blob Moeder", cv::WINDOW_NORMAL); //create a window called "Control"

    createTrackbar("minThreshold", "Blob Moeder", &settings.minThreshold, 255);
    createTrackbar("maxThreshold", "Blob Moeder", &settings.maxThreshold, 255);

    createTrackbar("filterByArea", "Blob Moeder", &settings.filterByArea, 1);
    createTrackbar("minArea", "Blob Moeder", &settings.minArea, 10000);
    createTrackbar("maxArea", "Blob Moeder", &settings.maxArea, 10000);


    createTrackbar("filterByCircularity", "Blob Moeder", &settings.filterByCircularity, 1);
    createTrackbar("minCircularity", "Blob Moeder", &settings.minCircularity, 100);
    createTrackbar("maxCircularity", "Blob Moeder", &settings.maxCircularity, 100);

    createTrackbar("filterByConvexity", "Blob Moeder", &settings.filterByConvexity, 1);
    createTrackbar("minConvexity", "Blob Moeder", &settings.minConvexity, 100);
    createTrackbar("maxConvexity", "Blob Moeder", &settings.maxConvexity, 100);


    createTrackbar("filterByInertia", "Blob Moeder", &settings.filterByInertia, 1);
    createTrackbar("minInertiaRatio", "Blob Moeder", &settings.minInertiaRatio, 100);
    createTrackbar("maxInertiaRatio", "Blob Moeder", &settings.maxInertiaRatio, 100);

#endif

    // Setup SimpleBlobDetector parameters.
    params.minThreshold = settings.minThreshold+1;
    params.maxThreshold = settings.maxThreshold+1;

    // Filter by Area.
    params.filterByArea = settings.filterByArea;
    params.minArea = settings.minArea+1;
    params.maxArea = settings.maxArea+1;

    // Filter by Circularity
    params.filterByCircularity = settings.filterByCircularity;
    params.minCircularity = ((float)settings.minCircularity)/100.0f;
    params.maxCircularity = ((float)settings.maxCircularity)/100.0f;

    // Filter by Convexity
    params.filterByConvexity = settings.filterByConvexity;
    params.minConvexity = ((float)settings.minConvexity)/100.0f;
    params.maxConvexity = ((float)settings.maxConvexity)/100.0f;

    // Filter by Inertia
    params.filterByInertia = settings.filterByInertia;
    params.minInertiaRatio = ((float)settings.minInertiaRatio)/100.0f;
    params.maxInertiaRatio = ((float)settings.maxInertiaRatio)/100.0f;

    params.minRepeatability = 0;
    params.minDistBetweenBlobs=0;
    params.filterByColor = 0;
    params.thresholdStep=1;

}

void DroneTracker::track(cv::Mat frameL, cv::Mat frameR) {
    updateParams();

    cv::Mat resFrameL,resFrameR;
    resFrameL = cv::Mat(frameL.rows/4, frameL.cols/4,CV_8UC3);
    resFrameR = cv::Mat(frameR.rows/4, frameR.cols/4,CV_8UC3);
    cv::resize(frameL,resFrameL,resFrameL.size(),0,0);
    cv::resize(frameR,resFrameR,resFrameR.size(),0,0);

    Mat imgHSVL,imgHSVR;
    cvtColor(resFrameL, imgHSVL, COLOR_BGR2HSV);
    cvtColor(resFrameR, imgHSVR, COLOR_BGR2HSV);

    Mat imgTRedL,imgTBlueL,imgTRedR,imgTBlueR;
    inRange(imgHSVL, Scalar(settings.iLowH1r, settings.iLowS1r, settings.iLowV1r), Scalar(settings.iHighH1r, settings.iHighS1r, settings.iHighV1r), imgTRedL); //Threshold the image
    inRange(imgHSVL, Scalar(settings.iLowH1b, settings.iLowS1b, settings.iLowV1b), Scalar(settings.iHighH1b, settings.iHighS1b, settings.iHighV1b), imgTBlueL);
    inRange(imgHSVR, Scalar(settings.iLowH1r, settings.iLowS1r, settings.iLowV1r), Scalar(settings.iHighH1r, settings.iHighS1r, settings.iHighV1r), imgTRedR);
    inRange(imgHSVR, Scalar(settings.iLowH1b, settings.iLowS1b, settings.iLowV1b), Scalar(settings.iHighH1b, settings.iHighS1b, settings.iHighV1b), imgTBlueR);

    // Set up detector with params
    SimpleBlobDetector detector(params);

    // Detect blobs.
    std::vector<KeyPoint> keypRedL,keypBlueL, keypRedR,keypBlueR;
    detector.detect( imgTRedL, keypRedL);
    detector.detect( imgTBlueL, keypBlueL);
    detector.detect( imgTRedR, keypRedR);
    detector.detect( imgTBlueR, keypBlueR);

    drawKeypoints( resFrameL, keypRedL, resFrame, Scalar(255,255,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    drawKeypoints( resFrameL, keypBlueL, resFrame, Scalar(0,255,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    drawKeypoints( resFrameR, keypRedR, resFrame, Scalar(255,255,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    drawKeypoints( resFrameR, keypBlueR, resFrame, Scalar(0,255,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );


    //tmp
    cv::Mat frameC;
    combineImage(frameL,frameR,&frameC);
    cv::imshow("In", frameC);    
    combineImage(resFrameL,resFrameR,&resFrame);

    std::cout << "Red: " << keypRedL.size() << ", " << keypRedR.size() << ", blue: " << keypBlueL.size() << ", " << keypBlueR.size()  << std::endl;
    //calculate blob disparity
    if (keypRedL.size() > 0 && keypRedR.size() > 0)
        std::cout << "KeyRed: " << keypRedL[0].pt.x - keypRedR[0].pt.x << std::endl;
    if (keypBlueL.size() > 0 && keypBlueR.size() > 0)
        std::cout << "KeyBlue: " << keypBlueL[0].pt.x - keypBlueR[0].pt.x << std::endl;

}

void DroneTracker::updateParams(){
#ifdef TUNING
        // Change thresholds
        params.minThreshold = settings.minThreshold+1;
        params.maxThreshold = settings.maxThreshold+1;

        // Filter by Area.
        params.filterByArea = settings.filterByArea;
        params.minArea = settings.minArea+1;
        params.maxArea = settings.maxArea+1;

        // Filter by Circularity
        params.filterByCircularity = settings.filterByCircularity;
        params.minCircularity = ((float)settings.minCircularity)/100.0f;
        params.maxCircularity = ((float)settings.maxCircularity)/100.0f;

        // Filter by Convexity
        params.filterByConvexity = settings.filterByConvexity;
        params.minConvexity = ((float)settings.minConvexity)/100.0f;
        params.maxConvexity = ((float)settings.maxConvexity)/100.0f;

        // Filter by Inertia
        params.filterByInertia = settings.filterByInertia;
        params.minInertiaRatio = ((float)settings.minInertiaRatio)/100.0f;
        params.maxInertiaRatio = ((float)settings.maxInertiaRatio)/100.0f;

        params.minRepeatability = 0;
        params.minDistBetweenBlobs=0;
        params.filterByColor = 0;
        params.thresholdStep=1;
#endif
}

void DroneTracker::close () {
    std::ofstream os(settingsFile, std::ios::binary);
     cereal::BinaryOutputArchive archive( os );
     archive( settings );
}
