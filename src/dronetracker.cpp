#include <iostream>
#include "dronetracker.h"
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "common.h"

using namespace cv;

bool DroneTracker::init(void) {

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

    drawKeypoints( resFrameL, keypRedL, resFrame, Scalar(255,255,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    drawKeypoints( resFrameL, keypBlueL, resFrame, Scalar(0,255,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    drawKeypoints( resFrameR, keypRedR, resFrame, Scalar(255,255,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    drawKeypoints( resFrameR, keypBlueR, resFrame, Scalar(0,255,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );


    cv::imshow("L", resFrameL);
    cv::imshow("R", resFrameR);

    //tmp
    combineImage(resFrameL,resFrameR,resFrame);

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
}

void DroneTracker::close () {

}
