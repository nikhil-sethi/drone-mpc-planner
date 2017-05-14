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
/*
    namedWindow("Thresh Blue", WINDOW_NORMAL); //create a window called "Control"
    createTrackbar("LowH1", "Thresh Blue", &settings.iLowH1b, 255);
    createTrackbar("HighH1", "Thresh Blue", &settings.iHighH1b, 255);
    createTrackbar("LowS1", "Thresh Blue", &settings.iLowS1b, 255);
    createTrackbar("HighS1", "Thresh Blue", &settings.iHighS1b, 255);
    createTrackbar("LowV1", "Thresh Blue", &settings.iLowV1b, 255);
    createTrackbar("HighV1", "Thresh Blue", &settings.iHighV1b, 255);
    createTrackbar("Opening1", "Thresh Blue", &settings.iOpen1b, 30);
    createTrackbar("Closing1", "Thresh Blue", &settings.iClose1b, 30);
*/
    namedWindow("Tuning", WINDOW_NORMAL); //create a window called "Control"
    createTrackbar("LowH1", "Tuning", &settings.iLowH1r, 255);
    createTrackbar("HighH1", "Tuning", &settings.iHighH1r, 255);
    createTrackbar("filterByArea", "Tuning", &settings.filterByArea, 1);
    createTrackbar("minArea", "Tuning", &settings.minArea, 10000);
    createTrackbar("maxArea", "Tuning", &settings.maxArea, 10000);
    createTrackbar("Opening1", "Tuning", &settings.iOpen1r, 30);
    createTrackbar("Closing1", "Tuning", &settings.iClose1r, 30);

    /*
    createTrackbar("LowS1", "Thresh Red", &settings.iLowS1r, 255);
    createTrackbar("HighS1", "Thresh Red", &settings.iHighS1r, 255);
    createTrackbar("LowV1", "Thresh Red", &settings.iLowV1r, 255);
    createTrackbar("HighV1", "Thresh Red", &settings.iHighV1r, 255);
    createTrackbar("Opening1", "Thresh Red", &settings.iOpen1r, 30);
    createTrackbar("Closing1", "Thresh Red", &settings.iClose1r, 30);
    */

    /*
    namedWindow("Blob Moeder", cv::WINDOW_NORMAL); //create a window called "Control"

    //the thresholds are not to be tuned
//    createTrackbar("minThreshold", "Blob Moeder", &settings.minThreshold, 255);
//    createTrackbar("maxThreshold", "Blob Moeder", &settings.maxThreshold, 255);

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
*/
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

    stopWatch.Start();

}
/*
float prevTime = 0;
float prevX,prevY,prevZ = 0;
void DroneTracker::track(cv::Mat frameL, cv::Mat frameR, cv::Mat Qf) {
    updateParams();

    cv::Mat resFrameL,resFrameR;
    resFrameL = cv::Mat(frameL.rows/4, frameL.cols/4,CV_8UC3);
    resFrameR = cv::Mat(frameR.rows/4, frameR.cols/4,CV_8UC3);
    cv::resize(frameL,resFrameL,resFrameL.size(),0,0);
    cv::resize(frameR,resFrameR,resFrameR.size(),0,0);

	cv::Rect myROI(resFrameL.cols/4,resFrameL.rows/2,resFrameL.cols/2,resFrameL.rows/2);
	cv::Mat croppedResFrameL = resFrameL(myROI);
	cv::Mat croppedResFrameR = resFrameR(myROI);

    Mat imgHSVL,imgHSVR;
    cvtColor(resFrameL, imgHSVL, COLOR_BGR2HSV);
    cvtColor(resFrameR, imgHSVR, COLOR_BGR2HSV);
    
    //cvtColor(croppedResFrameL, imgHSVL, COLOR_BGR2HSV);
    //cvtColor(croppedResFrameR, imgHSVR, COLOR_BGR2HSV);

    Mat imgTRedL,imgTBlueL,imgTRedR,imgTBlueR;
    inRange(imgHSVL, Scalar(settings.iLowH1r, settings.iLowS1r, settings.iLowV1r), Scalar(settings.iHighH1r, settings.iHighS1r, settings.iHighV1r), imgTRedL); //Threshold the image
    inRange(imgHSVL, Scalar(settings.iLowH1b, settings.iLowS1b, settings.iLowV1b), Scalar(settings.iHighH1b, settings.iHighS1b, settings.iHighV1b), imgTBlueL);
    inRange(imgHSVR, Scalar(settings.iLowH1r, settings.iLowS1r, settings.iLowV1r), Scalar(settings.iHighH1r, settings.iHighS1r, settings.iHighV1r), imgTRedR);
    inRange(imgHSVR, Scalar(settings.iLowH1b, settings.iLowS1b, settings.iLowV1b), Scalar(settings.iHighH1b, settings.iHighS1b, settings.iHighV1b), imgTBlueR);

    //erode(imgTRedL, imgTRedL, getStructuringElement(MORPH_ELLIPSE, Size(settings.iOpen1r+1, settings.iOpen1r+1)));
    dilate( imgTRedL, imgTRedL, getStructuringElement(MORPH_ELLIPSE, Size(settings.iClose1r+1, settings.iClose1r+1)));
    //erode(imgTBlueL, imgTBlueL, getStructuringElement(MORPH_ELLIPSE, Size(settings.iOpen1b+1, settings.iOpen1b+1)));
    dilate( imgTBlueL, imgTBlueL, getStructuringElement(MORPH_ELLIPSE, Size(settings.iClose1b+1, settings.iClose1b+1)));

    //erode(imgTRedR, imgTRedR, getStructuringElement(MORPH_ELLIPSE, Size(settings.iOpen1r+1, settings.iOpen1r+1)));
    dilate( imgTRedR, imgTRedR, getStructuringElement(MORPH_ELLIPSE, Size(settings.iClose1r+1, settings.iClose1r+1)));
    //erode(imgTBlueR, imgTBlueR, getStructuringElement(MORPH_ELLIPSE, Size(settings.iOpen1b+1, settings.iOpen1b+1)));
    dilate( imgTBlueR, imgTBlueR, getStructuringElement(MORPH_ELLIPSE, Size(settings.iClose1b+1, settings.iClose1b+1)));

    // Set up detector with params
    SimpleBlobDetector detector(params);

    // Detect blobs.
    std::vector<KeyPoint> keypRedL,keypBlueL, keypRedR,keypBlueR;
    detector.detect( imgTRedL, keypRedL);
    detector.detect( imgTBlueL, keypBlueL);
    detector.detect( imgTRedR, keypRedR);
    detector.detect( imgTBlueR, keypBlueR);

    cv::Mat frameLBlob = cv::Mat::zeros(resFrameL.cols,resFrameL.rows,CV_8UC3);
    cv::Mat frameRBlob = cv::Mat::zeros(resFrameR.cols,resFrameR.rows,CV_8UC3);
    drawKeypoints( frameLBlob, keypRedL, frameLBlob, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    drawKeypoints( frameLBlob, keypBlueL, frameLBlob, Scalar(255,0,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    drawKeypoints( frameRBlob, keypRedR, frameRBlob, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    drawKeypoints( frameRBlob, keypBlueR, frameRBlob, Scalar(255,0,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

#ifdef TUNING
    cv::Mat greenDummy = cv::Mat::zeros(imgTBlueL.rows,imgTBlueL.cols,CV_8UC1);
    std::vector<cv::Mat> channelsTL;
    channelsTL.push_back(imgTBlueL);
    channelsTL.push_back(greenDummy);
    channelsTL.push_back(imgTRedL);
    cv::Mat frameTrgbL;
    cv::merge(channelsTL,frameTrgbL);
    std::vector<cv::Mat> channelsTR;
    channelsTR.push_back(imgTBlueL);
    channelsTR.push_back(greenDummy);
    channelsTR.push_back(imgTRedL);
    cv::Mat frameTrgbR;
    cv::merge(channelsTR,frameTrgbR);
    cv::Mat frameCT;
    combineImage(frameTrgbL,frameTrgbR,&frameCT);
    cv::imshow("Thresh", frameCT);

    cv::Mat frameCB;
    combineImage(frameLBlob,frameRBlob,&frameCB);
    cv::imshow("Blob", frameCB);
#endif

    combineImage(resFrameL,resFrameR,&resFrame);
/**
    std::cout << "Red: " << keypRedL.size() << ", " << keypRedR.size() << ", blue: " << keypBlueL.size() << ", " << keypBlueR.size()  << std::endl;
    //calculate blob disparity

    if (keypRedL.size() > 0 && keypRedR.size() > 0)
        std::cout << "KeyRed: " << keypRedL[0].pt.x - keypRedR[0].pt.x << std::endl;
    if (keypBlueL.size() > 0 && keypBlueR.size() > 0)
        std::cout << "KeyBlue: " << keypBlueL[0].pt.x - keypBlueR[0].pt.x << std::endl;
* /

    float time = ((float)stopWatch.Read())/1000.0;
    float dt = time - prevTime;
    prevTime = time;

if (keypRedL.size() > 0 && keypRedR.size() > 0) {
/*
    data.posX = keypRedL[0].pt.x;
    data.posY = keypRedL[0].pt.y;
    data.posZ = 1.0f / (keypRedL[0].pt.x -keypRedR[0].pt.x);
    data.dx = data.posX - prevX;
    data.dy = data.posY - prevY;
    data.dy = data.posZ - prevZ;
    data.velX = data.dx / dt;
    data.velY = data.dy / dt;
    data.velZ = data.dz / dt;
    data.dt = dt;
* /
    
	
    if (keypRedL.size() == 1) {	
	std::vector<Point3f> camera_coordinates;   
    	std::vector<Point3f> world_coordinates;
    	camera_coordinates.push_back(Point3f(keypRedL[0].pt.x*4,keypRedL[0].pt.y*4,(keypRedL[0].pt.x - keypRedR[0].pt.x)*4));
    	cv::perspectiveTransform(camera_coordinates,world_coordinates,Qf);

	Point3f output = world_coordinates[0];
	//std::cout << "XYZ: " << data.posX << " " << data.posY << " " << keypRedL[0].pt.x -keypRedR[0].pt.x << std::endl;
	//std::cout << "Point3: " << output.x << " " << output.y << " " << output.z << std::endl;
	
	//std::cout << Qf.at<double>(0,0)<< " " << Qf.at<double>(0,1) << " " << Qf.at<double>(0,2) << " " << Qf.at<double>(0,3) << std::endl;
	//std::cout << Qf.at<double>(1,0)<< " " << Qf.at<double>(1,1) << " " << Qf.at<double>(1,2) << " " << Qf.at<double>(1,3) << std::endl;
	//std::cout << Qf.at<double>(2,0)<< " " << Qf.at<double>(2,1) << " " << Qf.at<double>(0,2) << " " << Qf.at<double>(2,3) << std::endl;
	//std::cout << Qf.at<double>(3,0)<< " " << Qf.at<double>(3,1) << " " << Qf.at<double>(3,2) << " " << Qf.at<double>(3,3) << std::endl;
	
	data.posX = output.x;
    	data.posY = -output.y;
    	data.posZ = output.z;
    	data.dx = data.posX - prevX;
    	data.dy = data.posY - prevY;
    	data.dy = data.posZ - prevZ;
    	data.velX = data.dx / dt;
    	data.velY = data.dy / dt;
    	data.velZ = data.dz / dt;
    	data.dt = dt;
        data.valid = true;

	prevX = data.posX;
	prevY = data.posY;
	prevZ = data.posZ;
    } else
	data.valid = false;
}

}
*/

cv::Mat prevFrame;
bool firstFrame;
cv::Mat backf;

std::vector<KeyPoint> dronepath;

void DroneTracker::track(cv::Mat frameL, cv::Mat frameR, cv::Mat Qf) {
    updateParams();
    resFrame = frameL.clone();


    cv::Mat tmpf;
    tmpf = cv::Mat(frameL.rows/4, frameL.cols/4,CV_8UC3);
    cv::resize(frameL,tmpf,tmpf.size(),0,0);
    cv::Mat framegray;
    cvtColor(tmpf,framegray,COLOR_BGR2GRAY);


    if (!firstFrame) {
        firstFrame = true;
        prevFrame = framegray.clone();
        backf = framegray.clone();
    }


    cv::Mat diff = framegray - prevFrame;
    prevFrame = framegray.clone();

    cv::Mat treshf;
    inRange(diff, settings.iLowH1r, settings.iHighH1r, treshf);


    dilate( treshf, treshf, getStructuringElement(MORPH_ELLIPSE, Size(settings.iClose1r+1, settings.iClose1r+1)));
    erode(treshf, treshf, getStructuringElement(MORPH_ELLIPSE, Size(settings.iOpen1r+1, settings.iOpen1r+1)));



    // Set up detector with params
    SimpleBlobDetector detector(params);
    // Detect blobs.
    std::vector<KeyPoint> keypoints;
    detector.detect( treshf, keypoints);


    if (keypoints.size() == 1 && dronepath.size() == 0) {
        dronepath.push_back(keypoints.at(0));
    } else if (keypoints.size()>0 &&  dronepath.size() > 0) {
        //find closest keypoint to last previously known location
        int mind = 999999999;
        cv::KeyPoint p = dronepath.at(dronepath.size()-1);
        cv::KeyPoint closest;
        for (int i = 0 ; i < keypoints.size();i++) {
            cv::KeyPoint k =keypoints.at(i);
            int d = (p.pt.x-k.pt.x) * (p.pt.x-k.pt.x) + (p.pt.y-k.pt.y)*(p.pt.y-k.pt.y);
            if (d < mind ) {
                mind = d;
                closest = keypoints.at(i);
            }

        }

        dronepath.push_back(closest);

    }




    if (dronepath.size() > 0) {
        drawKeypoints( framegray, dronepath, framegray, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        framegray.copyTo(resFrame(cv::Rect(0,0,framegray.cols, framegray.rows)));
    }

    if (dronepath.size() > 30)
        dronepath.erase(dronepath.begin());



    //imshow("FrameL",frameL);



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
