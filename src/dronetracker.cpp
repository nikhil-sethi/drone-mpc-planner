#include <iostream>
#include "dronetracker.h"
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "common.h"

using namespace cv;
using namespace std;

#ifdef _PC
#define DRAWVIZSL
#define DRAWVIZSR
#define TUNING
#endif

const string settingsFile = "../settings.dat";
bool DroneTracker::init(void) {

    if (checkFileExist(settingsFile)) {
        std::ifstream is(settingsFile, std::ios::binary);
        cereal::BinaryInputArchive archive( is );
        archive(settings);
    }

#ifdef TUNING
    namedWindow("Tuning", WINDOW_NORMAL); //create a window called "Control"
    createTrackbar("LowH1", "Tuning", &settings.iLowH1r, 255);
    createTrackbar("HighH1", "Tuning", &settings.iHighH1r, 255);
    createTrackbar("filterByArea", "Tuning", &settings.filterByArea, 1);
    createTrackbar("minArea", "Tuning", &settings.minArea, 10000);
    createTrackbar("maxArea", "Tuning", &settings.maxArea, 10000);
    createTrackbar("Opening1", "Tuning", &settings.iOpen1r, 30);
    createTrackbar("Closing1", "Tuning", &settings.iClose1r, 30);
#endif

    kfL = cv::KalmanFilter(stateSize, measSize, contrSize, type);

    stateL =cv::Mat(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
    measL = cv::Mat (measSize, 1, type);    // [z_x,z_y,z_w,z_h]

    cv::setIdentity(kfL.transitionMatrix);
    kfL.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    kfL.measurementMatrix.at<float>(0) = 1.0f;
    kfL.measurementMatrix.at<float>(7) = 1.0f;
    kfL.measurementMatrix.at<float>(16) = 1.0f;
    kfL.measurementMatrix.at<float>(23) = 1.0f;

    kfL.processNoiseCov.at<float>(0) = 1e-2;
    kfL.processNoiseCov.at<float>(7) = 1e-2;
    kfL.processNoiseCov.at<float>(14) = 5.0f;
    kfL.processNoiseCov.at<float>(21) = 5.0f;
    kfL.processNoiseCov.at<float>(28) = 1e-2;
    kfL.processNoiseCov.at<float>(35) = 1e-2;

    // Measures Noise Covariance Matrix R
    cv::setIdentity(kfL.measurementNoiseCov, cv::Scalar(1e-1));


    kfR = cv::KalmanFilter(stateSize, measSize, contrSize, type);

    stateR =cv::Mat(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
    measR = cv::Mat (measSize, 1, type);    // [z_x,z_y,z_w,z_h]

    cv::setIdentity(kfR.transitionMatrix);
    kfR.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    kfR.measurementMatrix.at<float>(0) = 1.0f;
    kfR.measurementMatrix.at<float>(7) = 1.0f;
    kfR.measurementMatrix.at<float>(16) = 1.0f;
    kfR.measurementMatrix.at<float>(23) = 1.0f;

    kfR.processNoiseCov.at<float>(0) = 1e-2;
    kfR.processNoiseCov.at<float>(7) = 1e-2;
    kfR.processNoiseCov.at<float>(14) = 5.0f;
    kfR.processNoiseCov.at<float>(21) = 5.0f;
    kfR.processNoiseCov.at<float>(28) = 1e-2;
    kfR.processNoiseCov.at<float>(35) = 1e-2;

    // Measures Noise Covariance Matrix R
    cv::setIdentity(kfR.measurementNoiseCov, cv::Scalar(1e-1));


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

float calculateDistance(float xr, float yr, float xl, float yl) {
    return 1.0 /(xr - xl); // kaihard kebowned
}

cv::Mat prevFrameL,prevFrameR;
bool firstFrame;

std::vector<KeyPoint> dronepathL,dronepathR;
std::vector<KeyPoint> predicted_dronepathL,predicted_dronepathR;

void DroneTracker::track(cv::Mat frameL, cv::Mat frameR, cv::Mat Qf) {
    updateParams();

    static int t_prev = 0;
    int t = stopWatch.Read();
    float dt= (t-t_prev)/1000.0;
    t_prev = t;
    cv::Mat resFrameL,resFrameR;
#ifdef DRAWVIZSL    
    cv::resize(frameL,resFrameL,cv::Size(2*frameL.cols,frameL.rows*2));
//    resFrameL = frameL.clone();
#endif
#ifdef DRAWVIZSR
    cv::resize(frameR,resFrameR,cv::Size(2*frameR.cols,frameR.rows*2));
//    resFrameR = frameR.clone();
#endif

    cv::Mat tmpfL,tmpfR;
    tmpfL = cv::Mat(frameL.rows/2, frameL.cols/2,CV_8UC3);
    tmpfR = cv::Mat(frameR.rows/2, frameR.cols/2,CV_8UC3);
    cv::resize(frameL,tmpfL,tmpfL.size(),0,0);
    cv::resize(frameR,tmpfR,tmpfR.size(),0,0);

    cv::Mat framegrayL,framegrayR;
    cvtColor(tmpfL,framegrayL,COLOR_BGR2GRAY);
    cvtColor(tmpfR,framegrayR,COLOR_BGR2GRAY);

    if (!firstFrame) {
        firstFrame = true;
        prevFrameL = framegrayL.clone();
        prevFrameR = framegrayR.clone();
    }

    cv::Mat diffL = framegrayL - prevFrameL;
    cv::Mat diffR = framegrayR - prevFrameR;
    prevFrameL = framegrayL.clone();
    prevFrameR = framegrayR.clone();
    cv::Mat treshfL,treshfR;

    inRange(diffL, settings.iLowH1r, settings.iHighH1r, treshfL);
    dilate( treshfL, treshfL, getStructuringElement(MORPH_ELLIPSE, Size(settings.iClose1r+1, settings.iClose1r+1)));
    erode(treshfL, treshfL, getStructuringElement(MORPH_ELLIPSE, Size(settings.iOpen1r+1, settings.iOpen1r+1)));
    inRange(diffR, settings.iLowH1r, settings.iHighH1r, treshfR);
    dilate( treshfR, treshfR, getStructuringElement(MORPH_ELLIPSE, Size(settings.iClose1r+1, settings.iClose1r+1)));
    erode(treshfR, treshfR, getStructuringElement(MORPH_ELLIPSE, Size(settings.iOpen1r+1, settings.iOpen1r+1)));


    static bool foundL = false;
    static bool foundR = false;
    static int notFoundCountL =0;
    static int notFoundCountR =0;

    cv::Point3f predicted_drone_locationL;
    cv::Point predicted_drone_locationR;
    if (foundL) {
        kfL.transitionMatrix.at<float>(2) = dt;
        kfL.transitionMatrix.at<float>(9) = dt;
        stateL = kfL.predict();
        cv::Rect predRect;
        predRect.width = stateL.at<float>(4);
        predRect.height = stateL.at<float>(5);
        predRect.x = stateL.at<float>(0) - predRect.width / 2;
        predRect.y = stateL.at<float>(1) - predRect.height / 2;

        predicted_drone_locationL.x = stateL.at<float>(0);
        predicted_drone_locationL.y = stateL.at<float>(1);
        predicted_drone_locationL.z = stateL.at<float>(2);
        cv::KeyPoint t;
        cv::Point beun;
        beun.x = predicted_drone_locationL.x;
        beun.y = predicted_drone_locationL.y;
        t.pt = beun;
        t.size = 3;
        predicted_dronepathL.push_back(t);
       // cout << "PredictionL: " << predicted_drone_locationL << std::endl;
#ifdef DRAWVIZSL
        drawKeypoints( framegrayL, predicted_dronepathL, framegrayL, Scalar(0,255,0), DrawMatchesFlags::DEFAULT );
#endif

    } else {
#ifdef DRAWVIZSL
        cvtColor(framegrayL,framegrayL,CV_GRAY2BGR);
#endif
    }
    if (foundR) {
        kfR.transitionMatrix.at<float>(2) = dt;
        kfR.transitionMatrix.at<float>(9) = dt;
        stateR = kfR.predict();
        //        cv::Rect predRect;
        //        predRect.width = stateR.at<float>(4);
        //        predRect.height = stateR.at<float>(5);
        //        predRect.x = stateR.at<float>(0) - predRect.width / 2;
        //        predRect.y = stateR.at<float>(1) - predRect.height / 2;

        predicted_drone_locationR.x = stateR.at<float>(0);
        predicted_drone_locationR.y = stateR.at<float>(1);
        cv::KeyPoint t;
        t.pt = predicted_drone_locationR;
        t.size = 3;
        predicted_dronepathR.push_back(t);
        //cout << "PredictionR: " << predicted_drone_locationR << std::endl;
#ifdef DRAWVIZSR
        drawKeypoints( framegrayR, predicted_dronepathR, framegrayR, Scalar(0,255,0), DrawMatchesFlags::DEFAULT );
#endif

    } else {
#ifdef DRAWVIZSR
        cvtColor(framegrayR,framegrayR,CV_GRAY2BGR);
#endif
    }


    // Set up detector with params
    SimpleBlobDetector detector(params);
    // Detect blobs.
    std::vector<KeyPoint> keypointsL,keypointsR;
    detector.detect( treshfL, keypointsL);
    detector.detect( treshfR, keypointsR);

    cv::KeyPoint closestL,closestR;
    if (keypointsL.size() == 1 && dronepathL.size() == 0) {
        dronepathL.push_back(keypointsL.at(0));
    } else if (keypointsL.size()>0 &&  dronepathL.size() > 0) {
        //find closest keypoint to new predicted location
        int mind = 999999999;
        for (int i = 0 ; i < keypointsL.size();i++) {
            cv::KeyPoint k =keypointsL.at(i);
            int d = (predicted_drone_locationL.x-k.pt.x) * (predicted_drone_locationL.x-k.pt.x) + (predicted_drone_locationL.y-k.pt.y)*(predicted_drone_locationL.y-k.pt.y);
            if (d < mind ) {
                mind = d;
                closestL = keypointsL.at(i);
            }
        }

        //cout << "MeasuredL: " <<  closestL.pt << std::endl;

        dronepathL.push_back(closestL);

    }
    if (keypointsR.size() == 1 && dronepathR.size() == 0) {
        dronepathR.push_back(keypointsR.at(0));
    } else if (keypointsR.size()>0 &&  dronepathR.size() > 0) {
        //find closest keypoint to new predicted location
        int mind = 999999999;
        for (int i = 0 ; i < keypointsR.size();i++) {
            cv::KeyPoint k =keypointsR.at(i);
            int d = (predicted_drone_locationR.x-k.pt.x) * (predicted_drone_locationR.x-k.pt.x) + (predicted_drone_locationR.y-k.pt.y)*(predicted_drone_locationR.y-k.pt.y);
            if (d < mind ) {
                mind = d;
                closestR = keypointsR.at(i);
            }
        }

        //cout << "MeasuredR: " <<  closestR.pt << std::endl;

        //turbo beuntje
        closestR.pt.x = closestL.pt.x - (closestL.pt.x - closestR.pt.x);


        dronepathR.push_back(closestR);
    }

    if (keypointsL.size() == 0 || keypointsR.size() == 0) {
        if (keypointsL.size() == 0) {
            notFoundCountL++;
            //cout << "notFoundCountL:" << notFoundCountL << endl;
            if( notFoundCountL >= 100 )
                foundL = false;
        }

        if (keypointsR.size() == 0) {
            notFoundCountR++;
            //cout << "notFoundCountR:" << notFoundCountR << endl;
            if( notFoundCountR >= 100 )
                foundR = false;
        }

    } else {


        float dist = 1.0 / (closestL.pt.x -closestR.pt.x );

        notFoundCountL = 0;
        cv::Mat measL(measSize, 1, type);
        measL.at<float>(0) = closestL.pt.x;
        measL.at<float>(1) = closestL.pt.y;
        measL.at<float>(2) = dist;
        measL.at<float>(3) = 0;


        if (!foundL) { // First detection!
            kfL.errorCovPre.at<float>(0) = 1; // px
            kfL.errorCovPre.at<float>(7) = 1; // px
            kfL.errorCovPre.at<float>(14) = 1;
            kfL.errorCovPre.at<float>(21) = 1;
            kfL.errorCovPre.at<float>(28) = 1; // px
            kfL.errorCovPre.at<float>(35) = 1; // px

            stateL.at<float>(0) = measL.at<float>(0);
            stateL.at<float>(1) = measL.at<float>(1);
            stateL.at<float>(2) = 0;
            stateL.at<float>(3) = 0;
            stateL.at<float>(4) = measL.at<float>(2);
            stateL.at<float>(5) = measL.at<float>(3);

            kfL.statePost = stateL;

            foundL = true;
        }
        else
            kfL.correct(measL);


        notFoundCountR = 0;
        cv::Mat measR(measSize, 1, type);
        measR.at<float>(0) = closestR.pt.x;
        measR.at<float>(1) = closestR.pt.y;
        measR.at<float>(2) = 0;
        measR.at<float>(3) = 0;

        if (!foundR) { // First detection!
            kfR.errorCovPre.at<float>(0) = 1; // px
            kfR.errorCovPre.at<float>(7) = 1; // px
            kfR.errorCovPre.at<float>(14) = 1;
            kfR.errorCovPre.at<float>(21) = 1;
            kfR.errorCovPre.at<float>(28) = 1; // px
            kfR.errorCovPre.at<float>(35) = 1; // px

            stateR.at<float>(0) = measR.at<float>(0);
            stateR.at<float>(1) = measR.at<float>(1);
            stateR.at<float>(2) = 0;
            stateR.at<float>(3) = 0;
            stateR.at<float>(4) = measR.at<float>(2);
            stateR.at<float>(5) = measR.at<float>(3);

            kfR.statePost = stateR;

            foundR = true;
        }
        else
            kfR.correct(measR);

        //calculate everything for the dronecontroller:

        std::vector<Point3f> camera_coordinates;
        std::vector<Point3f> world_coordinates;
        camera_coordinates.push_back(Point3f(closestL.pt.x*4,closestL.pt.y*4,(closestL.pt.x - closestR.pt.x)*4));
        cv::perspectiveTransform(camera_coordinates,world_coordinates,Qf);

        Point3f output = world_coordinates[0];

        static float prevX,prevY,prevZ =0;
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


    }

#ifdef DRAWVIZSL
    cvtColor(treshfL,treshfL,CV_GRAY2BGR);
    treshfL.copyTo(resFrameL(cv::Rect(resFrameL.cols - treshfL.cols,0,treshfL.cols, treshfL.rows)));
    if (dronepathL.size() > 0) {
        drawKeypoints( framegrayL, dronepathL, framegrayL, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    }
    framegrayL.copyTo(resFrameL(cv::Rect(0,0,framegrayL.cols, framegrayL.rows)));
#endif
#ifdef DRAWVIZSR
    cvtColor(treshfR,treshfR,CV_GRAY2BGR);
    treshfR.copyTo(resFrameR(cv::Rect(resFrameR.cols - treshfR.cols,0,treshfR.cols, treshfR.rows)));
    if (dronepathR.size() > 0) {
        drawKeypoints( framegrayR, dronepathR, framegrayR, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    }
    framegrayR.copyTo(resFrameR(cv::Rect(0,0,framegrayR.cols, framegrayR.rows)));


    resFrame = cv::Mat(resFrameL.rows,resFrameL.cols +resFrameR.cols ,CV_8UC3);
    resFrameL.copyTo(resFrame(cv::Rect(0,0,resFrameL.cols, resFrameL.rows)));
    resFrameR.copyTo(resFrame(cv::Rect(resFrameL.cols,0,resFrameL.cols, resFrameL.rows)));
#endif

    if (dronepathL.size() > 30)
        dronepathL.erase(dronepathL.begin());
    if (predicted_dronepathL.size() > 30)
        predicted_dronepathL.erase(predicted_dronepathL.begin());

    if (dronepathR.size() > 30)
        dronepathR.erase(dronepathR.begin());
    if (predicted_dronepathR.size() > 30)
        predicted_dronepathR.erase(predicted_dronepathR.begin());

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
