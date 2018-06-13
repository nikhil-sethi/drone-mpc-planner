#include <iostream>
#include "insect.h"
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "common.h"

using namespace cv;
using namespace std;

//#define USERIGHTCAM

#ifdef HASSCREEN
#define DRAWVIZSL
#ifdef USERIGHTCAM
#define DRAWVIZSR
#endif
//#define TUNING
#endif


const string settingsFile = "../settings2.dat";
bool Insect::init(std::ofstream *logger, Arduino * arduino) {
    _logger = logger;
    _arduino = arduino;
    (*_logger) << "insect_imLx; insect_imLy; insect_imRx; insect_imRy;";
    if (checkFileExist(settingsFile)) {
        std::ifstream is(settingsFile, std::ios::binary);
        cereal::BinaryInputArchive archive( is );
        archive(settings);
    }
    resFrame = cv::Mat::zeros(480, 1280,CV_8UC3);

#ifdef TUNING
    namedWindow("Tuning", WINDOW_NORMAL);
    createTrackbar("LowH1", "Tuning", &settings.iLowH1r, 255);
    createTrackbar("HighH1", "Tuning", &settings.iHighH1r, 255);
    createTrackbar("filterByArea", "Tuning", &settings.filterByArea, 1);
    createTrackbar("minArea", "Tuning", &settings.minArea, 10000);
    createTrackbar("maxArea", "Tuning", &settings.maxArea, 10000);
    createTrackbar("Opening1", "Tuning", &settings.iOpen1r, 30);
    createTrackbar("Closing1", "Tuning", &settings.iClose1r, 30);
    createTrackbar("LED", "Tuning", &(_arduino->ledpower), 255);
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

    sposX.init(5);
    sposY.init(5);
    sposZ.init(5);

}

void Insect::track(cv::Mat frameL, cv::Mat frameR, cv::Mat Qf) {
    updateParams();
    // Set up detector with params
#if CV_MAJOR_VERSION==3
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
#else
    SimpleBlobDetector * detector;
    detector = *SimpleBlobDetector(params);
#endif


    bool found_keypoints_in_bothLR = false;

    static int t_prev = 0;
    int t = stopWatch.Read();
    float dt= (t-t_prev)/1000.0;

    cv::Mat framegrayL;
    cv::Size smalsize(frameL.rows/IMSCALEF, frameL.cols/IMSCALEF);
    cv::resize(frameL,framegrayL,smalsize);
    //cvtColor(tmpfL,framegrayL,COLOR_BGR2GRAY);

    if (!firstFrame) {
        prevFrameL = framegrayL.clone();
    }

    cv::Mat treshfL;
    cv::Mat diffL = framegrayL - prevFrameL;
    prevFrameL = framegrayL.clone();
    inRange(diffL, settings.iLowH1r, settings.iHighH1r, treshfL);
    dilate( treshfL, treshfL, getStructuringElement(MORPH_ELLIPSE, Size(settings.iClose1r+1, settings.iClose1r+1)));
    erode(treshfL, treshfL, getStructuringElement(MORPH_ELLIPSE, Size(settings.iOpen1r+1, settings.iOpen1r+1)));
    static bool foundL = false;
    static int notFoundCountL =0;
    cv::Point3f predicted_insect_locationL;

    if (foundL) {
        kfL.transitionMatrix.at<float>(2) = dt;
        kfL.transitionMatrix.at<float>(9) = dt;
        stateL = kfL.predict();
        cv::Rect predRect;
        predRect.width = stateL.at<float>(4);
        predRect.height = stateL.at<float>(5);
        predRect.x = stateL.at<float>(0) - predRect.width / 2;
        predRect.y = stateL.at<float>(1) - predRect.height / 2;

        predicted_insect_locationL.x = stateL.at<float>(0);
        predicted_insect_locationL.y = stateL.at<float>(1);
        predicted_insect_locationL.z = stateL.at<float>(2);
        cv::KeyPoint t;
        cv::Point beun;
        beun.x = predicted_insect_locationL.x;
        beun.y = predicted_insect_locationL.y;
        t.pt = beun;
        t.size = 3;
        predicted_insect_pathL.push_back(t);
    }
    std::vector<KeyPoint> keypointsL;
    cv::KeyPoint closestL,closestR;
    detector->detect( treshfL, keypointsL);
    if (keypointsL.size() == 1 && insect_pathL.size() == 0) {
        insect_pathL.push_back(keypointsL.at(0));
    } else if (keypointsL.size()>0 &&  insect_pathL.size() > 0) {
        //find closest keypoint to new predicted location
        int mind = 999999999;
        for (int i = 0 ; i < keypointsL.size();i++) {
            cv::KeyPoint k =keypointsL.at(i);
            int d = (predicted_insect_locationL.x-k.pt.x) * (predicted_insect_locationL.x-k.pt.x) + (predicted_insect_locationL.y-k.pt.y)*(predicted_insect_locationL.y-k.pt.y);
            if (d < mind ) {
                mind = d;
                closestL = keypointsL.at(i);
            }
        }
        insect_pathL.push_back(closestL);
    }
    if (keypointsL.size() == 0) {
        notFoundCountL++;
        //cout << "notFoundCountL:" << notFoundCountL << endl;
        if( notFoundCountL >= 10 )
            foundL = false;
    } else {
        found_keypoints_in_bothLR = true;
        notFoundCountL = 0;
    }

    float dist = 0;
    float disparity =0;
#ifdef USERIGHTCAM
    cv::Mat tmpfR,framegrayR;
    if (!firstFrame) {
        prevFrameR = framegrayR.clone();
    }
    cv::resize(frameR,tmpfR,smalsize);
    cvtColor(tmpfR,framegrayR,COLOR_BGR2GRAY);


    //cv::KeyPoint closestR;
    cv::Mat treshfR;
    if (found_keypoints_in_bothLR) {
        std::vector<KeyPoint> keypointsR;
        cv::Mat diffR = framegrayR - prevFrameR;
        prevFrameR = framegrayR.clone();
        inRange(diffR, settings.iLowH1r, settings.iHighH1r, treshfR);
        dilate( treshfR, treshfR, getStructuringElement(MORPH_ELLIPSE, Size(settings.iClose1r+1, settings.iClose1r+1)));
        erode(treshfR, treshfR, getStructuringElement(MORPH_ELLIPSE, Size(settings.iOpen1r+1, settings.iOpen1r+1)));
        static bool foundR = false;
        static int notFoundCountR =0;
        cv::Point predicted_insect_locationR;
        if (foundR) {
            kfR.transitionMatrix.at<float>(2) = dt;
            kfR.transitionMatrix.at<float>(9) = dt;
            stateR = kfR.predict();
            //        cv::Rect predRect;
            //        predRect.width = stateR.at<float>(4);
            //        predRect.height = stateR.at<float>(5);
            //        predRect.x = stateR.at<float>(0) - predRect.width / 2;
            //        predRect.y = stateR.at<float>(1) - predRect.height / 2;

            predicted_insect_locationR.x = stateR.at<float>(0);
            predicted_insect_locationR.y = stateR.at<float>(1);
            cv::KeyPoint t;
            t.pt = predicted_insect_locationR;
            t.size = 3;
            predicted_insect_pathR.push_back(t);
            //cout << "PredictionR: " << predicted_insect_locationR << std::endl;
        }
        detector.detect( treshfR, keypointsR);

        if (keypointsR.size() == 1 && insect_pathR.size() == 0) {
            insect_pathR.push_back(keypointsR.at(0));
        } else if (keypointsR.size()>0 &&  insect_pathR.size() > 0) {
            //find closest keypoint to new predicted location
            int mind = 999999999;
            for (int i = 0 ; i < keypointsR.size();i++) {
                cv::KeyPoint k =keypointsR.at(i);
                int d = (predicted_insect_locationR.x-k.pt.x) * (predicted_insect_locationR.x-k.pt.x) + (predicted_insect_locationR.y-k.pt.y)*(predicted_insect_locationR.y-k.pt.y);
                if (d < mind ) {
                    mind = d;
                    closestR = keypointsR.at(i);
                }
            }

            //turbo beuntje
            closestR.pt.x = closestL.pt.x - (closestL.pt.x - closestR.pt.x);
            insect_pathR.push_back(closestR);
        }
        if (keypointsR.size() == 0) {
            notFoundCountR++;
            if( notFoundCountR >= 10 )
                foundR = false;
            found_keypoints_in_bothLR = false;
        } else
            notFoundCountR = 0;

        if (found_keypoints_in_bothLR) {
            dist = 1.0 / (closestL.pt.x -closestR.pt.x );
            disparity = ((closestR.pt.x - closestL.pt.x)*IMSCALEF);

            cv::Mat measR(measSize, 1, type);
            measR.at<float>(0) = closestR.pt.x;
            measR.at<float>(1) = closestR.pt.y;
            measR.at<float>(2) = dist;
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

        }
    } else
        found_keypoints_in_bothLR = false;
#endif
    data.valid = false;
    if (found_keypoints_in_bothLR)  {

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


        // attention: original image (1280x960) --> IMSCALEF factor 2 --> 640x480 ( = ook video size)
        // plaatje 640x480 --> ROI --> 432x432 (864/IMSCALEF)
        // 432x432 --> rectify --> 432x432 (met zwarte randen eventueel)(op deze size is Qf gebaseerd)
        // de track functie begint hierboven weer met nog een resize factor IMSCALEF --> 216x216

        //calculate everything for the dronecontroller:
        std::vector<Point3f> camera_coordinates;
        std::vector<Point3f> world_coordinates;

        camera_coordinates.push_back(Point3f(closestL.pt.x*IMSCALEF,closestL.pt.y*IMSCALEF,disparity));
        cv::perspectiveTransform(camera_coordinates,world_coordinates,Qf);

        Point3f output = world_coordinates[0];
        data.posX = output.x;
        data.posY = output.y;
        data.posZ = output.z;

        float csposX = sposX.addSample(data.posX);
        float csposY = sposY.addSample(data.posY);
        float csposZ = sposZ.addSample(data.posZ);

        static float prevX,prevY,prevZ =0;

        data.dx = csposX - prevX;
        data.dy = csposY - prevY;
        data.dz = csposZ - prevZ;
        data.velX = data.dx / dt;
        data.velY = data.dy / dt;
        data.velZ = data.dz / dt;
        data.dt = dt;
        t_prev = t; // update dt only if data valid
        data.valid = true;

        prevX = csposX;
        prevY = csposY;
        prevZ = csposZ;

#ifdef DRAWVIZSR
        cv::Mat resFrameR;
        drawKeypoints( framegrayR, predicted_insect_pathR, framegrayR, Scalar(0,255,0), DrawMatchesFlags::DEFAULT );
        cv::resize(frameR,resFrameR,cv::Size(2*frameR.cols,frameR.rows*2));
        cvtColor(treshfR,treshfR,CV_GRAY2BGR);
        treshfR.copyTo(resFrameR(cv::Rect(resFrameR.cols - treshfR.cols,0,treshfR.cols, treshfR.rows)));
        if (insect_pathR.size() > 0) {
            drawKeypoints( framegrayR, insect_pathR, framegrayR, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        }
        framegrayR.copyTo(resFrameR(cv::Rect(0,0,framegrayR.cols, framegrayR.rows)));
        resFrame = cv::Mat(resFrameR.rows,2*resFrameR.cols  ,CV_8UC3);
        resFrameR.copyTo(resFrame(cv::Rect(resFrameR.cols,0,resFrameR.cols, resFrameR.rows)));
#endif
#ifdef DRAWVIZSL
        cv::Mat resFrameL;
        cv::resize(frameL,resFrameL,cv::Size(2*frameL.cols,frameL.rows*2));
        cvtColor(resFrameL,resFrameL,CV_GRAY2BGR);
#ifndef DRAWVIZSR
        resFrame = cv::Mat(resFrameL.rows,resFrameL.cols  ,CV_8UC3);
#endif
        cvtColor(treshfL,treshfL,CV_GRAY2BGR);
        treshfL.copyTo(resFrameL(cv::Rect(resFrameL.cols - treshfL.cols,0,treshfL.cols, treshfL.rows)));
        drawKeypoints( framegrayL, predicted_insect_pathL, framegrayL, Scalar(0,255,0), DrawMatchesFlags::DEFAULT );
        if (insect_pathL.size() > 0) {
            drawKeypoints( framegrayL, insect_pathL, framegrayL, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        }
        framegrayL.copyTo(resFrameL(cv::Rect(0,0,framegrayL.cols, framegrayL.rows)));
        resFrameL.copyTo(resFrame(cv::Rect(0,0,resFrameL.cols, resFrameL.rows)));
#endif       
    }

    if (insect_pathL.size() > 30)
        insect_pathL.erase(insect_pathL.begin());
    if (predicted_insect_pathL.size() > 30)
        predicted_insect_pathL.erase(predicted_insect_pathL.begin());

    if (insect_pathR.size() > 30)
        insect_pathR.erase(insect_pathR.begin());
    if (predicted_insect_pathR.size() > 30)
        predicted_insect_pathR.erase(predicted_insect_pathR.begin());

    (*_logger) << closestL.pt.x  << "; " << closestL.pt.y  << "; " << closestR.pt.x  << "; " << closestR.pt.y  << "; ";
    firstFrame = true;
}

void Insect::updateParams(){
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

void Insect::close () {
    std::ofstream os(settingsFile, std::ios::binary);
    cereal::BinaryOutputArchive archive( os );
    archive( settings );
}
