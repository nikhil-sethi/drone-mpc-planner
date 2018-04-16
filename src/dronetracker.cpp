#include <iostream>
#include "dronetracker.h"
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "common.h"

using namespace cv;
using namespace std;

#if 1
#define DRAWVIZSL
#define DRAWVIZSR
//#define TUNING
#endif

//#define POSITIONTRACKBARS

const string settingsFile = "../settings.dat";
bool DroneTracker::init(std::ofstream *logger) {
    _logger = logger;
    (*_logger) << "imLx; imLy; disparity;";
    if (checkFileExist(settingsFile)) {
        std::ifstream is(settingsFile, std::ios::binary);
        cereal::BinaryInputArchive archive( is );
        archive(settings);
    }


    setpoints.push_back(cv::Point3i(SETPOINTXMAX / 2,SETPOINTYMAX / 2,1000)); // this is overwritten by position trackbars!!!
    setpoints.push_back(cv::Point3i(0,0,1200));
    setpoints.push_back(cv::Point3i(0,0,1200));
    setpoints.push_back(cv::Point3i(0,0,1200));
    setpoints.push_back(cv::Point3i(0,0,1200));


#ifdef TUNING
    namedWindow("Tuning", WINDOW_NORMAL);
    createTrackbar("LowH1", "Tuning", &settings.iLowH1r, 255);
    createTrackbar("HighH1", "Tuning", &settings.iHighH1r, 255);
    createTrackbar("filterByArea", "Tuning", &settings.filterByArea, 1);
    createTrackbar("minArea", "Tuning", &settings.minArea, 10000);
    createTrackbar("maxArea", "Tuning", &settings.maxArea, 10000);
    createTrackbar("Opening1", "Tuning", &settings.iOpen1r, 30);
    createTrackbar("Closing1", "Tuning", &settings.iClose1r, 30);
    createTrackbar("Min disparity", "Tuning", &settings.min_disparity, 255);
    createTrackbar("Max disparity", "Tuning", &settings.max_disparity, 255);
#endif
#ifdef POSITIONTRACKBARS
    namedWindow("Setpoint", WINDOW_NORMAL);
    createTrackbar("X [mm]", "Setpoint", &setpointX, SETPOINTXMAX);
    createTrackbar("Y [mm]", "Setpoint", &setpointY, SETPOINTYMAX);
    createTrackbar("Z [mm]", "Setpoint", &setpointZ, SETPOINTZMAX);
    createTrackbar("WP id", "Setpoint", &wpid, 4);

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

    sposX.init(3);
    sposY.init(3);
    sposZ.init(5);

    disp_smoothed.init(5);
}

float calculateDistance(float xr, float yr, float xl, float yl) {
    return 1.0 /(xr - xl); // kaihard kebowned
}

bool foundL = false;
int t_prev = 0;
bool DroneTracker::track(cv::Mat frameL, cv::Mat frameR, cv::Mat Qf) {
    updateParams();

    int t = stopWatch.Read();
    float dt= (t-t_prev)/1000.0;

    cv::Mat frameL_small;
    cv::Size smallsize(frameL.cols/IMSCALEF,frameL.rows/IMSCALEF);
    cv::resize(frameL,frameL_small,smallsize);
    if (!firstFrame) {
        firstFrame = true;
        frameL_prev = frameL_small.clone();
        frameL_prev_OK = frameL_prev;
        frameL_big_prev = frameL.clone();
        frameR_big_prev = frameR.clone();
        frameL_big_prev_OK = frameL.clone();
        frameR_big_prev_OK = frameR.clone();
    }
    //attempt to detect changed blobs
    cv::Mat treshfL = segment_drone(frameL_small,frameL_prev);
    SimpleBlobDetector detector(params);
    std::vector<KeyPoint> keypointsL;
    detector.detect( treshfL, keypointsL);

    bool update_prev = false;
    //check if changed blobs were detected
    if (keypointsL.size() == 0) { // if not, use the last frame that was confirmed to be working before...
        treshfL = segment_drone(frameL_small,frameL_prev_OK);
        detector.detect( treshfL, keypointsL);
    } else {
        update_prev = true;
    }

    data.valid = false; // reset the flag, set to true when drone is detected properly

    cv::Point3f predicted_drone_locationL = predict_drone(dt);

    bool bam = false;
    std::stringstream ss1,ss2,ss3;
    static int notFoundCountL =0;
    if (keypointsL.size() == 0) {
        notFoundCountL++;
        if( notFoundCountL >= 10 )
            foundL = false;
        update_prediction_state(cv::Point3f(predicted_drone_locationL.x,predicted_drone_locationL.y,predicted_drone_locationL.z));
    } else {
        notFoundCountL = 0;

        //static cv::KeyPoint closestL_prev;
        cv::KeyPoint closestL = match_closest_to_prediciton(predicted_drone_locationL,keypointsL);

        static float disparity_prev =0;
        int disparity = stereo_match(closestL,frameL_big_prev_OK,frameR_big_prev_OK,frameL,frameR,disparity_prev);

        disparity_prev =  disp_smoothed.addSample(disparity);

        if (fabs((float)abs(disparity) - fabs(disparity_prev)) > 3) {
            //do better matching?
            std::cout << disparity << "-> BAM <-" << disparity_prev << std::endl;
            disparity = disparity_prev;
            bam = true;
        }


        update_prediction_state(cv::Point3f(closestL.pt.x,closestL.pt.y,disparity));

        //calculate everything for the dronecontroller:
        std::vector<Point3f> camera_coordinates, world_coordinates;
        camera_coordinates.push_back(Point3f(closestL.pt.x*IMSCALEF,closestL.pt.y*IMSCALEF,-disparity));
        camera_coordinates.push_back(Point3f(predicted_drone_locationL.x*IMSCALEF,predicted_drone_locationL.y*IMSCALEF,-predicted_drone_locationL.z));
        cv::perspectiveTransform(camera_coordinates,world_coordinates,Qf);
        Point3f output = world_coordinates[0];
        Point3f predicted_output = world_coordinates[1];
        static Point3f output_prev;
        outlier_filter(output,output_prev,&ss1,&ss2,&ss3);
        update_tracker_ouput(output,dt);

        output_prev = output;

        t_prev = t; // update dt only if data valid

        (*_logger) << closestL.pt.x  << "; " << closestL.pt.y << "; " << disparity << "; ";
        ss3 << " " << disparity;
    }

    if (bam) {
        ss3 << " BAM!";
    }

    drawviz(frameL,treshfL,frameL_small, &ss1,&ss2,&ss3);
    if (!bam && !breakpause) { //TMP
        if (update_prev) {
            frameL_big_prev_OK = frameL_big_prev.clone();
            frameL_prev_OK = frameL_prev.clone();
            frameR_big_prev_OK = frameR_big_prev.clone();
        }
        frameL_prev = frameL_small.clone();
        frameL_big_prev = frameL.clone();
        frameR_big_prev = frameR.clone();
    }
    return bam;
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

cv::Mat DroneTracker::segment_drone(cv::Mat frame,cv::Mat frame_prev) {
    cv::Mat diffL = frame - frame_prev;


    cv::Mat treshfL;
    inRange(diffL, settings.iLowH1r, settings.iHighH1r, treshfL);
    dilate( treshfL, treshfL, getStructuringElement(MORPH_ELLIPSE, Size(settings.iClose1r+1, settings.iClose1r+1)));
    erode(treshfL, treshfL, getStructuringElement(MORPH_ELLIPSE, Size(settings.iOpen1r+1, settings.iOpen1r+1)));

    return treshfL;
}

cv::Point3f DroneTracker::predict_drone(float dt) {
    cv::Point3f predicted_drone_locationL;
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
        if (dronepathL.size() > 30)
            dronepathL.erase(dronepathL.begin());
        if (predicted_dronepathL.size() > 30)
            predicted_dronepathL.erase(predicted_dronepathL.begin());
        // cout << "PredictionL: " << predicted_drone_locationL << std::endl;
    }
    return predicted_drone_locationL;
}

cv::KeyPoint DroneTracker::match_closest_to_prediciton(cv::Point3f predicted_drone_locationL,std::vector<KeyPoint> keypointsL) {
    cv::KeyPoint closestL;
    if (keypointsL.size() == 1 && dronepathL.size() == 0) {
        closestL = keypointsL.at(0);
    } else if (dronepathL.size() > 0) {
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
    } else {
        //the case that we have multiple drone candidates, no good way to figure out which one is the drone
        //guess we'll just take the first one...
        closestL = keypointsL.at(0);
    }
    dronepathL.push_back(closestL);
    return closestL;
}

int DroneTracker::stereo_match(cv::KeyPoint closestL,cv::Mat prevFrameL_big,cv::Mat prevFrameR_big, cv::Mat frameL,cv::Mat frameR,int prevDisparity){
    int disparity_cor = 0;
    int disparity_err = 0;

    /****match the thresholdedL pixels in the gray images***/

    //calc retangle around blob / changed pixels
    float rectsize = closestL.size+1;
    if (rectsize < 3)
        rectsize = 3;
//    rectsize = ceil(rectsize);
    float rectsizeX = ceil(rectsize*2.0f);
    float rectsizeY = ceil(rectsize*1.4f);

    int x1,y1,x2,y2;
    x1 = (closestL.pt.x-rectsizeX)*IMSCALEF;
    x2 = 2*rectsizeX*IMSCALEF;
    y1 = (closestL.pt.y-rectsizeY)*IMSCALEF;
    y2 = 2*rectsizeY*IMSCALEF;
    if (x1 < 0)
        x1=0;
    if (y1 < 0)
        y1=0;
    if (x1+x2 >= frameL.cols)
        x2=frameL.cols-x1;
    if (y1+y2 >= frameL.rows)
        y2=frameL.rows-y1;

    cv::Rect roi(x1,y1,x2,y2);

    //retrieve changed pixels (through time)
    cv::Mat a = frameL(roi);
    cv::Mat b = prevFrameL_big(roi);
    cv::Mat diffroi = a-b;
    inRange(diffroi, settings.iLowH1r, settings.iHighH1r, diffroi);
    dilate( diffroi, diffroi, getStructuringElement(MORPH_ELLIPSE, Size(settings.iClose1r+10, settings.iClose1r+3)));

    cv::Mat fL,fL_16;
    frameL(roi).copyTo(fL, diffroi); //mask out the static pixels
    fL.convertTo(fL_16, CV_16UC1);

    cv::Mat fR = fL; // init it to something random

    //find the minumum MSE over the disparity range
    float maxcor = std::numeric_limits<float>::min();
    float minerr = std::numeric_limits<float>::max();
    int tmp_max_disp = settings.max_disparity;
    if (x1 - tmp_max_disp < 0)
        tmp_max_disp = x1;
    for (int i=settings.min_disparity; i<tmp_max_disp;i++) {
        cv::Rect roiR(x1-i,y1,x2,y2);
        cv::Mat fR_tmp;
        frameR(roiR).copyTo(fR_tmp, diffroi); //mask the static pixels

        cv::Mat fR_tmp_16;
        fR_tmp.convertTo(fR_tmp_16, CV_16UC1);
        cv::Mat corV_16 = fL_16.mul(fR_tmp_16);
        cv::Mat errV = abs(fL - fR_tmp);

        int cor_16 = cv::sum(corV_16 )[0]/256;
        int err = cv::sum(errV)[0];

         if (cor_16 > maxcor ) {
            fR = fR_tmp.clone();
            disparity_cor  = i;
            maxcor = cor_16;
        }
         if (err < minerr ) { //update min MSE
            //fR = fR_tmp.clone();
            disparity_err  = i;
            minerr = err;
        }
        if (breakpause) {
            cv::resize(fR_tmp,fR_tmp,cv::Size(fR_tmp.cols*4, fR_tmp.rows*4));
            std::stringstream ss;
            ss << i << "; " << cor_16 << " " << err;
            putText(fR_tmp,ss.str() ,cv::Point(0,12),cv::FONT_HERSHEY_SIMPLEX,0.5,255);

            imshow("shift",fR_tmp);

            unsigned char key = cv::waitKey(0);
            if (key == 'c')
                breakpause = false;

        }

    }

    if (tmp_max_disp > settings.min_disparity) {
        std::vector<cv::Mat> ims;
        ims.push_back(a);
        ims.push_back(b);
        ims.push_back(diffroi);
        ims.push_back(fL);
        ims.push_back(fR);
        showColumnImage(ims);
    }
    int disparity;
    if (disparity_cor != disparity_err) {
        if (abs(prevDisparity - disparity_cor) > abs(prevDisparity - disparity_err))
            disparity = disparity_err;
        else
            disparity = disparity_cor;
    } else
        disparity = disparity_cor;

    return disparity;
}

int trackingOK = 0;
#define OUTLIER_DISTANCE 5.0f
#define TRACKINGOK_THRESHOLD 3
bool DroneTracker::outlier_filter(Point3f measured_world_coordinates, Point3f predicted_world_coordinates, stringstream *ss1,stringstream *ss2,stringstream *ss3) {
    Point3f err;
    err.x = measured_world_coordinates.x-predicted_world_coordinates.x;
    err.y = measured_world_coordinates.y-predicted_world_coordinates.y;
    err.z = measured_world_coordinates.z-predicted_world_coordinates.z;
    float dist = sqrt(err.x*err.x + err.y*err.y + err.z*err.z);

    (*ss1).precision(2);
    (*ss2).precision(2);
    (*ss3).precision(2);

    (*ss1) << "[" << measured_world_coordinates.x << ", " << measured_world_coordinates.y << ", " << measured_world_coordinates.z << "] " ;
    (*ss2) << "[" << predicted_world_coordinates.x << ", " << predicted_world_coordinates.y << ", " << predicted_world_coordinates.z << "] " ;

    (*ss3) << dist << ":  ";
    if (dist > OUTLIER_DISTANCE) {
        trackingOK =0;
        (*ss3) << "TRACKING LOST";
    } else {
        (*ss3) << "TRACKING ...";
        trackingOK++;
    }

    if (trackingOK > TRACKINGOK_THRESHOLD) {
        (*ss3) << "OK";
        return true;
    } else {
        return false;
    }
}

void DroneTracker::drawviz(cv::Mat frameL,cv::Mat treshfL,cv::Mat framegrayL, stringstream *ss1,stringstream *ss2,stringstream *ss3) {

    cv::Mat resFrameL;
#ifdef DRAWVIZSL
    cv::resize(frameL,resFrameL,cv::Size(frameL.cols,frameL.rows));
    //    equalizeHist( resFrameL, resFrameL);
    cvtColor(resFrameL,resFrameL,CV_GRAY2BGR);


    if (foundL) {
        drawKeypoints( framegrayL, predicted_dronepathL, framegrayL, Scalar(0,255,0), DrawMatchesFlags::DEFAULT );

    } else {
        cvtColor(framegrayL,framegrayL,CV_GRAY2BGR);
    }

    cv::Size vizsizeL(resFrameL.cols/4,resFrameL.rows/4);
    cv::resize(treshfL,treshfL,vizsizeL);
    cvtColor(treshfL,treshfL,CV_GRAY2BGR);

    treshfL.copyTo(resFrameL(cv::Rect(resFrameL.cols - treshfL.cols,0,treshfL.cols, treshfL.rows)));

    if (dronepathL.size() > 0) {
        drawKeypoints( framegrayL, dronepathL, framegrayL, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    }
    cv::circle(framegrayL,cv::Point(setpoint.x,setpoint.y),2,cv::Scalar(150,255,200));
    cv::resize(framegrayL,framegrayL,vizsizeL);
    framegrayL.copyTo(resFrameL(cv::Rect(0,0,framegrayL.cols, framegrayL.rows)));
#endif
#ifdef DRAWVIZSR
    putText(resFrameL,(*ss1).str() ,cv::Point(220,20),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(125,125,255));
    putText(resFrameL,(*ss2).str() ,cv::Point(220,40),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(125,125,255));
    putText(resFrameL,(*ss3).str() ,cv::Point(220,60),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(125,125,255));
    resFrame = cv::Mat(resFrameL.rows,resFrameL.cols ,CV_8UC3);
    resFrameL.copyTo(resFrame(cv::Rect(0,0,resFrameL.cols, resFrameL.rows)));
    //    resFrameR.copyTo(resFrame(cv::Rect(resFrameR.cols,0,resFrameR.cols, resFrameR.rows)));
#endif
}

void DroneTracker::update_prediction_state(cv::Point3f p) {
    cv::Mat measL(measSize, 1, type);
    measL.at<float>(0) = p.x;
    measL.at<float>(1) = p.y;
    measL.at<float>(2) = p.z;
    //measL.at<float>(3) = 0;


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
        //            stateL.at<float>(3) = 0;
        //            stateL.at<float>(4) = measL.at<float>(2);
        //            stateL.at<float>(5) = measL.at<float>(3);

        kfL.statePost = stateL;
        foundL = true;
    }
    else
        kfL.correct(measL);
}

void DroneTracker::update_tracker_ouput(Point3f output,float dt) {
    cv::Point3i tmps;
    if (wpid > 0)
        tmps = setpoints[wpid];
    else { // read from position trackbars
        tmps.x = setpointX;
        tmps.y = setpointY;
        tmps.z = setpointZ;
    }

    setpointw.x = (tmps.x - SETPOINTXMAX/2) / 1000.0f;
    setpointw.y = (tmps.y - SETPOINTYMAX/2) / 1000.0f;
    setpointw.z = -(tmps.z) / 1000.0f;

    data.posX = output.x;
    data.posY = output.y;
    data.posZ = output.z;

    float csposX = sposX.addSample(data.posX);
    float csposY = sposY.addSample(data.posY);
    float csposZ = sposZ.addSample(data.posZ);

    data.csposX = csposX;
    data.csposY = csposY;
    data.csposZ = csposZ;

    static float prevX,prevY,prevZ =0;

    data.dx = csposX - prevX;
    data.dy = csposY - prevY;
    data.dz = csposZ - prevZ;
    data.velX = data.dx / dt;
    data.velY = data.dy / dt;
    data.velZ = data.dz / dt;
    data.dt = dt;
    data.valid = true;

    prevX = csposX;
    prevY = csposY;
    prevZ = csposZ;

    data.posErrX = data.posX - setpointw.x;
    data.posErrY = data.posY - setpointw.y;
    data.posErrZ = data.posZ - setpointw.z;
}

void DroneTracker::close () {
    std::ofstream os(settingsFile, std::ios::binary);
    cereal::BinaryOutputArchive archive( os );
    archive( settings );
}
