#include <iostream>
#include "dronetracker.h"
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "common.h"

using namespace cv;
using namespace std;

#if 1
#define DRAWVIZS
#define TUNING
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

    createTrackbar("Uncertain mult", "Tuning", &settings.uncertainty_multiplier, 255);
    createTrackbar("Uncertain pow", "Tuning", &settings.uncertainty_power, 255);
    createTrackbar("Uncertain back", "Tuning", &settings.uncertainty_background, 255);

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

    svelX.init(3);
    svelY.init(3);
    svelZ.init(5);

    disp_smoothed.init(5);

    data.drone_image_locationL = cv::Point(DRONE_IM_X_START,DRONE_IM_Y_START);
    data.landed = true;

    blurred_circle = createBlurryCircle(60,settings.uncertainty_background/255.0);
}

cv::Mat DroneTracker::createBlurryCircle(int size, float background) {
    float tmp = roundf(size/4);
    if (fabs((tmp / 2.f) - roundf(tmp / 2.f)) < 0.01)
        tmp +=1;

    cv::Mat res(size,size,CV_32F);
    res = background;
    cv::circle(res,cv::Point(size/2,size/2),tmp,1.0f,CV_FILLED);
    cv::GaussianBlur( res, res, cv::Size( tmp, tmp ), 0, 0 );
//    cv::GaussianBlur( res, res, cv::Size( tmp, tmp ), 0, 0 );
//    cv::GaussianBlur( res, res, cv::Size( tmp, tmp ), 0, 0 );
    return res;
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
    cv::Point3f predicted_drone_locationL = predict_drone(dt);

    cv::Mat frameL_small;
    cv::Size smallsize(frameL.cols/IMSCALEF,frameL.rows/IMSCALEF);
    cv::resize(frameL,frameL_small,smallsize);
    if (!firstFrame) {
        firstFrame = true;
        frameL_s_prev = frameL_small.clone();
        frameL_s_prev_OK = frameL_s_prev;
        frameL_big_prev = frameL.clone();
        frameR_big_prev = frameR.clone();
        frameL_big_prev_OK = frameL.clone();
        frameR_big_prev_OK = frameR.clone();
        empty_diffs = cv::Mat::zeros(frameL_small.rows,frameL_small.cols,CV_32SC1);
    }

    resFrame = frameL;

    find_drone(frameL_small);

    data.valid = false; // reset the flag, set to true when drone is detected properly



    bool bam = false;
    static int notFoundCountL =0;
    if (find_drone_result.keypointsL.size() == 0) {
        notFoundCountL++;
        if( notFoundCountL >= 10 )
            foundL = false;
        update_prediction_state(cv::Point3f(predicted_drone_locationL.x,predicted_drone_locationL.y,predicted_drone_locationL.z));
    } else {
        notFoundCountL = 0;

        cv::KeyPoint closestL = match_closest_to_prediciton(predicted_drone_locationL,find_drone_result.keypointsL);

        static float disparity_prev =0;
        int disparity = stereo_match(closestL,frameL_big_prev_OK,frameR_big_prev_OK,frameL,frameR,disparity_prev);

        disparity_prev =  disp_smoothed.addSample(disparity);

        if (fabs((float)abs(disparity) - fabs(disparity_prev)) > 3) {
            //do better matching?
            std::cout << disparity << "-> BAM <-" << disparity_prev << std::endl;
            disparity = disparity_prev;
            bam = true;
            find_drone_result.update_prev = false;
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
        update_tracker_ouput(output,dt,closestL.pt);

        output_prev = output;

        t_prev = t; // update dt only if data valid

        (*_logger) << closestL.pt.x  << "; " << closestL.pt.y << "; " << disparity << "; ";
    }

#ifdef DRAWVIZS
        drawviz(frameL,find_drone_result.treshfL,frameL_small);
#endif

    if (!bam && !breakpause) { //TMP
        if (find_drone_result.update_prev) {
            frameL_big_prev_OK = frameL_big_prev.clone();
            frameL_s_prev_OK = frameL_s_prev.clone();
            frameR_big_prev_OK = frameR_big_prev.clone();
        }
        frameL_s_prev = frameL_small.clone();
        frameL_big_prev = frameL.clone();
        frameR_big_prev = frameR.clone();
    }
    return false;
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

void DroneTracker::collect_no_drone_frames(cv::Mat diff) {
    static cv::Mat max_uncertainty_map = diff.clone();
    cv::Mat mask = diff > max_uncertainty_map;
    diff.copyTo(max_uncertainty_map,mask);

    uncertainty_map = 255 - max_uncertainty_map * settings.uncertainty_multiplier;
    uncertainty_map.convertTo(uncertainty_map ,CV_32F);
    uncertainty_map /=255.0f;
    cv::pow(uncertainty_map,settings.uncertainty_power,uncertainty_map);

}


void DroneTracker::find_drone(cv::Mat frameL_small) {
    cv::Point previous_drone_location;
    if (data.landed)
        data.drone_image_locationL = cv::Point(DRONE_IM_X_START,DRONE_IM_Y_START);
    previous_drone_location = data.drone_image_locationL;

    //attempt to detect changed blobs
    cv::Mat treshfL = segment_drone(frameL_small,frameL_s_prev,true,previous_drone_location);
    SimpleBlobDetector detector(params);
    std::vector<KeyPoint> keypointsL;
    detector.detect( treshfL, keypointsL);

    static int update_prev = 0;
    //check if changed blobs were detected
    if (keypointsL.size() == 0) { // if not, use the last frame that was confirmed to be working before...
        treshfL = segment_drone(frameL_small,frameL_s_prev_OK,false,previous_drone_location);
        detector.detect( treshfL, keypointsL);
        update_prev +=1;
        if (update_prev > 50)
            update_prev = 50;
        blurred_circle = createBlurryCircle(60+update_prev,settings.uncertainty_background/255.0);
    } else {
        blurred_circle = createBlurryCircle(60,settings.uncertainty_background/255.0);
        update_prev = 0;
    }

    if (data.landed)
        update_prev = 0;
    find_drone_result.keypointsL = keypointsL;
    find_drone_result.treshfL = treshfL;
    find_drone_result.update_prev = update_prev == 0 || update_prev > 5 ;

}

cv::Mat DroneTracker::segment_drone(cv::Mat frame,cv::Mat frame_prev, bool update_uncertainty_map, cv::Point previous_imageL_location) {
    cv::Mat diffL;
    cv::absdiff( frame ,frame_prev,diffL);

    if (!data.background_calibrated && update_uncertainty_map)
        collect_no_drone_frames(diffL); // calibration of background uncertainty map

    //filter out changes in areas that were noisy during calibration period
    cv::Mat diff_of_diff = diffL.clone();
    cv::Mat diff_before_uncertainty = diffL.clone();;
    diffL.convertTo(diffL, CV_32F);

    cv::Mat uncertainty_drone_map = get_uncertainty_map_with_drone(previous_imageL_location);

    diffL = diffL.mul(uncertainty_drone_map);
    //diffL *=10;
    diffL.convertTo(diffL, CV_8UC1);


#ifdef DRAWVIZS
    cv::absdiff( diff_of_diff ,diffL,diff_of_diff);
    equalizeHist(diff_of_diff, diff_of_diff);
    if (!data.background_calibrated)
        putText(diff_of_diff,"Building" ,cv::Point(0,12),cv::FONT_HERSHEY_SIMPLEX,0.5,255);
//    cv::Mat diffL_eq;
//    equalizeHist(diffL, diffL_eq);

    cv::Mat uncertainty_drone_map_8;
    uncertainty_drone_map_8 = uncertainty_drone_map.clone();
    uncertainty_drone_map_8 = uncertainty_drone_map_8 * 255.0f;
    uncertainty_drone_map_8.convertTo(uncertainty_drone_map_8, CV_8UC1);

    cv::Mat uncertainty_ori_map_8;
    uncertainty_ori_map_8 = uncertainty_map.clone();
    uncertainty_ori_map_8 = uncertainty_ori_map_8 * 255.0f;
    uncertainty_ori_map_8.convertTo(uncertainty_ori_map_8, CV_8UC1);
#endif

    cv::Mat treshfL;
    inRange(diffL, settings.iLowH1r, settings.iHighH1r, treshfL);
    dilate( treshfL, treshfL, getStructuringElement(MORPH_ELLIPSE, Size(settings.iClose1r+1, settings.iClose1r+1)));
    erode(treshfL, treshfL, getStructuringElement(MORPH_ELLIPSE, Size(settings.iOpen1r+1, settings.iOpen1r+1)));

#ifdef DRAWVIZS
    //equalizeHist(uncertainty_map_8, uncertainty_map_8);
    std::vector<cv::Mat> ims;
    ims.push_back(uncertainty_ori_map_8);
    ims.push_back(uncertainty_drone_map_8);
    ims.push_back(diff_before_uncertainty*10);
    ims.push_back(diffL*10);
    ims.push_back(treshfL);
    ims.push_back(diff_of_diff);
//    ims.push_back(diffL_eq);
    showColumnImage(ims,"uncertainty",CV_8UC1);
#endif
    return treshfL;
}

/* Takes the calibrated uncertainty map, and augments it with a highlight around p */
cv::Mat DroneTracker::get_uncertainty_map_with_drone(cv::Point p) {

    cv::Mat res = uncertainty_map.clone();


    cv::Rect roi_circle(0,0,blurred_circle.cols,blurred_circle.rows);
    int x1 = p.x-blurred_circle.cols/2;
    if (x1 < 0) {
        roi_circle.x = abs(p.x-blurred_circle.cols/2);
        roi_circle.width-=roi_circle.x;
    } else if (x1 + blurred_circle.cols >= res.cols)
        roi_circle.width = roi_circle.width  - abs(x1 + blurred_circle.cols - res.cols);

    int y1 = p.y-blurred_circle.rows/2;
    if (y1 < 0) {
        roi_circle.y = abs(p.y-blurred_circle.rows/2);
        roi_circle.height-=roi_circle.y;
    } else if (y1 + blurred_circle.rows >= res.rows)
        roi_circle.height = roi_circle.height - abs(y1 + blurred_circle.rows - res.rows);



    cv::Mat gray = cv::Mat::zeros(res.rows,res.cols,CV_32F);
    gray = settings.uncertainty_background/255.0;
    cv::Mat a = blurred_circle(roi_circle);


    x1 = p.x-blurred_circle.cols/2+roi_circle.x;
    int x2 = a.cols;
    y1 = p.y-blurred_circle.rows/2+roi_circle.y;
    int y2 = a.rows;

    cv::Rect roi(x1,y1,x2,y2);

    a.copyTo(gray(roi));

    res = gray.mul(res);

//    double min, max;
//    cv::minMaxLoc(blurred_circle, &min, &max);
//    std::cout << "min max " << min << " " << max << std::endl;

    return res;
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

    //get retangle around blob / changed pixels
    float rectsize = closestL.size+1;
    if (rectsize < 3)
        rectsize = 3;
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

    cv::Rect roiL(x1,y1,x2,y2);

    //retrieve changed pixels (through time) for left camera
    cv::Mat aL = frameL(roiL);
    cv::Mat bL = prevFrameL_big(roiL);
    cv::Mat diff_L_roi = aL-bL;
    cv::Mat diff_L_roi_16;
    diff_L_roi.convertTo(diff_L_roi_16, CV_16UC1);

#ifdef DRAWVIZS
    cv::Mat aR_viz,bR_viz,diff_R_roi_viz;
#endif

    //shift over the image to find the best match, shift = disparity
    int disparity_cor = 0,disparity_err = 0;
    float maxcor = std::numeric_limits<float>::min();
    float minerr = std::numeric_limits<float>::max();
    int tmp_max_disp = settings.max_disparity;
    if (x1 - tmp_max_disp < 0)
        tmp_max_disp = x1;
    for (int i=settings.min_disparity; i<tmp_max_disp;i++) {
        cv::Rect roiR(x1-i,y1,x2,y2);

        cv::Mat aR = frameR(roiR);
        cv::Mat bR = prevFrameR_big(roiR);
        cv::Mat diff_R_roi = aR-bR;
        cv::Mat diff_R_roi_16;
        diff_R_roi.convertTo(diff_R_roi_16, CV_16UC1);

        cv::Mat corV_16 = diff_L_roi_16.mul(diff_R_roi_16);
        cv::Mat errV = abs(diff_L_roi - diff_R_roi);

        int cor_16 = cv::sum(corV_16 )[0]/256;
        int err = cv::sum(errV)[0];

         if (cor_16 > maxcor ) {
            disparity_cor  = i;
            maxcor = cor_16;
        }
         if (err < minerr ) { //update min MSE       
            disparity_err  = i;
            minerr = err;
#ifdef DRAWVIZS
            diff_R_roi_viz = diff_R_roi.clone();
            aR_viz = aR;
            bR_viz = bR;
#endif
        } // for shift

#ifdef DRAWVIZS
        if (breakpause) {
            cv::Mat aL_shift,bL_shift,diff_L_roi_shift,aR_shift,bR_shift,diff_R_roi_shift;
            cv::resize(aL,aL_shift,cv::Size(aL.cols*4, aL.rows*4));
            cv::resize(bL,bL_shift,cv::Size(aL.cols*4, aL.rows*4));
            cv::resize(diff_L_roi,diff_L_roi_shift,cv::Size(aL.cols*4, aL.rows*4));
            cv::resize(aR,aR_shift,cv::Size(aL.cols*4, aL.rows*4));
            cv::resize(bR,bR_shift,cv::Size(aL.cols*4, aL.rows*4));
            cv::resize(diff_R_roi,diff_R_roi_shift,cv::Size(aL.cols*4, aL.rows*4));
            std::stringstream ss;
            ss << i << "; cor: " << cor_16 << " err: " << err;
            putText(aL_shift,ss.str() ,cv::Point(0,12),cv::FONT_HERSHEY_SIMPLEX,0.5,255);

            std::vector<cv::Mat> ims;
            ims.push_back(aL_shift);
            ims.push_back(bL_shift);
            ims.push_back(diff_L_roi_shift);
            ims.push_back(aR_shift);
            ims.push_back(bR_shift);
            ims.push_back(diff_R_roi_shift);
            showColumnImage(ims, "shift",CV_8UC1);

            unsigned char key = cv::waitKey(0);
            if (key == 'c')
                breakpause = false;

        }
#endif

    }
    int disparity;
    if (disparity_cor != disparity_err) {
        if (abs(prevDisparity - disparity_cor) > abs(prevDisparity - disparity_err))
            disparity = disparity_err;
        else
            disparity = disparity_cor;
    } else
        disparity = disparity_cor;

#ifdef DRAWVIZS
    if (tmp_max_disp > settings.min_disparity) {
        std::vector<cv::Mat> ims;
        ims.push_back(aL);
        ims.push_back(bL);
        ims.push_back(diff_L_roi);
        ims.push_back(aR_viz);
        ims.push_back(bR_viz);
        ims.push_back(diff_R_roi_viz);
//        showColumnImage(ims, "col");
    }
#endif

    return disparity;
}

void DroneTracker::drawviz(cv::Mat frameL,cv::Mat treshfL,cv::Mat framegrayL) {
#ifdef DRAWVIZS
    cv::Mat resFrameL;

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

    std::stringstream ss1,ss2,ss3;
    ss1.precision(2);
    ss2.precision(2);
    ss3.precision(2);

    ss1 << "[" << data.posX << ", " << data.posY << ", " << data.posZ << "] " ;
    ss2 << "[" << data.posErrX << ", " << data.posErrY << ", " << data.posErrZ << "] " ;
    ss3 << "Delta: " << sqrtf(data.posErrX*data.posErrX+data.posErrY*data.posErrY+data.posErrZ*data.posErrZ);

    putText(resFrameL,ss1.str() ,cv::Point(220,20),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(125,125,255));
    putText(resFrameL,ss2.str() ,cv::Point(220,40),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(125,125,255));
    putText(resFrameL,ss3.str() ,cv::Point(220,60),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(125,125,255));
    resFrame = resFrameL;
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

void DroneTracker::update_tracker_ouput(Point3f measured_world_coordinates,float dt, cv::Point measured_drone_image_location) {
    cv::Point3i tmps;
    if (wpid > 0)
        tmps = setpoints[wpid];
    else { // read from position trackbars
        tmps.x = setpointX;
        tmps.y = setpointY;
        tmps.z = setpointZ;
    }

    data.drone_image_locationL = measured_drone_image_location;

    setpointw.x = (tmps.x - SETPOINTXMAX/2) / 1000.0f;
    setpointw.y = (tmps.y - SETPOINTYMAX/2) / 1000.0f;
    setpointw.z = -(tmps.z) / 1000.0f;

    data.posX = measured_world_coordinates.x;
    data.posY = measured_world_coordinates.y;
    data.posZ = measured_world_coordinates.z;

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
    data.svelX = svelX.addSample(data.velX);
    data.svelY = svelX.addSample(data.velY);
    data.svelZ = svelX.addSample(data.velZ);
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
