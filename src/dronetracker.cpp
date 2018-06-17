#include <iostream>
#include "dronetracker.h"
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "common.h"

using namespace cv;
using namespace std;

#ifdef HASSCREEN
#define TUNING
#endif

const string settingsFile = "../dronetrackersettings.dat";
bool DroneTracker::init(std::ofstream *logger, VisionData *visdat) {
    _logger = logger;
    this->visdat = visdat;

    (*_logger) << "imLx; imLy; disparity;";
    if (checkFileExist(settingsFile)) {
        std::ifstream is(settingsFile, std::ios::binary);
        cereal::BinaryInputArchive archive( is );
        archive(settings);
    }



#ifdef TUNING
    createTrackbar("LowH1", "Tracking", &settings.iLowH1r, 255);
    createTrackbar("HighH1", "Tracking", &settings.iHighH1r, 255);
    createTrackbar("filterByArea", "Tracking", &settings.filterByArea, 1);
    createTrackbar("minArea", "Tracking", &settings.minArea, 10000);
    createTrackbar("maxArea", "Tracking", &settings.maxArea, 10000);
    createTrackbar("Opening1", "Tracking", &settings.iOpen1r, 30);
    createTrackbar("Closing1", "Tracking", &settings.iClose1r, 30);
    createTrackbar("Min disparity", "Tracking", &settings.min_disparity, 255);
    createTrackbar("Max disparity", "Tracking", &settings.max_disparity, 255);
    createTrackbar("roi_min_size", "InsectTracking", &settings.roi_min_size, 2000);
    createTrackbar("roi_max_grow", "InsectTracking", &settings.roi_max_grow, 500);
    createTrackbar("roi_grow_speed", "Tracking", &settings.roi_grow_speed, 256);
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

    smoother_posX.init(smooth_width_pos);
    smoother_posY.init(smooth_width_pos);
    smoother_posZ.init(smooth_width_pos);
    disp_smoothed.init(smooth_width_pos);

    smoother_velX.init(smooth_width_vel);
    smoother_velY.init(smooth_width_vel);
    smoother_velZ.init(smooth_width_vel);

    data.drone_image_locationL = cv::Point(DRONE_IM_X_START,DRONE_IM_Y_START);
    find_drone_result.smoothed_disparity = DRONE_DISPARITY_START;
    find_drone_result.disparity = DRONE_DISPARITY_START;
    data.landed = true;
}

std::vector<KeyPoint> DroneTracker::remove_ignores(std::vector<KeyPoint> keypoints, cv::Point2f ignore) {
    std::vector<KeyPoint> tmp = keypoints;
    for (int i = 0 ; i< tmp.size();i++){
        float dis = (tmp.at(i).pt.x - ignore.x)*(tmp.at(i).pt.x - ignore.x) +(tmp.at(i).pt.y - ignore.y)*(tmp.at(i).pt.y - ignore.y);

        if (dis < 200 || (ignore.x == 0 && ignore.y == 0))
            keypoints.erase(keypoints.begin() + i);
        else if (dis > 0)
            std::cout << dis << std::endl;
    }
    return keypoints;
}

void DroneTracker::track(float time, cv::Point3f setpoint_world, cv::Point2f ignore) {
    updateParams();

    float dt= (time-t_prev);
    cv::Point3f predicted_drone_locationL = predict_drone(dt);


    if (!firstFrame) {
        firstFrame = true;
        frameL_s_prev_OK = visdat->frameL_s_prev;
        frameL_prev_OK = visdat->frameL.clone();
        frameR_prev_OK = visdat->frameR.clone();
    }

    if (data.landed)
    {
        frameL_s_prev_OK = visdat->frameL_s_prev;
        frameL_prev_OK = visdat->frameL_prev;
        frameR_prev_OK = visdat->frameR_prev;
    }

    find_drone(visdat->frameL_small, frameL_s_prev_OK);

    data.valid = false; // reset the flag, set to true when drone is detected properly

    static int n_frames_lost =100;
    if (find_drone_result.keypointsL.size() > 0) { //if not lost

        cv::Point3f previous_drone_location(find_drone_result.best_image_locationL .pt.x,find_drone_result.best_image_locationL .pt.y,0);        
        std::vector<cv::KeyPoint> keypoint_candidates = remove_ignores(find_drone_result.keypointsL,ignore);
        Point3f output;
        int disparity;
        cv::KeyPoint match;
        while (keypoint_candidates.size() > 0) {
            int match_id = match_closest_to_prediciton(previous_drone_location,find_drone_result.keypointsL);
            match = find_drone_result.keypointsL.at(match_id);

            disparity = stereo_match(match,frameL_prev_OK,frameR_prev_OK,visdat->frameL,visdat->frameR,find_drone_result.smoothed_disparity);

            //calculate everything for the dronecontroller:
            std::vector<Point3f> camera_coordinates, world_coordinates;
            camera_coordinates.push_back(Point3f(match.pt.x*IMSCALEF,match.pt.y*IMSCALEF,-disparity));
            //camera_coordinates.push_back(Point3f(predicted_drone_locationL.x*IMSCALEF,predicted_drone_locationL.y*IMSCALEF,-predicted_drone_locationL.z));
            cv::perspectiveTransform(camera_coordinates,world_coordinates,visdat->Qf);
            output = world_coordinates[0];
            float theta = CAMERA_ANGLE / (360/6.28318530718);
            output.y = output.y * cosf(theta) + output.z * sinf(theta);
            output.z = -output.y * sinf(theta) + output.z * cosf(theta);

            if ((output.z < -drone_max_border_z) || (output.y < -drone_max_border_y) || disparity < settings.min_disparity || disparity > settings.max_disparity) { //TODO check min/max disparity > or =>!!!
                keypoint_candidates.erase(keypoint_candidates.begin() + match_id);
            } else {
                break;
            }
        }
        if (keypoint_candidates.size() == 0) { // if lost
            n_frames_lost++;
            if( n_frames_lost >= 10 )
                foundL = false;
            update_prediction_state(cv::Point3f(predicted_drone_locationL.x,predicted_drone_locationL.y,predicted_drone_locationL.z));
            reset_tracker_ouput(n_frames_lost);
        } else {
            //Point3f predicted_output = world_coordinates[1];
            update_prediction_state(cv::Point3f(match.pt.x,match.pt.y,disparity));
            update_tracker_ouput(output,dt,n_frames_lost,match,disparity,setpoint_world);
            n_frames_lost = 0; // update this after calling update_tracker_ouput, so that it can determine how long tracking was lost
            t_prev = time; // update dt only if drone was detected
            n_frames_tracking++;
        }
    }

    (*_logger) << find_drone_result.best_image_locationL.pt.x *IMSCALEF << "; " << find_drone_result.best_image_locationL.pt.y *IMSCALEF << "; " << find_drone_result.disparity << "; ";

    if (find_drone_result.update_prev_frame && ! breakpause) {
        frameL_prev_OK = visdat->frameL_prev.clone();
        frameL_s_prev_OK = visdat->frameL_s_prev.clone();
        frameR_prev_OK = visdat->frameR_prev.clone();
    }

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


void DroneTracker::find_drone(cv::Mat frameL_small, cv::Mat frameL_s_prev_OK) {
    static int nframes_since_update_prev = 0;
    cv::Point previous_drone_location;

    if (data.landed) {
        data.drone_image_locationL = cv::Point(DRONE_IM_X_START,DRONE_IM_Y_START);
        find_drone_result.smoothed_disparity = DRONE_DISPARITY_START;
        find_drone_result.disparity = DRONE_DISPARITY_START;
        nframes_since_update_prev = 0;
    }
    previous_drone_location = data.drone_image_locationL;

    cv::Point roi_size;
    if (settings.roi_min_size < 1)
        settings.roi_min_size = 1;
    roi_size.x=settings.roi_min_size/IMSCALEF+nframes_since_update_prev*(settings.roi_grow_speed / 16 / IMSCALEF);
    roi_size.y=settings.roi_min_size/IMSCALEF+nframes_since_update_prev*(settings.roi_grow_speed / 16 / IMSCALEF);
    if (roi_size.x > visdat->frameL_small.cols)
        roi_size.x = visdat->frameL_small.cols;
    if (roi_size.y > visdat->frameL_small.rows)
        roi_size.y = visdat->frameL_small.rows;

    //attempt to detect changed blobs
    treshL = segment_drone(visdat->diffL,previous_drone_location,roi_size);
#if CV_MAJOR_VERSION==3
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
#else
    SimpleBlobDetector * detector;
    detector = *SimpleBlobDetector(params);
#endif

    std::vector<KeyPoint> keypointsL;
    detector->detect( treshL, keypointsL);


    bool still_nothing = false;
    //check if changed blobs were detected
    if (keypointsL.size() == 0) { // if not, use the last frame that was confirmed to be working before...
        cv::Mat diffL_OK;
        cv::absdiff( frameL_small ,frameL_s_prev_OK,diffL_OK);
        treshL = segment_drone(diffL_OK,previous_drone_location,roi_size);
        detector->detect( treshL, keypointsL);
        nframes_since_update_prev +=1;
        if (nframes_since_update_prev > settings.roi_max_grow)
            nframes_since_update_prev = settings.roi_max_grow;
        if (keypointsL.size() == 0 )
            still_nothing = true;

    } else {
        nframes_since_update_prev = 0;
    }




    for (int i = 0 ; i < keypointsL.size();i++) {
        keypointsL.at(i).pt.x += find_drone_result.roi_offset.x;
        keypointsL.at(i).pt.y += find_drone_result.roi_offset.y;
    }

    find_drone_result.keypointsL = keypointsL;
    find_drone_result.treshL = treshL;
    find_drone_result.update_prev_frame = nframes_since_update_prev == 0 || (nframes_since_update_prev >= settings.roi_max_grow && !data.landed );
}

cv::Mat DroneTracker::segment_drone(cv::Mat diffL, cv::Point previous_imageL_location, cv::Point roi_size) {

    approx = get_approx_drone_cutout_filtered(previous_imageL_location,diffL,roi_size);

    inRange(approx, settings.iLowH1r, settings.iHighH1r, treshL);
    dilate( treshL, treshL, getStructuringElement(MORPH_ELLIPSE, Size(settings.iClose1r+1, settings.iClose1r+1)));
    erode(treshL, treshL, getStructuringElement(MORPH_ELLIPSE, Size(settings.iOpen1r+1, settings.iOpen1r+1)));
    return treshL;
}

/* Takes the calibrated uncertainty map, and augments it with a highlight around p */
cv::Mat DroneTracker::get_approx_drone_cutout_filtered(cv::Point p, cv::Mat diffL, cv::Point size) {

    //calc roi:
    cv::Rect roi_circle(0,0,size.x,size.y);
    int x1 = p.x-size.x/2;
    if (x1 < 0) {
        roi_circle.x = abs(p.x-size.x/2);
        roi_circle.width-=roi_circle.x;
    } else if (x1 + size.x >= diffL.cols)
        roi_circle.width = roi_circle.width  - abs(x1 + size.x - diffL.cols);

    int y1 = p.y-size.y/2;
    if (y1 < 0) {
        roi_circle.y = abs(p.y-size.y/2);
        roi_circle.height-=roi_circle.y;
    } else if (y1 + size.y >= diffL.rows)
        roi_circle.height = roi_circle.height - abs(y1 + size.y - diffL.rows);

    cv::Mat blurred_circle = createBlurryCircle(size);
    cir = blurred_circle(roi_circle);

    x1 = p.x-size.x/2+roi_circle.x;
    int x2 = roi_circle.width;
    y1 = p.y-size.y/2+roi_circle.y;
    int y2 = roi_circle.height;

    cv::Rect roi(x1,y1,x2,y2);
    find_drone_result.roi_offset = roi;

    bkg = visdat->uncertainty_map(roi);
    diffL(roi).convertTo(dif, CV_32F);
    cv::Mat res;
    res = cir.mul(dif).mul(bkg);
    res.convertTo(res, CV_8UC1);

    return res;
}

/* Takes the calibrated uncertainty map, and augments it with a highlight around p */
cv::Mat DroneTracker::show_uncertainty_map_in_image(cv::Point pd4, cv::Mat res) {

    cv::Point p;
    p.x = pd4.x *IMSCALEF;
    p.y = pd4.y *IMSCALEF;

    cv::Mat blurred_circle_big;
    cv::resize(blurred_circle,blurred_circle_big,cv::Size (blurred_circle.cols*IMSCALEF,blurred_circle.rows*IMSCALEF));

    cv::Rect roi_circle(0,0,blurred_circle_big.cols,blurred_circle_big.rows);
    int x1 = p.x-blurred_circle_big.cols/2;
    if (x1 < 0) {
        roi_circle.x = abs(p.x-blurred_circle_big.cols/2);
        roi_circle.width-=roi_circle.x;
    } else if (x1 + blurred_circle_big.cols >= res.cols)
        roi_circle.width = roi_circle.width  - abs(x1 + blurred_circle_big.cols - res.cols);

    int y1 = p.y-blurred_circle_big.rows/2;
    if (y1 < 0) {
        roi_circle.y = abs(p.y-blurred_circle_big.rows/2);
        roi_circle.height-=roi_circle.y;
    } else if (y1 + blurred_circle_big.rows >= res.rows)
        roi_circle.height = roi_circle.height - abs(y1 + blurred_circle_big.rows - res.rows);

    cv::Mat gray = cv::Mat::zeros(res.rows,res.cols,CV_32F);
    gray = visdat->get_uncertainty_background();
    cv::Mat a = blurred_circle_big(roi_circle);

    x1 = p.x-blurred_circle_big.cols/2+roi_circle.x;
    int x2 = a.cols;
    y1 = p.y-blurred_circle_big.rows/2+roi_circle.y;
    int y2 = a.rows;

    cv::Rect roi(x1,y1,x2,y2);
    a.copyTo(gray(roi));
    cvtColor(res,res,CV_BGR2GRAY);
    res.convertTo(res,CV_32F);
    res = gray.mul(res);
    res.convertTo(res,CV_8UC1);
    cvtColor(res,res,CV_GRAY2BGR);

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
        predicted_drone_pathL.push_back(t);
        if (drone_pathL.size() > 30)
            drone_pathL.erase(drone_pathL.begin());
        if (predicted_drone_pathL.size() > 30)
            predicted_drone_pathL.erase(predicted_drone_pathL.begin());
        // cout << "PredictionL: " << predicted_drone_locationL << std::endl;
    }
    return predicted_drone_locationL;
}

int DroneTracker::match_closest_to_prediciton(cv::Point3f predicted_drone_locationL,std::vector<KeyPoint> keypointsL) {
    int closestL;
    if (keypointsL.size() == 1 && drone_pathL.size() == 0) {
        closestL = 0;
    } else if (drone_pathL.size() > 0) {
        //find closest keypoint to new predicted location
        int mind = 999999999;
        for (int i = 0 ; i < keypointsL.size();i++) {
            cv::KeyPoint k =keypointsL.at(i);
            int d = (predicted_drone_locationL.x-k.pt.x) * (predicted_drone_locationL.x-k.pt.x) + (predicted_drone_locationL.y-k.pt.y)*(predicted_drone_locationL.y-k.pt.y);
            if (d < mind ) {
                mind = d;
                closestL = i;
            }
        }
    } else {
        //the case that we have multiple drone candidates, no good way to figure out which one is the drone
        //guess we'll just take the first one...
        closestL = 0;
    }

    return closestL;
}

int DroneTracker::stereo_match(cv::KeyPoint closestL,cv::Mat prevFrameL_big,cv::Mat prevFrameR_big, cv::Mat frameL,cv::Mat frameR,int prevDisparity){

    //get retangle around blob / changed pixels
    float rectsize = closestL.size+1;
    if (rectsize < 3)
        rectsize = 3;
    float rectsizeX = ceil(rectsize*4.0f);
    float rectsizeY = ceil(rectsize*3.f);

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
    cv::Mat diff_L_roi;
    cv::absdiff(aL,bL,diff_L_roi);
    cv::Mat diff_L_roi_16;
    diff_L_roi.convertTo(diff_L_roi_16, CV_16UC1);

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
        cv::Mat diff_R_roi;
        cv::absdiff(aR,bR,diff_R_roi);
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

        } // for shift

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

void DroneTracker::update_tracker_ouput(Point3f measured_world_coordinates,float dt,  int n_frames_lost, cv::KeyPoint match, int disparity,cv::Point3f setpoint_world) {

    find_drone_result.best_image_locationL = match;
    find_drone_result.disparity = disparity;
    data.drone_image_locationL = find_drone_result.best_image_locationL.pt;
    drone_pathL.push_back(find_drone_result.best_image_locationL);

    data.posX = measured_world_coordinates.x;
    data.posY = measured_world_coordinates.y;
    data.posZ = measured_world_coordinates.z;
    data.disparity = find_drone_result.disparity; // tmp, should not be in data

    static float prevX,prevY,prevZ =0;
    static int detected_after_take_off = 0;
    if (n_frames_lost >= smooth_width_vel || data.reset_filters) { // tracking was regained, after n_frames_lost frames
        // data.sdisparity = -1;
        disp_smoothed.reset();
        smoother_posX.reset();
        smoother_posY.reset();
        smoother_posZ.reset();
        smoother_velX.reset();
        smoother_velY.reset();
        smoother_velZ.reset();

        detected_after_take_off = 0;
        data.reset_filters = false; // TODO also reset t_prev?
    }

    static float sdisparity;
    sdisparity = disp_smoothed.addSample(data.disparity);
    data.sdisparity = sdisparity; // tmp, should not be in data

    if (fabs((float)abs(find_drone_result.disparity) - fabs(find_drone_result.smoothed_disparity)) > 3) {
        //do better matching?
        //std::cout << find_drone_result.disparity << "-> BAM <-" << find_drone_result.smoothed_disparity << std::endl;
        //find_drone_result.update_prev_frames = false;
    }
    find_drone_result.smoothed_disparity = sdisparity;

    data.sposX = smoother_posX.addSample(data.posX);;
    data.sposY = smoother_posY.addSample(data.posY);;
    data.sposZ = smoother_posZ.addSample(data.posZ);;

    if (data.landed) {
        prevX = data.sposX;
        prevY = data.sposY;
        prevZ = data.sposZ;
    }
    float tsvelX = 0;
    float tsvelY = 0;
    float tsvelZ = 0;
    if (detected_after_take_off > smooth_width_pos) {
        data.dx = data.sposX - prevX;
        data.dy = data.sposY - prevY;
        data.dz = data.sposZ - prevZ;
        data.velX = data.dx / dt;
        data.velY = data.dy / dt;
        data.velZ = data.dz / dt;
        tsvelX = smoother_velX.addSample(data.velX);
        tsvelY = smoother_velY.addSample(data.velY);
        tsvelZ = smoother_velZ.addSample(data.velZ);
    } else { // should be done earlier???
        data.dx = 0;
        data.dy = 0;
        data.dz = 0;
        data.velX = 0;
        data.velY = 0;
        data.velZ = 0;
    }

    //if (detected_after_take_off > smooth_width_pos) { // + smooth_width_vel, but than it is too late for autotakeoff... (by filling the smoother with 0's we assume vel = 0 , which is true at take off)
    data.svelX = tsvelX;
    data.svelY = tsvelY;
    data.svelZ = tsvelZ;
    //    } else {
    //        data.svelX = 0;
    //        data.svelY = 0;
    //        data.svelZ = 0;
    //    }

    data.posErrX = data.sposX - setpoint_world.x;
    data.posErrY = data.sposY - setpoint_world.y;
    data.posErrZ = data.sposZ - setpoint_world.z;

    prevX = data.sposX;
    prevY = data.sposY;
    prevZ = data.sposZ;

    data.valid = true;
    data.dt = dt;
    detected_after_take_off++;
}

void DroneTracker::reset_tracker_ouput(int n_frames_lost) {
    if (n_frames_lost > 10 || data.landed){
        data.reset_filters = true;
        data.velX = 0;
        data.velY = 0;
        data.velZ = 0;
        data.svelX = 0;
        data.svelY = 0;
        data.svelZ = 0;
    }
    if (n_frames_lost != 0)
        n_frames_tracking = 0;
}

void DroneTracker::close () {
    std::ofstream os(settingsFile, std::ios::binary);
    cereal::BinaryOutputArchive archive( os );
    archive( settings );
}
