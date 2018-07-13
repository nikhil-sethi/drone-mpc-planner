#include <iostream>
#include "itemtracker.h"
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "common.h"

using namespace cv;
using namespace std;

#ifdef HASSCREEN
#define TUNING
#endif


void ItemTracker::init(std::ofstream *logger, VisionData *visdat, std::string name) {
    _logger = logger;
    _visdat = visdat;
    _settingsFile = "../" + name + "settings.dat";
    std::string window_name = name + "_trkr";

    if (checkFileExist(_settingsFile)) {
        std::ifstream is(_settingsFile, std::ios::binary);
        cereal::BinaryInputArchive archive( is );
        archive(settings);
    } else {
        init_settings();
    }

#ifdef TUNING
    namedWindow(window_name, WINDOW_NORMAL);
    createTrackbar("LowH1", window_name, &settings.iLowH1r, 255);
    createTrackbar("HighH1", window_name, &settings.iHighH1r, 255);
    createTrackbar("filterByArea", window_name, &settings.filterByArea, 1);
    createTrackbar("minArea", window_name, &settings.minArea, 10000);
    createTrackbar("maxArea", window_name, &settings.maxArea, 10000);
    createTrackbar("Opening1", window_name, &settings.iOpen1r, 30);
    createTrackbar("Closing1", window_name, &settings.iClose1r, 30);
    createTrackbar("Min disparity", window_name, &settings.min_disparity, 255);
    createTrackbar("Max disparity", window_name, &settings.max_disparity, 255);
    createTrackbar("roi_min_size", window_name, &settings.roi_min_size, 2000);
    createTrackbar("roi_max_grow", window_name, &settings.roi_max_grow, 500);
    createTrackbar("roi_grow_speed", window_name, &settings.roi_grow_speed, 256);
#endif

    kfL = cv::KalmanFilter(stateSize, measSize, contrSize, type);
    stateL =cv::Mat(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
    _measL = cv::Mat (measSize, 1, type);    // [z_x,z_y,z_w,z_h]

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
    _measR = cv::Mat (measSize, 1, type);    // [z_x,z_y,z_w,z_h]

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

    data.image_locationL = cv::Point(848/2/IMSCALEF,480/2/IMSCALEF);
    find_result.smoothed_disparity = 0;
    find_result.disparity = 0;
}

std::vector<KeyPoint> ItemTracker::remove_ignores(std::vector<KeyPoint> keypoints, std::vector<track_item> ignore_path) {
    if (ignore_path.size() > 0) {
        cv::Point2f ignore = ignore_path.at(ignore_path.size()-1)._k.pt;
        cv::Point2f ignore_prev = ignore;
        if (ignore_path.size() > 1) {
            ignore_prev = ignore_path.at(ignore_path.size()-2)._k.pt;
        }
        std::vector<KeyPoint> tmp = keypoints;
        for (uint i = 0 ; i< tmp.size();i++){
            float dis = (tmp.at(i).pt.x - ignore.x)*(tmp.at(i).pt.x - ignore.x) +(tmp.at(i).pt.y - ignore.y)*(tmp.at(i).pt.y - ignore.y);
            if (dis < 1)
                keypoints.erase(keypoints.begin() + i);
            else  if (ignore_path.size() > 1) {
                dis = (tmp.at(i).pt.x - ignore_prev.x)*(tmp.at(i).pt.x - ignore_prev.x) +(tmp.at(i).pt.y - ignore_prev.y)*(tmp.at(i).pt.y - ignore_prev.y);
                if (dis < 1)
                    keypoints.erase(keypoints.begin() + i);
            }
        }
    }
    return keypoints;
}

void ItemTracker::track(float time, cv::Point3f setpoint_world, std::vector<track_item> ignore, float drone_max_border_y, float drone_max_border_z) {
    updateParams();

    float dt= (time-t_prev);
    cv::Point3f predicted_locationL = predict(dt,_visdat->_frame_id);

    if (!firstFrame) {
        firstFrame = true;
        frameL_s_prev_OK = _visdat->_frameL_s_prev;
        frameL_prev_OK = _visdat->_frameL.clone();
        frameR_prev_OK = _visdat->_frameR.clone();
    }

    find(_visdat->_frameL_small);

    data.valid = false; // reset the flag, set to true when item is detected properly

    static int n_frames_lost =100;
    if (find_result.keypointsL.size() > 0) { //if not lost

        cv::Point3f previous_location(find_result.best_image_locationL .pt.x,find_result.best_image_locationL .pt.y,0);
        std::vector<cv::KeyPoint> keypoint_candidates = remove_ignores(find_result.keypointsL,ignore);

        Point3f output;
        int disparity;
        cv::KeyPoint match;
        while (keypoint_candidates.size() > 0) {
            int match_id = match_closest_to_prediciton(previous_location,find_result.keypointsL);
            match = find_result.keypointsL.at(match_id);

            disparity = stereo_match(match,frameL_prev_OK,frameR_prev_OK,_visdat->_frameL,_visdat->_frameR,find_result.smoothed_disparity);

            //calculate everything for the itemcontroller:
            std::vector<Point3f> camera_coordinates, world_coordinates;
            camera_coordinates.push_back(Point3f(match.pt.x*IMSCALEF,match.pt.y*IMSCALEF,-disparity));
            //camera_coordinates.push_back(Point3f(predicted_locationL.x*IMSCALEF,predicted_locationL.y*IMSCALEF,-predicted_locationL.z));
            cv::perspectiveTransform(camera_coordinates,world_coordinates,_visdat->_Qf);
            output = world_coordinates[0];
            float theta = CAMERA_ANGLE / (180/(float)M_PI);
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
            update_prediction_state(cv::Point3f(predicted_locationL.x,predicted_locationL.y,predicted_locationL.z));
            reset_tracker_ouput(n_frames_lost);
        } else {
            //Point3f predicted_output = world_coordinates[1];
            update_prediction_state(cv::Point3f(match.pt.x,match.pt.y,disparity));
            update_tracker_ouput(output,dt,n_frames_lost,match,disparity,setpoint_world,_visdat->_frame_id);
            n_frames_lost = 0; // update this after calling update_tracker_ouput, so that it can determine how long tracking was lost
            t_prev = time; // update dt only if item was detected
            n_frames_tracking++;
        }
    }

    if (find_result.update_prev_frame && ! breakpause) {
        frameL_prev_OK = _visdat->_frameL_prev.clone();
        frameL_s_prev_OK = _visdat->_frameL_s_prev.clone();
        frameR_prev_OK = _visdat->_frameR_prev.clone();
    }    
}

void ItemTracker::updateParams(){
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

void ItemTracker::find(cv::Mat frameL_small) {
    cv::Point previous_location;

    previous_location = data.image_locationL;

    cv::Point roi_size;
    if (settings.roi_min_size < 1)
        settings.roi_min_size = 1;
    roi_size.x=settings.roi_min_size/IMSCALEF+nframes_since_update_prev*(settings.roi_grow_speed / 16 / IMSCALEF);
    roi_size.y=settings.roi_min_size/IMSCALEF+nframes_since_update_prev*(settings.roi_grow_speed / 16 / IMSCALEF);

    if (roi_size.x  >= _visdat->_frameL_small.cols) {
        roi_size.x = _visdat->_frameL_small.cols;
    }

    if (roi_size.y >= _visdat->_frameL_small.rows)
        roi_size.y = _visdat->_frameL_small.rows;

    //attempt to detect changed blobs
    _treshL = segment(_visdat->diffL,previous_location,roi_size);
#if CV_MAJOR_VERSION==3
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
#else
    SimpleBlobDetector * detector;
    detector = *SimpleBlobDetector(params);
#endif

    std::vector<KeyPoint> keypointsL;
    detector->detect( _treshL, keypointsL);

    //check if changed blobs were detected
    if (keypointsL.size() == 0) { // if not, use the last frame that was confirmed to be working before...
        cv::Mat diffL_OK;
        cv::absdiff( frameL_small ,frameL_s_prev_OK,diffL_OK);
        _treshL = segment(diffL_OK,previous_location,roi_size);
        detector->detect( _treshL, keypointsL);
        nframes_since_update_prev +=1;
        if (nframes_since_update_prev > settings.roi_max_grow)
            nframes_since_update_prev = settings.roi_max_grow;

    } else {
        nframes_since_update_prev = 0;
    }




    for (uint i = 0 ; i < keypointsL.size();i++) {
        keypointsL.at(i).pt.x += find_result.roi_offset.x;
        keypointsL.at(i).pt.y += find_result.roi_offset.y;
    }

    find_result.keypointsL = keypointsL;
    find_result.treshL = _treshL;
    find_result.update_prev_frame = nframes_since_update_prev == 0 || (nframes_since_update_prev >= settings.roi_max_grow  );
}

cv::Mat ItemTracker::segment(cv::Mat diffL, cv::Point previous_imageL_location, cv::Point roi_size) {

    _approx = get_approx_cutout_filtered(previous_imageL_location,diffL,roi_size);

    inRange(_approx, settings.iLowH1r, settings.iHighH1r, _treshL);
    dilate( _treshL, _treshL, getStructuringElement(MORPH_ELLIPSE, Size(settings.iClose1r+1, settings.iClose1r+1)));
    erode(_treshL, _treshL, getStructuringElement(MORPH_ELLIPSE, Size(settings.iOpen1r+1, settings.iOpen1r+1)));

    return _treshL;
}


cv::Mat ItemTracker::get_probability_cloud(cv::Point size) {
    return cv::Mat::ones(size.y,size.x,CV_32F); // dummy implementation. TODO: create proper probablity estimate
}

/* Takes the calibrated uncertainty map, and augments it with a highlight around p */
cv::Mat ItemTracker::show_uncertainty_map_in_image(cv::Point pd4, cv::Mat res) {

    cv::Point p;
    p.x = pd4.x *IMSCALEF;
    p.y = pd4.y *IMSCALEF;

    cv::Mat blurred_circle_big;
    cv::resize(_blurred_circle,blurred_circle_big,cv::Size (_blurred_circle.cols*IMSCALEF,_blurred_circle.rows*IMSCALEF));

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
    gray = _visdat->get_uncertainty_background();
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

cv::Point3f ItemTracker::predict(float dt, int frame_id) {
    cv::Point3f predicted_locationL;
    if (foundL) {
        kfL.transitionMatrix.at<float>(2) = dt;
        kfL.transitionMatrix.at<float>(9) = dt;
        stateL = kfL.predict();
        cv::Rect predRect;
        predRect.width = stateL.at<float>(4);
        predRect.height = stateL.at<float>(5);
        predRect.x = stateL.at<float>(0) - predRect.width / 2;
        predRect.y = stateL.at<float>(1) - predRect.height / 2;

        predicted_locationL.x = stateL.at<float>(0);
        predicted_locationL.y = stateL.at<float>(1);
        predicted_locationL.z = stateL.at<float>(2);
        cv::KeyPoint t;
        cv::Point beun;
        beun.x = predicted_locationL.x;
        beun.y = predicted_locationL.y;
        t.pt = beun;
        t.size = 3;
        predicted_pathL.push_back(track_item(t,frame_id));
        if (pathL.size() > 30)
            pathL.erase(pathL.begin());
        if (predicted_pathL.size() > 30)
            predicted_pathL.erase(predicted_pathL.begin());
        // cout << "PredictionL: " << predicted_locationL << std::endl;
    }
    return predicted_locationL;
}

int ItemTracker::match_closest_to_prediciton(cv::Point3f predicted_locationL, std::vector<KeyPoint> keypointsL) {
    int closestL;
    if (keypointsL.size() == 1 && pathL.size() == 0) {
        closestL = 0;
    } else if (pathL.size() > 0) {
        //find closest keypoint to new predicted location
        int mind = 999999999;
        for (uint i = 0 ; i < keypointsL.size();i++) {
            cv::KeyPoint k =keypointsL.at(i);
            int d = (predicted_locationL.x-k.pt.x) * (predicted_locationL.x-k.pt.x) + (predicted_locationL.y-k.pt.y)*(predicted_locationL.y-k.pt.y);
            if (d < mind ) {
                mind = d;
                closestL = i;
            }
        }
    } else {
        //the case that we have multiple item candidates, no good way to figure out which one is the item
        //guess we'll just take the first one...
        closestL = 0;
    }

    return closestL;
}

int ItemTracker::stereo_match(cv::KeyPoint closestL,cv::Mat prevFrameL_big,cv::Mat prevFrameR_big, cv::Mat frameL,cv::Mat frameR,int prevDisparity){

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


void ItemTracker::update_prediction_state(cv::Point3f p) {
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

void ItemTracker::update_tracker_ouput(Point3f measured_world_coordinates,float dt,  int n_frames_lost, cv::KeyPoint match, int disparity,cv::Point3f setpoint_world, int frame_id) {

    find_result.best_image_locationL = match;
    find_result.disparity = disparity;
    data.image_locationL = find_result.best_image_locationL.pt;
    pathL.push_back(track_item(find_result.best_image_locationL,frame_id));

    data.posX = measured_world_coordinates.x;
    data.posY = measured_world_coordinates.y;
    data.posZ = measured_world_coordinates.z;
    data.disparity = find_result.disparity; // tmp, should not be in data

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

    if (fabs((float)abs(find_result.disparity) - fabs(find_result.smoothed_disparity)) > 3) {
        //do better matching?
        //std::cout << find_result.disparity << "-> BAM <-" << find__result.smoothed_disparity << std::endl;
        //find_result.update_prev_frames = false;
    }
    find_result.smoothed_disparity = sdisparity;

    data.sposX = smoother_posX.addSample(data.posX);;
    data.sposY = smoother_posY.addSample(data.posY);;
    data.sposZ = smoother_posZ.addSample(data.posZ);;

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

void ItemTracker::reset_tracker_ouput() {

    data.reset_filters = true;
    data.velX = 0;
    data.velY = 0;
    data.velZ = 0;
    data.svelX = 0;
    data.svelY = 0;
    data.svelZ = 0;
}

void ItemTracker::reset_tracker_ouput(int n_frames_lost) {
    if (n_frames_lost > 10){
        reset_tracker_ouput();
    }
    if (n_frames_lost != 0)
        n_frames_tracking = 0;
}

void ItemTracker::close () {
    std::ofstream os(_settingsFile, std::ios::binary);
    cereal::BinaryOutputArchive archive( os );
    archive( settings );
}
