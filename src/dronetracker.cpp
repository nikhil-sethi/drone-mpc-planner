#include <iostream>
#include "dronetracker.h"
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "common.h"

using namespace cv;
using namespace std;

#ifdef HASSCREEN
#if 1
#define DRAWVIZS //slow!
#define TUNING
#endif


#endif

const string settingsFile = "../settings.dat";
bool DroneTracker::init(std::ofstream *logger) {
    _logger = logger;

    (*_logger) << "imLx; imLy; disparity;";
    if (checkFileExist(settingsFile)) {
        std::ifstream is(settingsFile, std::ios::binary);
        cereal::BinaryInputArchive archive( is );
        archive(settings);
    }



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

    blurred_circle = createBlurryCircle(60,settings.uncertainty_background/255.0);
}

cv::Mat DroneTracker::createBlurryCircle(int size, float background) {
    float tmp = roundf(((float)size)/4.f);
    if (fabs((tmp / 2.f) - roundf(tmp / 2.f)) < 0.01)
        tmp +=1;

    cv::Mat res(size,size,CV_32F);
    res = background;
    cv::circle(res,cv::Point(size/2,size/2),tmp,1.0f,CV_FILLED);
    cv::GaussianBlur( res, res, cv::Size( tmp, tmp ), 0, 0 );
    return res;
}

float calculateDistance(float xr, float yr, float xl, float yl) {
    return 1.0 /(xr - xl); // kaihard kebowned
}

bool foundL = false;
float t_prev = 0;
bool DroneTracker::track(cv::Mat frameL, cv::Mat frameR, cv::Mat Qf, float time, int frame_id, cv::Point3d setpoint, cv::Point3f setpoint_world) {
    updateParams();

    float dt= (time-t_prev);
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
        init_avg_prev_frame();
    }

    if (data.landed)
    {
        frameL_s_prev_OK = frameL_s_prev;
        frameL_big_prev_OK = frameL_big_prev;
        frameR_big_prev_OK = frameR_big_prev;
    }

    find_drone(frameL_small, frameL_s_prev, frameL_s_prev_OK);

    data.valid = false; // reset the flag, set to true when drone is detected properly

    static int n_frames_lost =100;
    if (find_drone_result.keypointsL.size() > 0) { //if not lost

        cv::Point3f previous_drone_location(find_drone_result.best_image_locationL .pt.x,find_drone_result.best_image_locationL .pt.y,0);
        std::vector<cv::KeyPoint> keypoint_candidates = find_drone_result.keypointsL;
        Point3f output;
        int disparity;
        cv::KeyPoint match;
        while (keypoint_candidates.size() > 0) {
            int match_id = match_closest_to_prediciton(previous_drone_location,find_drone_result.keypointsL);
            match = find_drone_result.keypointsL.at(match_id);

            disparity = stereo_match(match,frameL_big_prev_OK,frameR_big_prev_OK,frameL,frameR,find_drone_result.smoothed_disparity);



            //calculate everything for the dronecontroller:
            std::vector<Point3f> camera_coordinates, world_coordinates;
            camera_coordinates.push_back(Point3f(match.pt.x*IMSCALEF,match.pt.y*IMSCALEF,-disparity));
            //camera_coordinates.push_back(Point3f(predicted_drone_locationL.x*IMSCALEF,predicted_drone_locationL.y*IMSCALEF,-predicted_drone_locationL.z));
            cv::perspectiveTransform(camera_coordinates,world_coordinates,Qf);
            output = world_coordinates[0];
            float theta = CAMERA_ANGLE / (360/6.28318530718);
            output.y = output.y * cosf(theta) + output.z * sinf(theta);
            output.z = -output.y * sinf(theta) + output.z * cosf(theta);

            if ((output.z < -DRONE_MAX_BORDER_Z) || (output.y < -DRONE_MAX_BORDER_Z)) {
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
        }
    }



#ifdef BEEP
    beep(find_drone_result.best_image_locationL.pt,n_frames_lost,time,frameL_small);
#endif

    if (!data.background_calibrated && time > background_calib_time)
        data.background_calibrated= true;

    (*_logger) << find_drone_result.best_image_locationL.pt.x *IMSCALEF << "; " << find_drone_result.best_image_locationL.pt.y *IMSCALEF << "; " << find_drone_result.disparity << "; ";

#ifdef DRAWVIZS
    drawviz(frameL,find_drone_result.treshfL,frameL_small,data.drone_image_locationL, setpoint);
#endif


    if (find_drone_result.update_prev_frame && ! breakpause) {
        frameL_big_prev_OK = frameL_big_prev.clone();
        frameL_s_prev_OK = frameL_s_prev.clone();
        frameR_big_prev_OK = frameR_big_prev.clone();
    }
    frameL_s_prev = frameL_small.clone();
    frameL_big_prev = frameL.clone();
    frameR_big_prev = frameR.clone();

    return false;
}
void DroneTracker::beep(cv::Point2f drone, int n_frames_lost, float time, cv::Mat frameL_small) {

    if (!data.background_calibrated && time > background_calib_time) {
        system("canberra-gtk-play -f /usr/share/sounds/ubuntu/notifications/Mallet.ogg &");
    }

    //if (bam) {
    //system("canberra-gtk-play -f /usr/share/sounds/ubuntu/notifications/Soft\ delay.ogg &");
    //}

    static bool lost = true;
    if( n_frames_lost == 3 && !lost ) {
        // system("canberra-gtk-play -f /usr/share/sounds/ubuntu/notifications/Rhodes.ogg &");
        lost =true;
    }

    if (n_frames_lost < 3) {
        if (lost) {
            //system("canberra-gtk-play -f /usr/share/sounds/ubuntu/notifications/Positive.ogg &");
            lost =false;
        }

        //check if near image border:
        float marginx = 0.25*frameL_small.cols;
        float marginy = 0.25*frameL_small.rows;
        static float time_beep_prev = time;
        float beep_speed = 0.0;
        if (drone.x < marginx )
            beep_speed = ( fabs(drone.x-marginx)) / marginx;
        else if (drone.x > frameL_small.cols - marginx )
            beep_speed = ( fabs(drone.x-frameL_small.cols)) / marginx;

        if (drone.y < marginy )
            beep_speed += ( fabs(drone.y-marginy)) / marginy;
        else if (drone.y > frameL_small.rows - marginy )
            beep_speed += ( fabs(drone.y-frameL_small.rows)) / marginy;


        if (time-time_beep_prev > 0.1 / beep_speed) {
            system("canberra-gtk-play -f /usr/share/sounds/ubuntu/notifications/Blip.ogg &");
            time_beep_prev = time;
        }
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

void DroneTracker::collect_no_drone_frames(cv::Mat diff) {
    static cv::Mat max_uncertainty_map = diff.clone();
    cv::Mat mask = diff > max_uncertainty_map;
    diff.copyTo(max_uncertainty_map,mask);

    uncertainty_map = 255 - max_uncertainty_map * settings.uncertainty_multiplier;
    uncertainty_map.convertTo(uncertainty_map ,CV_32F);
    uncertainty_map /=255.0f;
    cv::pow(uncertainty_map,settings.uncertainty_power,uncertainty_map);

}

void DroneTracker::init_avg_prev_frame(void) {
    avg_prev_frame = cv::Mat::zeros(frameL_s_prev_OK.rows,frameL_s_prev_OK.cols,CV_32SC1);
    n_avg_prev_frames = 0;
}

void DroneTracker::collect_avg_prev_frame(cv::Mat frame) {
    cv::Mat frame32;
    frame.convertTo(frame32,CV_32SC1);
    n_avg_prev_frames+=1;
    avg_prev_frame +=frame32;
}

void DroneTracker::find_drone(cv::Mat frameL_small,cv::Mat frameL_s_prev, cv::Mat frameL_s_prev_OK) {
    cv::Point previous_drone_location;
    if (data.landed) {
        data.drone_image_locationL = cv::Point(DRONE_IM_X_START,DRONE_IM_Y_START);
        find_drone_result.smoothed_disparity = DRONE_DISPARITY_START;
        find_drone_result.disparity = DRONE_DISPARITY_START;
    }
    previous_drone_location = data.drone_image_locationL;

    //attempt to detect changed blobs
    cv::Mat treshfL = segment_drone(frameL_small,frameL_s_prev,true,previous_drone_location);
    SimpleBlobDetector detector(params);
    std::vector<KeyPoint> keypointsL;
    detector.detect( treshfL, keypointsL);

    static int nframes_since_update_prev = 0;
    bool still_nothing = false;
    //check if changed blobs were detected
    if (keypointsL.size() == 0) { // if not, use the last frame that was confirmed to be working before...
        treshfL = segment_drone(frameL_small,frameL_s_prev_OK,false,previous_drone_location);
        detector.detect( treshfL, keypointsL);
        nframes_since_update_prev +=1;
        if (nframes_since_update_prev > 50)
            nframes_since_update_prev = 50;
        blurred_circle = createBlurryCircle(60+nframes_since_update_prev,settings.uncertainty_background/255.0);
        if (keypointsL.size() == 0 )
            still_nothing = true;

    } else {
        blurred_circle = createBlurryCircle(60,settings.uncertainty_background/255.0);
        nframes_since_update_prev = 0;
    }

    //    if (data.landed) TMP DISABLED?
    //        nframes_since_update_prev = 0;
    find_drone_result.keypointsL = keypointsL;
    find_drone_result.treshfL = treshfL;
    find_drone_result.update_prev_frame = nframes_since_update_prev == 0 || (nframes_since_update_prev > 49 && !data.landed );
}

cv::Mat DroneTracker::segment_drone(cv::Mat frame,cv::Mat frame_prev, bool update_uncertainty_map, cv::Point previous_imageL_location) {
    cv::Mat diffL;
    cv::absdiff( frame ,frame_prev,diffL);

    if (!data.background_calibrated && update_uncertainty_map)
        collect_no_drone_frames(diffL); // calibration of background uncertainty map

    //filter out changes in areas that were noisy during calibration period
#ifdef DRAWVIZS2
    cv::Mat diff_of_diff = diffL.clone();
    cv::Mat diff_before_uncertainty = diffL.clone();;
#endif
    diffL.convertTo(diffL, CV_32F);

    cv::Mat uncertainty_drone_map = get_uncertainty_map_with_drone(previous_imageL_location);

    diffL = diffL.mul(uncertainty_drone_map);
    diffL.convertTo(diffL, CV_8UC1);

    cv::Mat treshfL;
    inRange(diffL, settings.iLowH1r, settings.iHighH1r, treshfL);
    dilate( treshfL, treshfL, getStructuringElement(MORPH_ELLIPSE, Size(settings.iClose1r+1, settings.iClose1r+1)));
    erode(treshfL, treshfL, getStructuringElement(MORPH_ELLIPSE, Size(settings.iOpen1r+1, settings.iOpen1r+1)));

#ifdef DRAWVIZS2
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

    //equalizeHist(uncertainty_map_8, uncertainty_map_8);
    std::vector<cv::Mat> ims;
    ims.push_back(uncertainty_ori_map_8);
    ims.push_back(uncertainty_drone_map_8);
    ims.push_back(diff_before_uncertainty*10);
    ims.push_back(diffL*10);
    ims.push_back(treshfL);
    ims.push_back(diff_of_diff);
    //    ims.push_back(diffL_eq);
    if (update_uncertainty_map)
        showColumnImage(ims,"uncertainty1",CV_8UC1);
    else
        showColumnImage(ims,"uncertainty2",CV_8UC1);
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
    gray = settings.uncertainty_background/255.0;
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
        predicted_dronepathL.push_back(t);
        if (dronepathL.size() > 30)
            dronepathL.erase(dronepathL.begin());
        if (predicted_dronepathL.size() > 30)
            predicted_dronepathL.erase(predicted_dronepathL.begin());
        // cout << "PredictionL: " << predicted_drone_locationL << std::endl;
    }
    return predicted_drone_locationL;
}

int DroneTracker::match_closest_to_prediciton(cv::Point3f predicted_drone_locationL,std::vector<KeyPoint> keypointsL) {
    int closestL;
    if (keypointsL.size() == 1 && dronepathL.size() == 0) {
        closestL = 0;
    } else if (dronepathL.size() > 0) {
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
#ifdef DRAWVIZS
            diff_R_roi_viz = diff_R_roi.clone();
            aR_viz = aR;
            bR_viz = bR;
#endif
        } // for shift

#ifdef DRAWVIZS
        //        if (breakpause) {
        //            cv::Mat aL_shift,bL_shift,diff_L_roi_shift,aR_shift,bR_shift,diff_R_roi_shift;
        //            cv::resize(aL,aL_shift,cv::Size(aL.cols*4, aL.rows*4));
        //            cv::resize(bL,bL_shift,cv::Size(aL.cols*4, aL.rows*4));
        //            cv::resize(diff_L_roi,diff_L_roi_shift,cv::Size(aL.cols*4, aL.rows*4));
        //            cv::resize(aR,aR_shift,cv::Size(aL.cols*4, aL.rows*4));
        //            cv::resize(bR,bR_shift,cv::Size(aL.cols*4, aL.rows*4));
        //            cv::resize(diff_R_roi,diff_R_roi_shift,cv::Size(aL.cols*4, aL.rows*4));
        //            std::stringstream ss;
        //            ss << i << "; cor: " << cor_16 << " err: " << err;
        //            putText(aL_shift,ss.str() ,cv::Point(0,12),cv::FONT_HERSHEY_SIMPLEX,0.5,255);

        //            std::vector<cv::Mat> ims;
        //            ims.push_back(aL_shift);
        //            ims.push_back(bL_shift);
        //            ims.push_back(diff_L_roi_shift);
        //            ims.push_back(aR_shift);
        //            ims.push_back(bR_shift);
        //            ims.push_back(diff_R_roi_shift);
        //            showColumnImage(ims, "shift",CV_8UC1);

        //            unsigned char key = cv::waitKey(0);
        //            if (key == 'c')
        //                breakpause = false;

        //        }
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
        //        std::vector<cv::Mat> ims;
        //        cv::Mat aL_eq,bL_eq,diff_L_roi_eq,aR_viz_eq,bR_viz_eq,diff_R_roi_viz_eq;
        //        equalizeHist(aL, aL_eq);
        //        equalizeHist(bL, bL_eq);
        //        equalizeHist(diff_L_roi, diff_L_roi_eq);

        //        equalizeHist(aR_viz, aR_viz_eq);
        //        equalizeHist(bR_viz, bR_viz_eq);
        //        equalizeHist(diff_R_roi_viz, diff_R_roi_viz_eq);

        //        ims.push_back(aL_eq);
        //        ims.push_back(bL_eq);
        //        ims.push_back(diff_L_roi_eq);
        //        ims.push_back(aR_viz_eq);
        //        ims.push_back(bR_viz_eq);
        //        ims.push_back(diff_R_roi_viz_eq);
        //        showColumnImage(ims, "col", CV_8UC1);
    }
#endif

    return disparity;
}

void DroneTracker::drawviz(cv::Mat frameL,cv::Mat treshfL,cv::Mat framegrayL,cv::Point  previous_imageL_location, cv::Point3d setpoint) {
#ifdef DRAWVIZS

    static int div = 0;
    if (div++ % 4 == 1) {

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


        //resFrameL = show_uncertainty_map_in_image(previous_imageL_location,  resFrameL);
        resFrame = resFrameL;
        cv::imshow("Results", resFrame);
    }
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

void DroneTracker::update_tracker_ouput(Point3f measured_world_coordinates,float dt,  int n_frames_lost, cv::KeyPoint match, int disparity,cv::Point3f setpoint_world) {

    find_drone_result.best_image_locationL = match;
    find_drone_result.disparity = disparity;
    data.drone_image_locationL = find_drone_result.best_image_locationL.pt;
    dronepathL.push_back(find_drone_result.best_image_locationL);

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
}

void DroneTracker::close () {
    std::ofstream os(settingsFile, std::ios::binary);
    cereal::BinaryOutputArchive archive( os );
    archive( settings );
}
