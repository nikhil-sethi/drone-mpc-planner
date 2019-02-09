#include <iostream>
#include "itemtracker.h"
#include <opencv2/features2d.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "common.h"
#include "vector"
#include "algorithm"
using namespace cv;
using namespace std;

#ifdef HASSCREEN
//#define TUNING
#endif

//#define OPENCV_BLOBTRACKER
//TODO: remove _treshL when removing OPENCV_BLOBTRACKER
// also remove the uncertainty_map, the max_uncertainty_map should give all necesary info

void ItemTracker::init(std::ofstream *logger, VisionData *visdat, std::string name) {
    _logger = logger;
    _visdat = visdat;
    _name = name;
    _settingsFile = "../" + name + "settings.dat";
    std::string window_name = name + "_trkr";

    if (checkFileExist(_settingsFile)) {
        std::ifstream is(_settingsFile, std::ios::binary);
        cereal::BinaryInputArchive archive( is );
        try {
            archive(settings);
        }catch (cereal::Exception e) {
            std::cout << "Itemtracker settings file error: " << e.what() << std::endl;
            std::cout << "Maybe delete the file: " << _settingsFile << std::endl;
            exit (1);
        }
        TrackerSettings tmp;
        if (settings.version < tmp.version){
            std::cout << "Itemtracker settings version too low!" << std::endl;
            std::cout << "Maybe delete the file: " << _settingsFile << std::endl;
            throw my_exit(1);
        }
    } else {
        init_settings();
    }

#ifdef TUNING
    namedWindow(window_name, WINDOW_NORMAL);
#ifdef OPENCV_BLOBTRACKER
    createTrackbar("LowH1", window_name, &settings.iLowH1r, 255);
    createTrackbar("HighH1", window_name, &settings.iHighH1r, 255);
    createTrackbar("filterByArea", window_name, &settings.filterByArea, 1);
    createTrackbar("minArea", window_name, &settings.minArea, 10000);
    createTrackbar("maxArea", window_name, &settings.maxArea, 10000);
    createTrackbar("Opening1", window_name, &settings.iOpen1r, 30);
    createTrackbar("Closing1", window_name, &settings.iClose1r, 30);
#else
    createTrackbar("#points / frame", window_name, &settings.max_points_per_frame, 255);
    createTrackbar("ignore circle size", window_name, &settings.ignore_circle_r_around_motion_max, 255);
    createTrackbar("Motion threshold", window_name, &settings.motion_thresh, 255);
#endif
    createTrackbar("Min disparity", window_name, &settings.min_disparity, 255);
    createTrackbar("Max disparity", window_name, &settings.max_disparity, 255);
    createTrackbar("roi_min_size", window_name, &settings.roi_min_size, 2000);
    createTrackbar("roi_max_grow", window_name, &settings.roi_max_grow, 500);
    createTrackbar("roi_grow_speed", window_name, &settings.roi_grow_speed, 256);
    createTrackbar("appear_void_max_distance", window_name, &settings.appear_void_max_distance, 250);
    createTrackbar("void_void_max_distance", window_name, &settings.void_void_max_distance, 20);
    createTrackbar("exclude_min_distance", window_name, &settings.exclude_min_distance, 250);
    createTrackbar("background_subtract_zone_factor", window_name, &settings.background_subtract_zone_factor, 100);

#endif

    init_kalman();

    updateParams();

    smoother_posX.init(smooth_width_pos);
    smoother_posY.init(smooth_width_pos);
    smoother_posZ.init(smooth_width_pos);

    disp_rate_smoothed2.init(6,0.4f);
    disp_smoothed.init(smooth_width_pos);

    smoother_velX2.init(6,0.4f);
    smoother_velY2.init(6,0.4f);
    smoother_velZ2.init(6,0.4f);

    smoother_velX.init(smooth_width_vel);
    smoother_velY.init(smooth_width_vel);
    smoother_velZ.init(smooth_width_vel);

    smoother_accX2.init(6,0.4f);
    smoother_accY2.init(6,0.4f);
    smoother_accZ2.init(6,0.4f);

    smoother_accX.init(smooth_width_acc);
    smoother_accY.init(smooth_width_acc);
    smoother_accZ.init(smooth_width_acc);

    find_result.best_image_locationL.pt.x = IMG_W/2/IMSCALEF;
    find_result.best_image_locationL.pt.y = IMG_H/2/IMSCALEF;
    find_result.disparity = 0;

    sub_disparity = 0;
    disparity_smoothed = 0;

    (*_logger) << "imLx_" << _name << "; ";
    (*_logger) << "imLy_" << _name << "; ";
    (*_logger) << "disparity_" << _name << "; ";
    (*_logger) << "imLx_pred_" << _name << "; ";
    (*_logger) << "imLy_pred_" << _name << "; ";
    (*_logger) << "n_frames_lost_" << _name << "; ";
    (*_logger) << "n_frames_tracking_" << _name << "; ";
    (*_logger) << "foundL_" << _name << "; ";
    (*_logger) << "posX_" << _name << "; ";
    (*_logger) << "posY_" << _name << "; ";
    (*_logger) << "posZ_" << _name << "; ";
    (*_logger) << "sposX_" << _name << "; ";
    (*_logger) << "sposY_" << _name << "; ";
    (*_logger) << "sposZ_" << _name << "; ";
    (*_logger) << "velX_" << _name << "; ";
    (*_logger) << "velY_" << _name << "; ";
    (*_logger) << "velZ_" << _name << "; ";
    (*_logger) << "accX_" << _name << "; ";
    (*_logger) << "accY_" << _name << "; ";
    (*_logger) << "accZ_" << _name << "; ";
}

void ItemTracker::init_kalman() {
    kfL = cv::KalmanFilter(stateSize, measSize, contrSize, type);
    stateL =cv::Mat(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
    _measL = cv::Mat (measSize, 1, type);    // [z_x,z_y,z_w,z_h]

    //TODO: tune kalman init values
    cv::setIdentity(kfL.transitionMatrix);
    kfL.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    kfL.measurementMatrix.at<float>(0) = 1.0f;
    kfL.measurementMatrix.at<float>(7) = 1.0f;
    kfL.measurementMatrix.at<float>(16) = 1.0f;
    kfL.measurementMatrix.at<float>(23) = 1.0f;

    kfL.processNoiseCov.at<float>(0) = 1e-2f;
    kfL.processNoiseCov.at<float>(7) = 1e-2f;
    kfL.processNoiseCov.at<float>(14) = 0.1f;
    kfL.processNoiseCov.at<float>(21) = 0.1f;
    kfL.processNoiseCov.at<float>(28) = 1e-2f;
    kfL.processNoiseCov.at<float>(35) = 1e-2f;

    // Measures Noise Covariance Matrix R
    cv::setIdentity(kfL.measurementNoiseCov, cv::Scalar(1e-1));
}

std::vector<ItemTracker::track_item> ItemTracker::remove_excludes(std::vector<track_item> keypoints, std::vector<track_item> exclude_path,std::vector<cv::Point2f> additional_ignores) {
    float dis1,dis2,dis = 0;

    //    if (_name.compare("insect")==0 && keypoints.size()>0){
    //        std::cout << "insect" << std::endl;
    //    }

    //    if (_name.compare("drone")==0 && keypoints.size()>0){
    //        std::cout << "drone" << std::endl;
    //    }


    for (uint j=0; j<additional_ignores.size();j++){
        float min_dis = 9999;
        uint min_dis_i;
        for (uint i=0; i<keypoints.size();i++){
            //find the keypoint closest to the ignore
            float d = sqrtf(powf(keypoints.at(i).x()-additional_ignores.at(j).x,2) + powf(keypoints.at(i).y()-additional_ignores.at(j).y,2));
            if (min_dis > d) {
                min_dis = d;
                min_dis_i = i;
            }
        }
        //and delete it, if it is close enough
        if (min_dis < settings.exclude_max_distance)
            keypoints.erase(keypoints.begin() + min_dis_i);
    }

    if (exclude_path.size() > 0 && keypoints.size ()>0) {
        track_item exclude = exclude_path.at(exclude_path.size()-1);
        track_item exclude_prev = exclude;
        if (exclude_path.size() > 1) {
            exclude_prev = exclude_path.at(exclude_path.size()-2);
        }
        std::vector<track_item> tmp = keypoints;
        int erase_cnt =0;
        for (uint i = 0 ; i< tmp.size();i++){
            dis1 = sqrtf(powf(tmp.at(i).k.pt.x - exclude.x(),2) +powf(tmp.at(i).k.pt.y - exclude.y(),2)) - exclude.k.size;
            dis2 = sqrtf(powf(tmp.at(i).k_void.pt.x - exclude.x(),2) +powf(tmp.at(i).k_void.pt.y - exclude.y(),2));
            float certainty_this_kp = calc_certainty(tmp.at(i).k);

            float threshold_dis = settings.exclude_min_distance / sqrtf(exclude.tracking_certainty);
            if (threshold_dis > settings.exclude_max_distance)
                threshold_dis = settings.exclude_max_distance;
            if ((dis1 < threshold_dis|| dis2 < threshold_dis)) {// TODO: && certainty_this_kp <= exclude.tracking_certainty) {
                keypoints.erase(keypoints.begin() + i - erase_cnt);
                erase_cnt++;
            } else  if (exclude_path.size() > 1) {
                dis = sqrtf(powf(tmp.at(i).x() - exclude_prev.x(),2) +powf(tmp.at(i).y() - exclude_prev.y(),2));
                threshold_dis = settings.exclude_min_distance / sqrtf(exclude_prev.tracking_certainty);
                if (threshold_dis > settings.exclude_max_distance)
                    threshold_dis = settings.exclude_max_distance;
                if (dis < threshold_dis && certainty_this_kp < exclude_prev.tracking_certainty) {
                    keypoints.erase(keypoints.begin() + i - erase_cnt);
                    erase_cnt++;
                }
            }
        }
    }
    //if (exclude_path.size() > 10 && keypoints.size()>0 && _name == "insect") {
    //    std::cout << "hmmm" << std::endl;
    //}
    return keypoints;
}

std::vector<ItemTracker::track_item> ItemTracker::remove_excludes_improved(std::vector<track_item> keypoints, std::vector<track_item> exclude_path) {
    float dis1,dis3,certainty_prediction = 0;
    bool check1,check2,check3;

    /*    if (_name.compare("insect")==0 && keypoints.size()>0){
        std::cout << "insect" << std::endl;
    }

    if (_name.compare("drone")==0 && keypoints.size()>0){
        std::cout << "drone" << std::endl;
    }*/



    if (exclude_path.size() > 0 && keypoints.size () > 0 && (this->predicted_pathL.size() > 0 || !this->foundL )  ) {
        std::vector<track_item> tmp = keypoints;
        int erase_cnt =0;
        for (uint i = 0 ; i< tmp.size();i++){
            for (uint j = 0; j<exclude_path.size(); j++){
                track_item exclude = exclude_path.at(j);

                float threshold_dis = settings.exclude_min_distance / sqrtf(exclude.tracking_certainty);
                if (threshold_dis > settings.exclude_max_distance)
                    threshold_dis = settings.exclude_max_distance;

                dis1 = sqrtf(powf(tmp.at(i).k.pt.x - exclude.x(),2) +powf(tmp.at(i).k.pt.y - exclude.y(),2)) - exclude.k.size;
                if (dis1 < 0)
                    dis1 = 0;
                if (this->predicted_pathL.size() > 0) {
                    dis3 = sqrtf(powf(tmp.at(i).k.pt.x - this->predicted_pathL.back().x(),2) +powf(tmp.at(i).k.pt.y - this->predicted_pathL.back().y(),2)) + exclude.k.size;
                    certainty_prediction = this->predicted_pathL.back().tracking_certainty;
                } else {
                    dis3 = threshold_dis;
                    certainty_prediction = 0.1f;
                }

                check1 = (dis1 < dis3 && dis1 < threshold_dis); // if keypoint is nearer to and close to other item
                check2 = fabs(dis1-dis3) < 2.0f && dis3 < 2.0f; // ignore zone
                check3 = dis3 > (settings.exclude_min_distance / sqrtf(certainty_prediction)); // if keypoint is too far away
                if ( check1 || check2 || check3 ) {
                    keypoints.erase(keypoints.begin() + i - erase_cnt);
                    erase_cnt++;
                    break;
                }
            }
        }
    }

    return keypoints;
}

void ItemTracker::track(float time, std::vector<track_item> exclude,std::vector<cv::Point2f> additional_ignores) {
#ifdef TUNING
    updateParams();
#endif

    float dt_tracking= (time-t_prev_tracking);
    float dt_predict= (time-t_prev_predict);
    t_prev_predict = time;

    cv::Point3f predicted_locationL;
    if (foundL) {
        predicted_locationL = predict(dt_predict,_visdat->frame_id);
        predicted_locationL_last = predicted_locationL;
    } else {
        predicted_locationL = predicted_locationL_last;
    }

    find(exclude,additional_ignores);

    std::vector<track_item> keypoint_candidates = find_result.keypointsL_wihout_voids;
    uint nCandidates = static_cast<uint>(keypoint_candidates.size());
    if (nCandidates) { //if nokeypointsLt lost

        cv::Point3f previous_location(find_result.best_image_locationL.pt.x,find_result.best_image_locationL.pt.y,0);

        Point3f output;
        float disparity = 0;
        track_item * match;

        while (keypoint_candidates.size() > 0) {
            uint match_id = match_closest_to_prediciton(previous_location,find_result.keypointsL_wihout_voids);
            match = &find_result.keypointsL_wihout_voids.at(match_id);

            disparity = stereo_match(match->k,_visdat->diffL,_visdat->diffR,find_result.disparity);
            disparity = update_disparity(disparity, dt_tracking);

            bool background_check_ok = true;
            bool disparity_in_range = true;
            if (disparity < settings.min_disparity || disparity > settings.max_disparity){
                disparity_in_range = false;
            } else {
                //calculate everything for the itemcontroller:
                std::vector<Point3d> camera_coordinates, world_coordinates;
                camera_coordinates.push_back(Point3d(match->x()*IMSCALEF,match->y()*IMSCALEF,disparity));
                cv::perspectiveTransform(camera_coordinates,world_coordinates,_visdat->Qf);
                output = world_coordinates[0];

                float dist_back = _visdat->depth_background_mm.at<float>(match->y()*IMSCALEF,match->x()*IMSCALEF);
                float dist_meas = sqrtf(powf(output.x,2) + powf(output.y,2) +powf(output.z,2));
                if (dist_meas > dist_back*(static_cast<float>(settings.background_subtract_zone_factor)/100.f))
                    background_check_ok = false;

                find_result.keypointsL_wihout_voids.at(match_id).distance = dist_meas;
                find_result.keypointsL_wihout_voids.at(match_id).distance_background = dist_back;

                float theta = _visdat->camera_angle * deg2rad;
                float temp_y = output.y * cosf(theta) + output.z * sinf(theta);
                output.z = -output.y * sinf(theta) + output.z * cosf(theta);
                output.y = temp_y;
            }
            if ((!background_check_ok && _enable_background_check) || !disparity_in_range) {
                keypoint_candidates.erase(keypoint_candidates.begin() + match_id);
            } else {
                break;
            }
        }
        nCandidates = static_cast<uint>(keypoint_candidates.size());
        if (keypoint_candidates.size() > 0) { // if !lost
            //Point3f predicted_output = world_coordinates[1];
            check_consistency(cv::Point3f(this->prevX,this->prevY,this->prevZ),output);
            update_tracker_ouput(output,dt_tracking,match,disparity);
            update_prediction_state(cv::Point3f(match->x(),match->y(),disparity),match->k.size);
            n_frames_lost = 0; // update this after calling update_tracker_ouput, so that it can determine how long tracking was lost
            t_prev_tracking = time; // update dt only if item was detected
            n_frames_tracking++;
            roi_size_cnt = 0;

        }
    }
    if (nCandidates == 0) {
        roi_size_cnt ++; //TODO: same as n_frames_lost?
        n_frames_lost++;
        if (roi_size_cnt > settings.roi_max_grow)
            roi_size_cnt = settings.roi_max_grow;

        if( n_frames_lost >= n_frames_lost_threshold || !foundL ) {
            foundL = false;
            reset_tracker_ouput();
        } else
            update_prediction_state(cv::Point3f(predicted_locationL.x,predicted_locationL.y,predicted_locationL.z),blob_size_last);

        if (n_frames_lost != 0)
            n_frames_tracking = 0;

    }

    if (pathL.size() > 0) {
        if (pathL.begin()->frame_id < _visdat->frame_id - path_buf_size)
            pathL.erase(pathL.begin());
        if (pathL.begin()->frame_id > _visdat->frame_id ) //at the end of a realsense video loop, frame_id resets
            pathL.clear();
    }
    if (predicted_pathL.size() > 0) {
        if (predicted_pathL.begin()->frame_id < _visdat->frame_id - path_buf_size)
            predicted_pathL.erase(predicted_pathL.begin());
        if (predicted_pathL.begin()->frame_id > _visdat->frame_id ) //at the end of a realsense video loop, frame_id resets
            predicted_pathL.clear();
    }


    append_log();

}

void ItemTracker::append_log() {
    //log all image stuff
    if (pathL.size()>0)
        (*_logger) << pathL.at(pathL.size()-1).x() * IMSCALEF << "; " << pathL.at(pathL.size()-1).y() * IMSCALEF << "; " << find_result.disparity << "; ";
    else
        (*_logger) << -1 << "; " << -1 << "; " << -1 << "; ";
    if (predicted_pathL.size()>0)
        (*_logger) << predicted_pathL.at(predicted_pathL.size()-1).x() * IMSCALEF << "; " << predicted_pathL.at(predicted_pathL.size()-1).y() * IMSCALEF << "; ";
    else
        (*_logger) << -1 << "; " << -1   << "; ";

    (*_logger) << n_frames_lost << "; " << n_frames_tracking << "; " << foundL << "; ";
    //log all world stuff
    trackData last = Last_track_data();
    (*_logger) << last.posX << "; " << last.posY << "; " << last.posZ << ";" ;
    (*_logger) << last.sposX << "; " << last.sposY << "; " << last.sposZ << ";";
    (*_logger) << last.svelX << "; " << last.svelY << "; " << last.svelZ << ";";
    (*_logger) << last.saccX << "; " << last.saccY << "; " << last.saccZ << ";";
}

void ItemTracker::updateParams(){
    // Change thresholds
    params.minThreshold = settings.minThreshold+1;
    params.maxThreshold = settings.maxThreshold+1;

    // Filter by Area.
    params.filterByArea = settings.filterByArea;
    params.minArea = (settings.minArea/(IMSCALEF*IMSCALEF));
    params.maxArea = (settings.maxArea/(IMSCALEF*IMSCALEF));

    // Filter by Circularity
    params.filterByCircularity = settings.filterByCircularity;
    params.minCircularity = static_cast<float>(settings.minCircularity)/100.0f;
    params.maxCircularity = static_cast<float>(settings.maxCircularity)/100.0f;

    // Filter by Convexity
    params.filterByConvexity = settings.filterByConvexity;
    params.minConvexity = static_cast<float>(settings.minConvexity)/100.0f;
    params.maxConvexity = static_cast<float>(settings.maxConvexity)/100.0f;

    // Filter by Inertia
    params.filterByInertia = settings.filterByInertia;
    params.minInertiaRatio = static_cast<float>(settings.minInertiaRatio)/100.0f;
    params.maxInertiaRatio = static_cast<float>(settings.maxInertiaRatio)/100.0f;

    params.minRepeatability = 1;
    params.minDistBetweenBlobs=0;
    params.filterByColor = 0;
    params.thresholdStep=1;


}


void ItemTracker::find(std::vector<track_item> exclude,std::vector<cv::Point2f> additional_ignores) {
    cv::Point previous_location;

    previous_location = find_result.best_image_locationL.pt;

    cv::Point roi_size;
    if (settings.roi_min_size < 1)
        settings.roi_min_size = 1;

    if (_enable_roi) {
        roi_size.x=settings.roi_min_size/IMSCALEF+roi_size_cnt*(settings.roi_grow_speed / 16 / IMSCALEF);
        roi_size.y=settings.roi_min_size/IMSCALEF+roi_size_cnt*(settings.roi_grow_speed / 16 / IMSCALEF);

        if (roi_size.x  >= _visdat->smallsize.width) {
            roi_size.x = _visdat->smallsize.width;
        }

        if (roi_size.y >= _visdat->smallsize.height)
            roi_size.y = _visdat->smallsize.height;
    } else {
        roi_size.x = _visdat->smallsize.width;
        roi_size.y = _visdat->smallsize.height;
        previous_location.x = roi_size.x / 2;
        previous_location.y = roi_size.y / 2;
    }


    std::vector<KeyPoint> keypointsL;
#ifdef OPENCV_BLOBTRACKER
    //attempt to detect changed blobs
    _treshL = segment(_visdat->diffL_small,previous_location,roi_size);
    //    cv::Mat tmp = createColumnImage({_treshL,_visdat->diffL*100,_visdat->frameL},CV_8UC1,0.25f);
    //    cv::imshow("trek",tmp);
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    detector->detect( _treshL, keypointsL);
#else
    find_max_change(previous_location,roi_size,_visdat->diffL_small,&keypointsL);

#endif
    vector<track_item> kps;
    for (uint i = 0 ; i < keypointsL.size();i++) {
        keypointsL.at(i).pt.x += find_result.roi_offset.x;
        keypointsL.at(i).pt.y += find_result.roi_offset.y;
        track_item t( keypointsL.at(i),_visdat->frame_id,0);
        kps.push_back(t);
    }

    // TODO: verify if remove_voids() can help improve tracking. Currently it does not seem to do so, as it does not exclude keypoints very frequently
    //       When it is does exclude keypoints, these are often 'good' keypoints, that are excluded because they seem to form a pair with keypoints which
    //       are actually 'bad' detections that are too far away to be correct.
    //std::vector<track_item> keypoint_candidates;
    //keypoint_candidates = remove_voids(kps,find_result.keypointsL);
    find_result.keypointsL_wihout_voids = remove_excludes(kps,exclude,additional_ignores);

    if (find_result.keypointsL_wihout_voids.size() ==0) {
        roi_size_cnt +=1;
        if (roi_size_cnt > settings.roi_max_grow)
            roi_size_cnt = settings.roi_max_grow;

    }
    find_result.excludes = exclude;
    find_result.keypointsL = kps;
    find_result.treshL = _treshL;
}

void ItemTracker::find_max_change(cv::Point prev,cv::Point roi_size,cv::Mat diff,std::vector<KeyPoint> * scored_points) {

    _approx = get_approx_cutout_filtered(prev,diff,roi_size);
    cv::Mat frame = _approx;

    //    cv::Rect r1(prev.x-roi_size.x/2, prev.y-roi_size.y/2, roi_size.x,roi_size.y);
    //    if (r1.x < 0)
    //        r1.x = 0;
    //    else if (r1.x+r1.width >= diff.cols)
    //        r1.x -= (r1.x+r1.width+1) - diff.cols;
    //    if (r1.y < 0)
    //        r1.y = 0;
    //    else if (r1.y+r1.height >= diff.rows)
    //        r1.y -= (r1.y+r1.height+1) - diff.rows ;
    //    cv::Mat frame(diff,r1);
    //    find_result.roi_offset = r1;


    int radius = settings.ignore_circle_r_around_motion_max;

    for (int i = 0; i < settings.max_points_per_frame; i++) {
        cv::Point mint;
        cv::Point maxt;
        double min, max;
        cv::minMaxLoc(frame, &min, &max, &mint, &maxt);

        cv::Mat bkg_frame_cutout = _visdat->max_uncertainty_map(find_result.roi_offset);
        uint8_t bkg = bkg_frame_cutout.at<uint8_t>(maxt.y,maxt.x);

        if (max > bkg+settings.motion_thresh) {
            //find the COG:
            //get the Rect containing the max movement:
            cv::Rect r2(maxt.x-radius, maxt.y-radius, radius*2,radius*2);
            if (r2.x < 0)
                r2.x = 0;
            else if (r2.x+r2.width >= frame.cols)
                r2.x -= (r2.x+r2.width+1) - frame.cols;
            if (r2.y < 0)
                r2.y = 0;
            else if (r2.y+r2.height >= frame.rows)
                r2.y -= (r2.y+r2.height+1) - frame.rows ;
            cv::Mat roi(frame,r2); // so, this is the cut out around the max point, in the cut out of _approx roi

            // make a black mask, same size:
            cv::Mat mask = cv::Mat::zeros(roi.size(), roi.type());
            // with a white, filled circle in it:
            circle(mask, Point(radius,radius), radius, 32765, -1);
            // combine roi & mask:
            cv::Mat cropped = roi & mask;

            cv::Moments mo = cv::moments(cropped,false);
            cv::Point2f COG = cv::Point(static_cast<float>(mo.m10) / static_cast<float>(mo.m00), static_cast<float>(mo.m01) / static_cast<float>(mo.m00));

            // relative it back to the _approx frame
            COG.x += r2.x;
            COG.y += r2.y;

            scored_points->push_back(cv::KeyPoint(COG, max));

            //remove this maximum:
            cv::circle(frame, maxt, 1, cv::Scalar(0), radius);

        } else {
            break;
        }
    }
}
//filter out keypoints that are caused by the drone leaving the spot (voids)
//we prefer keypoints that are of the new location of the drone (an appearance)
//this is somewhat ambigious, because the drone might 'move' to the almost same position in some cases
//todo: smooth extrapolate if the drone did not move enough to generate two seperate keypoints
std::vector<ItemTracker::track_item> ItemTracker::remove_voids(std::vector<track_item> keyps,std::vector<track_item> keyps_prev) {
    vector<track_item> keyps_no_voids =  keyps; // init with a copy of all current keypoints
    int n_erased_items = 0;
    if (keyps.size() > 1 && keyps_prev.size() > 0) { // in order to detect a void-appearance pair, there should be at least two current keypoints now, and one in the past
        for (uint i = 0 ; i < keyps.size();i++) {
            for (uint j = 0 ; j < keyps_prev.size();j++) {
                KeyPoint k1,k2;
                k1 = keyps.at(i).k;
                k2 = keyps_prev.at(j).k;
                float d1_2 = sqrtf(static_cast<float>(pow(k1.pt.x - k2.pt.x,2) + pow(k1.pt.y - k2.pt.y,2)));
                if (d1_2 < settings.appear_void_max_distance) { // found a keypoint in the prev list that is very close to one in the current list
                    KeyPoint closest_k1;
                    int closest_k1_id = -1;
                    float closest_k1_d = 99999999;
                    for (uint k = 0 ; k < keyps.size();k++) {
                        //find the closest item to k1 in the current list and *assume* together it forms a void-appearance pair
                        if (k!=i) {
                            KeyPoint  k1_mate = keyps.at(k).k;
                            float k1_mate_d = sqrtf(static_cast<float>(pow(k1.pt.x - k1_mate.pt.x,2) + pow(k1.pt.y - k1_mate.pt.y,2)));
                            if (k1_mate_d < closest_k1_d) {
                                closest_k1 = k1_mate;
                                closest_k1_d = k1_mate_d;
                                closest_k1_id = static_cast<int>(k);
                            }
                        }
                    }

                    // If there is no mate, don't do anything.
                    if (closest_k1_d < settings.void_void_max_distance && closest_k1_id >= 0) { // the threshold number depends on the speed of the drone, and dt
                        // If a mate is found we need to verify which of the items in the pair is the void.
                        // The void should be the one closest to k2
                        float dmate_2 = sqrtf(powf(k2.pt.x - closest_k1.pt.x,2) + powf(k2.pt.y - closest_k1.pt.y,2));
                        if (d1_2< dmate_2) { //k1 is apparantely the one closest to k2, closest_k1 should be the new position of the item
                            keyps_no_voids.erase(keyps_no_voids.begin() -n_erased_items + i,keyps_no_voids.begin() -n_erased_items + i+1);
                            keyps.at(static_cast<unsigned long>(closest_k1_id)).k_void = k1;
                            keyps.at(i).k_void = closest_k1;
                            n_erased_items++;
                            j=static_cast<uint>(keyps_prev.size())+1; //break from for j
                        }
                    }
                }
            }
        }
    }
    return keyps_no_voids;
}

/* binary segment the motion subtraction, and apply some filtering */
cv::Mat ItemTracker::segment(cv::Mat diffL, cv::Point previous_imageL_location, cv::Point roi_size) {

    _approx = get_approx_cutout_filtered(previous_imageL_location,diffL,roi_size);
    inRange(_approx, settings.iLowH1r, settings.iHighH1r, _treshL);
    //TODO: remove erode dilate and related if really obsolete:
    if (settings.iClose1r > 0)
        dilate( _treshL, _treshL, getStructuringElement(MORPH_ELLIPSE, Size(settings.iClose1r, settings.iClose1r)));
    if (settings.iOpen1r > 0)
        erode(_treshL, _treshL, getStructuringElement(MORPH_ELLIPSE, Size(settings.iOpen1r, settings.iOpen1r)));

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
    gray = static_cast<double>(_visdat->uncertainty_background());
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
    kfL.transitionMatrix.at<float>(2) = dt;
    kfL.transitionMatrix.at<float>(9) = dt;
    stateL = kfL.predict();
    cv::Rect predRect;
    predRect.width = static_cast<int>(stateL.at<float>(4));
    predRect.height = static_cast<int>(stateL.at<float>(5));
    predRect.x = static_cast<int>(stateL.at<float>(0)) - predRect.width / 2;
    predRect.y = static_cast<int>(stateL.at<float>(1)) - predRect.height / 2;

    predicted_locationL.x = stateL.at<float>(0);
    predicted_locationL.y = stateL.at<float>(1);
    predicted_locationL.z = stateL.at<float>(2);
    cv::KeyPoint t(predicted_locationL.x,predicted_locationL.y,blob_size_last);

    float certainty;
    if (predicted_pathL.size() > 0.f) {
        if (roi_size_cnt == 1 )
            certainty   = predicted_pathL.back().tracking_certainty - certainty_init;
        else if (roi_size_cnt > 1 )
            certainty  = 1-((1 - predicted_pathL.back().tracking_certainty) * certainty_factor); // certainty_factor times less certain that the previous prediction. The previous prediction certainty is updated with an meas vs predict error score
        else {
            certainty = 1-((1 - predicted_pathL.back().tracking_certainty) / certainty_factor);
        }
    }   else {
        certainty = certainty_init;
    }

    if (certainty < 0)
        certainty = 0;
    if (certainty > 1)
        certainty = 1;

    predicted_pathL.push_back(track_item(t,frame_id,certainty) );

    return predicted_locationL;
}

uint ItemTracker::match_closest_to_prediciton(cv::Point3f predicted_locationL, std::vector<track_item> keypointsL) {
    uint closestL;
    if (keypointsL.size() == 1 && pathL.size() == 0) {
        closestL = 0;
    } else if (pathL.size() > 0) {
        //find closest keypoint to new predicted location
        float mind = 999999999;
        closestL=0;
        for (uint i = 0 ; i < keypointsL.size();i++) {
            track_item k =keypointsL.at(i);
            float d = (predicted_locationL.x-k.x()) * (predicted_locationL.x-k.x()) + (predicted_locationL.y-k.y())*(predicted_locationL.y-k.y());
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

float ItemTracker::stereo_match(cv::KeyPoint closestL, cv::Mat diffL,cv::Mat diffR, float prev_disparity){
    //get retangle around blob / changed pixels
    float rectsize = settings.max_disparity;


    float rectsizeX = ceil(rectsize*0.5f); // *4.0 results in drone-insect disparity interaction
    float rectsizeY = ceil(rectsize*0.5f);  // *3.0
    int x1,y1,x2,y2;
    x1 = static_cast<int>((closestL.pt.x-rectsizeX)*IMSCALEF);
    x2 = static_cast<int>(2*rectsizeX*IMSCALEF);
    y1 = static_cast<int>((closestL.pt.y-rectsizeY)*IMSCALEF);
    y2 = static_cast<int>(2*rectsizeY*IMSCALEF);
    if (x1 < 0)
        x1=0;
    else if (x1 >= diffL.cols)
        x1  = diffL.cols-1;
    if (y1 < 0)
        y1=0;
    else if (y1 >= diffL.rows)
        y1  = diffL.rows-1;
    if (x1+x2 >= diffL.cols)
        x2=diffL.cols-x1;
    if (y1+y2 >= diffL.rows)
        y2=diffL.rows-y1;
    cv::Rect roiL(x1,y1,x2,y2);

    //shift over the image to find the best match, shift = disparity
    int disparity_err = 0;
    __attribute__((unused)) int disparity_cor = 0;
    float maxcor = -std::numeric_limits<float>::max();
    float minerr = std::numeric_limits<float>::max();
    int tmp_max_disp = settings.max_disparity;
    if (x1 - tmp_max_disp < 0)
        tmp_max_disp = x1;

    int disp_start = settings.min_disparity;
    int disp_end = tmp_max_disp;
    if (n_frames_tracking>5) {
        disp_start = std::max(static_cast<int>(floor(prev_disparity))-2,disp_start);
        disp_end = std::min(static_cast<int>(ceil(prev_disparity))+2,disp_end);
    }

    cv::Mat diffLroi = diffL(roiL);
    for (int i=disp_start; i<disp_end;i++) {
        cv::Rect roiR(x1-i,y1,x2,y2);

        cv::Mat corV_16 = diffL(roiL).mul(diffR(roiR));
        cv::Mat errV = abs(diffL(roiL) - diffR(roiR));

        cor_16[i] = static_cast<int>(cv::sum(corV_16 )[0]);
        err[i] = static_cast<int>(cv::sum(errV)[0]);

        if (cor_16[i] > maxcor ) {
            maxcor = cor_16[i];
            disparity_cor = i;
        }
        if (err[i] < minerr ) { //update min MSE
            disparity_err  = i;
            minerr = err[i];
        } // for shift

    }

    // TODO: check if disparity_err OR disparity_cor is a better choice, or a mix
    int disparity = disparity_err;

    // measured disparity value

    if (disparity>0)
        sub_disparity = estimate_sub_disparity(disparity);
    else
        sub_disparity = 0;

    return sub_disparity;
}

float ItemTracker::estimate_sub_disparity(int disparity) {

    float sub_disp;

    // matching costs of neighbors
    /*
    float y1 = -cor_16[disparity-1];
    float y2 = -cor_16[disparity];
    float y3 = -cor_16[disparity+1];
    */

    float y1 = err[disparity-1];
    float y2 = err[disparity];
    float y3 = err[disparity+1];

    // by assuming a hyperbola shape, the x-location of the hyperbola minimum is determined and used as best guess
    float h31 = (y3 - y1);
    float h21 = (y2 - y1) * 4;
    sub_disp = ((h21 - h31)) / (h21 - h31 * 2);
    sub_disp += sinf(sub_disp*2.0f*static_cast<float>(M_PI))*0.13f;
    sub_disp += (disparity-1);

    if (sub_disp<disparity-1 || sub_disp>disparity+1 || sub_disp != sub_disp)
        return disparity;

    return sub_disp;
}

float ItemTracker::update_disparity(float disparity, float dt) {

    if (n_frames_lost>0 || isnanf(disparity_smoothed) || reset_disp)
    {
        disparity_smoothed = disparity;
        disp_rate_smoothed2.reset();
        reset_disp = false;

    } else {

        //        cout << "\t\t\t\t\t\t\t\t sub_disparity: " << disparity << endl;
        //        cout << "\t\t\t\t\t\t\t\t sub_disparity_smoothed: " << disparity_smoothed << endl;

        // predicted disparity value
        float disp_predict = disparity_smoothed + disp_rate*dt;

        float disp_filt_rate;
        float disp_filt_pred;
        if (abs(disparity-disp_predict)<0.5f) {
            disp_filt_rate = 0.4f;
            disparity_smoothed = disparity*disp_filt_rate + disparity_smoothed*(1.0f-disp_filt_rate);
        }
        else if (abs(disparity-disp_predict)<1.0f) {
            disp_filt_rate = 0.4f;
            disp_filt_pred = 0.1f;
            disparity_smoothed = disparity*disp_filt_pred + disp_predict*(disp_filt_rate-disp_filt_pred) + disparity_smoothed*(1.0f-disp_filt_rate);
        }
        else {
            disp_filt_rate = 0.4f;
            disp_filt_pred = 0.0f;
            disparity_smoothed = disparity*disp_filt_pred + disp_predict*(disp_filt_rate-disp_filt_pred) + disparity_smoothed*(1.0f-disp_filt_rate);

            if (abs(disparity-disp_prev)<1.0f)
                reset_disp = true;
        }

        disp_rate = disp_rate_smoothed2.addSample(disparity_smoothed,dt);
        disp_prev = disparity;
    }

    return disparity_smoothed;
}

void ItemTracker::check_consistency(cv::Point3f prevLoc,cv::Point3f measLoc) {

    float abs_dist = (prevLoc.x-measLoc.x)*(prevLoc.x-measLoc.x) + (prevLoc.y-measLoc.y)*(prevLoc.y-measLoc.y) + (prevLoc.z-measLoc.z)*(prevLoc.z-measLoc.z);

    if (abs_dist > 0.15f)
        reset_filters = true;
}


void ItemTracker::update_prediction_state(cv::Point3f p, float blob_size) {
    cv::Mat measL(measSize, 1, type);
    measL.at<float>(0) = p.x;
    measL.at<float>(1) = p.y;
    measL.at<float>(2) = p.z;
    measL.at<float>(3) = 0; //TODO; remove...?


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

    blob_size_last = blob_size;
}

void ItemTracker::update_tracker_ouput(Point3f measured_world_coordinates,float dt,  track_item * best_match, float disparity) {

    find_result.best_image_locationL = best_match->k;
    find_result.disparity = disparity;

    float new_tracking_certainty = calc_certainty(best_match->k);

    track_item t(*best_match);
    t.tracking_certainty = new_tracking_certainty;

    pathL.push_back(t);

    trackData data ={0};
    data.valid = true;
    data.posX = measured_world_coordinates.x;
    data.posY = measured_world_coordinates.y;
    data.posZ = measured_world_coordinates.z;

    if (n_frames_lost >= smooth_width_vel || reset_filters) { // tracking was regained, after n_frames_lost frames
        disp_rate_smoothed2.reset();
        disp_smoothed.reset();
        smoother_posX.reset();
        smoother_posY.reset();
        smoother_posZ.reset();
        smoother_velX2.reset();
        smoother_velY2.reset();
        smoother_velZ2.reset();
        smoother_velX.reset();
        smoother_velY.reset();
        smoother_velZ.reset();
        smoother_accX2.reset();
        smoother_accY2.reset();
        smoother_accZ2.reset();
        smoother_accX.reset();
        smoother_accY.reset();
        smoother_accZ.reset();

        detected_after_take_off = 0;
    }

    // Position estimation

    // smooth position data with simple filter
    if (reset_filters) {
        posX_smoothed = data.posX;
        posY_smoothed = data.posY;
        posZ_smoothed = data.posZ;

    } else {
        float pos_filt_rate = 0.3f;
        posX_smoothed = data.posX*pos_filt_rate + posX_smoothed*(1.0f-pos_filt_rate);
        posY_smoothed = data.posY*pos_filt_rate + posY_smoothed*(1.0f-pos_filt_rate);
        posZ_smoothed = data.posZ*pos_filt_rate + posZ_smoothed*(1.0f-pos_filt_rate);

    }

    data.sposX = posX_smoothed;
    data.sposY = posY_smoothed;
    data.sposZ = posZ_smoothed;

    data.svelX = smoother_velX2.addSample(posX_smoothed,dt);
    data.svelY = smoother_velY2.addSample(posY_smoothed,dt);
    data.svelZ = smoother_velZ2.addSample(posZ_smoothed,dt);

    data.saccX = smoother_accX2.addSample(data.svelX,dt);
    data.saccY = smoother_accY2.addSample(data.svelY,dt);
    data.saccZ = smoother_accZ2.addSample(data.svelZ,dt);


    prevX = data.sposX;
    prevY = data.sposY;
    prevZ = data.sposZ;

    data.valid = true;
    data.dt = dt;
    detected_after_take_off++;

    reset_filters = false;

    track_history.push_back(data);
}

float ItemTracker::calc_certainty(KeyPoint item) {
    float new_tracking_certainty;
    if (predicted_pathL.size() > 0) {
        new_tracking_certainty = 1.f / powf(powf(predicted_pathL.back().k.pt.x - item.pt.x,2) + powf(predicted_pathL.back().k.pt.y - item.pt.y,2),0.3f);
        new_tracking_certainty*= predicted_pathL.back().tracking_certainty;
        if (new_tracking_certainty>1 || new_tracking_certainty<0 || isnanf(new_tracking_certainty)) { // weird -nan sometimes???
            new_tracking_certainty = 1;
        }
    } else // if there was no prediciton, certainty prolly is quite low
        new_tracking_certainty = certainty_init;
    return new_tracking_certainty;
}

void ItemTracker::reset_tracker_ouput() {
    trackData data = {0};
    reset_filters = true;

    track_history.push_back(data);
    init_kalman();
}

void ItemTracker::close () {
    std::ofstream os(_settingsFile, std::ios::binary);
    cereal::BinaryOutputArchive archive( os );
    archive( settings );
}
