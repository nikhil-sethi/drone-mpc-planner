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
//#define VIZ
#endif

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
            std::stringstream serr;
            serr << "cannot read itemtracker settings file: " << e.what() << ". Maybe delete the file: " << _settingsFile;
            throw my_exit(serr.str());
        }
        TrackerSettings tmp;
        if (tmp.version-settings.version > 0.001f){
            std::stringstream serr;
            serr << "itemtracker settings version too low! Maybe delete the file: " << _settingsFile;
            throw my_exit(serr.str());
        }
    } else {
        init_settings();
    }

#ifdef TUNING
    namedWindow(window_name, WINDOW_NORMAL);

    createTrackbar("#points / frame", window_name, &settings.max_points_per_frame, 255);
    createTrackbar("Radius", window_name, &settings.radius, 255);
    createTrackbar("Motion threshold", window_name, &settings.motion_thresh, 255);
    createTrackbar("Min disparity", window_name, &settings.min_disparity, 255);
    createTrackbar("Max disparity", window_name, &settings.max_disparity, 255);
    createTrackbar("roi_min_size", window_name, &settings.roi_min_size, 2000);
    createTrackbar("roi_max_grow", window_name, &settings.roi_max_grow, 500);
    createTrackbar("roi_grow_speed", window_name, &settings.roi_grow_speed, 256);
    createTrackbar("exclude_min_distance", window_name, &settings.exclude_min_distance, 250);
    createTrackbar("background_subtract_zone_factor", window_name, &settings.background_subtract_zone_factor, 100);

    createTrackbar("pixel_dist_landing_spot", window_name, &settings.pixel_dist_landing_spot, 100);
    createTrackbar("pixel_dist_seperation_min", window_name, &settings.pixel_dist_seperation_min, 100);
    createTrackbar("pixel_dist_seperation_max", window_name, &settings.pixel_dist_seperation_max, 100);

#endif

    init_kalman();

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
    (*_logger) << "svelX_" << _name << "; ";
    (*_logger) << "svelY_" << _name << "; ";
    (*_logger) << "svelZ_" << _name << "; ";
    (*_logger) << "saccX_" << _name << "; ";
    (*_logger) << "saccY_" << _name << "; ";
    (*_logger) << "saccZ_" << _name << "; ";

    initialized = true;
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

void ItemTracker::select_best_candidate(){



    std::vector<track_item> keypoint_candidates = find_result.keypointsL_wihout_excludes;
    uint nCandidates = static_cast<uint>(keypoint_candidates.size());
    wti.clear();
    if (nCandidates) {

        //first sort candidates based on image location odds, and check disparity range and background checks
        int n = keypoint_candidates.size();

//        if (_name.compare("drone")==0 && n > 0 )
//            cout << std::endl;
//        if (_name.compare("drone")==0 && n > 1 )
//            cout << std::endl;
        for (int i=0; i<n;i++)   {
            world_track_item w;
            uint match_id = match_closest_to_prediciton(predicted_locationL_last,keypoint_candidates);
            w.ti = keypoint_candidates.at(match_id);

            w.disparity = stereo_match(w.image_coordinates(),_visdat->diffL,_visdat->diffR,find_result.disparity);

            if (w.disparity < settings.min_disparity || w.disparity > settings.max_disparity){
                w.disparity_in_range = false;
            } else {
                w.disparity_in_range = true;
                //calculate everything for the itemcontroller:
                std::vector<Point3d> camera_coordinates, world_coordinates;
                camera_coordinates.push_back(Point3d(w.image_coordinates().x*IMSCALEF,w.image_coordinates().y*IMSCALEF,-w.disparity));
                cv::perspectiveTransform(camera_coordinates,world_coordinates,_visdat->Qf);
                w.world_coordinates = world_coordinates[0];

                float dist_back = _visdat->depth_background_mm.at<float>(w.image_coordinates().y*IMSCALEF,w.image_coordinates().x*IMSCALEF);
                float dist_meas = sqrtf(powf(w.world_coordinates.x,2) + powf(w.world_coordinates.y,2) +powf(w.world_coordinates.z,2));
                if ((dist_meas > dist_back*(static_cast<float>(settings.background_subtract_zone_factor)/100.f)) && _enable_depth_background_check)
                    w.background_check_ok = false;
                else
                    w.background_check_ok = true;

                w.ti.distance = dist_meas;
                w.ti.distance_background = dist_back;

                float theta = _visdat->camera_angle * deg2rad;
                float temp_y = w.world_coordinates.y * cosf(theta) + w.world_coordinates.z * sinf(theta);
                w.world_coordinates.z = -w.world_coordinates.y * sinf(theta) + w.world_coordinates.z * cosf(theta);
                w.world_coordinates.y = temp_y;

//                if ( ((w.world_coordinates.y < -1.2f || w.world_coordinates.z < -3.5f  ||  w.world_coordinates.x < -0.5f ||  w.world_coordinates.x > 2.3f) && _name == "insect") ||
//                     ((w.world_coordinates.y < -1.35f || w.world_coordinates.z < -3.5f  ||  w.world_coordinates.x < -0.5f ||  w.world_coordinates.x > 2.3f) && _name == "drone"))

                //w.background_check_ok = true; // manual override (optional)
            }
            if (w.background_check_ok && w.disparity_in_range)
                wti.push_back(w);
            keypoint_candidates.erase(keypoint_candidates.begin() + match_id);
        }
    }
}

std::vector<ItemTracker::track_item> ItemTracker::remove_excludes(std::vector<track_item> keypoints, std::vector<track_item> exclude_path,std::vector<cv::Point2f> additional_ignores) {
    float dis1 = 0;

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
        if (min_dis < settings.exclude_additional_max_distance)
            keypoints.erase(keypoints.begin() + min_dis_i);
    }

    if (exclude_path.size() > 0 && keypoints.size ()>0) {
        track_item exclude = exclude_path.at(exclude_path.size()-1);

//        if (exclude_path.size() > 1) {
//            exclude_prev = exclude_path.at(exclude_path.size()-2);
//        }
        std::vector<track_item> tmp = keypoints;
        int erase_cnt =0;
        for (uint i = 0 ; i< tmp.size();i++){
            dis1 = sqrtf(powf(tmp.at(i).k.pt.x - exclude.x(),2) +powf(tmp.at(i).k.pt.y - exclude.y(),2));
//            dis2 = sqrtf(powf(tmp.at(i).k_void.pt.x - exclude.x(),2) +powf(tmp.at(i).k_void.pt.y - exclude.y(),2));
//            float certainty_this_kp = calc_certainty(tmp.at(i).k);

            float threshold_dis = settings.exclude_min_distance / sqrtf(exclude.tracking_certainty);
            if (threshold_dis > settings.exclude_max_distance)
                threshold_dis = settings.exclude_max_distance;

            if ((dis1 < threshold_dis)) {// TODO: && certainty_this_kp <= exclude.tracking_certainty) {
                keypoints.erase(keypoints.begin() + i - erase_cnt);
                erase_cnt++;
            }
        }
    }
    return keypoints;
}

void ItemTracker::track(float time, std::vector<track_item> exclude,std::vector<cv::Point2f> additional_ignores) {

    float dt_tracking= (time-t_prev_tracking);
    float dt_predict= (time-t_prev_predict);
    t_prev_predict = time;

    cv::Point3f predicted_locationL;
    if (foundL) {
        predicted_locationL = predict(dt_predict,_visdat->frame_id);
        predicted_pathL.back().k.pt.x = predicted_locationL_last.x;
        predicted_pathL.back().k.pt.y = predicted_locationL_last.y;
    } else {
        predicted_locationL = predicted_locationL_last;
    }

    find(exclude,additional_ignores);
    select_best_candidate();

    if ( wti.size()>0) {
        world_track_item w = wti.at(0);
        update_disparity(wti.at(0).disparity, dt_tracking);

        //Point3f predicted_output = world_coordinates[1];
        check_consistency(cv::Point3f(this->prevX,this->prevY,this->prevZ),w.world_coordinates);
        update_tracker_ouput(w.world_coordinates,dt_tracking,time,&w.ti,w.disparity);
        update_prediction_state(cv::Point3f(w.image_coordinates().x,w.image_coordinates().y,w.disparity),w.ti.k.size);
        n_frames_lost = 0; // update this after calling update_tracker_ouput, so that it can determine how long tracking was lost
        t_prev_tracking = time; // update dt only if item was detected
        n_frames_tracking++;
        roi_size_cnt = 0;
    } else {

        roi_size_cnt ++; //TODO: same as n_frames_lost?
        n_frames_lost++;
        if (roi_size_cnt > settings.roi_max_grow)
            roi_size_cnt = settings.roi_max_grow;

        if( n_frames_lost >= n_frames_lost_threshold || !foundL ) {
            foundL = false;
            reset_tracker_ouput(time);
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

    while (track_history.size() > track_history_max_size)
        track_history.erase(track_history.begin());

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
    track_data last = Last_track_data();
    (*_logger) << last.posX << "; " << last.posY << "; " << last.posZ << ";" ;
    (*_logger) << last.sposX << "; " << last.sposY << "; " << last.sposZ << ";";
    (*_logger) << last.svelX << "; " << last.svelY << "; " << last.svelZ << ";";
    (*_logger) << last.saccX << "; " << last.saccY << "; " << last.saccZ << ";";
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
    find_max_change(previous_location,roi_size,_visdat->diffL_small,&keypointsL);

    vector<track_item> kps;
    for (uint i = 0 ; i < keypointsL.size();i++) {
        keypointsL.at(i).pt.x += find_result.roi_offset.x;
        keypointsL.at(i).pt.y += find_result.roi_offset.y;
        track_item t( keypointsL.at(i),_visdat->frame_id,0);
        kps.push_back(t);
    }

    find_result.keypointsL_wihout_excludes = remove_excludes(kps,exclude,additional_ignores);

    if (find_result.keypointsL_wihout_excludes.size() ==0) {
        roi_size_cnt +=1;
        if (roi_size_cnt > settings.roi_max_grow)
            roi_size_cnt = settings.roi_max_grow;

    }
    find_result.excludes = exclude;
    find_result.keypointsL = kps;
}

//Blob finder in the tracking ROI of the motion image. Looks for a limited number of
//maxima that are higher than a threshold, the area around the maximum
//is segmented from the background noise, and seen as a blob. It then
//tries if this blob can be further splitted if necessary.
void ItemTracker::find_max_change(cv::Point prev,cv::Point roi_size,cv::Mat diff,std::vector<KeyPoint> * scored_points) {

    _approx = get_approx_cutout_filtered(prev,diff,roi_size);
    cv::Mat frame = _approx;

    int radius = settings.radius;

#ifdef VIZ
    std::vector<cv::Mat> vizs;
#endif

    for (int i = 0; i < settings.max_points_per_frame; i++) {
        cv::Point mint;
        cv::Point maxt;
        double min, max;
        cv::minMaxLoc(frame, &min, &max, &mint, &maxt);

        cv::Mat bkg_frame_cutout = _visdat->max_uncertainty_map(find_result.roi_offset);
        uint8_t bkg = bkg_frame_cutout.at<uint8_t>(maxt.y,maxt.x);
        if (!_enable_motion_background_check)
            bkg = 0;

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
            cv::Scalar avg = cv::mean(cropped);
            cv::GaussianBlur(cropped,cropped,cv::Size(5,5),0);
            mask = cropped > (max-avg(0)) * 0.3; // TODO: factor 0,3 seems to work better than 0,5, but what makes sense here?
            cropped = mask;

            cv::Moments mo = cv::moments(cropped,true);
            cv::Point2f COG = cv::Point2f(static_cast<float>(mo.m10) / static_cast<float>(mo.m00), static_cast<float>(mo.m01) / static_cast<float>(mo.m00));


#ifdef VIZ
            cv::Mat viz = createRowImage({roi,mask},CV_8UC1,4);
            cv::cvtColor(viz,viz,CV_GRAY2BGR);
            cv::putText(viz,std::to_string(i),cv::Point(0, 13),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,255,0));
            vizs.push_back(viz);
#endif

            // relative it back to the _approx frame
            COG.x += r2.x;
            COG.y += r2.y;

            //check if the blob may be multiple blobs, first check distance between COG and max:
            float dist = pow(COG.x-maxt.x,2) + pow(COG.y-maxt.y,2);
            bool single_blob = true;
            bool COG_is_nan = false;
            if (dist > 4) {//todo: instead of just check the distance, we could also check the pixel value of the COG, if it is black it is probably multiple blobs
                //the distance between the COG and the max is quite large, now check if there are multiple contours:
                vector<vector<Point>> contours;
                cv::findContours(cropped,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
                if (contours.size()>1) {
                    //ok, definetely multiple blobs. Split them, and find the COG for each.
                    single_blob = false;
                    for (uint j = 0; j< contours.size();j++) {
                        cv::Moments mo2 = cv::moments(contours.at(j),true);
                        cv::Point2f COG2 = cv::Point2f(static_cast<float>(mo2.m10) / static_cast<float>(mo2.m00), static_cast<float>(mo2.m01) / static_cast<float>(mo2.m00));

                        if (COG2.x == COG2.x) {// if not nan
#ifdef VIZ
                            cv::circle(viz,COG2*4,1,cv::Scalar(0,0,255),1);
                            cv::putText(viz,to_string_with_precision(COG2.y*4+find_result.roi_offset.y,0),COG2*4,cv::FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(100,0,255));
#endif
                            // relative COG back to the _approx frame, and save it:
                            COG2.x += r2.x;
                            COG2.y += r2.y;
                            scored_points->push_back(cv::KeyPoint(COG2, mo2.m00));

                            //remove this COG from the ROI:
                            cv::circle(frame, COG2, 1, cv::Scalar(0), radius);

                        } else {
                            COG_is_nan = true;
                        }
                    }
                }
            }
            if (single_blob) { // we could not split this blob, so we can use the original COG
                if (COG.x == COG.x) { // if not nan
                    scored_points->push_back(cv::KeyPoint(COG, mo.m00));
#ifdef VIZ
                    cv::Point2f tmpCOG;
                    tmpCOG.x = COG.x - r2.x;
                    tmpCOG.y = COG.y - r2.y;
                    cv::circle(viz,tmpCOG*4,1,cv::Scalar(0,0,255),1);
#endif
                    //remove this COG from the ROI:
                    cv::circle(frame, COG, 1, cv::Scalar(0), radius);
                } else {
                    COG_is_nan = true;
                }
            }
            if (COG_is_nan)  //remove the actual maximum from the ROI if the COG algorithm failed:
                cv::circle(frame, maxt, 1, cv::Scalar(0), radius);

        } else {
            break; // done searching for maxima, they are too small now
        }
    }

#ifdef VIZ
    if (vizs.size()>0)
        viz_max_points = createColumnImage(vizs,CV_8UC3,1);
#endif

}

cv::Mat ItemTracker::get_probability_cloud(cv::Point size) {
    return cv::Mat::ones(size.y,size.x,CV_32F); // dummy implementation. TODO: create proper probablity estimate
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

float ItemTracker::stereo_match(cv::Point closestL, cv::Mat diffL,cv::Mat diffR, float prev_disparity){
    //get retangle around blob / changed pixels
    float rectsize = settings.max_disparity;


    float rectsizeX = ceil(rectsize*0.5f); // *4.0 results in drone-insect disparity interaction
    float rectsizeY = ceil(rectsize*0.5f);  // *3.0
    int x1,y1,x2,y2;
    x1 = static_cast<int>((closestL.x-rectsizeX)*IMSCALEF);
    x2 = static_cast<int>(2*rectsizeX*IMSCALEF);
    y1 = static_cast<int>((closestL.y-rectsizeY)*IMSCALEF);
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
        sub_disp = disparity;



    return sub_disp;

}

void ItemTracker::update_disparity(float disparity, float dt) {


    float disparity_prev_smoothed = disparity_smoothed;

    // predicted disparity value
    float disp_predict = disparity + disp_rate*dt;

    if (abs(disparity-disp_predict)<0.5f) {
        float disp_filt_rate = 0.4f;
        disparity_smoothed = disparity*disp_filt_rate + disparity_prev_smoothed*(1.0f-disp_filt_rate);
    } else if (abs(disparity-disp_predict)<1.0f) {
        float disp_filt_rate = 0.4f;
        float disp_filt_pred = 0.1f;
        disparity_smoothed = disparity*disp_filt_pred + disp_predict*(disp_filt_rate-disp_filt_pred) + disparity_prev_smoothed*(1.0f-disp_filt_rate);
    } else {
        float disp_filt_rate = 0.4f;
        float disp_filt_pred = 0.0f;
        disparity_smoothed = disparity*disp_filt_pred + disp_predict*(disp_filt_rate-disp_filt_pred) + disparity_prev_smoothed*(1.0f-disp_filt_rate);

        if (abs(disparity-disp_prev)<1.0f)
            reset_disp = true;
    }

    if (n_frames_lost>0 || isnanf(disparity_smoothed) || reset_disp)
    {
        disp_rate_smoothed2.reset();
        reset_disp = false;

    } else {
        disp_rate = disp_rate_smoothed2.addSample(disparity_smoothed,dt);
        disp_prev = disparity;
    }
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

void ItemTracker::update_tracker_ouput(Point3f measured_world_coordinates,float dt,  float time, track_item * best_match, float disparity) {

    find_result.best_image_locationL = best_match->k;
    find_result.disparity = disparity;

    float new_tracking_certainty = calc_certainty(best_match->k);

    track_item t(*best_match);
    t.tracking_certainty = new_tracking_certainty;

    pathL.push_back(t);

    track_data data ={0};
    data.pos_valid = true;
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

    data.vel_valid = smoother_velX2.ready();
    data.acc_valid = smoother_accX2.ready();

    prevX = data.sposX;
    prevY = data.sposY;
    prevZ = data.sposZ;

    data.time = time;
    last_sighting_time = time;
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

void ItemTracker::reset_tracker_ouput(float time) {
    track_data data = {0};
    reset_filters = true;
    data.time = time;
    track_history.push_back(data);
    init_kalman();
}

void ItemTracker::close () {
    if (initialized){
        std::cout << "Closing tracker: " << _name << std::endl;
        std::ofstream os(_settingsFile, std::ios::binary);
        cereal::BinaryOutputArchive archive( os );
        archive( settings );
        initialized = false;
    }
}
