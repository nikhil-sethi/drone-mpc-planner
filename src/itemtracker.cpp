#include <iostream>
#include "itemtracker.h"
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "common.h"
#include "vector"
#include "algorithm"
using namespace cv;
using namespace std;

#ifdef HASSCREEN
//#define TUNING
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
            std::cout << "Itemtracker settings file error: " << e.what() << std::endl;
            std::cout << "Maybe delete the file: " << _settingsFile << std::endl;
            exit (1);
        }
        TrackerSettings tmp;
        if (settings.version < tmp.version){
            std::cout << "Itemtracker settings version too low!" << std::endl;
            std::cout << "Maybe delete the file: " << _settingsFile << std::endl;
            exit(1);
        }
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
    createTrackbar("appear_void_max_distance", window_name, &settings.appear_void_max_distance, 250);
    createTrackbar("void_void_max_distance", window_name, &settings.void_void_max_distance, 20);
    createTrackbar("exclude_min_distance", window_name, &settings.exclude_min_distance, 250);

#endif

    init_kalman();

    updateParams();

    smoother_posX.init(smooth_width_pos);
    smoother_posY.init(smooth_width_pos);
    smoother_posZ.init(smooth_width_pos);

    disp_rate_smoothed2.init(6,0.4);
    disp_smoothed.init(smooth_width_pos);

    smoother_velX2.init(6,0.4);
    smoother_velY2.init(6,0.4);
    smoother_velZ2.init(6,0.4);

    smoother_velX.init(smooth_width_vel);
    smoother_velY.init(smooth_width_vel);
    smoother_velZ.init(smooth_width_vel);

    smoother_accX2.init(6,0.4);
    smoother_accY2.init(6,0.4);
    smoother_accZ2.init(6,0.4);

    smoother_accX.init(smooth_width_acc);
    smoother_accY.init(smooth_width_acc);
    smoother_accZ.init(smooth_width_acc);

    find_result.best_image_locationL.pt.x = IMG_W/2/IMSCALEF;
    find_result.best_image_locationL.pt.y = IMG_H/2/IMSCALEF;
    find_result.smoothed_disparity = 0;
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

std::vector<ItemTracker::track_item> ItemTracker::remove_excludes(std::vector<track_item> keypoints, std::vector<track_item> exclude_path) {
    float dis1,dis2,dis = 0;

    //    if (_name.compare("insect")==0 && keypoints.size()>0){
    //        std::cout << "insect" << std::endl;
    //    }

    //    if (_name.compare("drone")==0 && keypoints.size()>0){
    //        std::cout << "drone" << std::endl;
    //    }

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

void ItemTracker::track(float time, std::vector<track_item> exclude, float drone_max_border_y, float drone_max_border_z) {
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

    find(exclude);

    std::vector<track_item> keypoint_candidates = find_result.keypointsL_wihout_voids;
    int nCandidates = keypoint_candidates.size();
    if (nCandidates) { //if nokeypointsLt lost

        cv::Point3f previous_location(find_result.best_image_locationL .pt.x,find_result.best_image_locationL .pt.y,0);

        Point3f output;
        float disparity;
        cv::KeyPoint match;

        while (keypoint_candidates.size() > 0) {
            int match_id = match_closest_to_prediciton(previous_location,find_result.keypointsL_wihout_voids);
            match = find_result.keypointsL_wihout_voids.at(match_id).k;

            disparity = stereo_match(match,_visdat->frameL_prev,_visdat->frameR_prev,_visdat->frameL,_visdat->frameR);
            disparity = update_disparity(disparity, dt_tracking);

            //calculate everything for the itemcontroller:
            std::vector<Point3f> camera_coordinates, world_coordinates;
            camera_coordinates.push_back(Point3f(match.pt.x*IMSCALEF,match.pt.y*IMSCALEF,-disparity));
            //camera_coordinates.push_back(Point3f(predicted_locationL.x*IMSCALEF,predicted_locationL.y*IMSCALEF,-predicted_locationL.z));
            cv::perspectiveTransform(camera_coordinates,world_coordinates,_visdat->Qf);
            output = world_coordinates[0];
            float theta = CAMERA_ANGLE / (180.f/(float)M_PI);
            float temp_y = output.y * cosf(theta) + output.z * sinf(theta);
            output.z = -output.y * sinf(theta) + output.z * cosf(theta);
            output.y = temp_y;

            if ((output.x < -1.0f) || (output.x > 0.75f) ||(output.z < -drone_max_border_z) || (output.y < -drone_max_border_y) || disparity < settings.min_disparity || disparity > settings.max_disparity) { //TODO check min/max disparity > or =>!!!
                keypoint_candidates.erase(keypoint_candidates.begin() + match_id);
            } else {
                break;
            }
        }
        nCandidates = keypoint_candidates.size();
        if (keypoint_candidates.size() > 0) { // if !lost
            //Point3f predicted_output = world_coordinates[1];
            check_consistency(cv::Point3f(this->prevX,this->prevY,this->prevZ),output);
            update_tracker_ouput(output,dt_tracking,match,disparity,_visdat->frame_id);
            update_prediction_state(cv::Point3f(match.pt.x,match.pt.y,disparity),match.size);
            n_frames_lost = 0; // update this after calling update_tracker_ouput, so that it can determine how long tracking was lost
            t_prev_tracking = time; // update dt only if item was detected
            n_frames_tracking++;
            roi_size_cnt = 0;

        }
    }
    if (nCandidates == 0) {
        roi_size_cnt ++; //TODO: samen as n_frames_lost?
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
    trackData last = get_last_track_data();
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


}


void ItemTracker::find(std::vector<track_item> exclude) {
    cv::Point previous_location;

    previous_location = find_result.best_image_locationL.pt;

    cv::Point roi_size;
    if (settings.roi_min_size < 1)
        settings.roi_min_size = 1;
    roi_size.x=settings.roi_min_size/IMSCALEF+roi_size_cnt*(settings.roi_grow_speed / 16 / IMSCALEF);
    roi_size.y=settings.roi_min_size/IMSCALEF+roi_size_cnt*(settings.roi_grow_speed / 16 / IMSCALEF);

    if (roi_size.x  >= _visdat->smallsize.width) {
        roi_size.x = _visdat->smallsize.width;
    }

    if (roi_size.y >= _visdat->smallsize.height)
        roi_size.y = _visdat->smallsize.height;

    //attempt to detect changed blobs
    _treshL = segment(_visdat->diffL_small,previous_location,roi_size);

    //    cv::Mat tmp = createColumnImage({_treshL,_visdat->diffL*100,_visdat->frameL},CV_8UC1,0.25f);
    //    cv::imshow("trek",tmp);

#if CV_MAJOR_VERSION==3
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
#else./logging/test
    SimpleBlobDetector * detector;
    detector = *SimpleBlobDetector(params);
#endif

    std::vector<KeyPoint> keypointsL;
    detector->detect( _treshL, keypointsL);

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
    find_result.keypointsL_wihout_voids = remove_excludes(kps,exclude);

    if (find_result.keypointsL_wihout_voids.size() ==0) {
        roi_size_cnt +=1;
        if (roi_size_cnt > settings.roi_max_grow)
            roi_size_cnt = settings.roi_max_grow;

    }
    find_result.excludes = exclude;
    find_result.keypointsL = kps;
    find_result.treshL = _treshL;
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
                float d1_2 = sqrtf(pow(k1.pt.x - k2.pt.x,2) + pow(k1.pt.y - k2.pt.y,2));
                if (d1_2 < settings.appear_void_max_distance) { // found a keypoint in the prev list that is very close to one in the current list
                    KeyPoint closest_k1;
                    int closest_k1_id = -1;
                    float closest_k1_d = 99999999;
                    for (uint k = 0 ; k < keyps.size();k++) {
                        //find the closest item to k1 in the current list and *assume* together it forms a void-appearance pair
                        if (k!=i) {
                            KeyPoint  k1_mate = keyps.at(k).k;
                            float k1_mate_d = sqrtf(pow(k1.pt.x - k1_mate.pt.x,2) + pow(k1.pt.y - k1_mate.pt.y,2))  ;
                            if (k1_mate_d < closest_k1_d) {
                                closest_k1 = k1_mate;
                                closest_k1_d = k1_mate_d;
                                closest_k1_id = k;
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
                            keyps.at(closest_k1_id).k_void = k1;
                            keyps.at(i).k_void = closest_k1;
                            n_erased_items++;
                            j=keyps_prev.size()+1; //break from for j
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
    gray = _visdat->uncertainty_background();
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
    predRect.width = stateL.at<float>(4);
    predRect.height = stateL.at<float>(5);
    predRect.x = stateL.at<float>(0) - predRect.width / 2;
    predRect.y = stateL.at<float>(1) - predRect.height / 2;

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

int ItemTracker::match_closest_to_prediciton(cv::Point3f predicted_locationL, std::vector<track_item> keypointsL) {
    int closestL;
    if (keypointsL.size() == 1 && pathL.size() == 0) {
        closestL = 0;
    } else if (pathL.size() > 0) {
        //find closest keypoint to new predicted location
        int mind = 999999999;
        for (uint i = 0 ; i < keypointsL.size();i++) {
            track_item k =keypointsL.at(i);
            int d = (predicted_locationL.x-k.x()) * (predicted_locationL.x-k.x()) + (predicted_locationL.y-k.y())*(predicted_locationL.y-k.y());
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

float ItemTracker::stereo_match(cv::KeyPoint closestL,cv::Mat prevFrameL_big,cv::Mat prevFrameR_big, cv::Mat frameL,cv::Mat frameR){

    //get retangle around blob / changed pixels
    float rectsize = closestL.size; // keypoint.size is the diameter of the blob
    if (rectsize < 3)
        rectsize = 3;
    float rectsizeX = ceil(rectsize*0.5f); // *4.0 results in drone-insect disparity interaction
    float rectsizeY = ceil(rectsize*0.5f);  // *3.0

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
    int disparity_err = 0;
    int disparity_cor = 0;
    float maxcor = -std::numeric_limits<float>::max();
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

        cor_16[i] = cv::sum(corV_16 )[0];
        err[i] = cv::sum(errV)[0];

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
    sub_disp += sinf(sub_disp*2.0f*(float)M_PI)*0.13f;
    sub_disp += (disparity-1);

    if (sub_disp<disparity-1 || sub_disp>disparity+1)
        return disparity;

    return sub_disp;
}

float ItemTracker::update_disparity(float disparity, float dt) {

    if (n_frames_lost>0 || isnan(disparity_smoothed) || reset_disp)
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
            disp_filt_rate = 0.4;
            disparity_smoothed = disparity*disp_filt_rate + disparity_smoothed*(1.0f-disp_filt_rate);
        }
        else if (abs(disparity-disp_predict)<1.0f) {
            disp_filt_rate = 0.4;
            disp_filt_pred = 0.1;
            disparity_smoothed = disparity*disp_filt_pred + disp_predict*(disp_filt_rate-disp_filt_pred) + disparity_smoothed*(1.0f-disp_filt_rate);
        }
        else {
            disp_filt_rate = 0.4;
            disp_filt_pred = 0.0;
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

void ItemTracker::update_tracker_ouput(Point3f measured_world_coordinates,float dt,  cv::KeyPoint match, float disparity, int frame_id) {

    find_result.best_image_locationL = match;
    find_result.disparity = disparity;

    float new_tracking_certainty = calc_certainty(match);

    pathL.push_back(track_item(find_result.best_image_locationL,frame_id,new_tracking_certainty));

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
        float pos_filt_rate = 0.3;
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
        if (new_tracking_certainty>1 || new_tracking_certainty<0 || isnan(new_tracking_certainty)) { // weird -nan sometimes???
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
