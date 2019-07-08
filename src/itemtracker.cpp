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

void ItemTracker::update_world_candidate(){

//    if (_name.compare("insect")==0)
//        cout << std::endl;

    WorldTrackItem w;
    w.iti = _image_track_item;
    if (_image_track_item.valid && (
            (_image_track_item.score >= settings.score_threshold/1e6f) ||
            (n_frames_tracking == 0))) {
        _image_track_item.disparity = stereo_match(_image_track_item.pt(),_visdat->diffL,_visdat->diffR,find_result.disparity);
        w.iti.disparity = _image_track_item.disparity; // hmm

        if (_image_track_item.disparity < settings.min_disparity || _image_track_item.disparity > settings.max_disparity){
            w.disparity_in_range = false;
        } else {
            w.disparity_in_range = true;
            //calculate everything for the itemcontroller:
            std::vector<Point3d> camera_coordinates, world_coordinates;
            camera_coordinates.push_back(Point3d(w.image_coordinates().x*IMSCALEF,w.image_coordinates().y*IMSCALEF,-_image_track_item.disparity));
            cv::perspectiveTransform(camera_coordinates,world_coordinates,_visdat->Qf);
            w.pt = world_coordinates[0];

            float theta = _visdat->camera_angle * deg2rad;
            float temp_y = w.pt.y * cosf(theta) + w.pt.z * sinf(theta);
            w.pt.z = -w.pt.y * sinf(theta) + w.pt.z * cosf(theta);
            w.pt.y = temp_y;

            float dist_back = _visdat->depth_background_mm.at<float>(w.image_coordinates().y*IMSCALEF,w.image_coordinates().x*IMSCALEF);
            float dist_meas = sqrtf(powf(w.pt.x,2) + powf(w.pt.y,2) +powf(w.pt.z,2));
            if ((dist_meas > dist_back*(static_cast<float>(settings.background_subtract_zone_factor)/100.f)) && _enable_depth_background_check)
                w.background_check_ok = false;
            else
                w.background_check_ok = true;

            w.distance = dist_meas;
            w.distance_background = dist_back;


        }
        if (w.background_check_ok && w.disparity_in_range)
            w.valid = true;
        else
            w.valid = false;
        _world_track_item = w;
    } else {
        _world_track_item.valid = false;
    }
}

void ItemTracker::track(double time) {

    double dt_tracking= (time-t_prev_tracking);
    double dt_predict= (time-t_prev_predict);
    t_prev_predict = time;


    update_world_candidate();

    if ( _world_track_item.valid) {

        update_disparity(_world_track_item.iti.disparity, dt_tracking);

        //Point3f predicted_output = world_coordinates[1];
        check_consistency(cv::Point3f(prevX,prevY,prevZ),_world_track_item.pt);
        update_tracker_ouput(_world_track_item.pt,dt_tracking,time,&_world_track_item.iti,_world_track_item.iti.disparity);
        update_prediction_state(_image_track_item.pt(), _world_track_item.iti.disparity,_image_track_item.size);
        n_frames_lost = 0; // update this after calling update_tracker_ouput, so that it can determine how long tracking was lost
        t_prev_tracking = time; // update dt only if item was detected
        n_frames_tracking++;
        roi_size_cnt = 0;
    } else {

        roi_size_cnt ++; //TODO: same as n_frames_lost?
        n_frames_lost++;
        n_frames_tracking = 0;

        if (roi_size_cnt > settings.roi_max_grow)
            roi_size_cnt = settings.roi_max_grow;

        if( n_frames_lost >= n_frames_lost_threshold || !_tracking ) {
            _tracking = false;
            reset_tracker_ouput(time);
        }
    }

    if (path.size() > 0) {
        if (path.begin()->frame_id() < _visdat->frame_id - path_buf_size)
            path.erase(path.begin());
        if (path.begin()->frame_id() > _visdat->frame_id ) //at the end of a realsense video loop, frame_id resets
            path.clear();
    }

    if (_tracking) {
        predict(dt_predict,_visdat->frame_id);
    }

    if (predicted_image_path.size() > 0) {
        if (predicted_image_path.begin()->frame_id < _visdat->frame_id - path_buf_size)
            predicted_image_path.erase(predicted_image_path.begin());
        if (predicted_image_path.begin()->frame_id > _visdat->frame_id ) //at the end of a realsense video loop, frame_id resets
            predicted_image_path.clear();
    }

    append_log();

    while (track_history.size() > track_history_max_size)
        track_history.erase(track_history.begin());

}

void ItemTracker::append_log() {
    //log all image stuff
    if (path.size()>0)
        (*_logger) << path.back().image_coordinates().x * IMSCALEF << "; " << path.back().image_coordinates().y * IMSCALEF << "; " << find_result.disparity << "; ";
    else
        (*_logger) << -1 << "; " << -1 << "; " << -1 << "; ";
    if (predicted_image_path.size()>0)
        (*_logger) << predicted_image_path.back().x * IMSCALEF << "; " << predicted_image_path.back().y * IMSCALEF << "; ";
    else
        (*_logger) << -1 << "; " << -1   << "; ";

    (*_logger) << n_frames_lost << "; " << n_frames_tracking << "; " << _tracking << "; ";
    //log all world stuff
    track_data last = Last_track_data();
    (*_logger) << last.posX << "; " << last.posY << "; " << last.posZ << ";" ;
    (*_logger) << last.sposX << "; " << last.sposY << "; " << last.sposZ << ";";
    (*_logger) << last.svelX << "; " << last.svelY << "; " << last.svelZ << ";";
    (*_logger) << last.saccX << "; " << last.saccY << "; " << last.saccZ << ";";
}

void ItemTracker::predict(float dt, int frame_id) {
    cv::Point2f predicted_image_locationL;
    kfL.transitionMatrix.at<float>(2) = dt;
    kfL.transitionMatrix.at<float>(9) = dt;
    stateL = kfL.predict();
    cv::Rect predRect;
    predRect.width = static_cast<int>(stateL.at<float>(4));
    predRect.height = static_cast<int>(stateL.at<float>(5));
    predRect.x = static_cast<int>(stateL.at<float>(0)) - predRect.width / 2;
    predRect.y = static_cast<int>(stateL.at<float>(1)) - predRect.height / 2;

    predicted_image_locationL.x = stateL.at<float>(0);
    predicted_image_locationL.y = stateL.at<float>(1);
    float size = stateL.at<float>(2);

    float certainty;
    if (predicted_image_path.size() > 0.f) {
        if (roi_size_cnt == 1 )
            certainty   = predicted_image_path.back().certainty - certainty_init;
        else if (roi_size_cnt > 1 )
            certainty  = 1-((1 - predicted_image_path.back().certainty) * certainty_factor); // certainty_factor times less certain that the previous prediction. The previous prediction certainty is updated with an meas vs predict error score
        else {
            certainty = 1-((1 - predicted_image_path.back().certainty) / certainty_factor);
        }
    }   else {
        certainty = certainty_init;
    }

    if (certainty < 0)
        certainty = 0;
    if (certainty > 1)
        certainty = 1;

    //keep track of the error:
    if (predicted_image_path.size()>0 && _world_track_item.valid){
        predicted_image_path.back().x_measured = _image_track_item.x;
        predicted_image_path.back().y_measured = _image_track_item.y;
        predicted_image_path.back().prediction_error  = sqrtf( powf(predicted_image_path.back().x_measured - predicted_image_path.back().x,2) + powf(predicted_image_path.back().y_measured - predicted_image_path.back().y,2));

    }
    predicted_image_path.push_back(ImagePredictItem(predicted_image_locationL,certainty,size,frame_id) );
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

void ItemTracker::update_prediction_state(cv::Point2f image_location, float disparity, float size) {
    cv::Mat measL(measSize, 1, type);
    measL.at<float>(0) = image_location.x;
    measL.at<float>(1) = image_location.y;
    measL.at<float>(2) = disparity;
    measL.at<float>(3) = size;


    if (!_tracking) { // First detection!
        kfL.errorCovPre.at<float>(0) = 1; // px
        kfL.errorCovPre.at<float>(7) = 1; // px
        kfL.errorCovPre.at<float>(14) = 1;
        kfL.errorCovPre.at<float>(21) = 1;
        kfL.errorCovPre.at<float>(28) = 1; // px
        kfL.errorCovPre.at<float>(35) = 1; // px

        stateL.at<float>(0) = measL.at<float>(0);
        stateL.at<float>(1) = measL.at<float>(1);
        stateL.at<float>(3) = measL.at<float>(2);
        stateL.at<float>(4) = measL.at<float>(3);
        stateL.at<float>(2) = 0;
        //            stateL.at<float>(3) = 0;
        //            stateL.at<float>(4) = measL.at<float>(2);
        //            stateL.at<float>(5) = measL.at<float>(3);

        kfL.statePost = stateL;
        _tracking = true;
    }
    else
        kfL.correct(measL);
}

void ItemTracker::update_tracker_ouput(Point3f measured_world_coordinates,float dt,  double time, ImageTrackItem * best_match, float disparity) {

    find_result.best_image_locationL = best_match->k();
    find_result.disparity = disparity;

    _world_track_item.iti.certainty = calc_certainty(_world_track_item.iti.k());;

    path.push_back(_world_track_item);

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
    if (predicted_image_path.size() > 0) {
        new_tracking_certainty = 1.f / powf(powf(predicted_image_path.back().x - item.pt.x,2) + powf(predicted_image_path.back().y - item.pt.y,2),0.3f);
        new_tracking_certainty*= predicted_image_path.back().certainty;
        if (new_tracking_certainty>1 || new_tracking_certainty<0 || isnanf(new_tracking_certainty)) { // weird -nan sometimes???
            new_tracking_certainty = 1;
        }
    } else // if there was no prediciton, certainty prolly is quite low
        new_tracking_certainty = certainty_init;
    return new_tracking_certainty;
}

void ItemTracker::reset_tracker_ouput(double time) {
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
