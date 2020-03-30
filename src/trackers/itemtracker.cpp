#include <iostream>
#include "itemtracker.h"
#include <opencv2/features2d.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/core/core.hpp>
#include "common.h"
#include "vector"
#include "algorithm"
using namespace cv;
using namespace std;

namespace tracking {

void ItemTracker::init(std::ofstream *logger, VisionData *visdat, std::string name, int16_t viz_id) {
    static int16_t trkr_cntr = 0;
    _uid = trkr_cntr++;
    _viz_id = viz_id;
    _logger = logger;
    _visdat = visdat;
    _name = name;
    track_history_max_size = pparams.fps;
    settings_file = "../../xml/" + name + "tracker.xml";
    std::string window_name = name + "_trkr";

    deserialize_settings();

    if (pparams.insect_tracking_tuning ||pparams.drone_tracking_tuning) {
        namedWindow(window_name, WINDOW_NORMAL);
        createTrackbar("Min disparity", window_name, &min_disparity, 255);
        createTrackbar("Max disparity", window_name, &max_disparity, 255);
        createTrackbar("background_subtract_zone_factor", window_name, &background_subtract_zone_factor, 100);
    }

    init_kalman();

    smoother_posX.init(smooth_width_pos);
    smoother_posY.init(smooth_width_pos);
    smoother_posZ.init(smooth_width_pos);

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

    yaw_smoother.init(6);
    smoother_im_size.init(smooth_blob_props_width);
    smoother_score.init(smooth_blob_props_width);
    smoother_brightness.init(smooth_blob_props_width);

    disparity_prev = 0;

    init_logger();
    initialized = true;
}

void ItemTracker::init_logger() {
    if (_logger->is_open()) {
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
    }
}

void ItemTracker::init_kalman() {
    kfL = cv::KalmanFilter(stateSize, measSize, contrSize, kalman_type);
    stateL =cv::Mat(stateSize, 1, kalman_type);  // [x,y,v_x,v_y,w,h]

    //TODO: tune kalman init values
    cv::setIdentity(kfL.transitionMatrix);
    kfL.measurementMatrix = cv::Mat::zeros(measSize, stateSize, kalman_type);
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

void ItemTracker::calc_world_props_blob_generic(BlobProps * pbs, bool use_max) {
    if (pbs->world_props.trkr_id != _uid) {
        BlobWorldProps w;
        cv::Point2f p;
        if (use_max)
            p = pbs->pt_max;
        else
            p = Point2f(pbs->x, pbs->y);

        w.trkr_id = _uid;

        p*=pparams.imscalef;
        float size = pbs->size*pparams.imscalef;
        w.disparity = stereo_match(p,_visdat->diffL,_visdat->diffR,size);

        if (w.disparity < min_disparity || w.disparity > max_disparity) {
            w.disparity_in_range = false;
        } else {
            w.disparity_in_range = true;

            std::vector<Point3d> camera_coordinates, world_coordinates;
            camera_coordinates.push_back(Point3d(p.x,p.y,-w.disparity));
            camera_coordinates.push_back(Point3d(p.x+size,p.y,-w.disparity)); // to calc world radius
            cv::perspectiveTransform(camera_coordinates,world_coordinates,_visdat->Qf);

            w.radius = cv::norm(world_coordinates[0]-world_coordinates[1]) / 2;
            w.radius_in_range = w.radius < max_size;

            w.x = world_coordinates[0].x;
            w.y = world_coordinates[0].y;
            w.z = world_coordinates[0].z;
            //compensate camera rotation:
            float theta = _visdat->camera_angle * deg2rad;
            float temp_y = w.y * cosf(theta) + w.z * sinf(theta);
            w.z = -w.y * sinf(theta) + w.z * cosf(theta);
            w.y = temp_y;

            w.distance_bkg = _visdat->depth_background_mm.at<float>(p.y,p.x);
            w.distance = sqrtf(powf(w.x,2) + powf(w.y,2) +powf(w.z,2));
            if ((w.distance > w.distance_bkg*(static_cast<float>(background_subtract_zone_factor)/100.f)) )
                w.bkg_check_ok = false;
            else
                w.bkg_check_ok = true;
        }
        pbs->world_props = w;
    }
}

void ItemTracker::update_world_candidate() { //TODO: rename
    if (_world_item.valid) {
        if (!_image_item.blob_is_fused) {
            smoother_im_size.addSample(_image_item.size);
            smoother_brightness.addSample(_image_item.pixel_max);
            _blobs_are_fused_cnt = 0;
        }
        smoother_score.addSample(_image_item.score);
    }
}

void ItemTracker::track(double time) {

    double dt_tracking= (time-t_prev_tracking);
    double dt_predict= (time-t_prev_predict);
    t_prev_predict = time;

    update_world_candidate();

    if ( _world_item.valid) {
        check_consistency(dt_tracking);
        update_tracker_ouput(_world_item.pt,dt_tracking,time,_world_item.iti.disparity);
        update_prediction_state(_image_item.pt(), _world_item.iti.disparity);
        n_frames_lost = 0; // update this after calling update_tracker_ouput, so that it can determine how long tracking was lost
        t_prev_tracking = time; // update dt only if item was detected
        n_frames_tracking++;
    } else {
        n_frames_lost++;
        n_frames_tracking = 0;

        if( n_frames_lost >= n_frames_lost_threshold || !_tracking ) {
            _tracking = false;
            reset_tracker_ouput(time);
        }
    }

    if (_tracking)
        predict(dt_predict,_visdat->frame_id);
    else
        _image_predict_item.valid = false;

    cleanup_paths();

    append_log();
}

//make sure the vectors that contain the path data don't endlesly grow
void ItemTracker::cleanup_paths() {
    if (_visdat->frame_id > path_buf_size) {
        if (path.size() > 0) {
            if (path.begin()->frame_id() < _visdat->frame_id - path_buf_size)
                path.erase(path.begin());
            if (path.begin()->frame_id() > _visdat->frame_id ) //at the end of a realsense video loop, frame_id resets
                path.clear();
        }

        if (predicted_image_path.size() > 0) {
            if (predicted_image_path.begin()->frame_id < _visdat->frame_id - path_buf_size )
                predicted_image_path.erase(predicted_image_path.begin());
            if (predicted_image_path.begin()->frame_id > _visdat->frame_id ) //at the end of a realsense video loop, frame_id resets
                predicted_image_path.clear();
        }
    }

    while (track_history.size() > track_history_max_size)
        track_history.erase(track_history.begin());
}

void ItemTracker::append_log() {
    if (_logger->is_open()) {
        //log all image stuff
        if (path.size()>0)
            (*_logger) << _image_item.x * pparams.imscalef << "; " << _image_item.y * pparams.imscalef << "; " << _image_item.disparity << "; ";
        else
            (*_logger) << -1 << "; " << -1 << "; " << -1 << "; ";
        if (_image_predict_item.valid)
            (*_logger) << _image_predict_item.x * pparams.imscalef << "; " << _image_predict_item.y * pparams.imscalef << "; ";
        else
            (*_logger) << -1 << "; " << -1   << "; ";

        (*_logger) << n_frames_lost << "; " << n_frames_tracking << "; " << _tracking << "; ";
        //log all world stuff
        track_data last = Last_track_data();
        (*_logger) << last.state.pos.x << "; " << last.state.pos.y << "; " << last.state.pos.z << ";" ;
        (*_logger) << last.posX_smooth << "; " << last.posY_smooth << "; " << last.posZ_smooth << ";";
        (*_logger) << last.state.vel.x << "; " << last.state.vel.y << "; " << last.state.vel.z << ";" ;
        (*_logger) << last.state.acc.x << "; " << last.state.acc.y << "; " << last.state.acc.z << ";" ;
    }

    while (track_history.size() > track_history_max_size)
        track_history.erase(track_history.begin());
}

void ItemTracker::predict(float dt, int frame_id) {
    cv::Point2f predicted_image_locationL;
    kfL.transitionMatrix.at<float>(2) = dt;
    kfL.transitionMatrix.at<float>(9) = dt;
    stateL = kfL.predict();

    predicted_image_locationL.x = stateL.at<float>(0);
    predicted_image_locationL.y = stateL.at<float>(1);

    float certainty;
    if (_image_predict_item.valid) {
        certainty  = 1-((1 - _image_predict_item.certainty) * certainty_factor); // certainty_factor times less certain that the previous prediction. The previous prediction certainty is updated with an meas vs predict error score
    }   else {
        certainty = certainty_init;
    }

    if (certainty < 0)
        certainty = 0;
    if (certainty > 1)
        certainty = 1;

    _image_predict_item = ImagePredictItem(predicted_image_locationL,certainty,smoother_im_size.latest(),smoother_brightness.latest(),frame_id);
    predicted_image_path.push_back(_image_predict_item );
}

float ItemTracker::stereo_match(cv::Point closestL, cv::Mat diffL,cv::Mat diffR, float size) {
    //get retangle around blob / changed pixels
    float rectsize = size + 2.f;

    float rectsizeX = ceilf(rectsize*0.5f); // *4.0 results in drone-insect disparity interaction
    float rectsizeY = ceilf(rectsize*0.5f);  // *3.0
    int x1,y1,x2,y2;
    x1 = static_cast<int>((closestL.x-rectsizeX));
    x2 = static_cast<int>(2*rectsizeX);
    y1 = static_cast<int>((closestL.y-rectsizeY));
    y2 = static_cast<int>(2*rectsizeY);
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
    int disparity = 0;
    float minerr = std::numeric_limits<float>::max();
    int tmp_max_disp = max_disparity;
    if (x1 - tmp_max_disp < 0)
        tmp_max_disp = x1;

    int disp_start = min_disparity;
    int disp_end = tmp_max_disp;
    if (n_frames_tracking>5) {
        disp_start = std::max(static_cast<int>(floorf(disparity_prev))-2,disp_start);
        disp_end = std::min(static_cast<int>(ceilf(disparity_prev))+2,disp_end);
    }

    int err [tmp_max_disp] = {0};
    for (int i=disp_start; i<disp_end; i++) {
        cv::Rect roiR(x1-i,y1,x2,y2);
        cv::Mat errV = abs(diffL(roiL) - diffR(roiR));
        err[i] = cv::sum(errV)[0];
        if (err[i] < minerr ) {
            disparity  = i;
            minerr = err[i];
        }
    }

    if (disparity > 0) {
        float sub_err_disp = estimate_sub_disparity(disparity,err);
        return sub_err_disp;
    } else {
        return 0;
    }
}

float ItemTracker::estimate_sub_disparity(int disparity,int * err) {
    int y1 = -err[disparity-1];
    int y2 = -err[disparity];
    int y3 = -err[disparity+1];
    // by assuming a hyperbola shape, the x-location of the hyperbola minimum is determined and used as best guess
    int h31 = (y3 - y1);
    int h21 = (y2 - y1) * 4;
    float sub_disp = static_cast<float>((h21 - h31)) / static_cast<float>(h21 - h31 * 2);
    sub_disp += sinf(sub_disp*2.0f*M_PIf32)*0.13f;
    sub_disp += (disparity-1);

    if (sub_disp<disparity-1 || sub_disp>disparity+1 || sub_disp != sub_disp)
        sub_disp = disparity;

    return sub_disp;
}

void ItemTracker::check_consistency(float dt) {

    if (track_history.size()>1) {
        auto data = track_history.back();

        cv::Point3f prev_pos =cv::Point3f(data.posX_smooth,data.posY_smooth,data.posZ_smooth);
        cv::Point3f predicted_pos = dt * data.vel() + prev_pos;
        float dist = cv::norm(predicted_pos - _world_item.pt);

        if (dist > 0.4f )
            reset_filters = true;
    }
}

void ItemTracker::update_prediction() {
    track_data td = Last_track_data();
    cv::Point3f pos = td.pos();
    cv::Point3f vel = td.vel();
    cv::Point3f acc= td.acc();
    //todo: use control inputs to make prediction

    // predict insect position for next frame
    float dt_pred = 1.f/pparams.fps;
    cv::Point3f predicted_pos = pos + vel*dt_pred + 0.5*acc*powf(dt_pred,2);

    auto p = world2im_2d(predicted_pos,_visdat->Qfi,_visdat->camera_angle);

    //update tracker with prediciton
    _image_predict_item.x = std::clamp(static_cast<int>(p.x),0,IMG_W-1)/pparams.imscalef;
    _image_predict_item.y = std::clamp(static_cast<int>(p.y),0,IMG_H-1)/pparams.imscalef;
    _image_predict_item.size = world2im_size(_world_item.pt+cv::Point3f(dparams.radius,0,0),_world_item.pt-cv::Point3f(dparams.radius,0,0),_visdat->Qfi) / pparams.imscalef;
    //issue #108:
    predicted_image_path.back().x = _image_predict_item.x;
    predicted_image_path.back().y = _image_predict_item.y;
    predicted_image_path.back().size = _image_predict_item.size;
}

void ItemTracker::update_prediction_state(cv::Point2f image_location, float disparity) {
    cv::Mat measL(measSize, 1, kalman_type);
    measL.at<float>(0) = image_location.x;
    measL.at<float>(1) = image_location.y;
    measL.at<float>(2) = disparity;

    if (!_tracking) { // First detection!
        kfL.errorCovPre.at<float>(0) = 1; // px
        kfL.errorCovPre.at<float>(7) = 1; // px
        kfL.errorCovPre.at<float>(14) = 1;
        kfL.errorCovPre.at<float>(21) = 1;
        kfL.errorCovPre.at<float>(28) = 1; // px
        kfL.errorCovPre.at<float>(35) = 1; // px

        stateL.at<float>(0) = measL.at<float>(0);
        stateL.at<float>(1) = measL.at<float>(1);
        stateL.at<float>(2) = 0;

        kfL.statePost = stateL;
        _tracking = true;
    }
    else
        kfL.correct(measL);
}

void ItemTracker::update_tracker_ouput(Point3f measured_world_coordinates,float dt,  double time, float disparity) {

    disparity_prev = disparity;

    //_world_item.certainty = calc_certainty(_world_item.iti.k());; // TODO does not work properly

    path.push_back(_world_item);

    track_data data;
    data.pos_valid = true;
    data.state.pos = measured_world_coordinates;
    if(!isnan(_world_item.yaw)) {
        data.yaw = _world_item.yaw;
        data.yaw_valid = true;
    }
    data.yaw_smooth = yaw_smoother.addSample(_world_item.yaw);

    if (n_frames_lost >= smooth_width_vel || reset_filters) { // tracking was regained, after n_frames_lost frames
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
    if (reset_filters || track_history.size()<1) {
        data.posX_smooth = data.state.pos.x;
        data.posY_smooth = data.state.pos.y;
        data.posZ_smooth = data.state.pos.z;
    } else {
        float pos_filt_rate = 0.3f;
        auto data_prev = track_history.back();
        data.posX_smooth = data.state.pos.x*pos_filt_rate + data_prev.posX_smooth*(1.0f-pos_filt_rate);
        data.posY_smooth = data.state.pos.y*pos_filt_rate + data_prev.posY_smooth*(1.0f-pos_filt_rate);
        data.posZ_smooth = data.state.pos.z*pos_filt_rate + data_prev.posZ_smooth*(1.0f-pos_filt_rate);
    }

    if (!reset_filters) { // dt is making a big jump with reset_filters
        data.state.vel.x = smoother_velX2.addSample(data.posX_smooth,dt);
        data.state.vel.y = smoother_velY2.addSample(data.posY_smooth,dt);
        data.state.vel.z = smoother_velZ2.addSample(data.posZ_smooth,dt);
    }

    if (smoother_velX2.ready()) {
        data.state.acc.x = smoother_accX2.addSample(data.state.vel.x,dt);
        data.state.acc.y = smoother_accY2.addSample(data.state.vel.y,dt);
        data.state.acc.z = smoother_accZ2.addSample(data.state.vel.z,dt);
    } else if (track_history.size()>0 && !reset_filters) {
        auto data_prev = track_history.back();
        data.state.acc.x = smoother_accX2.addSample((data.posX_smooth-data_prev.posX_smooth)/dt,dt);
        data.state.acc.y = smoother_accY2.addSample((data.posY_smooth-data_prev.posY_smooth)/dt,dt);
        data.state.acc.z = smoother_accZ2.addSample((data.posZ_smooth-data_prev.posZ_smooth)/dt,dt);
    }

    data.vel_valid = smoother_velX2.ready();
    data.acc_valid = smoother_accX2.ready();

    data.time = time;
    last_sighting_time = time;
    data.dt = dt;
    detected_after_take_off++;

    reset_filters = false;

    track_history.push_back(data);
}

float ItemTracker::calc_certainty(KeyPoint item) {
    float new_tracking_certainty;
    if (_image_predict_item.valid) {
        new_tracking_certainty = 1.f / powf(powf(_image_predict_item.x - item.pt.x,2) + powf(_image_predict_item.y - item.pt.y,2),0.3f);
        new_tracking_certainty*= _image_predict_item.certainty;
        if (new_tracking_certainty>1 || new_tracking_certainty<0 || isnanf(new_tracking_certainty)) { // weird -nan sometimes???
            new_tracking_certainty = 1;
        }
    } else // if there was no prediciton, certainty prolly is quite low
        new_tracking_certainty = certainty_init;
    return new_tracking_certainty;
}

void ItemTracker::reset_tracker_ouput(double time) {
    track_data data;
    reset_filters = true;
    _image_predict_item.valid = false;
    data.time = time;
    track_history.push_back(data);
    init_kalman();
}

bool ItemTracker::check_ignore_blobs_generic(BlobProps * pbs) {
    bool in_ignore_zone = false;
    for (auto ignore : ignores_for_me) {
        float dist_ignore = sqrtf(powf(ignore.p.x-pbs->x,2)+powf(ignore.p.y-pbs->y,2));
        if (dist_ignore < pbs->size + ignore.radius ) {
            ignore.was_used = true;
            pbs->ignores.push_back(ignore);
            in_ignore_zone = true;
        }
    }

    return in_ignore_zone;
}

float ItemTracker::score(BlobProps blob, ImageItem ref) {
    float dist = sqrtf(powf(ref.x-blob.x,2)+powf(ref.y-blob.y,2));
    float im_size_diff = fabs(ref.size - blob.size) / (blob.size + ref.size);
    float score = 1.f / (dist + 15.f*im_size_diff); // TODO: certainty

    if (_image_predict_item.valid) {
        float dist_pred = sqrtf(powf(_image_predict_item.x-blob.x,2)+powf(_image_predict_item.y-blob.y,2));
        float ps = smoother_im_size.latest();
        float im_size_diff_pred = fabs(ps - blob.size) / (blob.size+ps);
        float score_pred = 1.f / (dist_pred + 15.f*im_size_diff_pred); // TODO: certainty
        if (score_pred > score)
            score = score_pred;
    }

    return score*1000.f;
}

void ItemTracker::deserialize_settings() {
    std::cout << "Reading settings from: " << settings_file << std::endl;
    TrackerParams params;
    if (file_exist(settings_file)) {
        std::ifstream infile(settings_file);
        std::string xmlData((std::istreambuf_iterator<char>(infile)),
                            std::istreambuf_iterator<char>());

        if (!xmls::Serializable::fromXML(xmlData, &params))
        {   // Deserialization not successful
            throw my_exit("Cannot read: " + settings_file);
        }
        TrackerParams tmp;
        auto v1 = params.getVersion();
        auto v2 = tmp.getVersion();
        if (v1 != v2) {
            throw my_exit("XML version difference detected from " + settings_file);
        }
    } else {
        throw my_exit("File not found: " + settings_file);
    }

    min_disparity = params.min_disparity.value();
    max_disparity = params.max_disparity.value();
    _score_threshold = params.score_threshold.value();
    background_subtract_zone_factor = params.background_subtract_zone_factor.value();
    max_size = params.max_size.value();
}

void ItemTracker::serialize_settings() {
    TrackerParams params;

    params.min_disparity = min_disparity;
    params.max_disparity = max_disparity;
    params.score_threshold = _score_threshold;
    params.background_subtract_zone_factor = background_subtract_zone_factor;
    params.max_size = max_size;

    std::string xmlData = params.toXML();
    std::ofstream outfile = std::ofstream (settings_file);
    outfile << xmlData ;
    outfile.close();
}

void ItemTracker::close () {
    if (initialized) {
        (*_logger) << std::flush;
        std::cout << "Closing tracker: " << _name << std::endl;
        if (pparams.insect_tracking_tuning || pparams.drone_tracking_tuning)
            serialize_settings();
        initialized = false;
    }
}

}
