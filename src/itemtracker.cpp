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

void ItemTracker::init(std::ofstream *logger, VisionData *visdat, std::string name) {
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

    smoother_im_size.init(smooth_blob_props_width);
    smoother_score.init(smooth_blob_props_width);
    smoother_brightness.init(smooth_blob_props_width);

    disparity_prev = 0;
    disparity_smoothed = 0;

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

ItemTracker::BlobWorldProps ItemTracker::calc_world_props_blob_generic(BlobProps * pbs){
    BlobWorldProps w;
    cv::Point2f p(pbs->x, pbs->y);

    w.disparity = stereo_match(p,_visdat->diffL,_visdat->diffR,pbs->radius); //TODO: wtf, inputs scaled with pparams.imscalef, but disparity is unscaled?
    p*=pparams.imscalef;

    if (w.disparity < min_disparity || w.disparity > max_disparity){
        w.disparity_in_range = false;
    } else {
        w.disparity_in_range = true;

        std::vector<Point3d> camera_coordinates, world_coordinates;
        camera_coordinates.push_back(Point3d(p.x,p.y,-w.disparity));
        camera_coordinates.push_back(Point3d(p.x+pbs->radius*pparams.imscalef,p.y,-w.disparity)); // to calc world radius
        cv::perspectiveTransform(camera_coordinates,world_coordinates,_visdat->Qf);

        w.radius = cv::norm(world_coordinates[0]-world_coordinates[1]);
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
    return w;
}

void ItemTracker::update_world_candidate(){ //TODO: rename
    if (_world_item.valid){
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
        update_disparity(_world_item.iti.disparity, dt_tracking);
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
        (*_logger) << last.sposX << "; " << last.sposY << "; " << last.sposZ << ";";
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

float ItemTracker::stereo_match(cv::Point closestL, cv::Mat diffL,cv::Mat diffR, float radius){
    //get retangle around blob / changed pixels
    float rectsize = radius*2.f + 2.f;

    float rectsizeX = ceil(rectsize*0.5f); // *4.0 results in drone-insect disparity interaction
    float rectsizeY = ceil(rectsize*0.5f);  // *3.0
    int x1,y1,x2,y2;
    x1 = static_cast<int>((closestL.x-rectsizeX)*pparams.imscalef);
    x2 = static_cast<int>(2*rectsizeX*pparams.imscalef);
    y1 = static_cast<int>((closestL.y-rectsizeY)*pparams.imscalef);
    y2 = static_cast<int>(2*rectsizeY*pparams.imscalef);
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
    int tmp_max_disp = max_disparity;
    if (x1 - tmp_max_disp < 0)
        tmp_max_disp = x1;

    int disp_start = min_disparity;
    int disp_end = tmp_max_disp;
    if (n_frames_tracking>5) {
        disp_start = std::max(static_cast<int>(floor(disparity_prev))-2,disp_start);
        disp_end = std::min(static_cast<int>(ceil(disparity_prev))+2,disp_end);
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

    float sub_disparity;
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
    sub_disp += sinf(sub_disp*2.0f*M_PIf32)*0.13f;
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

void ItemTracker::check_consistency(float dt) {

    if (track_history.size()>1) {
        auto data = track_history.back();

        cv::Point3f prev_pos =cv::Point3f(data.sposX,data.sposY,data.sposZ);
        cv::Point3f predicted_pos = dt * data.vel() + prev_pos;
        float dist = cv::norm(predicted_pos - _world_item.pt);

        if (dist > 0.4f )
            reset_filters = true;
    }
}

void ItemTracker::update_prediction_state(cv::Point2f image_location, float disparity) {
    cv::Mat measL(measSize, 1, type);
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
    data.heading = _world_item.heading;

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
    if (reset_filters || track_history.size()<1) {
        data.sposX = data.state.pos.x;
        data.sposY = data.state.pos.y;
        data.sposZ = data.state.pos.z;
    } else {
        float pos_filt_rate = 0.3f;
        auto data_prev = track_history.back();
        data.sposX = data.state.pos.x*pos_filt_rate + data_prev.sposX*(1.0f-pos_filt_rate);
        data.sposY = data.state.pos.y*pos_filt_rate + data_prev.sposY*(1.0f-pos_filt_rate);
        data.sposZ = data.state.pos.z*pos_filt_rate + data_prev.sposZ*(1.0f-pos_filt_rate);
    }

    if (!reset_filters){ // dt is making a big jump with reset_filters
        data.state.vel.x = smoother_velX2.addSample(data.sposX,dt);
        data.state.vel.y = smoother_velY2.addSample(data.sposY,dt);
        data.state.vel.z = smoother_velZ2.addSample(data.sposZ,dt);
    }

    if (smoother_velX2.ready()) {
        data.state.acc.x = smoother_accX2.addSample(data.state.vel.x,dt);
        data.state.acc.y = smoother_accY2.addSample(data.state.vel.y,dt);
        data.state.acc.z = smoother_accZ2.addSample(data.state.vel.z,dt);
    } else if (track_history.size()>0 && !reset_filters){
        auto data_prev = track_history.back();
        data.state.acc.x = smoother_accX2.addSample((data.sposX-data_prev.sposX)/dt,dt);
        data.state.acc.y = smoother_accY2.addSample((data.sposY-data_prev.sposY)/dt,dt);
        data.state.acc.z = smoother_accZ2.addSample((data.sposZ-data_prev.sposZ)/dt,dt);
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
    for (uint k=0; k<ignores_for_me.size();k++){
        cv::Point2f p_ignore = ignores_for_me.at(k).p;
        float dist_ignore = sqrtf(powf(p_ignore.x-pbs->x,2)+powf(p_ignore.y-pbs->y,2));
        if (dist_ignore < pbs->radius + ignores_for_me.at(k).radius ){
            ignores_for_me.at(k).was_used = true;
            pbs->ignores.push_back(ignores_for_me.at(k));
            in_ignore_zone = true;
        }
    }

    return in_ignore_zone;
}

void ItemTracker::deserialize_settings() {
    std::cout << "Reading settings from: " << settings_file << std::endl;
    TrackerParams params;
    if (file_exist(settings_file)) {
        std::ifstream infile(settings_file);
        std::string xmlData((std::istreambuf_iterator<char>(infile)),
                            std::istreambuf_iterator<char>());

        if (!xmls::Serializable::fromXML(xmlData, &params))
        { // Deserialization not successful
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
    static_ignores_dist_thresh = params.static_ignores_dist_thresh.value();
    _score_threshold = params.score_threshold.value();
    background_subtract_zone_factor = params.background_subtract_zone_factor.value();
    max_size = params.max_size.value();
}

void ItemTracker::serialize_settings() {
    TrackerParams params;

    params.min_disparity = min_disparity;
    params.max_disparity = max_disparity;
    params.static_ignores_dist_thresh = static_ignores_dist_thresh;
    params.score_threshold = _score_threshold;
    params.background_subtract_zone_factor = background_subtract_zone_factor;
    params.max_size = max_size;

    std::string xmlData = params.toXML();
    std::ofstream outfile = std::ofstream (settings_file);
    outfile << xmlData ;
    outfile.close();
}

void ItemTracker::close () {
    if (initialized){
        std::cout << "Closing tracker: " << _name << std::endl;
        if (pparams.insect_tracking_tuning || pparams.drone_tracking_tuning)
            serialize_settings();
        initialized = false;
    }
}
