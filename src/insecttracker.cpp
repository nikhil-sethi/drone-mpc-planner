#include "insecttracker.h"
#include "common.h"

using namespace cv;
using namespace std;

void InsectTracker::init(std::ofstream *logger, VisionData *visdat) {
    ItemTracker::init(logger,visdat,"insect");
}

void InsectTracker::update_from_log(LogReader::Log_Entry log, int frame_number, double time) {

    _image_item = ImageItem (log.ins_im_x/pparams.imscalef,log.ins_im_y/pparams.imscalef,log.ins_disparity,frame_number);
    ImagePredictItem ipi(cv::Point2f(log.ins_pred_im_x/pparams.imscalef,log.ins_pred_im_y/pparams.imscalef),1,1,255,frame_number);
    predicted_image_path.push_back(ipi);
    ipi.valid = ipi.x > 0 ;
    _image_predict_item = ipi;

    //TODO: recalculate this instead of reading from the log
    WorldItem w;
    w.iti = _image_item;
    w.valid = true;
    w.pt.x = log.ins_pos_x;
    w.pt.y = log.ins_pos_y;
    w.pt.z = log.ins_pos_z;
    path.push_back(w);
    _world_item = w;

    track_data data ={0};
    data.pos_valid = true;
    data.posX = log.ins_pos_x;
    data.posY = log.ins_pos_y;
    data.posZ = log.ins_pos_z;
    data.sposX = log.ins_spos_x;
    data.sposY = log.ins_spos_y;
    data.sposZ = log.ins_spos_z;
    data.svelX = log.ins_svel_x;
    data.svelY = log.ins_svel_y;
    data.svelZ = log.ins_svel_z;
    data.saccX = log.ins_sacc_x;
    data.saccY = log.ins_sacc_y;
    data.saccZ = log.ins_sacc_z;
    data.time = time;
    track_history.push_back(data);

    n_frames_lost = log.ins_n_frames_lost;
    n_frames_tracking = log.ins_n_frames_tracking;
    _tracking = log.ins_foundL;

    if (!_tracking) {
        predicted_image_path.clear();
        _image_predict_item.valid = false;
    }

    cleanup_paths();
    append_log();
}

void InsectTracker::track(double time) {

    ItemTracker::track(time);

    if (!_tracking) {
        predicted_image_path.clear();
        path.clear();
    } else {
        update_insect_prediction();
    }

}

void InsectTracker::update_insect_prediction() {
    track_data itd = Last_track_data();
    cv::Point3f insect_pos = {itd.posX,itd.posY,itd.posZ};
    cv::Point3f insect_vel = {itd.svelX,itd.svelY,itd.svelZ};
    //TODO: also consider acc?


    // predict insect position for next frame
    float dt_pred = 1.f/pparams.fps;
    cv::Point3f predicted_pos = insect_pos + insect_vel*dt_pred;

    //transform back to image coordinates
    std::vector<cv::Point3d> world_coordinates,camera_coordinates;
    cv::Point3f tmp(predicted_pos.x,predicted_pos.y,predicted_pos.z);

    //derotate camera and convert to double:
    cv::Point3d tmpd;
    float theta = -_visdat->camera_angle * deg2rad;
    float temp_y = tmp.y * cosf(theta) + tmp.z * sinf(theta);
    tmpd.z = -tmp.y * sinf(theta) + tmp.z * cosf(theta);
    tmpd.y = temp_y;
    tmpd.x = tmp.x;

    world_coordinates.push_back(tmpd);
    cv::perspectiveTransform(world_coordinates,camera_coordinates,_visdat->Qfi);

    //update tracker with prediciton
    cv::Point2f image_location;
    image_location.x= camera_coordinates.at(0).x/pparams.imscalef;
    image_location.y= camera_coordinates.at(0).y/pparams.imscalef;

    if (image_location.x < 0)
        image_location.x = 0;
    else if (image_location.x >= IMG_W/pparams.imscalef)
        image_location.x = IMG_W/pparams.imscalef-1;
    if (image_location.y < 0)
        image_location.y = 0;
    else if (image_location.y >= IMG_H/pparams.imscalef)
        image_location.y = IMG_H/pparams.imscalef-1;

    //issue #108:
    predicted_image_path.back().x = image_location.x;
    predicted_image_path.back().y = image_location.y;
    _image_predict_item.x = image_location.x;
    _image_predict_item.y = image_location.y;

}

ItemTracker::BlobWorldProps InsectTracker::calc_world_item(BlobProps * pbs, double time __attribute__((unused))){
    auto wbp = calc_world_props_blob_generic(pbs);
    wbp.valid = wbp.bkg_check_ok && wbp.disparity_in_range & wbp.radius_in_range;

    if (_blobs_are_fused_cnt > 1 * pparams.fps) // if the insect and drone are fused, the drone is accelerating through it and should become seperate again within a limited time
        wbp.valid = false;
    return wbp;
}

bool InsectTracker::check_ignore_blobs(BlobProps * pbs, uint id __attribute__((unused))) {
    return this->check_ignore_blobs_generic(pbs);
}
