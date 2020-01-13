#include "insecttracker.h"
#include "common.h"

using namespace cv;
using namespace std;
namespace tracking {

void InsectTracker::init(int id, VisionData *visdat, int16_t viz_id) {
    _insect_trkr_id = id;
    std::ofstream * logger = new std::ofstream(); // FIXME: use std::shared_ptr?
    std::string logger_fn;
    logger_fn = data_output_dir  + "log_itrk" + to_string(id) + ".csv";
    (*logger).open(logger_fn,std::ofstream::out);
    (*logger) << "RS_ID;time;";
    ItemTracker::init(logger,visdat,"insect",viz_id);
    (*logger) << std::endl;
    n_frames_lost = 0;
}
void InsectTracker::start_new_log_line(double time, unsigned long long frame_number) {
    (*_logger) << std::to_string(frame_number) << ";";
    (*_logger) << std::to_string(time) << ";";
}

void InsectTracker::append_log(double time, unsigned long long frame_number) {
    start_new_log_line(time,frame_number);
    ItemTracker::append_log();
    (*_logger) << std::endl;
}

void InsectTracker::track(double time) {

    start_new_log_line(time,_visdat->frame_id);
    ItemTracker::track(time);

    if (!_tracking) {
        predicted_image_path.clear();
        path.clear();
    } else {
        update_insect_prediction();
    }
    (*_logger) << std::endl;
}

void InsectTracker::update_insect_prediction() {
    track_data itd = Last_track_data();
    cv::Point3f insect_pos = itd.pos();
    cv::Point3f insect_vel = itd.vel();
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

void InsectTracker::calc_world_item(BlobProps * pbs, double time [[maybe_unused]]) {
    calc_world_props_blob_generic(pbs);
    pbs->world_props.valid = pbs->world_props.bkg_check_ok && pbs->world_props.disparity_in_range & pbs->world_props.radius_in_range;

    if (_blobs_are_fused_cnt > 1 * pparams.fps) // if the insect and drone are fused, the drone is accelerating through it and should become seperate again within a limited time
        pbs->world_props.valid = false;
}

bool InsectTracker::check_ignore_blobs(BlobProps * pbs) {
    return this->check_ignore_blobs_generic(pbs);
}

}
