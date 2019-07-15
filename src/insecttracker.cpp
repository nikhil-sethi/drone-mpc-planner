#include "insecttracker.h"
#include "common.h"

using namespace cv;
using namespace std;

void InsectTracker::init(std::ofstream *logger, VisionData *visdat) {
    ItemTracker::init(logger,visdat,"insect");

    cv::invert(visdat->Qf,Qfi);
}
void InsectTracker::init_settings() {
    settings.min_disparity=1;
}

void InsectTracker::update_from_log(LogReader::Log_Entry log, int frame_number) {

    _image_item = ImageItem (log.ins_im_x/IMSCALEF,log.ins_im_y/IMSCALEF,log.ins_disparity,frame_number);
    ImagePredictItem ipi(cv::Point2f(log.ins_pred_im_x/IMSCALEF,log.ins_pred_im_y/IMSCALEF),1,1,frame_number);
    predicted_image_path.push_back(ipi);

    //TODO: recalculate this instead of reading from the log
    WorldItem w;
    w.iti = _image_item;
    w.disparity_in_range = true;
    w.background_check_ok = true;
    w.valid = true;
    w.pt.x = log.ins_pos_x;
    w.pt.y = log.ins_pos_y;
    w.pt.z = log.ins_pos_z;
    path.push_back(w);

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
    track_history.push_back(data);

    n_frames_lost = log.ins_n_frames_lost;
    n_frames_tracking = log.ins_n_frames_tracking;
    _tracking = log.ins_foundL;

    if (path.size() > 0) {
        if (path.begin()->frame_id() < _visdat->frame_id - path_buf_size)
            path.erase(path.begin());
    }
    if (predicted_image_path.size() > 0) {
        if (predicted_image_path.begin()->frame_id < _visdat->frame_id - path_buf_size)
            predicted_image_path.erase(predicted_image_path.begin());
    }

    if (n_frames_lost > n_frames_lost_threshold || !_tracking)
        predicted_image_path.clear();

    append_log();
}

void InsectTracker::track(double time) {

    ItemTracker::track(time);

    if (n_frames_lost > n_frames_lost_threshold || !_tracking) {
        predicted_image_path.clear();
        path.clear();
        _tracking = false;
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
    float dt_pred = 1.f/VIDEOFPS;
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
    cv::perspectiveTransform(world_coordinates,camera_coordinates,Qfi);

    //update tracker with prediciton
    cv::Point2f image_location;
    image_location.x= camera_coordinates.at(0).x/IMSCALEF;
    image_location.y= camera_coordinates.at(0).y/IMSCALEF;

    if (image_location.x < 0)
        image_location.x = 0;
    else if (image_location.x >= IMG_W/IMSCALEF)
        image_location.x = IMG_W/IMSCALEF-1;
    if (image_location.y < 0)
        image_location.y = 0;
    else if (image_location.y >= IMG_H/IMSCALEF)
        image_location.y = IMG_H/IMSCALEF-1;

    //TODO: this seems very ugly:
    predicted_image_path.back().x = image_location.x;
    predicted_image_path.back().y = image_location.y;


}
