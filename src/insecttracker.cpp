#include "insecttracker.h"

using namespace cv;
using namespace std;

void InsectTracker::init(std::ofstream *logger, VisionData *visdat) {
    ItemTracker::init(logger,visdat,"insect");

    cv::invert(visdat->Qf,Qfi);
}
void InsectTracker::init_settings() {
    settings.min_disparity=1;
    settings.motion_thresh = 15;

    settings.roi_min_size = 200;
    settings.roi_max_grow = 160;
    settings.roi_grow_speed = 64;

}

void InsectTracker::update_from_log(LogReader::Log_Entry log, int frame_number) {

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

    track_item ti(cv::KeyPoint(cv::Point2f(log.ins_im_x/IMSCALEF,log.ins_im_y/IMSCALEF),1),frame_number,1);
    track_item tip(cv::KeyPoint(cv::Point2f(log.ins_pred_im_x/IMSCALEF,log.ins_pred_im_y/IMSCALEF),1),frame_number,1);

    pathL.push_back(ti);
    predicted_pathL.push_back(tip);

    n_frames_lost = log.ins_n_frames_lost;
    n_frames_tracking = log.ins_n_frames_tracking;
    foundL = log.ins_foundL;

    if (pathL.size() > 0) {
        if (pathL.begin()->frame_id < _visdat->frame_id - path_buf_size)
            pathL.erase(pathL.begin());
    }
    if (predicted_pathL.size() > 0) {
        if (predicted_pathL.begin()->frame_id < _visdat->frame_id - path_buf_size)
            predicted_pathL.erase(predicted_pathL.begin());
    }

    if (n_frames_lost > n_frames_lost_threshold || !foundL) {
        predicted_pathL.clear();
        foundL = false;
    }

    append_log();
}

void InsectTracker::track(float time, std::vector<track_item> exclude,std::vector<cv::Point2f> additional_ignores) {

    ItemTracker::track(time,exclude,additional_ignores);

    if (n_frames_lost > n_frames_lost_threshold || !foundL) {
        predicted_pathL.clear();
        pathL.clear();
        foundL = false;
    } else {
        update_insect_prediction();
    }

}


/* Takes the calibrated uncertainty map, and augments it with a highlight around p */
cv::Mat InsectTracker::get_approx_cutout_filtered(cv::Point p, cv::Mat diffL_small, cv::Point size) {

    cv::Mat blurred_circle = get_probability_cloud(size);

    int x = p.x-size.x/2;
    if (x < 0)
        x = 0;
    int width = size.x;
    if (x+width > diffL_small.cols)
        x -= (x+width) - diffL_small.cols;

    int y = p.y-size.y/2;
    if (y < 0)
        y = 0;
    int height = size.y;
    if (y+height>diffL_small.rows)
        y -= (y+height) - diffL_small.rows;

    cv::Rect roi(x,y,width,height);
    find_result.roi_offset = roi;

    diffL_small(roi).convertTo(_dif, CV_32F);
    cv::Mat res;
    res = blurred_circle.mul(_dif);
    res.convertTo(res, CV_8UC1);

    return res;
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

    predicted_locationL_last.x = image_location.x;
    predicted_locationL_last.y = image_location.y;

}
