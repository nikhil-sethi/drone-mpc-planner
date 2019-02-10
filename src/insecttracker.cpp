#include "insecttracker.h"

using namespace cv;
using namespace std;

void InsectTracker::init(std::ofstream *logger, VisionData *visdat) {
    ItemTracker::init(logger,visdat,"insect");
}
void InsectTracker::init_settings() {
    //thresh params
    settings.iLowH1r = 10;
    settings.iHighH1r = 255;
    settings.iLowS1r = 0;
    settings.iHighS1r = 255;
    settings.iLowV1r = 188;
    settings.iHighV1r = 255;
    settings.iOpen1r =0;
    settings.iClose1r =0;

    //blob params

    // Change thresholds
    settings.minThreshold = 10;
    settings.maxThreshold = 91;

    // Filter by Area.
    settings.filterByArea = 1;
    settings.minArea = 0;
    settings.maxArea = 80;

    // Filter by Circularity
    settings.filterByCircularity = 0;
    settings.minCircularity = 10;
    settings.maxCircularity = 100;

    // Filter by Convexity
    settings.filterByConvexity = 0;
    settings.minConvexity = 87;
    settings.maxConvexity = 100;

    // Filter by Inertia
    settings.filterByInertia = 0;
    settings.minInertiaRatio = 1;
    settings.maxInertiaRatio = 100;

    settings.min_disparity=1;
    settings.max_disparity=20;

    settings.roi_min_size = 200;
    settings.roi_max_grow = 160;
    settings.roi_grow_speed = 64;

}

void InsectTracker::update_from_log(LogReader::Log_Entry log, int frame_number) {

    trackData data ={0};
    data.valid = true;
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
}

void InsectTracker::track(float time, std::vector<track_item> exclude,std::vector<cv::Point2f> additional_ignores) {

    ItemTracker::track(time,exclude,additional_ignores);

    if (n_frames_lost > n_frames_lost_threshold || !foundL) {
        predicted_pathL.clear();
        foundL = false;
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
