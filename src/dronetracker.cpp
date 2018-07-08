#include "dronetracker.h"


bool DroneTracker::init(std::ofstream *logger, VisionData *visdat) {
    ItemTracker::init(logger,visdat,"drone");

    (*_logger) << "imLx; imLy; disparity;";

    data.image_locationL = cv::Point(DRONE_IM_X_START,DRONE_IM_Y_START);
    find_result.smoothed_disparity = DRONE_DISPARITY_START;
    find_result.disparity = DRONE_DISPARITY_START;
    data.landed = true;
    return false;
}
void DroneTracker::init_settings() {
    //thresh params
    settings.iLowH1r = 10;
    settings.iHighH1r = 255;
    settings.iLowS1r = 0;
    settings.iHighS1r = 255;
    settings.iLowV1r = 188;
    settings.iHighV1r = 255;
    settings.iOpen1r =0;
    settings.iClose1r =2;

    //blob params

    // Change thresholds
    settings.minThreshold = 10;
    settings.maxThreshold = 91;

    // Filter by Area.
    settings.filterByArea = 1;
    settings.minArea = 1;
    settings.maxArea = 40;

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

    settings.min_disparity=0;
    settings.max_disparity=20;

    settings.roi_min_size = 200;
    settings.roi_max_grow = 250;
    settings.roi_grow_speed = 64;
}

void DroneTracker::track(float time, cv::Point3f setpoint_world, std::vector<track_item> ignore) {

    if (data.landed) {
        frameL_s_prev_OK = _visdat->_frameL_s_prev;
        frameL_prev_OK = _visdat->_frameL_prev;
        frameR_prev_OK = _visdat->_frameR_prev;

        data.image_locationL = cv::Point(DRONE_IM_X_START,DRONE_IM_Y_START);
        find_result.smoothed_disparity = DRONE_DISPARITY_START;
        find_result.disparity = DRONE_DISPARITY_START;
        nframes_since_update_prev = 0;
    }

    ItemTracker::track(time,setpoint_world,ignore,drone_max_border_y,drone_max_border_z);

    if (data.landed) {
        find_result.update_prev_frame = true;
        reset_tracker_ouput();
    }

    (*_logger) << find_result.best_image_locationL.pt.x *IMSCALEF << "; " << find_result.best_image_locationL.pt.y *IMSCALEF << "; " << find_result.disparity << "; ";


}

cv::Mat DroneTracker::get_probability_cloud(cv::Point size) {
    //TODO: make proper probability esitmate based on control inputs and movement estimates
    return createBlurryCircle(size);
}
