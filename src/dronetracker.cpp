#include "dronetracker.h"


bool DroneTracker::init(std::ofstream *logger, VisionData *visdat) {
    ItemTracker::init(logger,visdat,"drone");

    find_result.best_image_locationL.pt.x = DRONE_IM_X_START;
    find_result.best_image_locationL.pt.y = DRONE_IM_Y_START;
    find_result.best_image_locationL.size = DRONE_IM_START_SIZE;
    find_result.smoothed_disparity = DRONE_DISPARITY_START;
    find_result.disparity = DRONE_DISPARITY_START;
    return false;
}
void DroneTracker::init_settings() {
    //thresh params
    settings.iLowH1r = 18;
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
    settings.minArea = 20;
    settings.maxArea = 640;

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
    settings.max_disparity=43;

    settings.roi_min_size = 200;
    settings.roi_max_grow = 50;
    settings.roi_grow_speed = 64;

    settings.background_subtract_zone_factor = 97;
}

void DroneTracker::track(float time, std::vector<track_item> ignore, bool drone_is_active) {

    switch (_drone_tracking_state) {
    case dts_initialize_blink_locater: {
        _blinking_drone_located = bds_start;
        break;
    } case dts_blinking: {
        //can only be taken out of this state externally
        break;
    } case dts_inactive: {
        //todo: change start up location to detected
        //todo: really no point in trying to detect the drone when it is inactive... (currently only necessary to properly write the log)
        find_result.best_image_locationL.pt.x = DRONE_IM_X_START;
        find_result.best_image_locationL.pt.y =  DRONE_IM_Y_START;
        find_result.smoothed_disparity = DRONE_DISPARITY_START;
        find_result.disparity = DRONE_DISPARITY_START;
        predicted_locationL_last.x = DRONE_IM_X_START;
        predicted_locationL_last.y = DRONE_IM_Y_START;
        if (drone_is_active)
            _drone_tracking_state = dts_active_detecting;
        break;
    } case dts_active_detecting: {
        if (!drone_is_active) {
            reset_tracker_ouput(); //TODO: double?
            _drone_tracking_state = dts_inactive;
        } else if (n_frames_lost==0)
            _drone_tracking_state = dts_found_after_takeoff;

        break;
    } case dts_found_after_takeoff: {
        break;
    }
    }

    switch (_blinking_drone_located) {
    case bds_none: {
        ItemTracker::track(time,ignore,drone_max_border_y,drone_max_border_z);
        break;
    } case bds_start: {
        _enable_roi = false;
        pathL.clear();
        predicted_pathL.clear();
        foundL = false; // todo: is this necessary?
        //todo: also disable background depth map?
        _blinking_drone_located = bds_resetting_background;
        _visdat->reset_motion_integration();
        ItemTracker::append_log(); // write a dummy entry
        break;
    } case bds_resetting_background: {
        _blinking_drone_located = bds_searching; // -> wait 1 frame
        ItemTracker::append_log(); // write a dummy entry
        break;
    } case bds_searching: {
        ItemTracker::track(time,ignore,drone_max_border_y,drone_max_border_z);
        if (foundL) {
            blink_location = find_result.best_image_locationL;
            _blinking_drone_located = bds_blink_off;
            _enable_roi = false;
            blink_time_start = time;
        }
        break;
    } case bds_blink_off: {
        ItemTracker::track(time,ignore,drone_max_border_y,drone_max_border_z);
        _blinking_drone_located = detect_blink(time, !foundL);
        break;
    } case bds_blink_on: {
        ItemTracker::track(time,ignore,drone_max_border_y,drone_max_border_z);
        _blinking_drone_located = detect_blink(time, foundL);
        break;
    } case bds_2nd_blink_off: {
        ItemTracker::track(time,ignore,drone_max_border_y,drone_max_border_z);
        _blinking_drone_located = detect_blink(time, !foundL);
        break;
    } case bds_2nd_blink_on: {
        ItemTracker::track(time,ignore,drone_max_border_y,drone_max_border_z);
        _blinking_drone_located = detect_blink(time, foundL);
        break;
    } case bds_3th_blink_off: {
        ItemTracker::track(time,ignore,drone_max_border_y,drone_max_border_z);
        _blinking_drone_located = detect_blink(time, !foundL);
        break;
    } case bds_3th_blink_on: {
        ItemTracker::track(time,ignore,drone_max_border_y,drone_max_border_z);
        _blinking_drone_located = detect_blink(time, foundL);
        break;
    } case bds_found: {
        //todo: do something profound here :)
        break;
    }
    }

    if (_drone_tracking_state == dts_active_detecting) {
        predicted_pathL.clear();
#ifndef INSECT_LOGGING_MODE
        //hack to disable the dronetracker
        predicted_pathL.push_back(track_item(find_result.best_image_locationL,_visdat->frame_id,0.1f));
#endif
        foundL = false;
        roi_size_cnt = 0;
    }
}

DroneTracker::blinking_drone_state DroneTracker::detect_blink(float time, bool found) {
    float blink_period = time - blink_time_start;
    if (found) {
        if ( blink_period > bind_blink_time - 0.16f && blink_period < bind_blink_time+0.16f) { //todo: figure out what's up with the 0.16
            blink_time_start = time;
            int tmp  =static_cast<int>(_blinking_drone_located)+1;
            return static_cast<blinking_drone_state>(tmp);
        } else {
            return bds_searching;
        }
    } else if (!found && blink_period > bind_blink_time +0.16f) {
        return bds_searching;
    }
    return _blinking_drone_located;
}

cv::Mat DroneTracker::get_probability_cloud(cv::Point size) {
    //TODO: make proper probability esitmate based on control inputs and movement estimates
    return createBlurryCircle(size);
}


/* Takes the calibrated uncertainty map, and augments it with a highlight around p */
cv::Mat DroneTracker::get_approx_cutout_filtered(cv::Point p, cv::Mat diffL, cv::Point size) {

    if (_enable_roi) {
        //calc roi:
        cv::Rect roi_circle(0,0,size.x,size.y);
        int x1 = p.x-size.x/2;
        if (x1 < 0) {
            roi_circle.x = abs(x1);
            roi_circle.width-=roi_circle.x;
        } else if (x1 + size.x >= diffL.cols)
            roi_circle.width = roi_circle.width  - abs(x1 + size.x - diffL.cols);

        int y1 = p.y-size.y/2;
        if (y1 < 0) {
            roi_circle.y = abs(y1);
            roi_circle.height-=roi_circle.y;
        } else if (y1 + size.y >= diffL.rows)
            roi_circle.height = roi_circle.height - abs(y1 + size.y - diffL.rows);

        cv::Mat blurred_circle = get_probability_cloud(size);
        cv::Mat cir = blurred_circle(roi_circle);
        _cir = cir;

        x1 = p.x-size.x/2+roi_circle.x;
        int x2 = roi_circle.width;
        y1 = p.y-size.y/2+roi_circle.y;
        int y2 = roi_circle.height;

        cv::Rect roi(x1,y1,x2,y2);
        find_result.roi_offset = roi;

        _bkg = _visdat->uncertainty_map(roi);
        diffL(roi).convertTo(_dif, CV_32F);
        cv::Mat res;
        res = cir.mul(_dif).mul(_bkg);
        res.convertTo(res, CV_8UC1);

        return res;
    } else {
        find_result.roi_offset = cv::Rect(0,0,diffL.cols,diffL.rows);
        return diffL;
    }
}
