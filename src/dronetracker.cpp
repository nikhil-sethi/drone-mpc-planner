#include "dronetracker.h"

bool DroneTracker::init(std::ofstream *logger, VisionData *visdat) {
#ifdef HASSCREEN
    enable_viz_diff = false;
#endif
    ItemTracker::init(logger,visdat,"drone");

    return false;
}
void DroneTracker::init_settings() {
    settings.roi_min_size = 150;
    settings.roi_max_grow = 50;
    settings.roi_grow_speed = 64;
    settings.background_subtract_zone_factor = 97;
}

void DroneTracker::track(double time, bool drone_is_active) {
    current_time = time;

    if (enable_viz_diff)
        cv::cvtColor(_visdat->diffL*10,diff_viz,CV_GRAY2BGR);

    switch (_drone_tracking_status) {
    case dts_init: {
        append_log(); // no tracking needed in this stage
        _drone_tracking_status = dts_inactive;
        break;
    }  case dts_inactive: {
        roi_size_cnt = 0; // don't grow roi in this stage
        start_take_off_time = time;
        predicted_image_path.clear();
        path.clear();
        _tracking = false;
        find_result.best_image_locationL.pt = _drone_blink_image_location;
        predicted_image_path.push_back(ImagePredictItem(_drone_blink_image_location,1,DRONE_IM_START_SIZE,_visdat->frame_id));
        reset_tracker_ouput(time);
        _drone_control_prediction_valid = false;
        if (drone_is_active)
            _drone_tracking_status = dts_detecting_takeoff_init;
        else {
            ItemTracker::append_log(); //really no point in trying to detect the drone when it is inactive...
            break;
        }
    } FALLTHROUGH_INTENDED; case dts_detecting_takeoff_init: {
        // remove the indefinite startup location and replace with a time out
        ignores_for_other_trkrs.clear();
        ignores_for_other_trkrs.push_back(IgnoreBlob(drone_startup_im_location(),time+startup_location_ignore_timeout, IgnoreBlob::landing_spot));
        _drone_tracking_status = dts_detecting_takeoff;
    } FALLTHROUGH_INTENDED; case dts_detecting_takeoff: {
        insert_control_predicted_drone_location();
        ItemTracker::track(time);

        if (enable_viz_diff){
            cv::Point2f tmpp = drone_startup_im_location();
            cv::circle(diff_viz,tmpp*IMSCALEF,1,cv::Scalar(255,0,0),1);
            cv::circle(diff_viz,drone_startup_im_location()*IMSCALEF,1,cv::Scalar(0,0,255),1);
            cv::circle(diff_viz, _world_item.image_coordinates()*IMSCALEF,3,cv::Scalar(0,255,0),2);
        }
        if (!drone_is_active)
            _drone_tracking_status = dts_inactive;
        else if (n_frames_lost==0 && _world_item.valid ){
            bool takeoff_spot_detected = false;
            bool drone_detected_near_takeoff_spot = false;

            for (uint i = 0; i< ignores_for_other_trkrs.size(); i++) {
                if (ignores_for_other_trkrs.at(i).ignore_type == IgnoreBlob::landing_spot && ignores_for_other_trkrs.at(i).was_used)
                    takeoff_spot_detected = true;
            }

            float dist2take_off = sqrt(pow(_image_item.x - drone_startup_im_location().x,2)+pow(_image_item.y - drone_startup_im_location().y,2));
            if (dist2take_off > settings.pixel_dist_seperation_min + DRONE_IM_START_SIZE && dist2take_off < settings.pixel_dist_seperation_max + DRONE_IM_START_SIZE){
                drone_detected_near_takeoff_spot = true;
                ignores_for_other_trkrs.push_back(IgnoreBlob(_image_item.pt(),time+taking_off_ignore_timeout, IgnoreBlob::drone_taking_off));
            } else if (dist2take_off < settings.pixel_dist_seperation_max + DRONE_IM_START_SIZE){
                ignores_for_other_trkrs.push_back(IgnoreBlob(_image_item.pt(),time+taking_off_ignore_timeout, IgnoreBlob::drone_taking_off));
            }

            if (takeoff_spot_detected &&drone_detected_near_takeoff_spot ) {
                _drone_tracking_status = dts_detected;
                _visdat->delete_from_motion_map(drone_startup_im_location()*IMSCALEF, DRONE_IM_START_SIZE);
            }
        }
        roi_size_cnt = 0; // don't grow roi in this stage
        break;
    } case dts_detecting: {
        insert_control_predicted_drone_location();
        ItemTracker::track(time);
        if (!drone_is_active)
            _drone_tracking_status = dts_inactive;
        else if (n_frames_lost==0)
            _drone_tracking_status = dts_detected;
        break;
    } case dts_detected: {
        insert_control_predicted_drone_location();
        ItemTracker::track(time);
        if (!drone_is_active)
            _drone_tracking_status = dts_inactive;
        else if (!_tracking)
            _drone_tracking_status = dts_detecting;
        break;
    }
    }
    clean_ignore_blobs(time);


}

//Removes all ignore points which timed out
void DroneTracker::clean_ignore_blobs(double time){
    std::vector<IgnoreBlob> new_ignores_for_insect_tracker;
    for (uint i = 0; i < ignores_for_other_trkrs.size(); i++) {
        if (ignores_for_other_trkrs.at(i).was_used && ignores_for_other_trkrs.at(i).invalid_after>=0)
            ignores_for_other_trkrs.at(i).invalid_after += 1./VIDEOFPS;
        ignores_for_other_trkrs.at(i).was_used = false;
        if (ignores_for_other_trkrs.at(i).invalid_after > time || ignores_for_other_trkrs.at(i).invalid_after<0)
            new_ignores_for_insect_tracker.push_back(ignores_for_other_trkrs.at(i));
    }
    ignores_for_other_trkrs= new_ignores_for_insect_tracker;
}
