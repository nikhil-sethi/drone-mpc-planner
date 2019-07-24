#include "dronetracker.h"

bool DroneTracker::init(std::ofstream *logger, VisionData *visdat) {
#ifdef HASSCREEN
    enable_viz_diff = false;
#endif
    ItemTracker::init(logger,visdat,"drone");

    return false;
}
void DroneTracker::init_settings() {
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
        start_take_off_time = time;
        predicted_image_path.clear();
        path.clear();
        _tracking = false;
        _image_predict_item = ImagePredictItem(_drone_blink_im_location,1,_drone_blink_im_size,255,_visdat->frame_id);
        predicted_image_path.push_back(_image_predict_item);
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
        start_take_off_time = time;
        ignores_for_other_trkrs.clear();
        ignores_for_other_trkrs.push_back(IgnoreBlob(drone_startup_im_location(),time+startup_location_ignore_timeout, IgnoreBlob::landing_spot));
        ignores_for_me.push_back(IgnoreBlob(drone_startup_im_location(),time+startup_location_ignore_timeout, IgnoreBlob::landing_spot));
        _drone_tracking_status = dts_detecting_takeoff;
    } FALLTHROUGH_INTENDED; case dts_detecting_takeoff: {
        if (!_world_item.valid){
            cv::Point2f expected_drone_location = _drone_blink_im_location;
            float dt = current_time - start_take_off_time;
            expected_drone_location.y+= dt * full_throttle_im_effect;
            _image_predict_item = ImagePredictItem(expected_drone_location,1,_drone_blink_im_size,255,_visdat->frame_id);
        }

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
            if (dist2take_off > settings.takeoff_seperation_min * _drone_blink_im_size &&
                    dist2take_off < settings.takeoff_seperation_max* _drone_blink_im_size &&
                    _image_item.size > _drone_blink_im_size*0.5f ){
                drone_detected_near_takeoff_spot = true;
                ignores_for_other_trkrs.push_back(IgnoreBlob(_image_item.pt(),time+taking_off_ignore_timeout, IgnoreBlob::drone_taking_off));
            } else if (dist2take_off < settings.takeoff_seperation_max + _drone_blink_im_size){
                ignores_for_other_trkrs.push_back(IgnoreBlob(_image_item.pt(),time+taking_off_ignore_timeout, IgnoreBlob::drone_taking_off));
            }

            if (takeoff_spot_detected &&drone_detected_near_takeoff_spot ) {
                _drone_tracking_status = dts_detected;
                _visdat->delete_from_motion_map(drone_startup_im_location()*IMSCALEF, _drone_blink_im_disparity,ceilf(_drone_blink_im_size*2.f)*IMSCALEF,VIDEOFPS/2);
            }
        }
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
        _visdat->exclude_drone_from_motion_fading(_image_item.pt()*IMSCALEF,_image_item.size*1.2f*IMSCALEF); // TODO: use this trick also in the blinktracker
        if (!drone_is_active)
            _drone_tracking_status = dts_inactive;
        else if (!_tracking)
            _drone_tracking_status = dts_detecting;
        break;
    }
    }
    clean_ignore_blobs(time);


}

ItemTracker::BlobWorldProps DroneTracker::calc_tmp_world_item(BlobProps * pbs) {
    auto wbp = calc_world_props_blob_generic(pbs);
    wbp.valid = wbp.bkg_check_ok && wbp.disparity_in_range;
    return wbp;
}

bool DroneTracker::check_ignore_blobs(BlobProps * pbs, uint id) {

    if ( this->check_ignore_blobs_generic(pbs))
        return true;

    if (taking_off() && !_manual_flight_mode) {

        cv::Point2f expected_drone_location = _drone_blink_im_location;
        float dt = current_time - start_take_off_time;
        expected_drone_location.y+= dt * full_throttle_im_effect;

        float d = sqrtf(powf(expected_drone_location.x-pbs->x,2)+powf(expected_drone_location.y-pbs->y,2));
        std::cout << id << ": " <<  pbs->x << ",  "  << pbs->y <<  " d: "  << d << std::endl;
        if (d > 20)
            return true;
    }
    return false;

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
