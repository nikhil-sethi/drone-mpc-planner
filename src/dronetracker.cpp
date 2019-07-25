#include "dronetracker.h"

bool DroneTracker::init(std::ofstream *logger, VisionData *visdat) {
#ifdef HASSCREEN
    enable_viz_diff = false;
#endif
    ItemTracker::init(logger,visdat,"drone");
    cv::invert(visdat->Qf,Qfi);
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
                //calculate take off speed
                float dy_to = _world_item.pt.y - _drone_blink_world_location.y;
                float dt_to = time - start_take_off_time;

                //assuming linear acceleration
                //x = 0.5a*t^2
                //a = (2x)/t^2
                float a_to = 2.f*dy_to / powf(dt_to,2);
                float v_final_to = a_to*dt_to;

                auto data = track_history.back(); //hacky
                track_history.clear(); // hack
                data.svelY = v_final_to;
                data.saccY = a_to;
                track_history.push_back(data);
                for (uint hack = 0; hack<70;hack++) {
                    smoother_velY2.addSample(_world_item.pt.y + (-69 * v_final_to * dt_to) + (v_final_to*dt_to*hack),dt_to); // extremely dirty hack
                    smoother_accY2.addSample(v_final_to + (-69 * a_to * dt_to) + (a_to*dt_to*hack),dt_to); // extremely dirty hack
                }

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
        update_drone_prediction();
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

    if (inactive())
        wbp.valid = false;

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

void DroneTracker::update_drone_prediction() {
    track_data td = Last_track_data();
    cv::Point3f pos = {td.posX,td.posY,td.posZ};
    cv::Point3f vel = {td.svelX,td.svelY,td.svelZ};
    //TODO: also consider acc?


    // predict insect position for next frame
    float dt_pred = 1.f/VIDEOFPS;
    cv::Point3f predicted_pos = pos + vel*dt_pred;

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

    //issue #108:
    predicted_image_path.back().x = image_location.x;
    predicted_image_path.back().y = image_location.y;
    _image_predict_item.x = image_location.x;
    _image_predict_item.y = image_location.y;

}

