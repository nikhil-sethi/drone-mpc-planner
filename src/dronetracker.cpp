#include "dronetracker.h"

#include "multimodule.h"

bool DroneTracker::init(std::ofstream *logger, VisionData *visdat) {
    enable_viz_diff = false;
    ItemTracker::init(logger,visdat,"drone");
    return false;
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
        if (drone_is_active)
            _drone_tracking_status = dts_detecting_takeoff_init;
        else {
            ItemTracker::append_log(); //really no point in trying to detect the drone when it is inactive...
            break;
        }
        [[fallthrough]];
    } case dts_detecting_takeoff_init: {
        // remove the indefinite startup location and replace with a time out
        start_take_off_time = time;
        ignores_for_other_trkrs.clear();
        ignores_for_other_trkrs.push_back(IgnoreBlob(drone_startup_im_location(),_drone_blink_im_size*5,time+startup_location_ignore_timeout, IgnoreBlob::takeoff_spot));
        ignores_for_me.push_back(IgnoreBlob(drone_startup_im_location(),_drone_blink_im_size*2,time+startup_location_ignore_timeout, IgnoreBlob::takeoff_spot));
        _drone_tracking_status = dts_detecting_takeoff;
        [[fallthrough]];
    } case dts_detecting_takeoff: {
        if (!_world_item.valid){
            //TODO: remove full_bat_and_throttle_im_effect and use acc to calc back to image coordinates
            cv::Point2f expected_drone_location = _drone_blink_im_location;
            float dt = current_time - start_take_off_time;
            expected_drone_location.y+= dt * dparams.full_bat_and_throttle_im_effect;
            _image_predict_item = ImagePredictItem(expected_drone_location,1,_drone_blink_im_size,255,_visdat->frame_id);
        }

        ItemTracker::track(time);

        if (enable_viz_diff){
            cv::Point2f tmpp = drone_startup_im_location();
            cv::circle(diff_viz,tmpp*pparams.imscalef,1,cv::Scalar(255,0,0),1);
            cv::circle(diff_viz,drone_startup_im_location()*pparams.imscalef,1,cv::Scalar(0,0,255),1);
            cv::circle(diff_viz, _world_item.image_coordinates()*pparams.imscalef,3,cv::Scalar(0,255,0),2);
        }
        if (!drone_is_active)
            _drone_tracking_status = dts_inactive;
        else if (n_frames_lost==0 && _world_item.valid){
            bool takeoff_spot_detected = false;

            for (uint i = 0; i< ignores_for_other_trkrs.size(); i++) {
                if (ignores_for_other_trkrs.at(i).ignore_type == IgnoreBlob::takeoff_spot && ignores_for_other_trkrs.at(i).was_used)
                    takeoff_spot_detected = true;
            }

            if (takeoff_spot_detected  ) {
                _drone_tracking_status = dts_detected;
                //calculate take off speed
                /* LUDWIG HELP!
                float dy_to = _world_item.pt.y - _drone_blink_world_location.y;
                float dt_to = static_cast<float>(time - start_take_off_time) - dparams.full_bat_and_throttle_spinup_time;

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
                float tmp_v,tmp_acc;
                for (uint hack = 0; hack<70;hack++) {
                    tmp_v = smoother_velY2.addSample(_world_item.pt.y + (-69 * v_final_to * dt_to) + (v_final_to*dt_to*hack),dt_to); // extremely dirty hack
                    tmp_acc = smoother_accY2.addSample(v_final_to + (-69 * a_to * dt_to) + (a_to*dt_to*hack),dt_to); // extremely dirty hack
                }
                std::cout << "Start dy: " << dy_to << ", dt: " << (time - start_take_off_time) << std::endl;
                std::cout << "Start v: " << tmp_v << ", a: " << tmp_acc << std::endl;
                std::cout << "Start v: " << data.svelY << ", a: " << data.saccY << std::endl;
*/
                _visdat->delete_from_motion_map(drone_startup_im_location()*pparams.imscalef, _drone_blink_im_disparity,ceilf(_drone_blink_im_size*2.f)*pparams.imscalef,pparams.fps/2);
            }
        }
        break;
    } case dts_detecting: {
        ItemTracker::track(time);
        if (!drone_is_active)
            _drone_tracking_status = dts_inactive;
        else if (n_frames_lost==0)
            _drone_tracking_status = dts_detected;
        break;
    } case dts_detected: {
        ItemTracker::track(time);
        update_drone_prediction();
        _visdat->exclude_drone_from_motion_fading(_image_item.pt()*pparams.imscalef,_image_item.size*1.2f*pparams.imscalef);
        if (!drone_is_active)
            _drone_tracking_status = dts_inactive;
        else if (!_tracking)
            _drone_tracking_status = dts_detecting;
        break;
    } case dts_landing_init: {
        ignores_for_other_trkrs.push_back(IgnoreBlob(drone_startup_im_location(),_drone_blink_im_size*5,time+landing_ignore_timeout, IgnoreBlob::landing_spot));
        _drone_tracking_status = dts_landing;
        [[fallthrough]];
    } case dts_landing: {
        ItemTracker::track(time);
        update_drone_prediction();
        _visdat->exclude_drone_from_motion_fading(_image_item.pt()*pparams.imscalef,_image_item.size*1.2f*pparams.imscalef);
        if (!drone_is_active)
            _drone_tracking_status = dts_inactive;
        else if (!_tracking)
            _drone_tracking_status = dts_detecting;
        break;
    }
    }
    clean_ignore_blobs(time);
}

ItemTracker::BlobWorldProps DroneTracker::calc_world_item(BlobProps * pbs, double time) {
    auto wbp = calc_world_props_blob_generic(pbs);
    wbp.valid = wbp.bkg_check_ok && wbp.disparity_in_range && wbp.radius_in_range;

    if (inactive()) {
        wbp.valid = false;
        return wbp;
    } else if (taking_off() && wbp.valid && !_manual_flight_mode) {

        float dt = time - start_take_off_time;
        if (dt < dparams.full_bat_and_throttle_spinup_time) { // spin up time
            wbp.valid = false;
            return wbp;
        }
        dt -= dparams.full_bat_and_throttle_spinup_time;

        cv::Point3f expected_drone_location = _drone_blink_world_location;
        expected_drone_location.y+= 0.5f*dparams.full_bat_and_throttle_take_off_acc * powf(dt,2);

        float err_y = wbp.y - expected_drone_location.y;
        float err_pos = static_cast<float>(norm(expected_drone_location - cv::Point3f(wbp.x,wbp.y,wbp.z)));
        float dy = wbp.y - _drone_blink_world_location.y;

        // only accept the drone blob when
        // 1) it is 10cm up in the air, because we then have a reasonable good seperation and can calculate the state
        // 2) it is reasonably close to the prediciton
        if (fabs(err_y) > 1.f || err_pos > 1.f || dy < 0.12f) {
            wbp.valid = false;
            return wbp;
        } else {
            float vel_y = dy/dt;
            //std::cout << "detection dy: " << dy << " dt: " << dt << " vy: " << vel_y<< std::endl;
            hover_throttle_estimation = dparams.hover_throttle_a*vel_y + dparams.hover_throttle_b ;
            //std::cout << "Initialising ht [-1, 1]: " << hover_throttle_estimation << std::endl;
            hover_throttle_estimation +=1;
            hover_throttle_estimation /= 2.f;
            hover_throttle_estimation *=JOY_BOUND_MAX - JOY_BOUND_MIN;
            hover_throttle_estimation+=JOY_BOUND_MIN;
        }
    }

    return wbp;
}

bool DroneTracker::check_ignore_blobs(BlobProps * pbs, uint id  __attribute__((unused))) {

    if ( this->check_ignore_blobs_generic(pbs))
        return true;

    return false;
}

//Removes all ignore points which timed out
void DroneTracker::clean_ignore_blobs(double time){
    std::vector<IgnoreBlob> new_ignores_for_insect_tracker;
    for (uint i = 0; i < ignores_for_other_trkrs.size(); i++) {
        if (ignores_for_other_trkrs.at(i).was_used && ignores_for_other_trkrs.at(i).invalid_after>=0)
            ignores_for_other_trkrs.at(i).invalid_after += 1./pparams.fps;
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
    float dt_pred = 1.f/pparams.fps;
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

