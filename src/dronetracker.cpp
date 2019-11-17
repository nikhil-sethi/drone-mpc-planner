#include "dronetracker.h"

#include "multimodule.h"

bool DroneTracker::init(std::ofstream *logger, VisionData *visdat) {
    enable_viz_diff = false;
    ItemTracker::init(logger,visdat,"drone");
    (*_logger) << "dtrkr_state;";
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

        if (_landing_pad_location_set && ignores_for_other_trkrs.size() == 0) {
            //some times there is some motion noise around the drone when it is just sitting on the ground
            //not sure what that is (might be a flickering led?), but the following makes the insect tracker
            //ignore it. Can be better fixed by having a specialized landingspot detector.
            ignores_for_other_trkrs.push_back(IgnoreBlob(drone_startup_im_location(),
                                                         _drone_blink_im_size*5,
                                                         time+startup_location_ignore_timeout,
                                                         IgnoreBlob::takeoff_spot));
        }

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
    } case dts_reset_heading: {
        ItemTracker::track(time);
        update_drone_prediction();
        _visdat->exclude_drone_from_motion_fading(_image_item.pt()*pparams.imscalef,_image_item.size*1.2f*pparams.imscalef);
        find_heading = true;
        break;
    } case dts_landing_init: {
        find_heading = false;
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
    (*_logger) << static_cast<int16_t>(_drone_tracking_status) << ";";
}

ItemTracker::BlobWorldProps DroneTracker::calc_world_item(BlobProps * pbs, double time) {
    auto wbp = calc_world_props_blob_generic(pbs);
    wbp.valid = wbp.bkg_check_ok && wbp.disparity_in_range && wbp.radius_in_range;

    if (inactive()) {
        wbp.valid = false;
        return wbp;
    } else if (taking_off() && wbp.valid && !_manual_flight_mode) {

        float dt = time - start_take_off_time;
        dt -= dparams.full_bat_and_throttle_spinup_duration;
        if (dt<0)
            dt=0;

        cv::Point3f expected_drone_location = _drone_blink_world_location;
        expected_drone_location.y+= 0.5f*dparams.full_bat_and_throttle_take_off_acc * powf(dt,2);

        float err_y = wbp.y - expected_drone_location.y;
        float err_pos = static_cast<float>(norm(expected_drone_location - cv::Point3f(wbp.x,wbp.y,wbp.z)));
        float dy = wbp.y - _drone_blink_world_location.y;

        // only accept the drone blob when
        // 1) it is 10cm up in the air, because we then have a reasonable good seperation and can calculate the state
        // 2) it is reasonably close to the prediciton
        float dy_takeoff_detected = 0.12f;
        if (fabs(err_y) > 1.f || err_pos > 1.f || dy < dy_takeoff_detected) {
            wbp.valid = false;
            takeoff_detection_dy_prev = dy;
            takeoff_detection_dt_prev = dt;
            return wbp;
        } else {
            // Interpolate time to the time of dy=dy_takeoff_detected:
            float t = takeoff_detection_dt_prev + (dt - takeoff_detection_dt_prev) / (dy - takeoff_detection_dy_prev)
                                                      * (dy_takeoff_detected - takeoff_detection_dy_prev);
            //            std::cout << "dt: " << dt << " dy: " << dy << " dt(k-1): " << takeoff_detection_dt_prev << " dy(k-1): " << takeoff_detection_dy_prev << std::endl;
            hover_throttle_estimation = dparams.hover_throttle_a*t + dparams.hover_throttle_b ;
            std::cout << "Initialising hover-throttle: " << hover_throttle_estimation << std::endl;
        }
    }
    if(find_heading==true){
        heading = calc_heading(pbs, false); // Set second argument to true to show the masks, otherwise set to false.
        wbp.heading = heading;
    }
    return wbp;
}

bool DroneTracker::check_ignore_blobs(BlobProps * pbs, double time) {

    if ( this->check_ignore_blobs_generic(pbs)) {
        for (auto ignore : pbs->ignores){ // TODO: use c+17 filter
            if (ignore.ignore_type == ItemTracker::IgnoreBlob::IgnoreType::takeoff_spot) {
                auto w = calc_world_item(pbs,time); // TODO: this is called a few times over the same data later in the tracker manager. May that should be streamlined.
                if (norm(w.pt() - drone_startup_location()) < 0.1)
                    return true;
            } else
                return true;
        }
    }

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
    cv::Point3f pos = td.pos();
    cv::Point3f vel = td.vel();
    cv::Point3f acc= td.acc();
    //todo: use control inputs to make prediction

    // predict insect position for next frame
    float dt_pred = 1.f/pparams.fps;
    cv::Point3f predicted_pos = pos + vel*dt_pred + 0.5*acc*powf(dt_pred,2);

    //transform back to image coordinates
    std::vector<cv::Point3d> world_coordinates,camera_coordinates;
    cv::Point3f tmp(predicted_pos.x,predicted_pos.y,predicted_pos.z);

    //derotate camera and convert to double:
    cv::Point3d tmpd;
    float theta = -_visdat->camera_angle * deg2rad;
    float temp_y = tmp.y * cosf(theta) + tmp.z * sinf(theta);
    tmpd.z = static_cast<double>(-tmp.y * sinf(theta) + tmp.z * cosf(theta));
    tmpd.y = static_cast<double>(temp_y);
    tmpd.x = static_cast<double>(tmp.x);

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

cv::Mat DroneTracker::get_big_blob(cv::Mat Mask, int connectivity){
    cv::Mat labels, stats, centroids;
    int nLabels = cv::connectedComponentsWithStats(Mask, labels, stats, centroids, connectivity, CV_32S);

    cv::Mat mask_big;
    int big_blob;
    int area = 0;

    for(int i = 1; i<=nLabels; i++){
        if(stats.at<int>(i,cv::CC_STAT_AREA) > area){
            big_blob = i;
            area = stats.at<int>(i,cv::CC_STAT_AREA);
        }
    }
    compare(labels, big_blob, mask_big, cv::CMP_EQ);
    return mask_big;
}

cv::Mat DroneTracker::extract_mask_column(cv::Mat mask_big, float range_left, float range_right, float side_percentage, enum side side_){
    cv::Mat half_ = mask_big(cv::Range::all(), cv::Range(range_left,range_right)).clone();
    for(int j=0; j<half_.rows; j++){
        half_.at<uchar>(j,(half_.cols-1)*(1-side_)) = round(side_percentage*half_.at<uchar>(j,(half_.cols-1)*(1-side_)));
    }
    return half_;
}

cv::Mat DroneTracker::split_mask_half(cv::Mat mask_big, enum side side_){
    cv::Moments mo = moments(mask_big,true);
    cv::Point2f COG = cv::Point2f(static_cast<float>(mo.m10) / static_cast<float>(mo.m00), static_cast<float>(mo.m01) / static_cast<float>(mo.m00));
    float delta = COG.x - mask_big.rows/2;
    float delta_frac, n;
    delta_frac = modf(delta, &n);
    cv::Mat half;

    if(side_==leftside && delta>=0 && delta<mask_big.rows/2){
        half = extract_mask_column(mask_big, 2*floor(delta), mask_big.rows/2+ceil(delta), delta_frac, side_);
    }
    else if(side_==leftside && delta<0 && delta>-mask_big.rows/2){
        half = extract_mask_column(mask_big, 0, mask_big.rows/2+ceil(delta), 1-abs(delta_frac), side_);
    }
    else if(side_==rightside && delta>=0 && delta<mask_big.rows/2){
        half = extract_mask_column(mask_big, mask_big.rows/2+floor(delta), mask_big.rows, 1-delta_frac, side_);
    }
    else if(side_==rightside && delta<0 && delta>-mask_big.rows/2){
        half = extract_mask_column(mask_big, mask_big.rows/2+floor(delta), mask_big.rows+2*floor(delta), abs(delta_frac), side_);
    }
    return half;
}

float DroneTracker::yaw_heading(cv::Mat left, cv::Mat right){ // Heading positive = Clockwise, Heading negative is Counter-Clockwise
    cv::Moments mo_l = moments(left,true);
    cv::Point2f COG_l = cv::Point2f(static_cast<float>(mo_l.m10) / static_cast<float>(mo_l.m00), static_cast<float>(mo_l.m01) / static_cast<float>(mo_l.m00));
    cv::Moments mo_r = moments(right,true);
    cv::Point2f COG_r = cv::Point2f(static_cast<float>(mo_r.m10) / static_cast<float>(mo_r.m00), static_cast<float>(mo_r.m01) / static_cast<float>(mo_r.m00));

    heading = COG_l.y-COG_r.y;
    return heading;
}

float DroneTracker::calc_heading(BlobProps * pbs, bool inspect_blob){ // Set inspect_blob = true to see mask. Otherwise set to false.
    if(inspect_blob==true){
        cout<<"Original Drone Mask: "<<endl;
        cout<<pbs->mask_<<endl;
    }
    if(!pbs->mask_.empty()){
        int kernel_int = 2;
        cv::Mat mask_erode;
        erode(pbs->mask_, mask_erode,getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_int,kernel_int)));
        if(inspect_blob==true){
            cout<<"eroded mask: "<<endl;
            cout<<mask_erode<<endl;
        }

        cv::Mat mask_big = get_big_blob(mask_erode, 4);
        if(inspect_blob==true){
            cout<<"New Drone Mask: "<<endl;
            cout<<mask_big<<endl;
        }

        int nrnonzero = countNonZero(mask_big);
        cv::Mat splitted_mask_left, splitted_mask_right;
        if(nrnonzero > 1){
            splitted_mask_left = split_mask_half(mask_big, leftside);
            splitted_mask_right = split_mask_half(mask_big, rightside);
        }
        else if(nrnonzero < 1){
            splitted_mask_left = split_mask_half(mask_erode, leftside);
            splitted_mask_right = split_mask_half(mask_erode, rightside);
        }
        if(inspect_blob==true){
            cout<<"Left: "<<splitted_mask_left<<endl;
            cout<<"Right: "<<splitted_mask_right<<endl;
        }
        heading = yaw_heading(splitted_mask_left, splitted_mask_right);
        if(inspect_blob==true){
            cout<<"Heading: "<<heading<<endl;
        }
    }
    return heading;
}
