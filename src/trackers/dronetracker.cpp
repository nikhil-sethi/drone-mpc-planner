#include "dronetracker.h"
#include "multimodule.h"

namespace tracking {

bool DroneTracker::init(std::ofstream *logger, VisionData *visdat, int16_t viz_id) {
    enable_viz_diff = false;
    ItemTracker::init(logger,visdat,"drone",viz_id);
    max_size = dparams.radius*3;
    landing_parameter.deserialize("../../xml/landing_location.xml");
    (*_logger) << "dtrkr_state;";
    return false;
}

void DroneTracker::update(double time, bool drone_is_active) {
    current_time = time;

    if (enable_viz_diff)
        cv::cvtColor(_visdat->diffL*10,diff_viz,cv::COLOR_GRAY2BGR);

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
        if (_landing_pad_location_set) {
            cv::Point3f p= world2im_3d(drone_takeoff_location(),_visdat->Qfi,_visdat->camera_angle);
            float size = _drone_takeoff_im_size;
            _image_predict_item = ImagePredictItem(p,1,size,255,_visdat->frame_id);
            predicted_image_path.push_back(_image_predict_item);
        } else {
            _image_predict_item.valid = false;
            reset_tracker_ouput(time);
        }

        if (_landing_pad_location_set && ignores_for_other_trkrs.size() == 0) {
            //some times there is some motion noise around the drone when it is just sitting on the ground
            //not sure what that is (might be a flickering led?), but the following makes the insect tracker
            //ignore it. Can be better fixed by having a specialized landingspot detector.
            ignores_for_other_trkrs.push_back(IgnoreBlob(drone_takeoff_im_location()/pparams.imscalef,
                                              _drone_takeoff_im_size/pparams.imscalef,
                                              time+takeoff_location_ignore_timeout,
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
        enable_takeoff_motion_delete = true;
        ignores_for_other_trkrs.clear();
        ignores_for_other_trkrs.push_back(IgnoreBlob(drone_takeoff_im_location()/pparams.imscalef,drone_takeoff_im_size()/pparams.imscalef,time+takeoff_location_ignore_timeout, IgnoreBlob::takeoff_spot));
        _drone_tracking_status = dts_detecting_takeoff;
        spinup_detected = 0;
        _take_off_detection_failed = false;
        take_off_frame_cnt = 0;
        [[fallthrough]];
    } case dts_detecting_takeoff: {
        ItemTracker::update(time);
        if (!_world_item.valid) {
            calc_takeoff_prediction();
        } else {
            update_prediction(time);
        }

        if (enable_viz_diff) {
            cv::Point2f tmpp = blnk_im_location();
            cv::circle(diff_viz,tmpp*pparams.imscalef,1,cv::Scalar(255,0,0),1);
            cv::circle(diff_viz,blnk_im_location()*pparams.imscalef,1,cv::Scalar(0,0,255),1);
            cv::circle(diff_viz, _world_item.image_coordinates()*pparams.imscalef,3,cv::Scalar(0,255,0),2);
        }
        if (!drone_is_active)
            _drone_tracking_status = dts_inactive;
        else if (_world_item.valid) {
            spinup_detected++;
            if (spinup_detected==3) {
                spinup_detect_time = time;
                std::cout << "Spin up detected after: " << time - start_take_off_time << "s" << std::endl;
            }
            if (detect_lift_off()) {
                std::cout << "Lift off detected after: " << time - start_take_off_time << "s" << std::endl;
                liftoff_detected = true;
                if (detect_takeoff()) {
                    std::cout << "Take off detected after: " << time - start_take_off_time << "s" << std::endl;
                    _drone_tracking_status = dts_tracking;
                }
            }
        } else if (spinup_detected < 3) {
            spinup_detected = 0;
        }
        if(spinup_detected<3 && (time - start_take_off_time) > 0.7) { // hmm spinup detection really does not work with improper lighting conditions. Have set the time really high. (should be ~0.3-0.4s)
            _take_off_detection_failed = true;
            std::cout << "No spin up detected in time!" << std::endl;
        } else if ((time - start_take_off_time) > 0.75) {
            _take_off_detection_failed = true;
            if (liftoff_detected)
                std::cout << "No takeoff detected in time!" << std::endl;
            else
                std::cout << "No liftoff_detected detected in time!" << std::endl;
        }
        break;
    } case dts_detecting: {
        ItemTracker::update(time);
        if (!drone_is_active)
            _drone_tracking_status = dts_inactive;
        else if (n_frames_lost==0)
            _drone_tracking_status = dts_tracking;
        break;
    } case dts_tracking: {
        ItemTracker::update(time);
        update_prediction(time); // use control inputs to make prediction #282
        _visdat->exclude_drone_from_motion_fading(_image_item.ptd(),_image_predict_item.size);
        if (!drone_is_active)
            _drone_tracking_status = dts_inactive;
        else if (!_tracking)
            _drone_tracking_status = dts_detecting;
        break;
    } case dts_detect_yaw: {
        ItemTracker::update(time);
        update_prediction(time);
        _visdat->exclude_drone_from_motion_fading(_image_item.ptd(),_image_predict_item.size);
        break;
    } case dts_landing_init: {
        ignores_for_other_trkrs.push_back(IgnoreBlob(drone_takeoff_im_location()/pparams.imscalef,drone_takeoff_im_size()/pparams.imscalef,time+landing_ignore_timeout, IgnoreBlob::landing_spot));
        _drone_tracking_status = dts_landing;
        [[fallthrough]];
    } case dts_landing: {
        ItemTracker::update(time);
        update_prediction(time);
        _visdat->exclude_drone_from_motion_fading(_image_item.ptd(),_image_predict_item.size);
        if (!drone_is_active)
            _drone_tracking_status = dts_inactive;
        else if (!_tracking)
            _drone_tracking_status = dts_detecting;
        break;
    }
    }

    delete_takeoff_fake_motion();
    clean_ignore_blobs(time);
    (*_logger) << static_cast<int16_t>(_drone_tracking_status) << ";";
}

void DroneTracker::delete_takeoff_fake_motion() {
    if (enable_takeoff_motion_delete) {
        _visdat->reset_spot_on_motion_map(drone_takeoff_im_location(), _blink_im_disparity,_drone_takeoff_im_size*2.5f,1);

        //to end the deletion of this area, we check if there are not blobs in this area anymore because
        //they leave a permanent mark if we stop prematurely. Two conditions:
        //1. the drone must have left the area with a margin of its size
        //2. other blobs must not be inside the area. (slightly more relaxed, because crop leave movements otherwise are holding this enabled indefinetely)
        if (_world_item.valid &&normf(cv::Point2f(_world_item.iti.x,_world_item.iti.y)*pparams.imscalef - drone_takeoff_im_location()) > _drone_takeoff_im_size*2.5f + _world_item.iti.size/2*pparams.imscalef ) {
            enable_takeoff_motion_delete = false;
            for (auto blob : _all_blobs) {
                if (normf(cv::Point2f(blob.x,blob.y)*pparams.imscalef - drone_takeoff_im_location()) < 2.6f * _drone_takeoff_im_size) {
                    enable_takeoff_motion_delete = true;
                }
            }
        }
        if (!enable_takeoff_motion_delete)
            std::cout << "takeoff_motion_delete done" << std::endl;
    }
}
void DroneTracker::delete_landing_motion(float duration) {
    int delete_dst;
    delete_dst = _drone_takeoff_im_size;
    delete_dst = std::clamp(delete_dst,30,60) + 5;
    _visdat->reset_spot_on_motion_map(drone_takeoff_im_location(), _blink_im_disparity,delete_dst,duration*pparams.fps);
}

void DroneTracker::calc_takeoff_prediction() {

    cv::Point3f acc = _target-drone_takeoff_location();
    acc = acc/(normf(acc));
    acc = acc * dparams.thrust;
    float dt = std::clamp(static_cast<float>(current_time - (spinup_detect_time + 0.3)),0.f,1.f);
    if (spinup_detected<3)
        dt = 0;
    cv::Point3f expected_drone_location = drone_takeoff_location() + 0.5* acc *powf(dt,2);


    _image_predict_item = ImagePredictItem(world2im_3d(expected_drone_location,_visdat->Qfi,_visdat->camera_angle),1,_drone_takeoff_im_size,255,_visdat->frame_id);
    predicted_image_path.push_back(_image_predict_item);
}

bool DroneTracker::detect_lift_off() {
    float dist2takeoff =normf(_world_item.pt - drone_takeoff_location());
    float takeoff_y =  _world_item.pt.y - drone_takeoff_location().y;

    if (dist2takeoff > 0.1f && takeoff_y > 0.05f && _world_item.size_in_image() > _blink_im_size/2.f && _world_item.radius < dparams.radius*4.f) {
        take_off_frame_cnt++;
        if (take_off_frame_cnt >= 3) {
            return true;
        }
    } else {
        take_off_frame_cnt = 0;
    }
    return false;
}

void DroneTracker::calc_world_item(BlobProps * props, double time [[maybe_unused]]) {

    bool use_max = false; //_visdat->camera_exposure < 5000 && landing();

    calc_world_props_blob_generic(props,use_max);

    props->world_props.valid = props->world_props.bkg_check_ok && props->world_props.disparity_in_range && props->world_props.radius_in_range;

    if (use_max)
        props->world_props.z -= dparams.radius; // because we track the led instead of the whole blob
    else if (inactive())
        props->world_props.valid = false;
    else if (taking_off() && !_manual_flight_mode) {
        float dist2takeoff = normf(props->world_props.pt() - drone_takeoff_location());
        float takeoff_y = props->world_props.y - drone_takeoff_location().y;
        if ( dist2takeoff < 0.2f && !props->world_props.bkg_check_ok)
            props->world_props.valid =  props->world_props.disparity_in_range && props->world_props.radius_in_range;

        // std::cout << to_string_with_precision(time,2) + "; dist2takeoff: " <<  to_string_with_precision(dist2takeoff,2) << " "
        //           << ", takeoff_y: " << to_string_with_precision(takeoff_y,2)
        //           << ", world size: " << to_string_with_precision(props->world_props.radius,2) << std::endl;

        if (takeoff_y < 0.02f && props->world_props.valid ) {
            props->world_props.valid = false;
            props->world_props.takeoff_reject = true;
        }
    } else if(correct_yaw() && props->world_props.valid) {
        yaw = calc_yaw(props, false);
        props->world_props.yaw = yaw;
    }
}

bool DroneTracker::detect_takeoff() {
    uint16_t closest_to_takeoff_im_dst = 999;
    cv::Point2i closest_to_takeoff_point;
    for (auto blob : _all_blobs) {
        float takeoff_im_dst = normf(cv::Point2f(blob.x,blob.y)*pparams.imscalef - drone_takeoff_im_location());
        if (takeoff_im_dst < closest_to_takeoff_im_dst) {
            closest_to_takeoff_im_dst = takeoff_im_dst;
            closest_to_takeoff_point = cv::Point2i(blob.x,blob.y);
        }
    }

    return _world_item.size_in_image() > _blink_im_size/2.f  && _world_item.radius < dparams.radius*4.f &&
           closest_to_takeoff_point.x == static_cast<int>(_world_item.iti.x) &&
           closest_to_takeoff_point.y == static_cast<int>(_world_item.iti.y); // maybe should create an id instead of checking the distance

}

bool DroneTracker::check_ignore_blobs(BlobProps * pbs) {
    return this->check_ignore_blobs_generic(pbs);
}

//Removes all ignore points which timed out
void DroneTracker::clean_ignore_blobs(double time) {
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

cv::Mat DroneTracker::get_big_blob(cv::Mat Mask, int connectivity) {
    cv::Mat labels, stats, centroids;
    int nLabels = cv::connectedComponentsWithStats(Mask, labels, stats, centroids, connectivity, CV_32S);

    cv::Mat mask_big;
    int big_blob;
    int area = 0;

    for(int i = 1; i<=nLabels; i++) {
        if(stats.at<int>(i,cv::CC_STAT_AREA) > area) {
            big_blob = i;
            area = stats.at<int>(i,cv::CC_STAT_AREA);
        }
    }
    compare(labels, big_blob, mask_big, cv::CMP_EQ);
    return mask_big;
}

cv::Mat DroneTracker::extract_mask_column(cv::Mat mask_big, float range_left, float range_right, float side_percentage, enum side side_) {
    cv::Mat half_ = mask_big(cv::Range::all(), cv::Range(range_left,range_right)).clone();
    for(int j=0; j<half_.rows; j++) {
        half_.at<uchar>(j,(half_.cols-1)*(1-side_)) = round(side_percentage*half_.at<uchar>(j,(half_.cols-1)*(1-side_)));
    }
    return half_;
}

cv::Mat DroneTracker::split_mask_half(cv::Mat mask_big, enum side side_) {
    cv::Moments mo = moments(mask_big,true);
    cv::Point2f COG = cv::Point2f(static_cast<float>(mo.m10) / static_cast<float>(mo.m00), static_cast<float>(mo.m01) / static_cast<float>(mo.m00));
    float delta = COG.x - mask_big.rows/2;
    float delta_frac, n;
    delta_frac = modf(delta, &n);
    cv::Mat half;

    if(side_==leftside && delta>=0 && delta<mask_big.rows/2) {
        half = extract_mask_column(mask_big, 2*floor(delta), mask_big.rows/2+ceil(delta), delta_frac, side_);
    }
    else if(side_==leftside && delta<0 && delta>-mask_big.rows/2) {
        half = extract_mask_column(mask_big, 0, mask_big.rows/2+ceil(delta), 1-abs(delta_frac), side_);
    }
    else if(side_==rightside && delta>=0 && delta<mask_big.rows/2) {
        half = extract_mask_column(mask_big, mask_big.rows/2+floor(delta), mask_big.rows, 1-delta_frac, side_);
    }
    else if(side_==rightside && delta<0 && delta>-mask_big.rows/2) {
        half = extract_mask_column(mask_big, mask_big.rows/2+floor(delta), mask_big.rows+2*floor(delta), abs(delta_frac), side_);
    }
    return half;
}

float DroneTracker::yaw_from_splitted_mask(cv::Mat left, cv::Mat right) { // yaw positive = Clockwise, yaw negative is Counter-Clockwise
    cv::Moments mo_l = moments(left,true);
    cv::Point2f COG_l = cv::Point2f(static_cast<float>(mo_l.m10) / static_cast<float>(mo_l.m00), static_cast<float>(mo_l.m01) / static_cast<float>(mo_l.m00));
    cv::Moments mo_r = moments(right,true);
    cv::Point2f COG_r = cv::Point2f(static_cast<float>(mo_r.m10) / static_cast<float>(mo_r.m00), static_cast<float>(mo_r.m01) / static_cast<float>(mo_r.m00));
    yaw = COG_l.y-COG_r.y;
    return yaw;
}

float DroneTracker::calc_yaw(BlobProps * pbs, bool inspect_blob) { // Set inspect_blob = true to see mask. Otherwise set to false.
    if(inspect_blob==true) {
        cout<<"Original Drone Mask: "<<endl;
        cout<<pbs->mask<<endl;
    }
    if(!pbs->mask.empty()) {
        int kernel_int = 2;
        cv::Mat mask_erode;
        erode(pbs->mask, mask_erode,getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_int,kernel_int)));
        if(inspect_blob==true) {
            cout<<"eroded mask: "<<endl;
            cout<<mask_erode<<endl;
        }

        cv::Mat mask_big = get_big_blob(mask_erode, 4);
        int nrnonzero = countNonZero(mask_big);
        if(nrnonzero > 1) {
            if(inspect_blob==true) {
                cout<<"New Drone Mask: "<<endl;
                cout<<mask_big<<endl;
            }
        }

        cv::Mat splitted_mask_left, splitted_mask_right;
        if(nrnonzero > 1) {
            splitted_mask_left = split_mask_half(mask_big, leftside);
            splitted_mask_right = split_mask_half(mask_big, rightside);
        }
        else if(nrnonzero < 1) {
            splitted_mask_left = split_mask_half(mask_erode, leftside);
            splitted_mask_right = split_mask_half(mask_erode, rightside);
        }
        if(inspect_blob==true) {
            cout<<"Left: "<<splitted_mask_left<<endl;
            cout<<"Right: "<<splitted_mask_right<<endl;
        }
        yaw = yaw_from_splitted_mask(splitted_mask_left, splitted_mask_right);
        if(inspect_blob==true) {
            cout<<"Yaw: "<<yaw<<endl;
        }
    }
    return yaw;
}

}
