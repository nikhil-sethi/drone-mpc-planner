#include "dronetracker.h"
#include "multimodule.h"
#include <numeric>

namespace tracking {

bool DroneTracker::init(std::ofstream *logger, VisionData *visdat, int motion_thresh, int16_t viz_id) {
    enable_viz_diff = false;
    ItemTracker::init(logger,visdat,motion_thresh,"drone",viz_id);
    max_size = dparams.radius*3;
    expected_radius = dparams.radius;
    landing_parameter.deserialize("../../xml/landing_location.xml");
    (*_logger) << "dtrkr_state;yaw_deviation;";
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
        min_disparity = std::clamp(static_cast<int>(roundf(_blink_im_disparity))-5,params.min_disparity.value(),params.max_disparity.value());
        max_disparity = std::clamp(static_cast<int>(roundf(_blink_im_disparity))+5,params.min_disparity.value(),params.max_disparity.value());
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
        float takeoff_duration = static_cast<float>(time - start_take_off_time);
        if (!drone_is_active)
            _drone_tracking_status = dts_inactive;
        else if (_world_item.valid) {
            spinup_detected++;
            if (spinup_detected==3) {
                spinup_detect_time = time;
                std::cout << "Spin up detected after: " << takeoff_duration << "s" << std::endl;
            }
            if (detect_lift_off()) {
                if (!liftoff_detected)
                    std::cout << "Lift off detected after: " << takeoff_duration << "s" << std::endl;
                liftoff_detected = true;
                if (detect_takeoff()) {
                    std::cout << "Take off detected after: " << takeoff_duration << "s" << std::endl;
                    _drone_tracking_status = dts_tracking;
                    break;
                }
            }
        } else if (spinup_detected < 3) {
            spinup_detected = 0;
        }

        if(spinup_detected<3 && takeoff_duration > dparams.full_bat_and_throttle_spinup_duration + 0.3f) { // hmm spinup detection really does not work with improper lighting conditions. Have set the time really high. (should be ~0.3-0.4s)
            _take_off_detection_failed = true;
            std::cout << "No spin up detected in time!" << std::endl;
        } else if (takeoff_duration > dparams.full_bat_and_throttle_spinup_duration + 0.35f) {
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
        else if (_n_frames_lost==0)
            _drone_tracking_status = dts_tracking;
        break;
    } case dts_tracking: {
        ItemTracker::update(time);
        update_prediction(time); // use control inputs to make prediction #282
        min_disparity = std::clamp(static_cast<int>(roundf(_image_predict_item.disparity))-5,params.min_disparity.value(),params.max_disparity.value());
        max_disparity = std::clamp(static_cast<int>(roundf(_image_predict_item.disparity))+5,params.min_disparity.value(),params.max_disparity.value());
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
        detect_deviation_angle();
        break;
    } case dts_landing_init: {
        ignores_for_other_trkrs.push_back(IgnoreBlob(drone_takeoff_im_location()/pparams.imscalef,drone_takeoff_im_size()/pparams.imscalef,time+landing_ignore_timeout, IgnoreBlob::landing_spot));
        _drone_tracking_status = dts_landing;
        [[fallthrough]];
    } case dts_landing: {
        ItemTracker::update(time);
        update_prediction(time);
        min_disparity = std::clamp(static_cast<int>(roundf(_image_predict_item.disparity))-5,params.min_disparity.value(),params.max_disparity.value());
        max_disparity = std::clamp(static_cast<int>(roundf(_image_predict_item.disparity))+5,params.min_disparity.value(),params.max_disparity.value());
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
    (*_logger) << static_cast<int16_t>(_drone_tracking_status) << ";" << last_track_data().yaw_deviation << ";";
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
    props->world_props.im_pos_ok = true;
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

void DroneTracker::detect_deviation_angle() {

    static vector<track_data> bowling_vector;

    uint index_difference = 15;

    track_history.back().yaw_deviation_valid = false;

    bowling_vector.push_back(last_track_data());

    if(bowling_vector.size()>= 2*index_difference) {

        cv::Point3f point1_3f = bowling_vector.at(0).spos();
        cv::Point3f point2_3f = bowling_vector.at(index_difference).spos();
        cv::Point3f point3_3f = bowling_vector.at(2*index_difference-1).spos();

        cv::Mat point1 = cv::Mat::zeros(3,1, CV_64F);
        cv::Mat point2 =cv::Mat::zeros(3,1, CV_64F);
        cv::Mat point3 = cv::Mat::zeros(3,1, CV_64F);

        point1.at<double>(0,0) = point1_3f.x;
        point1.at<double>(1,0) = point1_3f.y;
        point1.at<double>(2,0) = point1_3f.z;

        point2.at<double>(0,0) = point2_3f.x;
        point2.at<double>(1,0) = point2_3f.y;
        point2.at<double>(2,0) = point2_3f.z;

        point3.at<double>(0,0) = point3_3f.x;
        point3.at<double>(1,0) = point3_3f.y;
        point3.at<double>(2,0) = point3_3f.z;

        cv::Mat vec1 = point2 - point1;
        cv::Mat vec2 = point3 - point2;

        // std::cout << "point1_3f: " << point1_3f << ", point2_3f: " << point2_3f << ", point3_3f: " << point3_3f  << std::endl;
        // std::cout << "vec1: " << vec1.at<double>(0) << "," << vec1.at<double>(1) << "," << vec1.at<double>(2) << std::endl;
        // std::cout << "vec2: " << vec1.at<double>(0) << "," << vec1.at<double>(1) << "," << vec1.at<double>(2) << std::endl;
        vec1.at<double>(1) = 0;
        vec2.at<double>(1) = 0;

        deviation_vec1_length = norm(vec1);
        deviation_vec2_length = norm(vec2);

        // normalize vectors
        cv::Mat unit_vec1 = vec1.mul(1./norm(vec1));
        cv::Mat unit_vec2 = vec2.mul(1./norm(vec2));

        cv::Mat cross_vec1_vec2 = unit_vec1.cross(unit_vec2);

        // create skew symetric matrix
        cv::Mat skew_sym_cross_prod = cv::Mat::zeros(3,3, CV_64F);
        skew_sym_cross_prod.at<double>(0,1) = -cross_vec1_vec2.at<double>(2);
        skew_sym_cross_prod.at<double>(0,2) = cross_vec1_vec2.at<double>(1);
        skew_sym_cross_prod.at<double>(1,0) = cross_vec1_vec2.at<double>(2);
        skew_sym_cross_prod.at<double>(1,2) = -cross_vec1_vec2.at<double>(0);
        skew_sym_cross_prod.at<double>(2,0) = -cross_vec1_vec2.at<double>(1);
        skew_sym_cross_prod.at<double>(2,1) = cross_vec1_vec2.at<double>(0);

        double norm_cross = sqrt(pow(cross_vec1_vec2.at<double>(0),2) +
                                 pow(cross_vec1_vec2.at<double>(1),2) +
                                 pow(cross_vec1_vec2.at<double>(2),2));

        double scalar_comp = (1- unit_vec1.dot(unit_vec2))/(pow(norm_cross,2));

        cv::Mat skew_sym_squared = skew_sym_cross_prod * skew_sym_cross_prod;
        cv::Mat skew_sym_squared_times_scalar = skew_sym_squared.mul(scalar_comp);

        cv::Mat rot_mat = cv::Mat::eye(3,3,CV_64F) + skew_sym_cross_prod
                          +  skew_sym_squared_times_scalar;

        deviation_angle = std::acos(rot_mat.at<double>(0,0)) ;

        if (rot_mat.at<double>(2,0) > 0) {
            deviation_angle = -deviation_angle;
        }

        bowling_vector.erase(bowling_vector.begin());

        track_history.back().yaw_deviation = deviation_angle;
        track_history.back().yaw_deviation_valid = true;

    }

}

}
