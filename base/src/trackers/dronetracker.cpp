#include "dronetracker.h"
#include "multimodule.h"
#include <numeric>
#include "common.h"
#include <math.h>

namespace tracking {

bool DroneTracker::init(VisionData *visdat, int motion_thresh, int16_t viz_id) {
    enable_viz_motion = false;
    ItemTracker::init(visdat, motion_thresh, "drone", viz_id);
    max_radius = dparams.radius * 3;
    min_radius = dparams.radius / 6;
    expected_radius = dparams.radius;

    return false;
}
void DroneTracker::init_flight(std::ofstream *logger, double time) {
    _logger = logger;
    ItemTracker::init_logger(logger);
    (*_logger) << "yaw_deviation;dtrkr_state;";

    if (_pad_location_valid) {
        cv::Point3f p = world2im_3d(pad_location(), _visdat->Qfi, _visdat->camera_roll(), _visdat->camera_pitch());
        float size = _drone_on_pad_im_size;
        _image_predict_item = ImagePredictItem(p, size, 255, _visdat->frame_id);
        std::cout << "Pad location: " << pad_location() << std::endl;
    } else {
        _image_predict_item.valid = false;
        reset_tracker_ouput(time);
    }

    if (_pad_location_valid && ignores_for_other_trkrs.size() == 0) {
        //some times there is some motion noise around the drone when it is just sitting on the ground
        //not sure what that is (might be a flickering led?), but the following makes the insect tracker
        //ignore it. Can be better fixed by having a specialized landingspot detector.
        ignores_for_other_trkrs.push_back(IgnoreBlob(_pad_im_location / im_scaler,
                                          _pad_im_size / im_scaler,
                                          time + takeoff_location_ignore_timeout,
                                          IgnoreBlob::takeoff_spot));
    }

    _drone_tracking_status = dts_detecting_takeoff;
    start_take_off_time = time;
    takeoff_is_aborted_time = 0;
    start_burn_time = 0;
    _tracking = false;
    delete_motion_shadow(_pad_im_location, _pad_im_size, _pad_disparity);
    ignores_for_other_trkrs.clear();
    ignores_for_other_trkrs.push_back(IgnoreBlob(_pad_im_location / im_scaler, _pad_im_size / im_scaler, time + takeoff_location_ignore_timeout, IgnoreBlob::takeoff_spot));
    liftoff_detected = false;
    take_off_frame_cnt = 0;
    min_disparity = std::clamp(static_cast<int>(roundf(_pad_disparity)) - 5, params.min_disparity.value(), params.max_disparity.value());
    max_disparity = std::clamp(static_cast<int>(roundf(_pad_disparity)) + 5, params.min_disparity.value(), params.max_disparity.value());

    takeoff_prediction_pos = pad_location();
    takeoff_prediction_vel = cv::Point3f(0.f, 0.f, 0.f);
    _takeoff_direction_predicted = cv::Point2f(0.f, -1.f);

    _image_template_item = ImageItem(roundf(pad_im_location().x), roundf(pad_im_location().y), roundf(pad_disparity()), _visdat->frame_id);
    _image_template_item.size = make_even(_drone_on_pad_im_size);
    cv::Rect crop_template(_image_template_item.x - _image_template_item.size / 2, _image_template_item.y - _image_template_item.size / 2, _image_template_item.size, _image_template_item.size);
    crop_template = clamp_rect(crop_template, IMG_W, IMG_H);
    _template = _visdat->frameL(crop_template);
    template_deviation_detected = false;
    _template_tracking = false;
}

void DroneTracker::update(double time) {
    _time = time;
    if (enable_viz_motion)
        cv::cvtColor(_visdat->diffL * 10, diff_viz, cv::COLOR_GRAY2BGR);

    switch (_drone_tracking_status) {
        case dts_detecting_takeoff: {
                min_disparity = std::clamp(static_cast<int>(roundf(_pad_disparity)) - 5, params.min_disparity.value(), params.max_disparity.value());
                max_disparity = std::clamp(static_cast<int>(roundf(_pad_disparity)) + 5, params.min_disparity.value(), params.max_disparity.value());
                _visdat->disable_cloud_rejection = true;
                ItemTracker::update(time);
                if (_world_item.valid) {
                    update_prediction(time);
                } else if (!_tracking) {
                    TrackData data;
                    data.predicted_image_item = _image_predict_item;
                    data.time = time;
                    if (!_track.empty())
                        data.dt = static_cast<float>(time - _track.back().time);
                    _track.push_back(data);
                }

                if (enable_viz_motion) {
                    cv::Point2f tmpp = _pad_im_location;
                    cv::circle(diff_viz, tmpp, 1, cv::Scalar(255, 0, 0), 1);
                    cv::circle(diff_viz, _pad_im_location, 1, cv::Scalar(0, 0, 255), 1);
                    cv::circle(diff_viz, _world_item.image_coordinates(), 3, cv::Scalar(0, 255, 0), 2);
                }

                match_template();
                cv::Point3f acc = *_commanded_acceleration;
                acc.y = std::max(0.f, acc.y - GRAVITY);
                if (normf(acc) && start_burn_time == 0) {
                    start_burn_time = time;
                    std::cout << "Expecting drone movement in about 10 frames from now" << std::endl;
                }

                float takeoff_duration = static_cast<float>(time - start_take_off_time);
                if (post_burn_start(time)) {
                    calc_takeoff_prediction(time, acc);
                    if (_world_item.valid) {
                        if (detect_lift_off()) {
                            if (!liftoff_detected)
                                std::cout << "Lift off detected after: " << takeoff_duration << "s" << std::endl;
                            liftoff_detected = true;
                            drone_on_pad = false;
                            _visdat->disable_cloud_rejection = false;
                            _drone_tracking_status = dts_tracking;
                        }
                    }
                }
                if (takeoff_duration > max_allowed_takeoff_duration || takeoff_is_aborted(time)) {
                    _visdat->disable_cloud_rejection = false;
                    _drone_tracking_status = dts_detecting_takeoff_failure;
                    if (liftoff_detected)
                        std::cout << "No takeoff detected in time!" << std::endl;
                    else
                        std::cout << "No liftoff_detected detected in time!" << std::endl;
                }
                break;
        } case dts_detecting_takeoff_failure: {
                ItemTracker::update(time);
                match_template();
                break;
        } case dts_detecting: {
                ItemTracker::update(time);
                TrackData data;
                data.predicted_image_item = _image_predict_item;
                data.time = time;
                _track.push_back(data);
                update_drone_prediction(time);
                match_template();
                handle_brightness_change(time);
                if (_n_frames_lost > 3  && _drone_tracking_status_before_tracking_loss == dts_landing)
                    _drone_tracking_status = dts_landed;
                else if (_n_frames_lost == 0)
                    _drone_tracking_status = _drone_tracking_status_before_tracking_loss;
                break;
        } case dts_tracking: {
                ItemTracker::update(time);
                update_drone_prediction(time);
                match_template();
                handle_brightness_change(time);
                min_disparity = std::clamp(static_cast<int>(roundf(_image_predict_item.disparity)) - 5, params.min_disparity.value(), params.max_disparity.value());
                max_disparity = std::clamp(static_cast<int>(roundf(_image_predict_item.disparity)) + 5, params.min_disparity.value(), params.max_disparity.value());
                _visdat->exclude_drone_from_motion_fading(_image_item.ptd(), _image_predict_item.size);
                if (!_tracking) {
                    _drone_tracking_status = dts_detecting;
                    _drone_tracking_status_before_tracking_loss = dts_tracking;
                }
                break;
        } case dts_detect_yaw: {
                ItemTracker::update(time);
                update_drone_prediction(time);
                match_template();
                handle_brightness_change(time);
                min_disparity = std::clamp(static_cast<int>(roundf(_image_predict_item.disparity)) - 5, params.min_disparity.value(), params.max_disparity.value());
                max_disparity = std::clamp(static_cast<int>(roundf(_image_predict_item.disparity)) + 5, params.min_disparity.value(), params.max_disparity.value());
                _visdat->exclude_drone_from_motion_fading(_image_item.ptd(), _image_predict_item.size);
                detect_deviation_yaw_angle();
                if (!_tracking) {
                    _drone_tracking_status = dts_detecting;
                    _drone_tracking_status_before_tracking_loss = dts_detect_yaw;
                }
                break;
        } case dts_landing_init: {
                ignores_for_other_trkrs.push_back(IgnoreBlob(_pad_im_location / im_scaler, _pad_im_size / im_scaler, time + landing_ignore_timeout, IgnoreBlob::landing_spot));
                _drone_tracking_status = dts_landing;
                [[fallthrough]];
        } case dts_landing: {
                ItemTracker::update(time);
                update_prediction(time);
                match_template();
                handle_brightness_change(time);
                min_disparity = std::clamp(static_cast<int>(roundf(_image_predict_item.disparity)) - 5, params.min_disparity.value(), params.max_disparity.value());
                max_disparity = std::clamp(static_cast<int>(roundf(_image_predict_item.disparity)) + 5, params.min_disparity.value(), params.max_disparity.value());
                _visdat->exclude_drone_from_motion_fading(_image_item.ptd(), _image_predict_item.size);
                if (!_tracking) {
                    _drone_tracking_status = dts_detecting;
                    _drone_tracking_status_before_tracking_loss = dts_landing;
                }
                break;
        }  case dts_landed: {
                ItemTracker::update(time);
                TrackData data;
                data.predicted_image_item = _image_predict_item;
                data.time = time;
                _track.push_back(data);
                update_drone_prediction(time);
                if (_n_frames_lost == 0)
                    _drone_tracking_status = dts_landing;
                break;
            }

    }

    delete_motion_shadow_run();
    clean_ignore_blobs(time);
    (*_logger) << last_track_data().yaw_deviation << ";" << drone_tracking_state() << ";";
}

float DroneTracker::score(BlobProps *blob) {
    if (_drone_tracking_status == dts_detecting && _image_predict_item.out_of_image) {
        float im_dist_err_ratio, im_size_pred_err_ratio;
        cv::Point3f predicted_world_pos = im2world(_image_predict_item.pt, _image_predict_item.disparity, _visdat->Qf, _visdat->camera_roll(), _visdat->camera_pitch());
        float max_im_dist = world2im_dist(predicted_world_pos, max_world_dist, _visdat->Qfi, _visdat->camera_roll(), _visdat->camera_pitch());
        float world_projected_im_err = normf(blob->pt_unscaled() - _image_predict_item.pt);
        im_dist_err_ratio = world_projected_im_err / max_im_dist;
        im_size_pred_err_ratio = fabs(_image_predict_item.size - blob->size_unscaled()) / (blob->size_unscaled() + _image_predict_item.size);

        float score = im_dist_err_ratio * 0.25f + 0.75f * im_size_pred_err_ratio;
        if (!properly_tracking() && (blob->in_overexposed_area || blob->false_positive))
            score *= 1.5f;
        else if (blob->in_overexposed_area || blob->false_positive)
            score *= 1.25f;

        return score;
    } else {
        float angle_diff = 1;
        float score = ItemTracker::score(blob, &_image_item);
        if (_image_item.valid && _world_item.valid && _drone_tracking_status == dts_detecting_takeoff) {
            cv::Point2f pad_to_blob = blob->pt_unscaled() - _pad_im_location;
            cv::Point2f pad_to_drone = _image_item.pt() - _pad_im_location;
            angle_diff = acosf((pad_to_blob.dot(pad_to_drone)) / (normf(pad_to_blob) * normf(pad_to_drone)));
            angle_diff = angle_diff / M_PIf32 + 0.5f;
        } else if (_image_predict_item.valid && _drone_tracking_status == dts_detecting_takeoff) {
            cv::Point2f pad_to_blob = blob->pt_unscaled() - _pad_im_location;
            cv::Point2f pad_to_drone = _image_predict_item.pt - _pad_im_location;
            angle_diff = acosf((pad_to_blob.dot(pad_to_drone)) / (normf(pad_to_blob) * normf(pad_to_drone)));
            angle_diff = angle_diff / M_PIf32 + 0.5f;
        }
        return score * angle_diff;
    }
}
float DroneTracker::score_threshold() {
    return std::clamp(_score_threshold + _n_frames_lost * 0.3f * _score_threshold, 0.f, 1.5f * _score_threshold);
}
void DroneTracker::update_drone_prediction(double time) { // need to use control inputs to make prediction #282
    if (enable_motion_shadow_delete) {
        _image_predict_item = ImagePredictItem(_image_template_item.ptd(), _image_template_item.size, 255, _visdat->frame_id);
        return;
    }
    if (_tracking)
        update_prediction(time);
    else if (lost()) {
        _image_predict_item.valid = false;
    } else { // just keep looking for the drone on the last known location:
        _image_predict_item.frame_id = _visdat->frame_id;
    }
}
void DroneTracker::calc_world_item(BlobProps *props, double time [[maybe_unused]]) {

    calc_world_props_blob_generic(props);
    props->world_props.im_pos_ok = true;
    props->world_props.radius_in_range = min_radius <= props->world_props.radius && props->world_props.radius < max_radius;
    props->world_props.valid = props->world_props.bkg_check_ok && props->world_props.disparity_in_range && props->world_props.radius_in_range;

    if (taking_off() && !_manual_flight_mode) {
        if (std::isnan(props->world_props.disparity) && _image_template_item.disparity > params.min_disparity.value() && _image_template_item.disparity < params.max_disparity.value()) {
            props->world_props.disparity = _image_template_item.disparity;
            props->x = _image_template_item.x / im_scaler;
            props->y = _image_template_item.y / im_scaler;
            props->size = _image_template_item.size / im_scaler;
            props->world_props.disparity_in_range = true;
            cv::Point2f p = props->pt_unscaled();
            cv::Point3f world_coordinates = im2world(p, _image_template_item.disparity, _visdat->Qf, _visdat->camera_roll(), _visdat->camera_pitch());
            props->world_props.x = world_coordinates.x;
            props->world_props.y = world_coordinates.y;
            props->world_props.z = world_coordinates.z;
            props->world_props.radius = normf(im2world((p + cv::Point2f(props->size_unscaled(), 0.f)), _image_template_item.disparity, _visdat->Qf, _visdat->camera_roll(), _visdat->camera_pitch())) / 2.f;
            props->world_props.radius_in_range = (min_radius <= props->world_props.radius) && (props->world_props.radius < max_radius);
            props->world_props.distance_bkg = _visdat->depth_background_mm.at<float>(p.y, p.x);
            props->world_props.distance = sqrtf(powf(world_coordinates.x, 2) + powf(world_coordinates.y, 2) + powf(world_coordinates.z, 2));
            if ((props->world_props.distance > props->world_props.distance_bkg * (static_cast<float>(background_subtract_zone_factor) / 100.f)))
                props->world_props.bkg_check_ok = false;
            else
                props->world_props.bkg_check_ok = true;
            props->world_props.valid = props->world_props.bkg_check_ok && props->world_props.radius_in_range;

        }
        float dist2takeoff = normf(props->world_props.pt() - _pad_world_location);
        float takeoff_y = props->world_props.y - _pad_world_location.y;
        if (dist2takeoff < dparams.pad_radius * 2.f && !props->world_props.bkg_check_ok)
            props->world_props.valid = props->world_props.disparity_in_range && props->world_props.radius_in_range;

        // std::cout << to_string_with_precision(time,2) + "; dist2takeoff: " <<  to_string_with_precision(dist2takeoff,2) << " "
        //           << ", takeoff_y: " << to_string_with_precision(takeoff_y,2)
        //           << ", world size: " << to_string_with_precision(props->world_props.radius,2) << std::endl;

        if (takeoff_y < 0.02f && props->world_props.valid) {
            props->world_props.valid = false;
            props->world_props.takeoff_reject = true;
        }
    }
}

void DroneTracker::handle_brightness_change(double time) {
    if (_visdat->brightness_change_event(time)) {
        _template_tracking = true;
        if (_image_item.valid)
            delete_motion_shadow(_image_item.pt(), _image_item.size, _image_item.disparity);
        else if (_image_predict_item.valid)
            delete_motion_shadow(_image_predict_item.pt, _image_predict_item.size, _image_predict_item.disparity);
    }
}

void DroneTracker::delete_motion_shadow(cv::Point2f im_location, float im_size, float disparity) {
    motion_shadow_im_size = im_size;
    motion_shadow_im_location = im_location;
    motion_shadow_disparity = disparity;
    enable_motion_shadow_delete = true;
    n_frames_lost_threshold = pparams.fps;
}

void DroneTracker::delete_motion_shadow_run() {
    if (enable_motion_shadow_delete) {
        _visdat->reset_spot_on_motion_map(motion_shadow_im_location, motion_shadow_disparity, motion_shadow_im_size, 1); // radius = 2 x the pad radius = _pad_size, for some margin

        //to end the deletion of this area, we check if there are no blobs in this area anymore because
        //they leave a permanent mark if we stop prematurely. Two conditions:
        //1. the drone must have left the area with a margin of its size
        //2. other blobs must not be inside the area. (slightly more relaxed, because crop leaf movements otherwise are holding this enabled indefinetely)
        if (_world_item.valid && normf(_world_item.image_item.pt() - motion_shadow_im_location) > motion_shadow_im_size + 0.6f * _world_item.image_item.size) {
            enable_motion_shadow_delete = false;
            for (auto blob : _all_blobs) {
                if (normf(blob.pt_unscaled() - motion_shadow_im_location) < (motion_shadow_im_size + blob.size_unscaled()) / 2.f) {
                    enable_motion_shadow_delete = true;
                    break;
                }
            }
        }
        if (!enable_motion_shadow_delete) {
            n_frames_lost_threshold = pparams.fps / 5;
            _template_tracking = false;
        }
    }
}
void DroneTracker::delete_landing_motion(float duration) {
    int delete_dst;
    delete_dst = _pad_im_size;
    delete_dst = std::clamp(delete_dst, 30, 60) + 5;
    _visdat->reset_spot_on_motion_map(_pad_im_location, _pad_disparity, delete_dst, duration * pparams.fps);
}

void DroneTracker::calc_takeoff_prediction(double time, cv::Point3f acc) {




    tracking::TrackData last_drone_detection;
    if (_track.size())
        last_drone_detection = _track.back();

    if (last_drone_detection.pos_valid) {
        last_valid_trackdata_for_prediction = last_drone_detection;
        if (last_drone_detection.vel_valid)
            last_vel_valid_trackdata_for_prediction = last_drone_detection;
    }

    float dt;
    if (last_drone_detection.time)
        dt = static_cast<float>(time - last_drone_detection.time) + 1.f / pparams.fps;
    else
        dt = 1.f / pparams.fps;
    cv::Point3f dv = acc * dt;
    takeoff_prediction_vel += dv;
    takeoff_prediction_pos += takeoff_prediction_vel * dt;
    cv::Point2f expected_drone_location_image = world2im_2d(takeoff_prediction_pos, _visdat->Qfi, _visdat->camera_roll(), _visdat->camera_pitch());
    _takeoff_direction_predicted = (expected_drone_location_image - pad_im_location()) / normf(expected_drone_location_image - pad_im_location());
    cv::Point2f takeoff_direction_measured;
    float measured_versus_predicted_angle_diff = -1;
    if (_image_item.valid)
        takeoff_direction_measured = _image_item.pt() - pad_im_location();
    else if (last_valid_trackdata_for_prediction.pos_valid)
        takeoff_direction_measured = last_valid_trackdata_for_prediction.world_item.image_coordinates() - pad_im_location();
    else {
        takeoff_direction_measured = {0.f};
    }
    if (takeoff_direction_measured != cv::Point2f(0.f, 0.f))
        measured_versus_predicted_angle_diff = acosf((takeoff_direction_measured.dot(_takeoff_direction_predicted)) / (normf(takeoff_direction_measured) * normf(_takeoff_direction_predicted)));
    reset_takeoff_im_prediction_if_direction_bad(takeoff_direction_measured, measured_versus_predicted_angle_diff);
}

void DroneTracker::reset_takeoff_im_prediction_if_direction_bad(cv::Point2f takeoff_direction_measured, float measured_versus_predicted_angle_diff) {
    if (measured_versus_predicted_angle_diff > M_PIf32 / 4.f) {
        cv::Point2f reset_prediction_pos_im;
        reset_prediction_pos_im = pad_im_location() + _takeoff_direction_predicted * normf(takeoff_direction_measured);
        _image_predict_item.pt = reset_prediction_pos_im;
        cv::Point3f reset_prediction_pos = im2world(reset_prediction_pos_im, _pad_disparity, _visdat->Qf, _visdat->camera_roll(), _visdat->camera_pitch());
        float size = world2im_size(reset_prediction_pos + cv::Point3f(expected_radius, 0, 0), reset_prediction_pos - cv::Point3f(expected_radius, 0, 0), _visdat->Qfi, _visdat->camera_roll(), _visdat->camera_pitch());
        _image_predict_item.size = size;
    }
}

bool DroneTracker::detect_lift_off() {
    float dist2takeoff = normf(_world_item.pt - pad_location());
    float takeoff_y =  _world_item.pt.y - pad_location().y;
    if ((dist2takeoff > 0.1f
            && takeoff_y > 0.05f
            && _world_item.radius < dparams.radius * 2.f
            && _world_item.radius > dparams.radius / 2.5f)
            || (dist2takeoff > dparams.pad_radius
                && _world_item.radius < dparams.radius * 2.f
                && _world_item.radius > dparams.radius / 2.5f)) {
        take_off_frame_cnt++;
        if (take_off_frame_cnt >= 3) {
            return true;
        }
    } else {
        take_off_frame_cnt = std::max(0, take_off_frame_cnt - 1);
    }
    return false;
}

bool DroneTracker::takeoff_is_aborted(double time) {
    bool inside_takeoff_area = _takeoff_area.inside(takeoff_prediction_pos, bare);
    if (!inside_takeoff_area && !takeoff_is_aborted_time) {
        takeoff_is_aborted_time = time;
        return false;
    } else if (!inside_takeoff_area && time > takeoff_is_aborted_time + 1.0)
        return true;
    else
        return false;
}

bool DroneTracker::check_ignore_blobs(BlobProps *pbs) {
    if (_drone_tracking_status == dts_detecting_takeoff)
        return false;
    return this->check_ignore_blobs_generic(pbs);
}
//Removes all ignore points which timed out
void DroneTracker::clean_ignore_blobs(double time) {
    std::vector<IgnoreBlob> new_ignores_for_insect_tracker;
    for (size_t i = 0; i < ignores_for_other_trkrs.size(); i++) {
        if (ignores_for_other_trkrs.at(i).was_used && ignores_for_other_trkrs.at(i).invalid_after >= 0)
            ignores_for_other_trkrs.at(i).invalid_after += 1. / pparams.fps;
        ignores_for_other_trkrs.at(i).was_used = false;
        if (ignores_for_other_trkrs.at(i).invalid_after > time || ignores_for_other_trkrs.at(i).invalid_after < 0)
            new_ignores_for_insect_tracker.push_back(ignores_for_other_trkrs.at(i));
    }
    ignores_for_other_trkrs = new_ignores_for_insect_tracker;
}

void DroneTracker::hover_mode(bool value)  {
    if (value != _hover_mode) {
        _hover_mode = value;
        if (_hover_mode) {
            pos_smth_width = pparams.fps / 15;
            vel_smth_width = pparams.fps / 15;
            acc_smth_width = pparams.fps / 15;
            disparity_filter_rate = 0.7;
        } else {
            pos_smth_width = pparams.fps / 30;
            vel_smth_width = pparams.fps / 30;
            acc_smth_width = pparams.fps / 30;
            disparity_filter_rate = 0.8;
        }
        smoother_posX.change_width(pos_smth_width);
        smoother_posY.change_width(pos_smth_width);
        smoother_posZ.change_width(pos_smth_width);
        smoother_accX.change_width(acc_smth_width);
        smoother_accY.change_width(acc_smth_width);
        smoother_accZ.change_width(acc_smth_width);
    }
}

void DroneTracker::detect_deviation_yaw_angle() {

    static vector<TrackData> bowling_vector;

    uint index_difference = 15;

    _track.back().yaw_deviation_valid = false;

    bowling_vector.push_back(last_track_data());

    if (bowling_vector.size() >= 2 * index_difference) {

        cv::Point3f point1_3f = bowling_vector.at(0).spos();
        cv::Point3f point2_3f = bowling_vector.at(index_difference).spos();
        cv::Point3f point3_3f = bowling_vector.at(2 * index_difference - 1).spos();

        cv::Mat point1 = cv::Mat::zeros(3, 1, CV_64F);
        cv::Mat point2 = cv::Mat::zeros(3, 1, CV_64F);
        cv::Mat point3 = cv::Mat::zeros(3, 1, CV_64F);

        point1.at<double>(0, 0) = point1_3f.x;
        point1.at<double>(1, 0) = point1_3f.y;
        point1.at<double>(2, 0) = point1_3f.z;

        point2.at<double>(0, 0) = point2_3f.x;
        point2.at<double>(1, 0) = point2_3f.y;
        point2.at<double>(2, 0) = point2_3f.z;

        point3.at<double>(0, 0) = point3_3f.x;
        point3.at<double>(1, 0) = point3_3f.y;
        point3.at<double>(2, 0) = point3_3f.z;

        cv::Mat vec1 = point2 - point1;
        cv::Mat vec2 = point3 - point2;

        // std::cout << "point1_3f: " << point1_3f << ", point2_3f: " << point2_3f << ", point3_3f: " << point3_3f  << std::endl;
        // std::cout << "vec1: " << vec1.at<double>(0) << "," << vec1.at<double>(1) << "," << vec1.at<double>(2) << std::endl;
        // std::cout << "vec2: " << vec1.at<double>(0) << "," << vec1.at<double>(1) << "," << vec1.at<double>(2) << std::endl;
        vec1.at<double>(1) = 0;
        vec2.at<double>(1) = 0;

        float deviation_vec1_length = norm(vec1);
        float deviation_vec2_length = norm(vec2);
        yaw_deviation_vec_length_OK = deviation_vec1_length < min_deviate_vec_length && deviation_vec2_length < min_deviate_vec_length;

        // normalize vectors
        cv::Mat unit_vec1 = vec1.mul(1. / norm(vec1));
        cv::Mat unit_vec2 = vec2.mul(1. / norm(vec2));

        cv::Mat cross_vec1_vec2 = unit_vec1.cross(unit_vec2);

        // create skew symetric matrix
        cv::Mat skew_sym_cross_prod = cv::Mat::zeros(3, 3, CV_64F);
        skew_sym_cross_prod.at<double>(0, 1) = -cross_vec1_vec2.at<double>(2);
        skew_sym_cross_prod.at<double>(0, 2) = cross_vec1_vec2.at<double>(1);
        skew_sym_cross_prod.at<double>(1, 0) = cross_vec1_vec2.at<double>(2);
        skew_sym_cross_prod.at<double>(1, 2) = -cross_vec1_vec2.at<double>(0);
        skew_sym_cross_prod.at<double>(2, 0) = -cross_vec1_vec2.at<double>(1);
        skew_sym_cross_prod.at<double>(2, 1) = cross_vec1_vec2.at<double>(0);

        double norm_cross = sqrt(pow(cross_vec1_vec2.at<double>(0), 2) +
                                 pow(cross_vec1_vec2.at<double>(1), 2) +
                                 pow(cross_vec1_vec2.at<double>(2), 2));

        double scalar_comp = (1 - unit_vec1.dot(unit_vec2)) / (pow(norm_cross, 2));

        cv::Mat skew_sym_squared = skew_sym_cross_prod * skew_sym_cross_prod;
        cv::Mat skew_sym_squared_times_scalar = skew_sym_squared.mul(scalar_comp);

        cv::Mat rot_mat = cv::Mat::eye(3, 3, CV_64F) + skew_sym_cross_prod
                          +  skew_sym_squared_times_scalar;

        deviation_angle = std::acos(rot_mat.at<double>(0, 0)) ;

        if (rot_mat.at<double>(2, 0) > 0) {
            deviation_angle = -deviation_angle;
        }

        bowling_vector.erase(bowling_vector.begin());

        _track.back().yaw_deviation = deviation_angle;
        _track.back().yaw_deviation_valid = true;

    }
}
bool DroneTracker::check_yaw(double time) {
    if (!yaw_deviation_vec_length_OK || !last_track_data().yaw_deviation_valid)
        time_yaw_not_ok = time;

    if (time - time_yaw_not_ok > min_yaw_ok_time && time_yaw_not_ok > 0.1)
        return false;
    else
        return true;
}

void DroneTracker::set_pad_location_from_blink(cv::Point3f blink_world) {
    _pad_world_location = blink_world + cv::Point3f(0, 0, -dparams.radius);
    _target = _pad_world_location;
    _pad_im_size = world2im_size(_pad_world_location + cv::Point3f(dparams.pad_radius, 0, 0), _pad_world_location - cv::Point3f(dparams.pad_radius, 0, 0), _visdat->Qfi, _visdat->camera_roll(), _visdat->camera_pitch());
    _drone_on_pad_im_size = world2im_size(_pad_world_location + cv::Point3f(dparams.radius, 0, 0), _pad_world_location - cv::Point3f(dparams.radius, 0, 0), _visdat->Qfi, _visdat->camera_roll(), _visdat->camera_pitch());
    auto pt_im3 = world2im_3d(pad_location(), _visdat->Qfi, _visdat->camera_roll(), _visdat->camera_pitch());
    _pad_im_location =  cv::Point2f(pt_im3.x, pt_im3.y);
    _pad_disparity =  pt_im3.z;
    _pad_location_valid = true;
}

void DroneTracker::set_pad_location(cv::Point3f pad_world) {
    _pad_world_location = pad_world;
    _pad_im_size = world2im_size(_pad_world_location + cv::Point3f(dparams.pad_radius, 0, 0), _pad_world_location - cv::Point3f(dparams.pad_radius, 0, 0), _visdat->Qfi, _visdat->camera_roll(), _visdat->camera_pitch());
    _drone_on_pad_im_size = world2im_size(_pad_world_location + cv::Point3f(dparams.radius, 0, 0), _pad_world_location - cv::Point3f(dparams.radius, 0, 0), _visdat->Qfi, _visdat->camera_roll(), _visdat->camera_pitch());
    auto pt_im3 = world2im_3d(_pad_world_location, _visdat->Qfi, _visdat->camera_roll(), _visdat->camera_pitch());
    _pad_im_location =  cv::Point2f(pt_im3.x, pt_im3.y);
    _pad_disparity =  pt_im3.z;
    _pad_location_valid = true;
}

cv::Point3f DroneTracker::pad_location(bool landing_hack) {
    cv::Point3f hack = {0};
    if (landing_hack)
        hack = cv::Point3f(0, 0, 0.02); //Landing in the back rather then landing to the front seems to work better

    return _pad_world_location + hack;
}

void DroneTracker::match_template() {
    if (_image_template_item.frame_id == _visdat->frame_id)
        return;
    cv::Point3f last_world_pos = im2world(_image_template_item.pt(), _image_template_item.disparity, _visdat->Qf, _visdat->camera_roll(), _visdat->camera_pitch());
    int max_im_dist = static_cast<int>(world2im_dist(last_world_pos, max_world_dist, _visdat->Qfi, _visdat->camera_roll(), _visdat->camera_pitch()) + _image_template_item.size / 2);
    int roi_size = std::clamp(max_im_dist, _template.cols * 2, _template.cols * 10);
    double min_val_L, min_val_R, max_val_L, max_val_R;
    cv::Point min_loc_L, min_loc_R, max_loc_L, max_loc_R;
    cv::Point3i last_im_pos = _image_template_item.ptd();
    cv::Rect crop_L = clamp_rect(cv::Rect(last_im_pos.x - roi_size / 2, last_im_pos.y - roi_size / 2, roi_size, roi_size), IMG_W, IMG_H);
    cv::Mat roi_L = _visdat->frameL(crop_L);
    cv::Mat result_template_match_L, result_template_match_R;
    cv::matchTemplate(edge_detector(roi_L), edge_detector(_template), result_template_match_L, 2);
    cv::minMaxLoc(result_template_match_L, &min_val_L, &max_val_L, &min_loc_L, &max_loc_L);

    cv::Rect crop_R = clamp_rect(cv::Rect(last_im_pos.x - last_im_pos.z - roi_size / 2, last_im_pos.y - roi_size / 2 + max_loc_L.y - 1, roi_size, _template.cols + 2), _template.cols, _template.rows, IMG_W, IMG_H);
    cv::Mat roi_R = _visdat->frameR(crop_R);
    cv::matchTemplate(edge_detector(roi_R), edge_detector(_template), result_template_match_R, 2);
    cv::minMaxLoc(result_template_match_R, &min_val_R, &max_val_R, &min_loc_R, &max_loc_R);

    float temp_size = _image_template_item.size;
    _image_template_item = ImageItem(last_im_pos.x - roi_size / 2 + max_loc_L.x + ceilf(_template.cols / 2.f), last_im_pos.y - roi_size / 2 + max_loc_L.y + ceilf(_template.rows / 2.f), last_im_pos.z, _visdat->frame_id);
    _image_template_item.size = temp_size;
    _image_template_item.disparity += (crop_R.width / 2 - max_loc_R.x - _template.cols / 2);
    if (!taking_off()) {
        recenter_template();
    } else {
        _image_template_item.size = make_even(_drone_on_pad_im_size);
        _image_predict_item = ImagePredictItem(_image_template_item.ptd(), _image_template_item.size, 255, _visdat->frame_id);
    }
    cv::Rect crop_template(_image_template_item.x - _image_template_item.size / 2, _image_template_item.y - _image_template_item.size / 2, _image_template_item.size, _image_template_item.size);
    crop_template = clamp_rect(crop_template, IMG_W, IMG_H);
    _template = _visdat->frameL(crop_template);
    return;
}

void DroneTracker::recenter_template() {
    if (_image_item.valid && !template_tracking()) {
        cv::Point2f image_item_pos = _image_item.pt();
        cv::Point2f image_template_item_pos = _image_template_item.pt();
        if (normf(image_item_pos - image_template_item_pos) <= _template.cols) {
            if (normf(image_item_pos - image_template_item_pos) > 1) {
                cv::Point2i move_direction = (image_item_pos - image_template_item_pos) / normf(image_item_pos - image_template_item_pos);
                _image_template_item.x += move_direction.x;
                _image_template_item.y += move_direction.y;
                bool template_size_larger_than_blob = _image_template_item.size > _image_item.size;
                _image_template_item.size += (-2 * template_size_larger_than_blob);//converge to blob size
                _image_template_item.size = std::max(_image_template_item.size, 2.f);// >0
            }
        } else if (template_deviation_detected) {
            _image_template_item = ImageItem(roundf(_image_item.x), roundf(_image_item.y), roundf(_image_item.disparity), _visdat->frame_id);
            _image_template_item.size = make_even(_image_item.size);
            _image_template_item.size = std::max(_image_template_item.size, 2.f);// >0
            template_deviation_detected = false;
        } else
            template_deviation_detected = true;
    } else if (template_tracking()) {
        const uint8_t minimal_drone_brightness = 200;
        if (_template.at<uint8_t>(_template.cols / 2, _template.cols / 2) > minimal_drone_brightness) {
            cv::Moments template_drone_moments = cv::moments(_template);
            cv::Point2i center_of_mass(template_drone_moments.m10 / template_drone_moments.m00, template_drone_moments.m01 / template_drone_moments.m00);
            if (normf(center_of_mass - cv::Point2i(_template.cols / 2, _template.cols / 2)) > 1) {
                cv::Point2i move_direction = (center_of_mass - cv::Point2i(_template.cols / 2, _template.cols / 2)) / normf(center_of_mass - cv::Point2i(_template.cols / 2, _template.cols / 2));
                _image_template_item.x += move_direction.x;
                _image_template_item.y += move_direction.y;
            }
        }
        cv::Point3f current_world_pos = im2world(_image_template_item.pt(), _image_template_item.disparity, _visdat->Qf, _visdat->camera_roll(), _visdat->camera_pitch());
        float expected_drone_im_size = world2im_size(current_world_pos + cv::Point3f(expected_radius, 0, 0), current_world_pos - cv::Point3f(expected_radius, 0, 0), _visdat->Qfi, _visdat->camera_roll(), _visdat->camera_pitch());
        _image_template_item.size = make_even(expected_drone_im_size);
        _image_template_item.size = std::max(_image_template_item.size, 2.f);//and >0
    }
}
}
