#include <iostream>
#include "itemtracker.h"
#include <opencv2/features2d.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/core/core.hpp>
#include "common.h"
#include "vector"
#include "algorithm"
using namespace cv;
using namespace std;

namespace tracking {

void ItemTracker::init(std::ofstream *logger, VisionData *visdat, int motion_thresh, std::string name, int16_t viz_id) {
    init(visdat, motion_thresh, name, viz_id);
    init_logger(logger);
}
void ItemTracker::init(VisionData *visdat, int motion_thresh, std::string name, int16_t viz_id) {
    static int16_t trkr_cntr = 0;
    _uid = trkr_cntr++;
    _viz_id = viz_id;
    _visdat = visdat;
    _motion_thresh = motion_thresh;
    _name = name;
    track_history_max_size = pparams.fps;
    n_frames_lost_threshold = pparams.fps / 9;
    settings_file = "../xml/" + name + "tracker.xml";
    std::string window_name = name + "_trkr";

    _world_item = WorldItem();

    deserialize_settings();

    if (pparams.insect_tracking_tuning || pparams.drone_tracking_tuning) {
        namedWindow(window_name, WINDOW_NORMAL);
        createTrackbar("Min disparity", window_name, &min_disparity, 255);
        createTrackbar("Max disparity", window_name, &max_disparity, 255);
        createTrackbar("background_subtract_zone_factor", window_name, &background_subtract_zone_factor, 100);
    }
    if (pos_smth_width < 0)
        pos_smth_width = pparams.fps / 20;
    if (vel_smth_width < 0)
        vel_smth_width = pparams.fps / 20;
    if (acc_smth_width < 0)
        acc_smth_width = pparams.fps / 20;
    smoother_posX.init(pos_smth_width);
    smoother_posY.init(pos_smth_width);
    smoother_posZ.init(pos_smth_width);

    smoother_accX.init(acc_smth_width);
    smoother_accY.init(acc_smth_width);
    smoother_accZ.init(acc_smth_width);

    smoother_im_size.init(smooth_blob_props_width);
    smoother_brightness.init(smooth_blob_props_width);
    vel_filt.init(1.f / pparams.fps, 1, 0.020, 0.020);

    disparity_prev = -1;

    initialized = true;
}
void ItemTracker::init_logger(std::ofstream *logger) {
    _logger = logger;
    if (_logger->is_open()) {
        (*_logger) << "imLx_" << _name << ";";
        (*_logger) << "imLy_" << _name << ";";
        (*_logger) << "disparity_" << _name << ";";
        (*_logger) << "size_" << _name << ";";
        (*_logger) << "score_" << _name << ";";
        (*_logger) << "motion_sum_" << _name << ";";
        (*_logger) << "imLx_pred_" << _name << ";";
        (*_logger) << "imLy_pred_" << _name << ";";
        (*_logger) << "n_frames_lost_" << _name << ";";
        (*_logger) << "n_frames_tracking_" << _name << ";";
        (*_logger) << "foundL_" << _name << ";";
        (*_logger) << "pos_valid_" << _name << ";";
        (*_logger) << "posX_" << _name << ";";
        (*_logger) << "posY_" << _name << ";";
        (*_logger) << "posZ_" << _name << ";";
        (*_logger) << "sposX_" << _name << ";";
        (*_logger) << "sposY_" << _name << ";";
        (*_logger) << "sposZ_" << _name << ";";
        (*_logger) << "vel_valid_" << _name << ";";
        (*_logger) << "svelX_" << _name << ";";
        (*_logger) << "svelY_" << _name << ";";
        (*_logger) << "svelZ_" << _name << ";";
        (*_logger) << "acc_valid_" << _name << ";";
        (*_logger) << "saccX_" << _name << ";";
        (*_logger) << "saccY_" << _name << ";";
        (*_logger) << "saccZ_" << _name << ";";
        (*_logger) << "radius_" << _name << ";";
    }
    initialized_logger = true;
}

void ItemTracker::calc_world_props_blob_generic(BlobProps *blob) {
    if (blob->world_props.trkr_id != _uid) {
        BlobWorldProps w = {0};
        cv::Point2f p = blob->pt_unscaled();
        float size = blob->size_unscaled();
        w.trkr_id = _uid;

        float disparity = stereo_match(blob);

        if (disparity < params.min_disparity.value() || disparity > params.max_disparity.value()) {
            w.disparity_in_range = false;
            w.disparity  = NAN;
        } else {
            w.disparity_in_range = true;

            if (disparity_prev > 0 && _n_frames_tracking > 5)
                w.disparity = (disparity_filter_rate * disparity) + ((1 - disparity_filter_rate) * disparity_prev); // moving average on disparity
            else
                w.disparity = disparity;

            cv::Point3f world_coordinates = im2world(p, w.disparity, _visdat->Qf, _visdat->camera_roll, _visdat->camera_pitch);
            w.x = world_coordinates.x;
            w.y = world_coordinates.y;
            w.z = world_coordinates.z;

            cv::Point2f p_size = p;
            p_size.x += size;
            cv::Point3f world_coordinates_size = im2world(p_size, w.disparity, _visdat->Qf, _visdat->camera_roll, _visdat->camera_pitch);

            w.radius = static_cast<float>(cv::norm(world_coordinates_size - world_coordinates)) / 2.f;
            w.radius_in_range = w.radius < max_radius;
            w.motion_sum = blob->motion_sum;

            w.distance_bkg = _visdat->depth_background_mm.at<float>(p.y, p.x);
            w.distance = sqrtf(powf(w.x, 2) + powf(w.y, 2) + powf(w.z, 2));
            if ((w.distance > w.distance_bkg * (static_cast<float>(background_subtract_zone_factor) / 100.f)))
                w.bkg_check_ok = false;
            else
                w.bkg_check_ok = true;
        }
        blob->world_props = w;
    }
}

void ItemTracker::update_blob_filters() {
    if (!_image_item.blob_is_fused) {
        smoother_im_size.addSample(_image_item.size);
        smoother_brightness.addSample(_image_item.pixel_max);
        _blobs_are_fused_cnt = 0;
    }
    disparity_prev = _world_item.image_item.disparity;
}

void ItemTracker::update(double time) {
    _n_frames++;
    if (_world_item.valid) {
        update_state(_world_item.pt, time, false);
        update_blob_filters();
        _tracking = true;
        _n_frames_lost = 0; // update this after calling update_state, so that it can determine how long tracking was lost
        _n_frames_tracking++;
        _n_frames_tracked++;
    } else {
        _n_frames_lost++;
        _n_frames_tracking = 0;
        if ((_n_frames_lost >= n_frames_lost_threshold && ! _image_item.blob_is_fused) || !_tracking) {
            _tracking = false;
        } else
            update_state(im2world(_image_predict_item.pt_unbound, _image_predict_item.disparity, _visdat->Qf, _visdat->camera_roll, _visdat->camera_pitch), time, true);
    }

    cleanup_history();

    append_log();
}

//make sure the vectors that contain the path data don't endlesly grow
void ItemTracker::cleanup_history() {
    while (_track.size() > track_history_max_size)
        _track.erase(_track.begin());
}

void ItemTracker::append_log() {
    if (_logger->is_open()) {
        //log all image stuff
        if (_tracking)
            (*_logger) << _image_item.x << ";" << _image_item.y << ";" << _image_item.disparity << ";"
                       << _image_item.size  << ";" << _image_item.score  << ";" << _image_item.motion_sum  << ";";
        else
            (*_logger) << -1 << ";" << -1 << ";" << -1 << ";" << -1 << ";" << -1 << ";" << -1 << ";";
        if (_image_predict_item.valid)
            (*_logger) << _image_predict_item.pt_unbound.x << ";" << _image_predict_item.pt_unbound.y << ";";
        else
            (*_logger) << -1 << ";" << -1   << ";";

        (*_logger) << _n_frames_lost << ";" << _n_frames_tracking << ";" << _tracking << ";";
        //log all world stuff
        TrackData last = last_track_data();
        (*_logger) << last.pos_valid << ";" << last.state.pos.x << ";" << last.state.pos.y << ";" << last.state.pos.z << ";" ;
        (*_logger) << last.state.spos.x << ";" << last.state.spos.y << ";" << last.state.spos.z << ";";
        (*_logger) << last.vel_valid << ";" << last.state.vel.x << ";" << last.state.vel.y << ";" << last.state.vel.z << ";" ;
        (*_logger) << last.acc_valid << ";" << last.state.acc.x << ";" << last.state.acc.y << ";" << last.state.acc.z << ";" ;
        if (_world_item.valid)
            (*_logger) << _world_item.radius << ";";
        else
            (*_logger) << -1 << ";";

    }

    while (_track.size() > track_history_max_size)
        _track.erase(_track.begin());
}

float ItemTracker::stereo_match(BlobProps *blob) {
    cv::Point2f im_posL = blob->pt_unscaled();
    float size = blob->size_unscaled();
    int radius = ceilf((size + 4.f) * 0.5f);

    if (im_posL.x - _image_predict_item.disparity <= 0 && _image_predict_item.valid)
        return -1; // if the center of the blob falls out of the right image, disparity becomes tricky, return out of range which must be handled outside this function

    // When the blob gets too close to the left edge, the blob may already be partially out of the frameR.
    // We can tolerate that by shifting the center of the blob used for the disparity matching inwards the image a bit.
    // In other words, we then only use the rightmost part of the blob for matching, and through away the part that has a
    // high chance of not being visible in frameR anyway. And as long as we do it in both images, for matching it doesn't
    // matter too much.
    // A better alternative may be to calculate the disparity using the frameR as a base instead of frameL in the case the
    // blob is in the left half of the image, but that requires a more work.
    BlobProps blob_shifted = *blob;
    if (im_posL.x - radius - max_disparity <= 0) {
        size /= 2.f;
        im_posL.x += size;
        radius = ceilf((size + 4.f) * 0.5f);
        blob_shifted.size /= 2;
        blob_shifted.x += blob_shifted.size;
    }

    //limit the patches for CPU optimization
    //We don't resize, but just select the middle rect of the full patch. This way we
    //still have full resolution (very important for disparity precision), but limit
    //the amount of pixels being matched, without needing to do cpu intensive resizing. Win win :)
    if (radius > 20)
        radius = 20;

    cv::Mat diffL, diffR, grayL, grayR, motion_filtered_noise_mapL, motion_filtered_noise_mapR;
    diffL = _visdat->diffL;
    diffR = _visdat->diffR;
    motion_filtered_noise_mapL = _visdat->motion_filtered_noise_mapL;
    motion_filtered_noise_mapR = _visdat->motion_filtered_noise_mapR;
    grayL = _visdat->frameL;
    grayR = _visdat->frameR;

    int x, y, width, height;
    x = std::clamp(static_cast<int>(roundf(im_posL.x)) - radius, 0, diffL.cols - 1);
    width = 2 * radius;
    y = std::clamp(static_cast<int>(roundf(im_posL.y)) - radius, 0, diffL.rows - 1);
    height = 2 * radius;

    if (x + width >= diffL.cols)
        width = diffL.cols - x;
    if (y + height >= diffL.rows)
        height = diffL.rows - y;
    cv::Rect roiL(x, y, width, height);

    auto [disp_start, disp_pred, disp_rng, disp_end, calculated_rough_disp] =  disparity_search_rng(&blob_shifted, x, radius);
    if (disp_rng < 3) {
        return -1; // return out of range which must be handled outside this function, this should in theory already been caught above...
    }

    float npixels = static_cast<float>(roiL.width * roiL.height);
    float err_masked [disp_end] = {0};

    float masked_pixel_ratio[disp_end] = {0};
    int disparity_masked = 0;
    const float min_pxl_ratio = 0.25f;
    bool not_enough_pixels = false;
    bool masked_method_was_tried = false;

    int disp_cnt = 0; //keep track of how many shifts are calculated until we found the minimum
    float min_err = INFINITY;
    cv::Mat grayL_masked_final, grayR_masked_final, mask_final;

    if (motion_filtered_noise_mapL.cols) {
        //since the background often isn't a solid color we do matching on the raw image data instead of the motion
        //using the motion from both images as a mask, we match the disparity over the masked gray image

        float used_motion_thresh = _motion_thresh;
        cv::Mat diffL_mask_patch = diffL(roiL) > motion_filtered_noise_mapL(roiL) + used_motion_thresh;
        if (cv::countNonZero(diffL_mask_patch) / npixels > min_pxl_ratio) {
            cv::Rect roiR_disparity_rng(x - (disp_end - 1), y, width + (disp_end - disp_start - 1), height);
            cv::Mat diffR_mask_patch = diffR(roiR_disparity_rng) > motion_filtered_noise_mapR(roiR_disparity_rng) + used_motion_thresh;
            cv::Mat grayL_patch = _visdat->frameL(roiL);
            cv::Mat grayR_patch = _visdat->frameR(roiR_disparity_rng);

            masked_method_was_tried = true;
            bool err_calculated [disp_end] = {false};
            //search for a minimum matching error
            int ii = roundf(disp_pred);
            if (ii + 2 > disp_end) // this can happen if the blob is on the edge of the screen and traveling outwards. Probably this blob cannot have a valid disparity as it left the right camera already
                ii = disp_end - 2;
            while (!err_calculated[ii - 1] || !err_calculated[ii] || !err_calculated[ii + 1]) {
                if (!err_calculated[ii - 1]) {
                    std::tie(masked_pixel_ratio[ii - 1], err_masked[ii - 1]) = calc_match_score_masked(ii - 1, disp_end, width, height, diffL_mask_patch, diffR_mask_patch, grayL_patch, grayR_patch, npixels);
                    err_calculated[ii - 1] = true;
                    disp_cnt++;
                }
                if (!err_calculated[ii]) {
                    std::tie(masked_pixel_ratio[ii], err_masked[ii]) = calc_match_score_masked(ii, disp_end, width, height, diffL_mask_patch, diffR_mask_patch, grayL_patch, grayR_patch, npixels);
                    err_calculated[ii] = true;
                    disp_cnt++;
                }
                if (!err_calculated[ii + 1]) {
                    std::tie(masked_pixel_ratio[ii + 1], err_masked[ii + 1]) = calc_match_score_masked(ii + 1, disp_end, width, height, diffL_mask_patch, diffR_mask_patch, grayL_patch, grayR_patch, npixels);
                    err_calculated[ii + 1] = true;
                    disp_cnt++;
                }

                if (err_masked[ii - 1] >= err_masked[ii] && err_masked[ii + 1] >= err_masked[ii] && min_err > err_masked[ii]) { // minimum found
                    disparity_masked  = ii;
                    min_err = err_masked[ii];
                }
                if (masked_pixel_ratio[ii - 1] < min_pxl_ratio && masked_pixel_ratio[ii] < min_pxl_ratio && masked_pixel_ratio[ii + 1] < min_pxl_ratio) {
                    not_enough_pixels = true;
                    break; // it is very unlikely that no pixels at all are available, Try again with the fallback strategy
                }
                if (err_masked[ii - 1] >= err_masked[ii] && err_masked[ii + 1] >= err_masked[ii] && min_err >= err_masked[ii]) { // global minimum found
                    break;
                } else if (err_masked[ii - 1] > err_masked[ii + 1]) { // no minumum here, determine search direction based on slope
                    int ii_cnt = 0;
                    while (err_calculated[ii + 1] && ii_cnt < disp_end - disp_start) {
                        ii_cnt++;
                        ii++;
                        if (ii > disp_end - 2)
                            ii = disp_start + 1;
                    }
                } else {
                    int ii_cnt = 0;
                    while (err_calculated[ii - 1] && ii_cnt < disp_end - disp_start) {
                        ii_cnt++;
                        ii--;
                        if (ii < disp_start + 1)
                            ii = disp_end - 2;
                    }
                }
            }

            if (enable_draw_stereo_viz && !not_enough_pixels && disparity_masked) {
                cv::Rect roiR_patch = cv::Rect(disp_end - 1 - disparity_masked, 0, width, height);
                cv::bitwise_and(diffL_mask_patch, diffR_mask_patch(roiR_patch), mask_final);
                cv::bitwise_and(grayL_patch, grayL_patch, grayL_masked_final, mask_final);
                cv::bitwise_and(grayR_patch(roiR_patch), grayR_patch(roiR_patch), grayR_masked_final, mask_final);
            }

        }
    }

    int disparity;
    float *err;
    float err_motion [disp_end] = {0};
    if (not_enough_pixels || !masked_method_was_tried) {
        //back up strategy: ignore the background, hope for the best, and just do matching directly on the motion
        int disparity_motion = 0;
        float diffL_sum = cv::sum(diffL(roiL))[0];
        bool err_calculated [disp_end] = {false};

        int ii = roundf(disp_pred);
        if (ii + 2 > disp_end) // this can happen if the blob is on the edge of the screen and traveling outwards. Probably this blob cannot have a valid disparity as it left the right camera already
            ii = disp_end - 2;
        while (!err_calculated[ii - 1] || !err_calculated[ii] || !err_calculated[ii + 1]) {

            if (!err_calculated[ii - 1]) {
                err_motion[ii - 1] = calc_match_score_motion(ii - 1, x, y, width, height, diffL_sum, diffL(roiL), diffR);
                err_calculated[ii - 1] = true;
                disp_cnt++;
            }
            if (!err_calculated[ii]) {
                err_motion[ii] = calc_match_score_motion(ii, x, y, width, height, diffL_sum, diffL(roiL), diffR);
                err_calculated[ii] = true;
                disp_cnt++;
            }
            if (!err_calculated[ii + 1]) {
                err_motion[ii + 1] = calc_match_score_motion(ii + 1, x, y, width, height, diffL_sum, diffL(roiL), diffR);
                err_calculated[ii + 1] = true;
                disp_cnt++;
            }

            if (err_motion[ii - 1] >= err_motion[ii] && err_motion[ii + 1] >= err_motion[ii] && min_err > err_motion[ii]) { // minimum found
                disparity_motion  = ii;
                min_err = err_motion[ii];
            }
            if (err_motion[ii - 1] >= err_motion[ii] && err_motion[ii + 1] >= err_motion[ii] && min_err >= err_motion[ii]) { // global minimum found
                break;
            } else if (err_motion[ii - 1] > err_motion[ii + 1]) { // no minumum here, determine search direction based on slope
                int ii_cnt = 0;
                while (err_calculated[ii + 1] && ii_cnt < disp_end - disp_start) {
                    ii_cnt++;
                    ii++;
                    if (ii > disp_end - 2)
                        ii = disp_start + 1;
                }
            } else {
                int ii_cnt = 0;
                while (err_calculated[ii - 1] && ii_cnt < disp_end - disp_start) {
                    ii_cnt++;
                    ii--;
                    if (ii < disp_start + 1)
                        ii = disp_end - 2;
                }
            }
        }

        disparity = disparity_motion;
        err = err_motion;
    } else {
        disparity = disparity_masked;
        err = err_masked;
    }

    if (disp_cnt > 20) // this shoudlnt happen anymore since #506, leaving the message for now to see if that's true
        std::cout << "Warning large disparity search range: " << disp_cnt << std::endl;

    if (disparity > 0) {
        float sub_disp = estimate_sub_disparity(disparity, err);

        if (enable_draw_stereo_viz) {
            int viz_scale = 4;
            cv::Rect roiR = cv::Rect(x - round(sub_disp), y, width, height);

            cv::Mat viz_gray = create_column_image({grayL(roiL), grayR(roiR)}, CV_8UC1, viz_scale);
            cv::Mat viz_motion_abs = create_column_image({diffL(roiL), diffR(roiR)}, CV_8UC1, viz_scale);
            cv::Mat viz_noise = create_column_image({motion_filtered_noise_mapL(roiL), motion_filtered_noise_mapR(roiR)}, CV_8UC1, viz_scale);
            if (motion_filtered_noise_mapL.cols) {
                cv::Mat viz_mask = create_column_image({diffL(roiL) > motion_filtered_noise_mapL(roiL) + _motion_thresh, diffR(roiR) > motion_filtered_noise_mapR(roiR) + _motion_thresh}, CV_8UC1, viz_scale);
                if (grayL_masked_final.cols) {
                    cv::Mat viz_gray_masked = create_column_image({grayL_masked_final, grayR_masked_final}, CV_8UC1, viz_scale);
                    cv::Mat viz_mask_combined = create_column_image({mask_final, mask_final}, CV_8UC1, viz_scale);
                    viz_disp = create_row_image({viz_gray, viz_motion_abs, viz_noise, viz_mask, viz_mask_combined, viz_gray_masked}, CV_8UC1, 1);
                } else {
                    viz_disp = create_row_image({viz_gray, viz_motion_abs, viz_noise, viz_mask}, CV_8UC1, 1);
                }
            } else {
                viz_disp = create_row_image({viz_gray, viz_motion_abs, viz_noise}, CV_8UC1, 1);
            }
        }
        return sub_disp;
    } else {
        // disparity matching failed for unknown reasons. This is a bit of a pickle, for now we just return the predicted
        // disparity, unless the sanity check using rough disparity does not agree... Further discussion in #362
        if (calculated_rough_disp)
            return disp_pred;
        else {
            auto disp_rough = calc_rough_disparity(&blob_shifted, radius);
            if (properly_tracking() && fabs(disp_pred - disp_rough) < 2)
                return disp_pred;
            else
                return disp_rough;
        }
    }
}

float ItemTracker::calc_match_score_motion(int i, int x, int y, int width, int height, float diffL_sum, cv::Mat diffL_roi, cv::Mat diffR) {
    cv::Rect roiR = cv::Rect(x - i, y, width, height);
    float diff_sum = diffL_sum + static_cast<float>(cv::sum(diffR(roiR))[0]);
    cv::Mat errV;
    absdiff(diffL_roi, diffR(roiR), errV);
    return static_cast<float>(cv::sum(errV)[0]) / diff_sum;
}

std::tuple<float, float> ItemTracker::calc_match_score_masked(int i, int disp_end, int width, int height, cv::Mat diffL_mask_patch, cv::Mat diffR_mask_patch, cv::Mat grayL_patch, cv::Mat grayR_patch, int npixels) {
    cv::Rect roiR_patch = cv::Rect(disp_end - 1 - i, 0, width, height);
    cv::Mat errV;
    cv::Mat grayL_masked, grayR_masked, mask;

    cv::bitwise_and(diffL_mask_patch, diffR_mask_patch(roiR_patch), mask);
    cv::bitwise_and(grayL_patch, grayL_patch, grayL_masked, mask);
    cv::bitwise_and(grayR_patch(roiR_patch), grayR_patch(roiR_patch), grayR_masked, mask);
    float cnz = cv::countNonZero(mask);
    absdiff(grayL_masked, grayR_masked, errV);

    return std::make_tuple(cnz / npixels, static_cast<float>(cv::sum(errV)[0]) / cnz);
}

std::tuple<int, float, int, int, bool> ItemTracker::disparity_search_rng(BlobProps *blob, int x, int radius) {

    int tmp_max_disp = max_disparity;
    if (x - max_disparity < 0)
        tmp_max_disp = x;

    int disp_start = min_disparity;
    int disp_end = tmp_max_disp;
    float disp_pred;
    bool calculated_rough_disp = false;

    if (_image_predict_item.valid && _n_frames_lost < n_frames_lost_threshold) {
        disp_pred = _image_predict_item.disparity;
        disp_start = std::max(static_cast<int>(floorf(disp_pred)) - 4, disp_start);
        disp_end = std::min(static_cast<int>(ceilf(disp_pred)) + 4, disp_end);
    } else if (_n_frames_tracking > 5 && disparity_prev > 0) {
        disp_pred = disparity_prev;
        disp_start = std::max(static_cast<int>(floorf(disp_pred)) - 4, disp_start);
        disp_end = std::min(static_cast<int>(ceilf(disp_pred)) + 4, disp_end);
    } else {
        disp_pred = calc_rough_disparity(blob, radius);
        calculated_rough_disp = true;
        if (disp_pred < 0)
            return std::make_tuple(-1, -1, -1, -1, true);
        int r = std::clamp(static_cast<int>(roundf(blob->size_unscaled() / 2.f)), 4, 8);
        disp_start = std::max(static_cast<int>(floorf(disp_pred)) - r, params.min_disparity.value());
        disp_end = std::min(static_cast<int>(ceilf(disp_pred)) + r, params.max_disparity.value());
        if (x - disp_end < 0)
            disp_end = x;
        min_disparity = disp_start;
        max_disparity = disp_end;
    }
    if (disp_start < 1)
        disp_start = 1;
    if (disp_pred < disp_start + 1)
        disp_pred = disp_start + 1;


    int disp_rng = disp_end - disp_start;

    return std::make_tuple(disp_start, disp_pred, disp_rng, disp_end, calculated_rough_disp);
}
int ItemTracker::calc_rough_disparity(BlobProps *blob, int radius) {
    //find the maximum motion location in the right image
    //only need to look at the line where the blob is found in the left image
    auto pt = blob->pt_unscaled();
    int max_disp = params.max_disparity.value();
    cv::Rect roiR(roundf(pt.x - radius) - max_disp, roundf(pt.y - 1), radius * 2 + max_disp, 3);
    int offset = 0;
    if (roiR.x < 0)
        offset = roiR.x;
    roiR = clamp_rect(roiR, IMG_W, IMG_H);
    cv::Mat diffR_roi = _visdat->diffR(roiR);
    cv::Mat blurred;
    GaussianBlur(diffR_roi, blurred, Size(9, 3), 0);
    Point mint;
    Point maxt;
    double min, max;
    minMaxLoc(blurred, &min, &max, &mint, &maxt);
    int disparity = blurred.cols - maxt.x - radius + offset;
    if (_visdat->motion_filtered_noise_mapR.cols) {
        cv::Mat noiseR_roi = _visdat->motion_filtered_noise_mapR(roiR);
        auto pm = noiseR_roi.at<uint8_t>(maxt);
        if (max < pm)
            disparity = -1;
    } else if (max < 3)
        disparity = -1;
    return disparity;
}
float ItemTracker::estimate_sub_disparity(int disparity, float *err) {
    float y1 = -err[disparity - 1];
    float y2 = -err[disparity];
    float y3 = -err[disparity + 1];
    // by assuming a hyperbola shape, the x-location of the hyperbola minimum is determined and used as best guess
    float h31 = (y3 - y1);
    float h21 = (y2 - y1) * 4.f;
    float sub_disp = (h21 - h31) / (h21 - h31 * 2.f);
    sub_disp += sinf(sub_disp * 2.f * M_PIf32) * 0.13f;
    sub_disp += (disparity - 1);

    if (sub_disp < disparity - 1 || sub_disp > disparity + 1 || sub_disp != sub_disp)
        sub_disp = disparity;

    return sub_disp;
}

void ItemTracker::update_prediction(double time) {
    if (_track.back().pos_valid) {
        last_valid_trackdata_for_prediction = _track.back();
        if (_track.back().vel_valid)
            last_vel_valid_trackdata_for_prediction = _track.back();
    }

    if (!last_valid_trackdata_for_prediction.pos_valid) {
        _image_predict_item.valid = false;
    } else {
        cv::Point3f pos = last_valid_trackdata_for_prediction.pos();
        cv::Point3f vel = last_vel_valid_trackdata_for_prediction.vel();
        cv::Point3f acc = last_vel_valid_trackdata_for_prediction.acc();
        if (!last_vel_valid_trackdata_for_prediction.vel_valid)
            vel = {0};
        if (!last_vel_valid_trackdata_for_prediction.acc_valid)
            acc = {0};

        // predict insect position for next frame
        float dt_pred = static_cast<float>(time - last_valid_trackdata_for_prediction.time) + 1.f / pparams.fps;
        if (dt_pred > 0.3f) {
            _image_predict_item.valid = false;
            return;
        }
        cv::Point3f predicted_pos = pos + vel * dt_pred + 0.5 * acc * powf(dt_pred, 2);

        auto p = world2im_3d(predicted_pos, _visdat->Qfi, _visdat->camera_roll, _visdat->camera_pitch);
        float size = world2im_size(last_valid_trackdata_for_prediction.world_item.pt + cv::Point3f(expected_radius, 0, 0), last_valid_trackdata_for_prediction.world_item.pt - cv::Point3f(expected_radius, 0, 0), _visdat->Qfi, _visdat->camera_roll, _visdat->camera_pitch);
        float pixel_max = _image_predict_item.pixel_max;
        if (_image_item.valid)
            pixel_max = _image_item.pixel_max;
        _image_predict_item = ImagePredictItem(p, size, pixel_max, _visdat->frame_id);
    }
}

void ItemTracker::update_state(Point3f measured_world_coordinates, double time, bool using_prediction) {
    TrackData data;
    data.using_prediction = using_prediction;
    data.pos_valid = true;
    data.state.pos = measured_world_coordinates;

    TrackData data_prev;
    if (_track.size() > 0)
        data_prev = _track.back();

    if (reset_smoothers) {
        smoother_posX.reset();
        smoother_posY.reset();
        smoother_posZ.reset();
        smoother_accX.reset();
        smoother_accY.reset();
        smoother_accZ.reset();
    }
    float dt = n_frames_lost_threshold / pparams.fps;
    if (reset_smoothers) {
        data.state.spos = data.state.pos;
    } else {
        data.state.spos.x = smoother_posX.addSample(data.state.pos.x);
        data.state.spos.y = smoother_posY.addSample(data.state.pos.y);
        data.state.spos.z = smoother_posZ.addSample(data.state.pos.z);

        if (data_prev.pos_valid) {
            dt = static_cast<float>(time - data_prev.time);
            data.state.vel_unfiltered = (data.state.pos - data_prev.state.pos) / dt;
            cv::Point3f v_pt = vel_filt.new_sample(data.state.vel_unfiltered);
            data.state.vel = v_pt;
            data.vel_valid = true;
            if (data_prev.vel_valid) {
                float ax = (data.state.vel.x - data_prev.state.vel.x) / dt;
                float ay = (data.state.vel.y - data_prev.state.vel.y) / dt;
                float az = (data.state.vel.z - data_prev.state.vel.z) / dt;
                data.state.acc.x = smoother_accX.addSample(ax);
                data.state.acc.y = smoother_accY.addSample(ay);
                data.state.acc.z = smoother_accZ.addSample(az);
                data.acc_valid = smoother_accX.ready();
            }
        }

    }

    data.time = time;
    _last_detection = time;
    data.dt = dt;
    data.world_item = _world_item;
    data.predicted_image_item = _image_predict_item;
    reset_smoothers = false;
    _track.push_back(data);
}

void ItemTracker::reset_tracker_ouput(double time) {
    TrackData data;
    reset_smoothers = true;
    disparity_prev = -1;
    _image_predict_item.valid = false;
    data.time = time;
    _track.push_back(data);
}

bool ItemTracker::check_ignore_blobs_generic(BlobProps *blob) {
    bool in_ignore_zone = false;
    for (auto ignore : ignores_for_me) {
        float dist_ignore = normf(ignore.p - blob->pt());
        if (dist_ignore < blob->size + ignore.radius) {
            ignore.was_used = true;
            blob->ignores.push_back(ignore);
            in_ignore_zone = true;
        }
    }
    return in_ignore_zone;
}

float ItemTracker::score(BlobProps *blob, ImageItem *ref) {
    float im_dist_err_ratio, im_size_pred_err_ratio;
    const float max_world_dist = 0.05f; // max distance a blob can travel in one frame

    if (_image_item.valid && _world_item.valid) {
        cv::Point3f last_world_pos = im2world(_image_item.pt(), _image_item.disparity, _visdat->Qf, _visdat->camera_roll, _visdat->camera_pitch);
        float max_im_dist = world2im_dist(last_world_pos, max_world_dist, _visdat->Qfi, _visdat->camera_roll, _visdat->camera_pitch) + _image_item.size / 2;
        float world_projected_im_err = normf(blob->pt_unscaled() - _image_item.pt());
        float world_projected_pred_im_err = INFINITY;
        if (_image_predict_item.valid)
            world_projected_pred_im_err = normf(blob->pt_unscaled() - _image_predict_item.pt);
        if (world_projected_pred_im_err < world_projected_im_err)
            world_projected_im_err = world_projected_pred_im_err;
        im_dist_err_ratio = world_projected_im_err / max_im_dist;
        float prev_size = smoother_im_size.latest();
        im_size_pred_err_ratio = fabs(prev_size - blob->size_unscaled()) / (blob->size_unscaled() + prev_size);
        if (!smoother_im_size.ready())
            im_size_pred_err_ratio *= 0.5f;
        if (!properly_tracking()) {
            float im_dist = normf(blob->pt_unscaled() - ref->pt());
            if (im_dist_err_ratio > 0.1f * (im_dist / prev_size))
                im_dist_err_ratio = 0.1f * (im_dist / prev_size);
        }
    } else if (_image_predict_item.valid) {
        cv::Point3f predicted_world_pos = im2world(_image_predict_item.pt, _image_predict_item.disparity, _visdat->Qf, _visdat->camera_roll, _visdat->camera_pitch);
        float max_im_dist = world2im_dist(predicted_world_pos, max_world_dist, _visdat->Qfi, _visdat->camera_roll, _visdat->camera_pitch) + _image_predict_item.size / 2;
        float world_projected_im_err = normf(blob->pt_unscaled() - _image_predict_item.pt);
        im_dist_err_ratio = world_projected_im_err / max_im_dist;
        im_size_pred_err_ratio = fabs(_image_predict_item.size - blob->size_unscaled()) / (blob->size_unscaled() + _image_predict_item.size);
    } else if (ref->valid) {
        im_size_pred_err_ratio = fabs(ref->size - blob->size_unscaled()) / (blob->size_unscaled() + ref->size);
        float max_im_dist = 25;
        im_dist_err_ratio = sqrtf(powf(ref->x - blob->x * pparams.imscalef, 2) + powf(ref->y - blob->y * pparams.imscalef, 2)) / max_im_dist;
    } else {
        return INFINITY;
    }

    float score = im_dist_err_ratio * 0.75f + 0.25f * im_size_pred_err_ratio;
    if (!properly_tracking() && (blob->in_overexposed_area || blob->false_positive))
        score *= 1.5f;
    else if (blob->in_overexposed_area || blob->false_positive)
        score *= 1.25f;

    return score;
}

void ItemTracker::deserialize_settings() {
    // std::cout << "Reading settings from: " << settings_file << std::endl;
    if (file_exist(settings_file)) {
        std::ifstream infile(settings_file);
        std::string xmlData((std::istreambuf_iterator<char>(infile)),
                            std::istreambuf_iterator<char>());

        if (!xmls::Serializable::fromXML(xmlData, &params))
        {   // Deserialization not successful
            throw MyExit("Cannot read: " + settings_file);
        }
        TrackerParams tmp;
        auto v1 = params.getVersion();
        auto v2 = tmp.getVersion();
        if (v1 != v2) {
            throw MyExit("XML version difference detected from " + settings_file);
        }
        infile.close();
    } else {
        throw MyExit("File not found: " + settings_file);
    }

    min_disparity = params.min_disparity.value();
    max_disparity = params.max_disparity.value();
    _score_threshold = params.score_threshold.value();
    background_subtract_zone_factor = params.background_subtract_zone_factor.value();
    max_radius = params.max_size.value();
}
void ItemTracker::serialize_settings() {
    params.min_disparity = min_disparity;
    params.max_disparity = max_disparity;
    params.score_threshold = _score_threshold;
    params.background_subtract_zone_factor = background_subtract_zone_factor;
    params.max_size = max_radius;

    std::string xmlData = params.toXML();
    std::ofstream outfile = std::ofstream(settings_file);
    outfile << xmlData ;
    outfile.close();
}

void ItemTracker::close() {
    if (initialized_logger) {
        (*_logger) << std::flush;
        _logger->close();
    }
    if (initialized && (pparams.insect_tracking_tuning || pparams.drone_tracking_tuning))
        serialize_settings();
    initialized_logger = false;
    initialized = false;
}

}
