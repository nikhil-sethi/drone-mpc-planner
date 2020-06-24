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

void ItemTracker::init(std::ofstream *logger, VisionData *visdat, std::string name, int16_t viz_id) {
    static int16_t trkr_cntr = 0;
    _uid = trkr_cntr++;
    _viz_id = viz_id;
    _logger = logger;
    _visdat = visdat;
    _name = name;
    track_history_max_size = pparams.fps;
    settings_file = "../../xml/" + name + "tracker.xml";
    std::string window_name = name + "_trkr";

    _world_item = WorldItem();

    deserialize_settings();

    if (pparams.insect_tracking_tuning ||pparams.drone_tracking_tuning) {
        namedWindow(window_name, WINDOW_NORMAL);
        createTrackbar("Min disparity", window_name, &min_disparity, 255);
        createTrackbar("Max disparity", window_name, &max_disparity, 255);
        createTrackbar("background_subtract_zone_factor", window_name, &background_subtract_zone_factor, 100);
    }
    if (pos_smth_width<0)
        pos_smth_width = pparams.fps/20;
    if (vel_smth_width<0)
        vel_smth_width = pparams.fps/20;
    if (acc_smth_width<0)
        acc_smth_width = pparams.fps/20;
    smoother_posX.init(pos_smth_width);
    smoother_posY.init(pos_smth_width);
    smoother_posZ.init(pos_smth_width);

    smoother_velX.init(vel_smth_width);
    smoother_velY.init(vel_smth_width);
    smoother_velZ.init(vel_smth_width);

    smoother_accX.init(acc_smth_width);
    smoother_accY.init(acc_smth_width);
    smoother_accZ.init(acc_smth_width);

    smoother_im_size.init(smooth_blob_props_width);
    smoother_score.init(smooth_blob_props_width);
    smoother_brightness.init(smooth_blob_props_width);

    disparity_prev = -1;

    init_logger();
    initialized = true;

    if (pparams.has_screen)
        enable_draw_stereo_viz = false;

}

void ItemTracker::init_logger() {
    if (_logger->is_open()) {
        (*_logger) << "imLx_" << _name << ";";
        (*_logger) << "imLy_" << _name << ";";
        (*_logger) << "disparity_" << _name << ";";
        (*_logger) << "size_" << _name << ";";
        (*_logger) << "score_" << _name << ";";
        (*_logger) << "imLx_pred_" << _name << ";";
        (*_logger) << "imLy_pred_" << _name << ";";
        (*_logger) << "n_frames_lost_" << _name << ";";
        (*_logger) << "n_frames_tracking_" << _name << ";";
        (*_logger) << "foundL_" << _name << ";";
        (*_logger) << "posX_" << _name << ";";
        (*_logger) << "posY_" << _name << ";";
        (*_logger) << "posZ_" << _name << ";";
        (*_logger) << "sposX_" << _name << ";";
        (*_logger) << "sposY_" << _name << ";";
        (*_logger) << "sposZ_" << _name << ";";
        (*_logger) << "svelX_" << _name << ";";
        (*_logger) << "svelY_" << _name << ";";
        (*_logger) << "svelZ_" << _name << ";";
        (*_logger) << "saccX_" << _name << ";";
        (*_logger) << "saccY_" << _name << ";";
        (*_logger) << "saccZ_" << _name << ";";
        (*_logger) << "radius_" << _name << ";";
    }
}

void ItemTracker::calc_world_props_blob_generic(BlobProps * pbs, bool use_max) {
    if (pbs->world_props.trkr_id != _uid) {
        BlobWorldProps w;
        cv::Point2f p;
        if (use_max)
            p = pbs->pt_max;
        else
            p = Point2f(pbs->x, pbs->y);

        w.trkr_id = _uid;

        p*=pparams.imscalef;
        float size = pbs->size*pparams.imscalef;
        float disparity = stereo_match(p,size);

        if (disparity < min_disparity || disparity > max_disparity) {
            w.disparity_in_range = false;
        } else {
            w.disparity_in_range = true;

            if (disparity_prev>0 && n_frames_tracking> 5)
                w.disparity = (disparity_filter_rate * disparity ) + ((1-disparity_filter_rate)*disparity_prev); // moving average on disparity
            else
                w.disparity = disparity;

            std::vector<Point3d> camera_coordinates, world_coordinates;
            camera_coordinates.push_back(Point3d(p.x,p.y,-w.disparity));
            camera_coordinates.push_back(Point3d(p.x+size,p.y,-w.disparity)); // to calc world radius
            cv::perspectiveTransform(camera_coordinates,world_coordinates,_visdat->Qf);

            w.radius = cv::norm(world_coordinates[0]-world_coordinates[1]) / 2;
            w.radius_in_range = w.radius < max_size;

            w.x = world_coordinates[0].x;
            w.y = world_coordinates[0].y;
            w.z = world_coordinates[0].z;
            //compensate camera rotation:
            float theta = _visdat->camera_angle * deg2rad;
            float temp_y = w.y * cosf(theta) + w.z * sinf(theta);
            w.z = -w.y * sinf(theta) + w.z * cosf(theta);
            w.y = temp_y;

            w.distance_bkg = _visdat->depth_background_mm.at<float>(p.y,p.x);
            w.distance = sqrtf(powf(w.x,2) + powf(w.y,2) +powf(w.z,2));
            if ((w.distance > w.distance_bkg*(static_cast<float>(background_subtract_zone_factor)/100.f)) )
                w.bkg_check_ok = false;
            else
                w.bkg_check_ok = true;
        }
        pbs->world_props = w;
    }
}

void ItemTracker::update_blob_filters() {
    if (!_image_item.blob_is_fused) {
        smoother_im_size.addSample(_image_item.size);
        smoother_brightness.addSample(_image_item.pixel_max);
        _blobs_are_fused_cnt = 0;
    }
    smoother_score.addSample(_image_item.score);
    disparity_prev = _world_item.iti.disparity;
}

void ItemTracker::update(double time) {
    if ( _world_item.valid) {
        path.push_back(_world_item);
        update_state(_world_item.pt,time);
        update_blob_filters();
        _tracking = true;
        n_frames_lost = 0; // update this after calling update_state, so that it can determine how long tracking was lost
        n_frames_tracking++;
    } else {
        n_frames_lost++;
        n_frames_tracking = 0;
        if( n_frames_lost >= n_frames_lost_threshold || !_tracking ) {
            _tracking = false;
            reset_tracker_ouput(time);
        } else {
            track_data data;
            data.time = time;
            track_history.push_back(data);
        }
    }

    cleanup_paths();

    append_log();
}

//make sure the vectors that contain the path data don't endlesly grow
void ItemTracker::cleanup_paths() {
    if (_visdat->frame_id > path_buf_size) {
        if (path.size() > 0) {
            if (path.begin()->frame_id() < _visdat->frame_id - path_buf_size)
                path.erase(path.begin());
            if (path.begin()->frame_id() > _visdat->frame_id ) //at the end of a realsense video loop, frame_id resets
                path.clear();
        }

        if (predicted_image_path.size() > 0) {
            if (predicted_image_path.begin()->frame_id < _visdat->frame_id - path_buf_size )
                predicted_image_path.erase(predicted_image_path.begin());
            if (predicted_image_path.begin()->frame_id > _visdat->frame_id ) //at the end of a realsense video loop, frame_id resets
                predicted_image_path.clear();
        }
    }

    while (track_history.size() > track_history_max_size)
        track_history.erase(track_history.begin());
}

void ItemTracker::append_log() {
    if (_logger->is_open()) {
        //log all image stuff
        if (path.size()>0)
            (*_logger) << _image_item.x * pparams.imscalef << ";" << _image_item.y * pparams.imscalef << ";" << _image_item.disparity << ";"
                       << _image_item.size  << ";" << _image_item.score  << ";";
        else
            (*_logger) << -1 << ";" << -1 << ";" << -1 << ";" << -1 << ";" << -1 << ";";
        if (_image_predict_item.valid)
            (*_logger) << _image_predict_item.x << ";" << _image_predict_item.y << ";";
        else
            (*_logger) << -1 << ";" << -1   << ";";

        (*_logger) << n_frames_lost << ";" << n_frames_tracking << ";" << _tracking << ";";
        //log all world stuff
        track_data last = Last_track_data();
        (*_logger) << last.state.pos.x << ";" << last.state.pos.y << ";" << last.state.pos.z << ";" ;
        (*_logger) << last.state.spos.x << ";" << last.state.spos.y << ";" << last.state.spos.z << ";";
        (*_logger) << last.state.vel.x << ";" << last.state.vel.y << ";" << last.state.vel.z << ";" ;
        (*_logger) << last.state.acc.x << ";" << last.state.acc.y << ";" << last.state.acc.z << ";" ;
        if (_world_item.valid)
            (*_logger) << _world_item.radius << ";";
        else
            (*_logger) << -1 << ";";

    }

    while (track_history.size() > track_history_max_size)
        track_history.erase(track_history.begin());
}

float ItemTracker::stereo_match(cv::Point2f im_posL,float size) {

    cv::Mat diffL,diffR,grayL,grayR,motion_noise_mapL,motion_noise_mapR;

    diffL = _visdat->diffL;
    diffR = _visdat->diffR;
    motion_noise_mapL = _visdat->motion_noise_mapL;
    motion_noise_mapR = _visdat->motion_noise_mapR;
    grayL = _visdat->frameL;
    grayR = _visdat->frameR;
    int radius = ceilf((size + 2.f)*0.5f);

    //limit the patches for CPU optimization
    //We don't resize, but just select the middle rect of the full patch. This way we
    //still have full resolution (very important for disparity precision), but limit
    //the amount of pixels being matched, without needing to do cpu intensive resizing. Win win :)
    if (radius > 20)
        radius = 20;

    int x,y,width,height;
    x = std::clamp(static_cast<int>(roundf(im_posL.x))-radius,0,diffL.cols-1);
    width = 2*radius;
    y = std::clamp(static_cast<int>(roundf(im_posL.y))-radius,0,diffL.rows-1);
    height = 2*radius;

    if (x+width >= diffL.cols)
        width=diffL.cols-x;
    if (y+height >= diffL.rows)
        height=diffL.rows-y;
    cv::Rect roiL(x,y,width,height);

    auto [disp_start,disp_pred,disp_rng,disp_end] =  disparity_search_rng(x);
    if (disp_rng < 3) {
        return -1; // return out of range which must be handled outside this function
    }

    float npixels = static_cast<float>(roiL.width*roiL.height);
    float err_masked [disp_end] = {0};

    float masked_pixel_ratio[disp_end] = {0};
    int disparity_masked = 0;
    const float min_pxl_ratio = 0.25f;

    if (motion_noise_mapL.cols) {
        //since the background often isn't a solid color we do matching on the raw image data instead of the motion
        //using the motion from both images as a mask, we match the disparity over the masked gray image

        cv::Mat diffL_mask_patch = diffL(roiL)>motion_noise_mapL(roiL);
        if (cv::countNonZero(diffL_mask_patch) / npixels > min_pxl_ratio) {
            cv::Rect roiR_disparity_rng(x-(disp_end-1),y,width+(disp_end-disp_start-1),height);
            cv::Mat diffR_mask_patch = diffR(roiR_disparity_rng)>motion_noise_mapR(roiR_disparity_rng);
            cv::Mat grayL_patch = _visdat->frameL(roiL);
            cv::Mat grayR_patch = _visdat->frameR(roiR_disparity_rng);


            bool err_calculated [disp_end] = {false};
            //search for a minimum matching error
            int ii = roundf(disp_pred);
            while(!err_calculated[ii-1] || !err_calculated[ii] || !err_calculated[ii+1] ) {
                if (!err_calculated[ii-1]) {
                    std::tie(masked_pixel_ratio[ii-1],err_masked[ii-1]) = calc_match_score_masked(ii-1, disp_end, width, height,diffL_mask_patch,diffR_mask_patch,grayL_patch, grayR_patch,npixels);
                    err_calculated[ii-1] = true;
                }
                if (!err_calculated[ii]) {
                    std::tie(masked_pixel_ratio[ii],err_masked[ii]) = calc_match_score_masked(ii, disp_end, width, height,diffL_mask_patch,diffR_mask_patch,grayL_patch, grayR_patch,npixels);
                    err_calculated[ii] = true;
                }
                if (!err_calculated[ii+1]) {
                    std::tie(masked_pixel_ratio[ii+1],err_masked[ii+1]) = calc_match_score_masked(ii+1, disp_end, width, height,diffL_mask_patch,diffR_mask_patch,grayL_patch, grayR_patch,npixels);
                    err_calculated[ii+1] = true;
                }

                if (err_masked[ii-1] >= err_masked[ii] && err_masked[ii+1] >= err_masked[ii]) { // minimum found
                    disparity_masked  = ii;
                    break;
                } else if (err_masked[ii-1] > err_masked[ii+1]) { // no minumum here, determine search direction based on slope
                    int ii_cnt = 0;
                    while (err_calculated[ii+1] && ii_cnt < disp_end-disp_start) {
                        ii_cnt++;
                        ii++;
                        if (ii > disp_end-2)
                            ii = disp_start+1;
                    }
                } else {
                    int ii_cnt = 0;
                    while (err_calculated[ii-1] && ii_cnt < disp_end-disp_start) {
                        ii_cnt++;
                        ii--;
                        if (ii < disp_start+1)
                            ii = disp_end-2;
                    }
                }
            }
        }
    }

    int disparity;
    float * err;
    float err_motion [disp_end] = {0};
    if (masked_pixel_ratio[disparity_masked] < min_pxl_ratio || !motion_noise_mapL.cols) {
        //back up strategy: ignore the background, hope for the best, and just do matching directly on the motion
        int disparity_motion = 0;
        float tmp_diffL_sum = cv::sum(diffL(roiL))[0];
        bool err_calculated [disp_end] = {false};

        int ii = roundf(disp_pred);
        while(!err_calculated[ii-1] || !err_calculated[ii] || !err_calculated[ii+1] ) {

            if (!err_calculated[ii-1]) {
                err_motion[ii-1] = calc_match_score_motion(ii-1,x,y,width,height,tmp_diffL_sum,diffL(roiL),diffR);
                err_calculated[ii-1] = true;
            }
            if (!err_calculated[ii]) {
                err_motion[ii] = calc_match_score_motion(ii,x,y,width,height,tmp_diffL_sum,diffL(roiL),diffR);
                err_calculated[ii] = true;
            }
            if (!err_calculated[ii+1]) {
                err_motion[ii+1] = calc_match_score_motion(ii+1,x,y,width,height,tmp_diffL_sum,diffL(roiL),diffR);
                err_calculated[ii+1] = true;
            }

            if (err_motion[ii-1] >= err_motion[ii] && err_motion[ii+1] >= err_motion[ii]) { // minimum found
                disparity_motion  = ii;
                break;
            } else if (err_motion[ii-1] > err_motion[ii+1]) { // no minumum here, determine search direction based on slope
                int ii_cnt = 0;
                while (err_calculated[ii+1] && ii_cnt < disp_end-disp_start) {
                    ii_cnt++;
                    ii++;
                    if (ii > disp_end-2)
                        ii = disp_start+1;
                }
            } else {
                int ii_cnt = 0;
                while (err_calculated[ii-1] && ii_cnt < disp_end-disp_start) {
                    ii_cnt++;
                    ii--;
                    if (ii < disp_start+1)
                        ii = disp_end-2;
                }
            }
        }

        disparity = disparity_motion;
        err = err_motion;
    } else {
        disparity = disparity_masked;
        err = err_masked;
    }

    if (disparity > 0) {
        float sub_disp = estimate_sub_disparity(disparity,err);

        if (enable_draw_stereo_viz) {
            int viz_scale = 4;
            cv::Rect roiR = cv::Rect (x-round(sub_disp),y,width,height);

            cv::Mat viz_gray = create_column_image({grayL(roiL),grayR(roiR)},CV_8UC1,viz_scale);
            cv::Mat viz_motion_abs = create_column_image({diffL(roiL),diffR(roiR)},CV_8UC1,viz_scale);
            if (motion_noise_mapL.cols) {
                cv::Mat viz_test = create_column_image({diffL(roiL)>motion_noise_mapL(roiL),diffR(roiR)>motion_noise_mapR(roiR)},CV_8UC1,viz_scale);
                cv::Mat viz_noise = create_column_image({motion_noise_mapL(roiL),motion_noise_mapR(roiR)},CV_8UC1,viz_scale);
                viz_disp = create_row_image({viz_gray,viz_motion_abs,viz_noise,viz_test},CV_8UC1,1);
            } else {
                viz_disp = create_row_image({viz_gray,viz_motion_abs},CV_8UC1,1);
            }
        }
        return sub_disp;
    } else {
        return 0;
    }
}


float ItemTracker::calc_match_score_motion(int i,int x, int y, int width, int height,float tmp_diffL_sum, cv::Mat diffL_roi, cv::Mat diffR) {
    cv::Rect roiR = cv::Rect (x-i,y,width,height);
    float tmp_diff_sum = tmp_diffL_sum + static_cast<float>(cv::sum(diffR(roiR))[0]);
    cv::Mat errV;
    absdiff(diffL_roi,diffR(roiR),errV);
    return static_cast<float>(cv::sum(errV)[0]) / tmp_diff_sum;
}

std::tuple<float,float> ItemTracker::calc_match_score_masked(int i, int disp_end, int width, int height, cv::Mat diffL_mask_patch, cv::Mat diffR_mask_patch, cv::Mat grayL_patch, cv::Mat grayR_patch, int npixels) {
    cv::Rect roiR_patch = cv::Rect (disp_end-1-i,0,width,height);
    cv::Mat errV;
    cv::Mat grayL_masked,grayR_masked,mask;

    cv::bitwise_and(diffL_mask_patch,diffR_mask_patch(roiR_patch),mask);
    cv::bitwise_and(grayL_patch,grayL_patch,grayL_masked,mask);
    cv::bitwise_and(grayR_patch(roiR_patch),grayR_patch(roiR_patch),grayR_masked,mask);
    float cnz = cv::countNonZero(mask);
    absdiff(grayL_masked,grayR_masked,errV);

    return std::make_tuple(cnz / npixels,static_cast<float>(cv::sum(errV)[0]) / cnz);
}

std::tuple<int,float,int,int> ItemTracker::disparity_search_rng(int x) {

    int tmp_max_disp = max_disparity;
    if (x - tmp_max_disp < 0)
        tmp_max_disp = x;

    int disp_start = min_disparity;
    int disp_end = tmp_max_disp;
    float disp_pred;

    if (_image_predict_item.valid && _image_predict_item.certainty > 0.9f) {
        disp_start = std::max(static_cast<int>(floorf(_image_predict_item.disparity))-4,disp_start);
        disp_pred = _image_predict_item.disparity;
        disp_end = std::min(static_cast<int>(ceilf(_image_predict_item.disparity))+4,disp_end);
    } else if (n_frames_tracking>5) {
        auto tmp_disp_prev = disparity_prev;
        disp_start = std::max(static_cast<int>(floorf(tmp_disp_prev))-4,disp_start);
        disp_end = std::min(static_cast<int>(ceilf(tmp_disp_prev))+4,disp_end);
        disp_pred = (disp_end - disp_start)/2.f + disp_start;
    } else {
        disp_pred = (disp_end - disp_start)/2.f + disp_start;
    }

    if (disp_start < 1)
        disp_start = 1;
    if (disp_pred < disp_start + 1)
        disp_pred = disp_start + 1;

    int disp_rng = disp_end - disp_start;
    if (disp_rng > 20)
        std::cout << "Warning large disparity search range: " << disp_rng << std::endl;

    return std::make_tuple(disp_start,disp_pred,disp_rng,disp_end);
}

float ItemTracker::estimate_sub_disparity(int disparity,float * err) {
    float y1 = -err[disparity-1];
    float y2 = -err[disparity];
    float y3 = -err[disparity+1];
    // by assuming a hyperbola shape, the x-location of the hyperbola minimum is determined and used as best guess
    float h31 = (y3 - y1);
    float h21 = (y2 - y1) * 4.f;
    float sub_disp = (h21 - h31) / (h21 - h31 * 2.f);
    sub_disp += sinf(sub_disp*2.f*M_PIf32)*0.13f;
    sub_disp += (disparity-1);

    if (sub_disp<disparity-1 || sub_disp>disparity+1 || sub_disp != sub_disp)
        sub_disp = disparity;

    return sub_disp;
}

void ItemTracker::update_prediction(double time) {
    vector<track_data>::reverse_iterator td;
    for (td = track_history.rbegin(); td != track_history.rend(); ++td) { // TODO: isn't there some stl algorithm for this?
        if (td->pos_valid)
            break;
    }
    if (!td->pos_valid) {
        _image_predict_item.valid = false;
    } else {
        cv::Point3f pos = td->pos();
        cv::Point3f vel = td->vel();
        cv::Point3f acc= td->acc();
        //todo: use control inputs to make prediction

        // predict insect position for next frame
        float dt_pred = static_cast<float>(time - td->time)+1.f/pparams.fps;
        cv::Point3f predicted_pos = pos + vel*dt_pred + 0.5*acc*powf(dt_pred,2);

        auto p = world2im_3d(predicted_pos,_visdat->Qfi,_visdat->camera_angle);

        //update tracker with prediciton
        _image_predict_item.x = std::clamp(static_cast<int>(p.x),0,IMG_W-1);
        _image_predict_item.y = std::clamp(static_cast<int>(p.y),0,IMG_H-1);
        _image_predict_item.disparity = std::clamp(p.z,0.f,static_cast<float>(max_disparity));
        _image_predict_item.size = world2im_size(_world_item.pt+cv::Point3f(dparams.radius,0,0),_world_item.pt-cv::Point3f(dparams.radius,0,0),_visdat->Qfi,_visdat->camera_angle);
    }
    //issue #108:
    predicted_image_path.push_back(_image_predict_item);
}

void ItemTracker::update_state(Point3f measured_world_coordinates,double time) {
    track_data data;
    data.pos_valid = true;
    data.state.pos = measured_world_coordinates;

    track_data data_prev;
    if (track_history.size()>0)
        data_prev = track_history.back();

    if (reset_filters) {
        smoother_posX.reset();
        smoother_posY.reset();
        smoother_posZ.reset();
        smoother_velX.reset();
        smoother_velY.reset();
        smoother_velZ.reset();
        smoother_accX.reset();
        smoother_accY.reset();
        smoother_accZ.reset();
    }
    float dt = n_frames_lost_threshold / pparams.fps;
    if (reset_filters) {
        data.state.spos = data.state.pos;
    } else {
        data.state.spos.x = smoother_posX.addSample(data.state.pos.x);
        data.state.spos.y = smoother_posY.addSample(data.state.pos.y);
        data.state.spos.z = smoother_posZ.addSample(data.state.pos.z);
        data.spos_valid = smoother_posX.ready();

        if (data_prev.pos_valid && (data.spos_valid || skip_wait_smth_spos )) {
            dt = static_cast<float>(time - data_prev.time);
            float vx = (data.state.spos.x - data_prev.state.spos.x) / dt;
            float vy = (data.state.spos.y - data_prev.state.spos.y) / dt;
            float vz = (data.state.spos.z - data_prev.state.spos.z) / dt;
            data.state.vel.x = smoother_velX.addSample(vx);
            data.state.vel.y = smoother_velY.addSample(vy);
            data.state.vel.z = smoother_velZ.addSample(vz);

            if (data_prev.vel_valid) {
                float ax = (data.state.vel.x - data_prev.state.vel.x) / dt;
                float ay = (data.state.vel.y - data_prev.state.vel.y) / dt;
                float az = (data.state.vel.z - data_prev.state.vel.z) / dt;
                data.state.acc.x = smoother_accX.addSample(ax);
                data.state.acc.y = smoother_accY.addSample(ay);
                data.state.acc.z = smoother_accZ.addSample(az);
            }
        }
    }

    data.vel_valid = smoother_velX.ready();
    data.acc_valid = smoother_accX.ready();

    data.time = time;
    last_sighting_time = time;
    data.dt = dt;
    reset_filters = false;
    track_history.push_back(data);
}

float ItemTracker::calc_certainty(KeyPoint item) {
    float new_tracking_certainty;
    if (_image_predict_item.valid) {
        new_tracking_certainty = 1.f / powf(powf(_image_predict_item.x - item.pt.x*pparams.imscalef,2) + powf(_image_predict_item.y - item.pt.y*pparams.imscalef,2),0.3f);
        new_tracking_certainty*= _image_predict_item.certainty;
        if (new_tracking_certainty>1 || new_tracking_certainty<0 || isnanf(new_tracking_certainty)) { // weird -nan sometimes???
            new_tracking_certainty = 1;
        }
    } else // if there was no prediciton, certainty prolly is quite low
        new_tracking_certainty = certainty_init;
    return new_tracking_certainty;
}

void ItemTracker::reset_tracker_ouput(double time) {
    track_data data;
    reset_filters = true;
    disparity_prev = -1;
    _image_predict_item.valid = false;
    data.time = time;
    track_history.push_back(data);
}

bool ItemTracker::check_ignore_blobs_generic(BlobProps * pbs) {
    bool in_ignore_zone = false;
    for (auto ignore : ignores_for_me) {
        float dist_ignore = sqrtf(powf(ignore.p.x-pbs->x,2)+powf(ignore.p.y-pbs->y,2));
        if (dist_ignore < pbs->size + ignore.radius ) {
            ignore.was_used = true;
            pbs->ignores.push_back(ignore);
            in_ignore_zone = true;
        }
    }

    return in_ignore_zone;
}

float ItemTracker::score(BlobProps blob, ImageItem ref) {
    float dist = sqrtf(powf(ref.x-blob.x,2)+powf(ref.y-blob.y,2));
    float im_size_diff = fabs(ref.size - blob.size) / (blob.size + ref.size);
    float score = 1.f / (dist + 15.f*im_size_diff); // TODO: certainty

    if (_image_predict_item.valid) {
        float dist_pred = sqrtf(powf(_image_predict_item.x-blob.x*pparams.imscalef,2)+powf(_image_predict_item.y-blob.y*pparams.imscalef,2));
        float ps = smoother_im_size.latest();
        float im_size_diff_pred = fabs(ps - blob.size) / (blob.size+ps);
        float score_pred = 1.f / (dist_pred + 15.f*im_size_diff_pred); // TODO: certainty
        if (score_pred > score)
            score = score_pred;
    }

    return score*1000.f;
}

void ItemTracker::deserialize_settings() {
    std::cout << "Reading settings from: " << settings_file << std::endl;
    if (file_exist(settings_file)) {
        std::ifstream infile(settings_file);
        std::string xmlData((std::istreambuf_iterator<char>(infile)),
                            std::istreambuf_iterator<char>());

        if (!xmls::Serializable::fromXML(xmlData, &params))
        {   // Deserialization not successful
            throw my_exit("Cannot read: " + settings_file);
        }
        TrackerParams tmp;
        auto v1 = params.getVersion();
        auto v2 = tmp.getVersion();
        if (v1 != v2) {
            throw my_exit("XML version difference detected from " + settings_file);
        }
        infile.close();
    } else {
        throw my_exit("File not found: " + settings_file);
    }

    min_disparity = params.min_disparity.value();
    max_disparity = params.max_disparity.value();
    _score_threshold = params.score_threshold.value();
    background_subtract_zone_factor = params.background_subtract_zone_factor.value();
    max_size = params.max_size.value();
}

void ItemTracker::serialize_settings() {
    params.min_disparity = min_disparity;
    params.max_disparity = max_disparity;
    params.score_threshold = _score_threshold;
    params.background_subtract_zone_factor = background_subtract_zone_factor;
    params.max_size = max_size;

    std::string xmlData = params.toXML();
    std::ofstream outfile = std::ofstream (settings_file);
    outfile << xmlData ;
    outfile.close();
}

void ItemTracker::close () {
    if (initialized) {
        (*_logger) << std::flush;
        std::cout << "Closing tracker: " << _name << std::endl;
        if (pparams.insect_tracking_tuning || pparams.drone_tracking_tuning)
            serialize_settings();
        initialized = false;
    }
}

}
