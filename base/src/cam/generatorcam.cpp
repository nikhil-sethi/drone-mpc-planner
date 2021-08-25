#include "generatorcam.h"
#include <iostream>
#include <string.h>
#include <unistd.h>       //usleep

#include "opencv2/imgproc/imgproc.hpp"



#include "stopwatch.h"
static stopwatch_c swc;

void GeneratorCam::init () {
    camparams.deserialize(calib_rfn);
    camparams.serialize(calib_wfn);

    calibration();

    depth_background = cv::Mat::ones(IMG_H,IMG_W,CV_16UC1);
    depth_background = 10000; // basically disable the depth background map if it is not found

    convert_depth_background_to_world();

    cv::Mat clt = cv::Mat::zeros(5000,5000,CV_8UC1);
    cv::Mat cdt = cv::Mat::zeros(5000,5000,CV_8UC1);
    int cirk_size = 150;

    cv::circle(clt,cv::Point2i(clt.rows/2,clt.cols/2),cirk_size,180,cv::FILLED);
    cv::GaussianBlur(clt,clt,cv::Size(55,55),25);
    circ_template_light = clt(cv::Rect(clt.rows/2-250,clt.cols/2-250,500,500));

    cv::circle(cdt,cv::Point2i(cdt.rows/2,cdt.cols/2),cirk_size,80,cv::FILLED);
    cv::GaussianBlur(cdt,cdt,cv::Size(55,55),25);
    circ_template_dark = cdt(cv::Rect(cdt.rows/2-250,cdt.cols/2-250,500,500));

    int wb = IMG_W,hb=IMG_H;
    frame_bkg = cv::Mat::zeros(hb,wb,CV_8UC1);
    for (int i = 0; i < wb; i+=wb/5) {
        cv::rectangle(frame_bkg,cv::Point2i(i,0),cv::Point2i(i,0)+cv::Point2i(wb/15,hb),0,cv::FILLED);
        cv::rectangle(frame_bkg,cv::Point2i(i+wb/15,0),cv::Point2i(i+wb/15,0)+cv::Point2i(wb/15,hb),64,cv::FILLED);
        cv::rectangle(frame_bkg,cv::Point2i(i+2*wb/15,0),cv::Point2i(i+2*wb/15,0)+cv::Point2i(wb/15,hb),255,cv::FILLED);
    }
    swc.Start();
    update();

    initialized = true;
    clt.release();
    cdt.release();
}

void GeneratorCam::calibration() {
    intr = new rs2_intrinsics();
    intr->fx = camparams.fx;
    intr->fy = camparams.fy;
    intr->ppx = camparams.ppx;
    intr->ppy = camparams.ppy;
    intr->height = camparams.height;
    intr->width = camparams.width;
    intr->model = static_cast<rs2_distortion>(camparams.model);
    intr->coeffs[0] = camparams.coeffs[0];
    intr->coeffs[1] = camparams.coeffs[1];
    intr->coeffs[2] = camparams.coeffs[2];
    intr->coeffs[3] = camparams.coeffs[3];
    intr->coeffs[4] = camparams.coeffs[4];

    float focal_length = camparams.fx; // same as fy
    float cx = camparams.ppx; // same for both cameras
    float cy = camparams.ppy;
    float baseline = camparams.baseline;
    baseline = fabs(baseline);
    Qf = (cv::Mat_<double>(4, 4) << 1.0, 0.0, 0.0, -cx, 0.0, 1.0, 0.0, -cy, 0.0, 0.0, 0.0, focal_length, 0.0, 0.0, 1/baseline, 0.0);
    cv::invert(Qf,Qfi);
}

StereoPair * GeneratorCam::update() {
    if (_rc->throttle > RC_BOUND_MIN || takeoff_start_time > 0) {
        if (takeoff_start_time<0)
            takeoff_start_time = _frame_time;
        float dt_takeoff = static_cast<float>(_frame_time - takeoff_start_time) - dparams.full_bat_and_throttle_spinup_duration;
        if (dt_takeoff < 0.2f && dt_takeoff>0)
            current_drone_pos =drone_start_pos+cv::Point3f(0,0.2f*dparams.default_thrust*powf(dt_takeoff,2),0);
        else if (dt_takeoff < 0)
            current_drone_pos =drone_start_pos+cv::Point3f(0,0.2f*dparams.default_thrust*powf(0,2),0);
        else if (dt_takeoff < 1)
            current_drone_pos = drone_start_pos+cv::Point3f(0,0.2f*dparams.default_thrust*powf(0.2,2),0);
        else if (dt_takeoff < 2) {
            auto pos1 = drone_start_pos +  cv::Point3f(0,0.2f*dparams.default_thrust*powf(0.2,2),0);
            auto pos2 = drone_start_pos  + cv::Point3f(sinf(dt_takeoff)/M_PIf32*2.f- 0.15f,0,cosf(dt_takeoff)/M_PIf32*2.f - 0.15f);
            current_drone_pos = (1- (dt_takeoff - 1)) * pos1 + (dt_takeoff - 1) * pos2;
        } else { //do some flight:
            current_drone_pos = drone_start_pos  + cv::Point3f(sinf(dt_takeoff)/M_PIf32*2.f- 0.15f,0,cosf(dt_takeoff)/M_PIf32*2.f - 0.15f);
        }
    }

    auto drone_im_pos = world2im_3d(current_drone_pos,Qfi,camparams.camera_angle_x,camparams.camera_angle_y);
    float drone_im_half_size = world2im_sizef(current_drone_pos - cv::Point3f(dparams.radius,0,0),current_drone_pos + cv::Point3f(dparams.radius,0,0),Qfi,camparams.camera_angle_x,camparams.camera_angle_y)/4;

    generated_world_pos = current_drone_pos;
    generated_im_pos = drone_im_pos;
    generated_im_size = 2*drone_im_half_size;

    std::cout << "Generated drone world pos:" << generated_world_pos << " im: " << generated_im_pos << " size: " << generated_im_size << std::endl;

    drone_im_half_size*=3.f; // the circle drawn in the template is 1/3 the size of the template to give room for the gaussian filter. Also, the blurring makes the circle smaller (when tresholded)
    cv::Mat frameL_buf = frame_bkg.clone();
    cv::Mat frameR_buf = frame_bkg.clone();

    if (drone_im_pos.x+drone_im_half_size < IMG_W && drone_im_pos.x-drone_im_pos.z-drone_im_half_size >= 0 && drone_im_pos.y+drone_im_half_size<IMG_H && drone_im_pos.y-drone_im_half_size >= 0) {
        float upscalef = 30;

        int x_ref = floorf(drone_im_pos.x-drone_im_half_size - drone_im_pos.z);
        int x_ref_up = x_ref*upscalef;
        float x_offset_up = ((drone_im_pos.x-drone_im_half_size - drone_im_pos.z) * upscalef) - x_ref_up;
        int x_offset = ceilf(x_offset_up / upscalef);
        int x_range = ceilf(drone_im_half_size*2.f) + ceilf(drone_im_pos.z) + x_offset;

        cv::Rect target_rect = cv::Rect(x_ref,floorf(drone_im_pos.y-drone_im_half_size),x_range,ceilf(drone_im_half_size*2.f));

        int patch_size_upscaled  = target_rect.height*upscalef;

        cv::Size roi_size_upscaled(target_rect.width*upscalef,target_rect.height*upscalef);
        cv::Size roi_size(target_rect.width,target_rect.height);

        cv::Rect dispL_rect_upscaled = cv::Rect(roundf(x_offset_up+drone_im_pos.z*upscalef),0,patch_size_upscaled,patch_size_upscaled);
        cv::Rect dispR_rect_upscaled = cv::Rect(roundf(x_offset_up),0,patch_size_upscaled,patch_size_upscaled);

        cv::Mat im_roiL_upscaled = cv::Mat::zeros(roi_size_upscaled,CV_8UC1);
        cv::Mat im_roiR_upscaled = im_roiL_upscaled.clone();

        cv::Mat im_roiL_upscaled_disp = im_roiL_upscaled(dispL_rect_upscaled);
        cv::Mat im_roiR_upscaled_disp = im_roiR_upscaled(dispR_rect_upscaled);

        cv::Mat circ_template_resized;
        if (_rc->LED_drone())
            cv::resize(circ_template_light,circ_template_resized,cv::Size(patch_size_upscaled,patch_size_upscaled),0,0,cv::INTER_NEAREST);
        else
            cv::resize(circ_template_dark,circ_template_resized,cv::Size(patch_size_upscaled,patch_size_upscaled),0,0,cv::INTER_NEAREST);

        circ_template_resized.copyTo(im_roiL_upscaled_disp);
        circ_template_resized.copyTo(im_roiR_upscaled_disp);

        cv::Mat im_roiL_resized,im_roiR_resized;
        cv::resize(im_roiL_upscaled,im_roiL_resized,roi_size,0,0,cv::INTER_NEAREST);
        cv::resize(im_roiR_upscaled,im_roiR_resized,roi_size,0,0,cv::INTER_NEAREST);

        cv::Mat im_roiL_resized_gt0;
        cv::Mat im_roiR_resized_gt0;
        cv::compare(im_roiL_resized,cv::Scalar(0),im_roiL_resized_gt0,cv::CMP_GT);
        cv::compare(im_roiR_resized,cv::Scalar(0),im_roiR_resized_gt0,cv::CMP_GT);
        cv::bitwise_and(im_roiL_resized,im_roiL_resized,frameL_buf(target_rect),im_roiL_resized_gt0);
        cv::bitwise_and(im_roiR_resized,im_roiR_resized,frameR_buf(target_rect),im_roiR_resized_gt0);
    }

    cv::Mat frameL,frameR;
    frameL_buf.copyTo(frameL);
    frameR_buf.copyTo(frameR);
    _frame_number++;
    _frame_time = _frame_number / static_cast<float>(pparams.fps);
    StereoPair * sp = new StereoPair(frameL,frameR,_frame_number,_frame_time);
    _current = sp;
    buf.insert(std::pair(_frame_number,sp));
    delete_old_frames();

    if (!turbo) {
        while(swc.Read() < (1.f/pparams.fps)*1e3f) {
            usleep(10);
        }
        swc.Restart();
    }
    return _current;
}

void GeneratorCam::close () {
    frame_bkg.release();
    circ_template_dark.release();
    circ_template_light.release();
    Qfi.release();
    Cam::close();
}
