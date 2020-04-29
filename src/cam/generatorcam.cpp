#include "generatorcam.h"
#include <iostream>
#include <string.h>
#include <unistd.h>       //usleep

#include "opencv2/imgproc/imgproc.hpp"



#include "stopwatch.h"
static stopwatch_c swc;

void GeneratorCam::init () {
    camparams.deserialize(calib_rfn);

    calibration();

    depth_background = cv::Mat::ones(IMG_H,IMG_W,CV_16UC1);
    depth_background = 10000; // basically disable the depth background map if it is not found

    convert_depth_background_to_world();

    camera_volume = def_volume();

    cv::Mat clt = cv::Mat::zeros(5000,5000,CV_8UC1) + 128;
    cv::Mat cdt = clt.clone();
    int cirk_size = 150;

    cv::circle(clt,cv::Point2i(clt.rows/2,clt.cols/2),cirk_size,180,CV_FILLED);
    cv::GaussianBlur(clt,clt,cv::Size(55,55),25);
    circ_template_light = clt(cv::Rect(clt.rows/2-250,clt.cols/2-250,500,500)).clone();

    cv::circle(cdt,cv::Point2i(cdt.rows/2,cdt.cols/2),cirk_size,80,CV_FILLED);
    cv::GaussianBlur(cdt,cdt,cv::Size(55,55),25);
    circ_template_dark = cdt(cv::Rect(cdt.rows/2-250,cdt.cols/2-250,500,500)).clone();

    // cv::imshow("crk",circ_template_light );

    swc.Start();
    update();

    initialized = true;
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

void GeneratorCam::update() {

    if (_rc->throttle > JOY_BOUND_MIN || takeoff_start_time > 0) {
        if (takeoff_start_time<0)
            takeoff_start_time = _frame_time;
        float dt_takeoff = static_cast<float>(_frame_time - takeoff_start_time) - dparams.full_bat_and_throttle_spinup_duration;
        if (dt_takeoff < 0.2f && dt_takeoff>0)
            current_drone_pos =drone_start_pos+cv::Point3f(0,0.5f*dparams.thrust*powf(dt_takeoff,2),0);
        else if (dt_takeoff>0) { //do some flight:
            current_drone_pos = drone_start_pos  + cv::Point3f(0,0,cosf(dt_takeoff)/M_PIf32*2.f);
            // current_drone_pos = cv::Point3f(0, -1.3, -2.06449);
        }
    }

    auto drone_im_pos = world2im_3d(current_drone_pos,Qfi,camparams.camera_angle_y);
    float drone_im_half_size = world2im_sizef(current_drone_pos - cv::Point3f(dparams.radius,0,0),current_drone_pos + cv::Point3f(dparams.radius,0,0),Qfi,camparams.camera_angle_y)/4;

    generated_world_pos = current_drone_pos;
    generated_im_pos = drone_im_pos;
    generated_im_size = 2*drone_im_half_size;

    std::cout << "Generated drone world pos:" << generated_world_pos << " im: " << generated_im_pos << " size: " << generated_im_size << std::endl;

    drone_im_half_size*=3.f; // the circle drawn in the template is 1/3 the size of the template to give room for the gaussian filter. Also, the blurring makes the circle smaller (when tresholded)
    cv::Mat frameL_buf = cv::Mat::zeros(cv::Size(IMG_W,IMG_H),CV_8UC1)+128;
    cv::Mat frameR_buf = cv::Mat::zeros(cv::Size(IMG_W,IMG_H),CV_8UC1)+128;

    if (drone_im_pos.x+drone_im_half_size < IMG_W && drone_im_pos.x-drone_im_pos.z-drone_im_half_size >= 0 && drone_im_pos.y+drone_im_half_size<IMG_H && drone_im_pos.y-drone_im_half_size >= 0) {

        float upscalef = 10;
        int up_patch_size = roundf(drone_im_half_size*2*upscalef);
        cv::Mat im_roiL = cv::Mat::zeros(cv::Size(ceilf((drone_im_half_size*2+drone_im_pos.z)*upscalef),up_patch_size),CV_8UC1)+128;
        cv::Mat im_roiR = im_roiL.clone();
        cv::Mat im_roiL_disp = im_roiL(cv::Rect(roundf(drone_im_pos.z*upscalef),0,up_patch_size,im_roiL.rows));
        cv::Mat im_roiR_disp = im_roiR(cv::Rect(0,0,up_patch_size,im_roiR.rows));

        if (_rc->LED_drone()) {
            // cv::circle(im_roiL,cv::Point2i(roundf((drone_im_size+drone_im_pos.z)*upscalef),im_roiL.rows/2),roundf(drone_im_size*upscalef),180,CV_FILLED);
            // cv::circle(im_roiL_disp,cv::Point2i(roundf(drone_im_size*upscalef),roundf(drone_im_size*upscalef)),roundf(drone_im_size*upscalef),180,CV_FILLED);
            // cv::circle(im_roiR,cv::Point2i(roundf(drone_im_size*upscalef),roundf(drone_im_size*upscalef)),roundf(drone_im_size*upscalef),180,CV_FILLED);
            cv::Mat circ_template_resized;
            cv::resize(circ_template_light,circ_template_resized,cv::Size(up_patch_size,up_patch_size),0,0,CV_INTER_AREA);

            cv::circle(circ_template_resized,cv::Point2i(circ_template_resized.cols/2,circ_template_resized.rows/2),1,0,CV_FILLED);
            circ_template_resized.copyTo(im_roiL_disp);
            circ_template_resized.copyTo(im_roiR_disp);
            // cv::resize(circ_template_light,im_roiR,cv::Size(up_patch_size,up_patch_size),0,0,CV_INTER_AREA);
            cv::imwrite("patch_L.png",im_roiL);
            cv::imwrite("patch_R.png",im_roiR);
            // cv::circle(im_roiL_disp,cv::Point2i(roundf(drone_im_half_size*upscalef),roundf(drone_im_half_size*upscalef)),5,255,CV_FILLED);
            // cv::circle(im_roiR,cv::Point2i(roundf(drone_im_half_size*upscalef),roundf(drone_im_half_size*upscalef)),5,255,CV_FILLED);
        } else {
            cv::resize(circ_template_dark,im_roiL_disp,cv::Size(up_patch_size,up_patch_size),0,0,CV_INTER_AREA);
            cv::resize(circ_template_dark,im_roiR,cv::Size(up_patch_size,up_patch_size),0,0,CV_INTER_AREA);
        }
        cv::resize(im_roiL,frameL_buf(cv::Rect(roundf(drone_im_pos.x-drone_im_pos.z-drone_im_half_size),roundf(drone_im_pos.y-drone_im_half_size),im_roiL.cols/upscalef,im_roiL.rows/upscalef)),cv::Size(im_roiL.cols/upscalef,im_roiL.rows/upscalef),0,0,CV_INTER_AREA);
        cv::resize(im_roiR,frameR_buf(cv::Rect(roundf(drone_im_pos.x-drone_im_pos.z-drone_im_half_size),roundf(drone_im_pos.y-drone_im_half_size),im_roiR.cols/upscalef,im_roiR.rows/upscalef)),cv::Size(im_roiR.cols/upscalef,im_roiR.rows/upscalef),0,0,CV_INTER_AREA);
    }



    frameL = frameL_buf.clone();
    frameR = frameR_buf.clone();
    _frame_number++;
    _frame_time = _frame_number / static_cast<float>(pparams.fps);

    if (!turbo) {
        while(swc.Read() < (1.f/pparams.fps)*1e3f) {
            usleep(10);
        }
        swc.Restart();
    }

    while(frame_by_frame) {
        unsigned char k = cv::waitKey(1);
        if (k == 'f')
            break;
        else if (k== ' ') {
            frame_by_frame = false;
            break;
        }
    }
}

void GeneratorCam::close () {

}
