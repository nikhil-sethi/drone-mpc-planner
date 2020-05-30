#pragma once
#include "common.h"
#include "cameraview.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <unistd.h>
#include <thread>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API


#define IMG_W 848
#define IMG_H 480

class Cam {
protected:
    uint replay_skip_n_frames = 0;
    unsigned long long _frame_number = 0;
    double _frame_time = 0;
    bool initialized = false;

    //read file names
    std::string calib_rfn = "cam_calib.xml";
    const std::string calib_template_rfn = "../../xml/" + calib_rfn;
    std::string depth_map_rfn = "depth_filtered.png";
    std::string depth_unfiltered_map_rfn = "depth.png";
    std::string disparity_map_rfn = "disparity.png";
    std::string brightness_map_rfn = "brightness.png";

    //write file names:
    std::string calib_wfn;
    std::string depth_map_wfn;
    std::string depth_unfiltered_map_wfn;
    std::string disparity_map_wfn;
    std::string brightness_map_wfn;

    rs2_intrinsics * intr;
    xmls::CamCalibrationData camparams;

    void set_file_paths(std::string replay_dir);
    void convert_depth_background_to_world();
    void def_volume();
    cv::Point3f get_SlopesOfPixel(uint x, uint y);

public:
    bool frame_by_frame = false;
    bool turbo = false;
    cv::Mat Qf;
    cv::Mat frameL,frameR;
    cv::Mat depth_background;
    cv::Mat depth_background_3mm;
    cv::Mat depth_background_3mm_world;
    cv::Mat depth_background_mm;
    cv::Mat disparity_background;
    CameraView camera_volume;

    virtual void init() = 0;
    virtual void close() {
        delete intr;
        Qf.release();
        frameL.release();
        frameR.release();
        depth_background.release();
        depth_background_3mm.release();
        depth_background_3mm_world.release();
        depth_background_mm.release();
        disparity_background.release();
        camera_volume.release();
    }
    virtual void update() = 0;
    void skip(float duration) {
        replay_skip_n_frames+=roundf(pparams.fps*duration);
    }
    void skip_one_sec() {
        replay_skip_n_frames+=pparams.fps;
    }
    virtual void back_one_sec() =0;
    float camera_angle() { return camparams.camera_angle_y; }
    float measured_exposure() { return camparams.measured_exposure; }
    float measured_gain() { return camparams.measured_gain; }
    unsigned long long frame_number() {return _frame_number;}
    double frame_time() {return _frame_time;}
};
