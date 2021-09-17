#pragma once
#include "common.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <unistd.h>
#include <thread>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

struct ViewLimit {
    cv::Point3f point_left_top;
    cv::Point3f point_right_top;
    cv::Point3f point_left_bottom;
    cv::Point3f point_right_bottom;

    ViewLimit() {}
    ViewLimit(cv::Point3f plt, cv::Point3f prt, cv::Point3f plb, cv::Point3f prb) : point_left_top(plt),point_right_top(prt), point_left_bottom(plb),point_right_bottom(prb)  {}
};

class Cam {
protected:
    int _id = 0;
    uint replay_skip_n_frames = 0;
    bool initialized = false;
    bool _stop = false;
    int _frame_loss_cnt = 0;
    std::map<unsigned long long,StereoPair *> buf;
    StereoPair * _current;
    StereoPair * _last;

    //read file names
    std::string calib_rfn;
    std::string rgb_rfn;
    const std::string calib_template_rfn;
    std::string depth_map_rfn;
    std::string depth_unfiltered_map_rfn;
    std::string disparity_map_rfn;
    std::string brightness_map_rfn;

    //write file names:
    std::string calib_wfn;
    std::string depth_map_wfn;
    std::string depth_unfiltered_map_wfn;
    std::string disparity_map_wfn;

    rs2_intrinsics * intr;
    xmls::CamCalibration camparams;

    void set_read_file_paths(std::string replay_dir);
    void set_write_file_paths(std::string output_dir);
    void convert_depth_background_to_world();
    virtual void delete_old_frames();
    virtual void delete_all_frames();

public:
    bool frame_by_frame = false;
    bool turbo = true;
    cv::Mat Qf;
    cv::Mat depth_background;
    cv::Mat depth_background_3mm;
    cv::Mat depth_background_3mm_world;
    cv::Mat depth_background_mm;
    cv::Mat disparity_background;

    virtual void init() = 0;
    virtual void close() {
        delete intr;
        Qf.release();
        depth_background.release();
        depth_background_3mm.release();
        depth_background_3mm_world.release();
        depth_background_mm.release();
        disparity_background.release();
        delete_all_frames();
    }
    void stop() {_stop = true;}
    virtual StereoPair * update() = 0;
    float camera_pitch() { return camparams.camera_angle_y; }
    float camera_roll() { return camparams.camera_angle_x; }
    ViewLimit view_limits ();

    float measured_exposure() { return camparams.measured_exposure; }
    float measured_gain() { return camparams.measured_gain; }

    StereoPair * current() { return _current; }
    StereoPair * frame(unsigned long long rs_id) {return buf.at(rs_id);}

    int frame_loss_cnt() {return _frame_loss_cnt;}
};
