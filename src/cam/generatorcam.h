#pragma once
#include <opencv2/highgui/highgui.hpp>
#include "common.h"
#include "cam.h"
#include "multimodule.h"

class GeneratorCam : public Cam {
private:

    MultiModule * _rc;
    cv::Point3f drone_start_pos;
    cv::Mat Qfi;
    void calibration();

    cv::Mat circ_template_light;
    cv::Mat circ_template_dark;
    cv::Mat frame_bkg;

    double takeoff_start_time=-1;
    cv::Point3f current_drone_pos;

public:

    GeneratorCam() {
        calib_wfn = "./logging/" + calib_rfn;
        calib_rfn = "../../xml/generator_cam_calib.xml";
        depth_map_rfn = "./logging/" + depth_map_rfn;
        depth_unfiltered_map_rfn = "./logging/" + depth_unfiltered_map_rfn;
        disparity_map_rfn = "./logging/" + disparity_map_rfn;
        brightness_map_rfn = "./logging/" + brightness_map_rfn;
        depth_map_wfn = depth_map_rfn;
        depth_unfiltered_map_wfn = depth_unfiltered_map_rfn;
        disparity_map_wfn = disparity_map_rfn;
        brightness_map_wfn = brightness_map_rfn;

        drone_start_pos = cv::Point3f(0,-1.3,-1.5);
        current_drone_pos = drone_start_pos;
    }

    cv::Point3f generated_world_pos = {0};
    float generated_world_size = 0 ;

    cv::Point3f generated_im_pos = {0};
    float generated_im_size = 0;

    void init();
    void rc(MultiModule * rc) {_rc = rc;}
    void close();
    void update();

    void back_one_sec() {}
};
