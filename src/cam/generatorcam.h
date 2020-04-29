#pragma once
#include "defines.h"
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

    double takeoff_start_time=-1;
    cv::Point3f current_drone_pos;

public:

    GeneratorCam() {
        calib_rfn = "../../xml/generator_cam_calib.xml";
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
