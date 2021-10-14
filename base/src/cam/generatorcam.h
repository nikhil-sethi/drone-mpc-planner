#pragma once
#include <opencv2/highgui/highgui.hpp>
#include "common.h"
#include "cam.h"
#include "rc.h"

class GeneratorCam : public Cam {
private:

    unsigned long long _frame_number = 0;
    double _frame_time = 0;

    Rc *_rc;
    cv::Point3f drone_start_pos;
    cv::Mat Qfi;
    void calibration();

    cv::Mat circ_template_light;
    cv::Mat circ_template_dark;
    cv::Mat frame_bkg;

    double takeoff_start_time = -1;
    cv::Point3f current_drone_pos;

public:

    GeneratorCam() {
        drone_start_pos = cv::Point3f(0, -1.3, -1.5);
        current_drone_pos = drone_start_pos;
    }

    cv::Point3f generated_world_pos = {0};
    float generated_world_size = 0 ;

    cv::Point3f generated_im_pos = {0};
    float generated_im_size = 0;

    void init();
    void rc(Rc *rc) {_rc = rc;}
    void close();
    StereoPair *update();
};
