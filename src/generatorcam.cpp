#include "generatorcam.h"
#include <iostream>
#include <string.h>
#include <unistd.h>       //usleep

#include "opencv2/imgproc.hpp"

bool GeneratorCam::init (int argc __attribute__((unused)), char **argv __attribute__((unused))) {
    camRunning = true;
    float focal_length = 425.680267; // same as fy
    float cx = 419.639923; // same for both cameras
    float cy = 235.088562;
    float baseline = 0.0499379635;
    Qf = (cv::Mat_<double>(4, 4) << 1.0, 0.0, 0.0, -cx, 0.0, 1.0, 0.0, -cy, 0.0, 0.0, 0.0, focal_length, 0.0, 0.0, 1/baseline, 0.0);
    _mode = cam_mode_stereo;

    drone.x = DRONE_IM_X_START*IMSCALEF;
    drone.y = DRONE_IM_Y_START*IMSCALEF;
    insect.x = 0;
    insect.y = 0;

    return false;
}

void GeneratorCam::switch_mode(cam_mode_enum mode){
    if (mode != _mode) {
        if (mode == cam_mode_disabled )
            go_disabled();
        else if (mode == cam_mode_color)
            go_color();
        else if (mode == cam_mode_stereo)
            go_stereo();
    }
}
void GeneratorCam::go_color() {
    _mode = cam_mode_color;
}

void GeneratorCam::go_stereo() {
    _mode = cam_mode_stereo;
}

void GeneratorCam::go_disabled() {
    _mode = cam_mode_disabled;
}


void GeneratorCam::close () {
    camRunning = false;
}

void GeneratorCam::update() {
    if (_mode == cam_mode_color) {
        generateRGB();
        frame_id_rgb++;
    } else if(_mode == cam_mode_stereo) {
        generateStereo();
        frame_id_stereo++;
    }
    frame_id++;

    float delay = VIDEOFPS;
    delay = 1/delay;
    delay *=1e6f;
    usleep(delay);
}

void GeneratorCam::generateStereo() {

    cv::Mat resL = cv::Mat::zeros(IMG_H,IMG_W,CV_8UC1);
    cv::Mat resR = cv::Mat::zeros(IMG_H,IMG_W,CV_8UC1);

    frameL = resL;
    frameR = resR;

//    static bool bounce = -1;
//    drone.x = (drone.x + bounce ) % (IMG_W-3);
//    drone.y = (drone.y + bounce ) % (IMG_H-3);
    drone.x = DRONE_IM_X_START*IMSCALEF + sinf((float)(frame_id_stereo-2)/1000.f)*100 ;
    drone.y = DRONE_IM_Y_START*IMSCALEF + cosf((float)(frame_id_stereo-2)/1000.f)*100 - 100;

    std::cout << "Generated drone location: [" << drone.x/IMSCALEF << "," << drone.y/IMSCALEF << "]" << std::endl;

    insect.x = (insect.x + 1 ) % (IMG_W-1);
    insect.y = (insect.y + 1 ) % (IMG_H-1);

    bound(&drone,2);
    bound(&insect,1);
    if (frame_id_stereo > 2) { // circumvent background calib
        cv::circle(resL,drone,2,255,2);
        // cv::circle(resL,insect,1,200,1);
    }
}

void GeneratorCam::bound(cv::Point * p,int radius){
    if (p->x > IMG_W-radius)
        p->x = IMG_W-radius;
    else if (p->x-radius < 0)
        p->x = radius;
    if (p->y > IMG_H-radius)
        p->y = IMG_H-radius;
    else if (p->y-radius < 0)
        p->y = radius;
}

void GeneratorCam::generateRGB() {

}

void GeneratorCam::nextFrame() {
    frame_id++;
}
