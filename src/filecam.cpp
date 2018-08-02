#include "filecam.h"
#include <iostream>
#include <string.h>
#include <unistd.h>       //usleep


bool FileCam::init (int argc __attribute__((unused)), char **argv __attribute__((unused))) {
    file_rgb = std::string(argv[1]) + "videoRawRGB.avi";
    file_stereo = std::string(argv[1]) + "videoRawL.avi";

    videoLength_rgb = 999999;
    video_rgb = cv::VideoCapture(file_rgb);
    video_stereo = cv::VideoCapture(file_stereo);

    if (!video_rgb.isOpened()) {
        std::cerr << "Error opening video file!\n";
        return true;
    } else {
        im_width_rgb = (int) video_rgb.get(CV_CAP_PROP_FRAME_WIDTH);
        im_height_rgb = (int)video_rgb.get(CV_CAP_PROP_FRAME_HEIGHT);
        nFrames_rgb = (int) video_rgb.get(CV_CAP_PROP_FRAME_COUNT);
        video_rgb.set(CV_CAP_PROP_POS_FRAMES,0);
        if (videoLength_rgb > nFrames_rgb) {
            //videoLength = nFrames;
        }
        std::cout << "Opened filecam, nFrames: " << nFrames_rgb << std::endl;
        frame_id_rgb=0;

        //skip start
        for (int i =0; i < skipstart;i++) {
            video_rgb >> frame_rgb;
            frame_id_rgb++;
        }
        camRunning = true;
        return false;
    }

    float focal_length = 425.680267; // same as fy
    float cx = 419.639923; // same for both cameras
    float cy = 235.088562;
    float baseline = 0.0499379635;
    Qf = (cv::Mat_<double>(4, 4) << 1.0, 0.0, 0.0, -cx, 0.0, 1.0, 0.0, -cy, 0.0, 0.0, 0.0, focal_length, 0.0, 0.0, 1/baseline, 0.0);

}

void FileCam::switch_mode(cam_mode_enum mode){
    if (mode != _mode) {
        if (mode == cam_mode_disabled )
            go_disabled();
        else if (mode == cam_mode_color)
            go_color();
        else if (mode == cam_mode_stereo)
            go_stereo();
    }
}
void FileCam::go_color() {
    _mode = cam_mode_color;
}

void FileCam::go_stereo() {
    _mode = cam_mode_stereo;
}

void FileCam::go_disabled() {
    _mode = cam_mode_disabled;
}


void FileCam::close () {
    camRunning = false;
    video_rgb.release();
}

void FileCam::update() {
    static int tmpcnt = frame_id;
    if (_mode == cam_mode_color) {
        if (tmpcnt != frame_id) {
            video_rgb >> frame_rgb;
            tmpcnt = frame_id;
        }
        frame_id_rgb++;
        if (frame_rgb.empty() || frame_id_rgb >= videoLength_rgb)
            camRunning=false;
    } else if(_mode == cam_mode_stereo) {
        video_stereo >> frameL;
        frame_id_stereo++;
        if (frameL.empty() || frame_id_stereo >= videoLength_stereo)
            camRunning=false;
    }
    frame_id++;

    float delay = VIDEOFPS;
    delay = 1/delay;
    delay *=1e6f;
    usleep(delay);
}

void FileCam::nextFrame() {
    frame_id++;
}
