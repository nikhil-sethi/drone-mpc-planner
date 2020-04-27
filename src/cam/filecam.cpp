#include "filecam.h"
#include <iostream>
#include <string.h>
#include <unistd.h>       //usleep

#include "opencv2/imgproc/imgproc.hpp"
#include "third_party/stopwatch.h"
#include <experimental/filesystem>

#include "stopwatch.h"
static stopwatch_c swc;

void FileCam::init () {
    if (!file_exist(video_fn)) {
        std::stringstream serr;
        serr << "cannot not find " << video_fn;
        throw my_exit(serr.str());
    }
    std::cout << "Reading video from " << video_fn << std::endl;
    video = cv::VideoCapture(video_fn);

    if (!video.isOpened()) {
        throw my_exit("Error opening video file!");
    } else {
        im_width = static_cast<int>(video.get(CV_CAP_PROP_FRAME_WIDTH));
        im_height = static_cast<int>(video.get(CV_CAP_PROP_FRAME_HEIGHT));
        nFrames = static_cast<int>(video.get(CV_CAP_PROP_FRAME_COUNT));
        video.set(CV_CAP_PROP_POS_FRAMES,0);
        std::cout << "Opened filecam, nFrames: " << nFrames << std::endl;
    }

    if (file_exist(calib_rfn))
        camparams.deserialize(calib_rfn);
    else
        camparams.deserialize(calib_template_rfn);
    calibration();

    if (!file_exist(depth_map_rfn)) {
        //todo: make gui warning of this:
        std::cout << "Warning: could not find " << depth_map_rfn << std::endl;
        depth_background = cv::Mat::ones(IMG_H,IMG_W,CV_16UC1);
        depth_background = 10000; // basically disable the depth background map if it is not found
    } else {
        depth_background = cv::imread(depth_map_rfn,CV_LOAD_IMAGE_ANYDEPTH);
    }

    convert_depth_background_to_world();

    camera_volume = def_volume();
    read_frame_ids();
    swc.Start();
    update();
    video.release(); // reset after reading a frame, because this frame was not saved in the log in the first place
    video = cv::VideoCapture(video_fn);
    frame_cnt = 0;

    initialized = true;
}

void FileCam::read_frame_ids() {
    string framesfile = replay_dir + "/frames.csv";
    ifstream infile(framesfile);
    string line;
    while (getline(infile, line)) {
        try {
            auto data = logging::split_csv_line(line);
            frame_id_entry entry;
            entry.raw_video_frame_counter = stoi(data.at(0));
            entry.imgcount = stoi(data.at(1));
            entry.RS_id = stol(data.at(2));
            entry.time = stod(data.at(3));
            frames.push_back(entry);
        } catch (exception& exp ) {
            throw my_exit("Could not read log! File: " +framesfile + '\n' + "Line: " + string(exp.what()) + " at: " + line);
        }
    }
}

void FileCam::calibration() {
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
}

void FileCam::update() {
    cv::Mat frameLR;
    for (uint i = 0; i < replay_skip_n_frames+1; i++) {
        video >> frameLR;
        frame_cnt++;
    }
    replay_skip_n_frames = 0;
    if (frameLR.empty()) {
        std::cout << "Video end, exiting" << std::endl;
        throw bag_video_ended();
    }
    cvtColor(frameLR,frameLR,CV_BGR2GRAY);
    frameL = frameLR(cv::Rect(cv::Point(0,0),cv::Point(frameLR.cols/2,frameLR.rows)));
    frameR = frameLR(cv::Rect(cv::Point(frameLR.cols/2,0),cv::Point(frameLR.cols,frameLR.rows)));
    _frame_number = frames.at(frame_cnt-1).RS_id;
    _frame_time = frames.at(frame_cnt-1).time;

    if (_frame_number==ULONG_MAX) {
        std::cout << "Log end, exiting. Video frames left: " << nFrames - frame_cnt << std::endl;
        throw bag_video_ended();
    }

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

void FileCam::close () {
    video.release();
}
