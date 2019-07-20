#ifndef VIZDAT_H
#define VIZDAT_H

#include <fstream>
#include <vector>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/imgproc.hpp>

#include <cereal/types/unordered_map.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/archives/binary.hpp>
#include <fstream>

#include <condition_variable>

#include "cam.h"
#include "defines.h"
#include "common.h"

class VisionData{

private:

    int motion_update_iterator = 0;
    bool _calibrating_background = false;
    double calibrating_background_end_time = 0;
    int skip_background_frames = 0;
    std::mutex lock_data;

    struct BaseVisionSettings{

        int motion_update_iterator_max = 4;
        float brightness_event_tresh = 5;
        double brightness_check_period = 1;

        float version = 1.35f;

        template <class Archive>
        void serialize( Archive & ar )
        {
            ar(version, motion_update_iterator_max,
               brightness_event_tresh,brightness_check_period);
        }
    };

    std::string motion_noise_map_wfn = "max_motion_noise.png";
    const std::string settingsFile = "../basevisionsettings.dat";
    BaseVisionSettings settings;

    cv::Mat diffL16;
    cv::Mat frameL16;

    cv::Mat diffR16;
    cv::Mat frameR16;

    cv::Mat diffL16_back;
    double _current_frame_time = 0;

    double prev_time_brightness_check = 0;
    float prev_brightness;
    bool _reset_motion_integration = false;
    cv::Point delete_motion_spot = {0};
    int delete_motion_r = 0;
    int delete_motion_frame_cnt_duration = 0;

    bool _exclude_drone_from_motion_fading = false;
    cv::Point exclude_drone_from_motion_fading_spot = {0};
    int exclude_drone_from_motion_fading_r = 0;

    bool enable_viz_diff = false;

    bool initialized = false;

    void collect_no_drone_frames(cv::Mat dL);
    void track_avg_brightness(cv::Mat frame, double time);

    void fade(cv::Mat diff16, bool exclude_drone);

public:
    cv::Mat frameL,frameR;
    cv::Mat frameL_prev,frameR_prev;
    cv::Mat motion_noise_map;
    cv::Mat diffL,diffR,diffL_small,diffR_small;

    cv::Mat viz_frame;

    uint frame_id;
    cv::Size smallsize;
    cv::Mat Qf;
    float camera_angle;
    float camera_gain;
    cv::Mat depth_background;
    cv::Mat disparity_background;
    cv::Mat depth_background_mm;

    bool disable_fading = false;
    float current_time() {return _current_frame_time;}

    void init(bool fromfile, cv::Mat new_Qf, cv::Mat new_frameL, cv::Mat new_frameR, float new_camera_angle, float new_camera_gain, cv::Mat new_depth_background_mm);
    void close() {
        if (initialized){
            std::cout << "Closing visdat" << std::endl;
            std::ofstream os(settingsFile, std::ios::binary);
            cereal::BinaryOutputArchive archive( os );
            archive( settings );
            initialized = false;
        }
    }
    void update(cv::Mat new_frameL, cv::Mat new_frameR, double time, unsigned long long new_frame_id);
    void reset_motion_integration() {
        _reset_motion_integration = true;
    }
    void enable_background_motion_map_calibration(double duration);
    bool calibrating_background() {return _calibrating_background;}

    void delete_from_motion_map(cv::Point p, int radius, int duration);
    void exclude_drone_from_motion_fading(cv::Point p, int radius);
};

#endif // VIZDAT_H
