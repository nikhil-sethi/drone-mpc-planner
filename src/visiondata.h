#ifndef VIZDAT_H
#define VIZDAT_H

#include <fstream>
#include <vector>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d/features2d.hpp"

#include <cereal/types/unordered_map.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/archives/binary.hpp>
#include <fstream>

#include <condition_variable>

#include "cam.h"
#include "defines.h"

class VisionData{

private:

    int motion_update_iterator = 0;
    bool _calibrating_background = false;
    float calibrating_background_end_time = 0;
    int skip_background_frames = 0;
    std::mutex lock_data;

    struct BaseVisionSettings{

        int motion_update_iterator_max = 30;
        float brightness_event_tresh = 5;
        float brightness_check_period = 1;

        float version = 1.33f;

        template <class Archive>
        void serialize( Archive & ar )
        {
            ar(version, motion_update_iterator_max,
               brightness_event_tresh,brightness_check_period);
        }
    };

    std::string motion_noise_map_fn = "max_motion_noise.png";
    const std::string settingsFile = "../basevisionsettings.dat";
    BaseVisionSettings settings;

    cv::Mat diffL16;
    cv::Mat frameL16;

    cv::Mat diffR16;
    cv::Mat frameR16;

    cv::Mat diffL16_back;
    float _current_frame_time = 0;

    float prev_time_brightness_check = 0;
    float prev_brightness;
    bool _reset_motion_integration = false;
    bool delete_motion = false;
    cv::Point delete_motion_spot = {0};
    int delete_motion_r = 0;

    void collect_no_drone_frames(cv::Mat dL);
    void track_avg_brightness(cv::Mat frame,float time);

    void fade(cv::Mat diff16);

public:
    cv::Mat frameL,frameR;
    cv::Mat frameL_prev,frameR_prev;
    cv::Mat max_uncertainty_map; // IMSCALEF smaller than original, CV8UC1 max motion background map
    cv::Mat diffL,diffR,diffL_small,diffR_small;

    int frame_id;
    cv::Size smallsize;
    cv::Mat Qf;
    float camera_angle;
    float camera_gain;
    cv::Mat depth_background;
    cv::Mat disparity_background;
    cv::Mat depth_background_mm;

    float current_time() {return _current_frame_time;}

    void init(bool fromfile, string bag_dir, cv::Mat new_Qf, cv::Mat new_frameL, cv::Mat new_frameR, float new_camera_angle, float new_camera_gain, cv::Mat new_depth_background_mm);
    void close() {
        std::ofstream os(settingsFile, std::ios::binary);
        cereal::BinaryOutputArchive archive( os );
        archive( settings );
    }
    void update(cv::Mat new_frameL, cv::Mat new_frameR, float time, int new_frame_id);
    void reset_motion_integration() {
        _reset_motion_integration = true;
    }
    void enable_background_motion_map_calibration(float duration);
    bool calibrating_background() {return _calibrating_background;}

    void delete_from_motion_map(cv::Point p, int radius);
};

#endif // VIZDAT_H
