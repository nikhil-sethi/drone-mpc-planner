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
    bool _background_calibrated;
    std::mutex lock_data;

    struct BaseVisionSettings{

        int uncertainty_multiplier = 2;
        int uncertainty_power = 6;
        int uncertainty_background = 0.3*255.0;
        int background_calib_time = 5;
        int motion_update_iterator_max = 1;
        float brightness_event_tresh = 5;
        float brightness_check_period = 1;

        float version = 1.2f;

        template <class Archive>
        void serialize( Archive & ar )
        {
            ar(version, uncertainty_power,uncertainty_multiplier,
               uncertainty_background,background_calib_time,motion_update_iterator_max,
               brightness_event_tresh,brightness_check_period);
        }
    };

    const std::string settingsFile = "../basevisionsettings.dat";
    BaseVisionSettings settings;

    cv::Mat diffL16;
    cv::Mat frameL16;
    cv::Mat frameL_prev16;

    cv::Mat diffR16;
    cv::Mat frameR16;
    cv::Mat frameR_prev16;

    float prev_time_brightness_check = 0;
    float prev_brightness;
    bool _reset_motion_integration = false;

    void collect_no_drone_frames(cv::Mat diff);
    void track_avg_brightness(cv::Mat frame,float time);

    void fade(cv::Mat diff16);

public:
    cv::Mat frameL,frameR;
    cv::Mat frameL_prev,frameR_prev;
    cv::Mat uncertainty_map;
    cv::Mat max_uncertainty_map; // IMSCALEF smaller than original, CV8UC1 max motion background map
    cv::Mat diffL,diffR,diffL_small,diffR_small;

    int frame_id;
    cv::Size smallsize;
    cv::Mat Qf;
    float camera_angle;
    cv::Mat depth_background;
    cv::Mat disparity_background;
    cv::Mat depth_background_mm;

    void init(cv::Mat new_Qf, cv::Mat new_frameL, cv::Mat new_frameR, float new_camera_angle, cv::Mat new_depth_background_mm);
    void close() {
        std::ofstream os(settingsFile, std::ios::binary);
        cereal::BinaryOutputArchive archive( os );
        archive( settings );
    }
    void update(cv::Mat new_frameL, cv::Mat new_frameR, float time, int new_frame_id);
    float uncertainty_background() {return settings.uncertainty_background / 255.0f;}
    void reset_motion_integration() {
        _reset_motion_integration = true;
    }
    bool background_calibrated() {return _background_calibrated;}
};

#endif // VIZDAT_H
