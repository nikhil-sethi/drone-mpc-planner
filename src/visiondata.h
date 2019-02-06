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

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

class VisionData{

private:

    cv::Mat avg_prev_frame;
    int n_avg_prev_frames = 0;
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

    cv::Mat threshL,diffL16,diffL;
    cv::Mat frameL16;
    cv::Mat frameL_prev16;

    float prev_time_brightness_check = 0;
    float prev_brightness;
    bool _reset_motion_integration = false;

    void init_avg_prev_frame(void);
    void collect_avg_prev_frame(cv::Mat frame);
    void collect_no_drone_frames(cv::Mat diff);
    void track_avg_brightness(cv::Mat frame,float time);

    const rs2_intrinsics * intr;

public:
    cv::Mat frameL,frameR;
    cv::Mat frameL_prev,frameR_prev;
    cv::Mat uncertainty_map;
    cv::Mat max_uncertainty_map;
    cv::Mat diffL_small;

    int frame_id;
    cv::Size smallsize;
    cv::Mat Qf;
    float camera_angle;
    float depth_scale;
    cv::Mat depth_background;
    cv::Mat disparity_background;


#if CAMMODE == CAMMODE_GENERATOR
    void init(cv::Mat new_Qf, cv::Mat new_frameL, cv::Mat new_frameR, float new_camera_angle, cv::Mat new_depth_background);
#else
    void init(cv::Mat new_Qf, cv::Mat new_frameL, cv::Mat new_frameR, float new_camera_angle, cv::Mat new_depth_background, float new_depth_scale, rs2_intrinsics *new_intr);
#endif
    void close() {
        std::ofstream os(settingsFile, std::ios::binary);
        cereal::BinaryOutputArchive archive( os );
        archive( settings );
    }
    void update(cv::Mat new_frameL, cv::Mat new_frameR, float time, int new_frame_id);
    float uncertainty_background() {return settings.uncertainty_background / 255.0f;}
    void deproject(float pixel[2],float dist, float p[3]) {
#if CAMMODE != CAMMODE_GENERATOR
        //todo remove this dependency (#39)
        rs2_deproject_pixel_to_point(p, intr, pixel, dist);
#else
        dist++; // kill warning
        p[0] = pixel[0];
        p[1] = pixel[1];
        p[2] = 10000;
#endif

    }
    void reset_motion_integration() {
        _reset_motion_integration = true;
    }
    bool background_calibrated() {return _background_calibrated;}
};

#endif // VIZDAT_H
