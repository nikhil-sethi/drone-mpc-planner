#ifndef VIZDAT_H
#define VIZDAT_H

#include <fstream>
#include <vector>
#include <math.h>
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

    cv::Mat avg_prev_frame;
    int n_avg_prev_frames = 0;
    int motion_update_iterator = 0;
    bool background_calibrated;
    std::mutex lock_data;

    struct BaseVisionSettings{

        int uncertainty_multiplier = 2;
        int uncertainty_power = 6;
        int uncertainty_background = 0.3*255.0;
        int background_calib_time = 5;
        int motion_update_iterator_max = 10;

        float version = 1.1f;

        template <class Archive>
        void serialize( Archive & ar )
        {
            ar(version, uncertainty_power,uncertainty_multiplier,uncertainty_background,background_calib_time,motion_update_iterator_max);
        }
    };

    const std::string settingsFile = "../basevisionsettings.dat";
    BaseVisionSettings settings;

    cv::Mat threshL,diffL16,diffL;
    cv::Mat frameL16;
    cv::Mat frameL_prev16;

    void init_avg_prev_frame(void);
    void collect_avg_prev_frame(cv::Mat frame);
    void collect_no_drone_frames(cv::Mat diff);

public:
    cv::Mat frameL,frameR;
    cv::Mat frameL_prev,frameR_prev;
    cv::Mat uncertainty_map;
    cv::Mat diffL_small;

    int frame_id;
    cv::Size smallsize;
    cv::Mat Qf;

    void init(cv::Mat new_Qf, cv::Mat new_frameL, cv::Mat new_frameR);
    void close() {
        std::ofstream os(settingsFile, std::ios::binary);
        cereal::BinaryOutputArchive archive( os );
        archive( settings );
    }
    void update(cv::Mat new_frameL, cv::Mat new_frameR, float time, int new_frame_id);
    float uncertainty_background() {return settings.uncertainty_background / 255.0;}
};

#endif // VIZDAT_H
