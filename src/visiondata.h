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

#include "cam.h"
#include "defines.h"

class VisionData{

private:


    cv::Mat avg_prev_frame;
    int n_avg_prev_frames = 0;
    int motion_update_iterator = 0;
    const int motion_update_iterator_max = 10;
    bool background_calibrated;

    struct BaseVisionSettings{

        int uncertainty_multiplier = 2;
        int uncertainty_power = 6;
        int uncertainty_background = 0.3*255.0;
        int background_calib_time = 5;

        float version = 1.0f;

        template <class Archive>
        void serialize( Archive & ar )
        {
            ar(version, uncertainty_power,uncertainty_multiplier,uncertainty_background,background_calib_time);
        }
    };

    const std::string settingsFile = "../basevisionsettings.dat";
    BaseVisionSettings settings;

public:
    cv::Mat frameL,frameR;
    cv::Mat frameL_prev,frameR_prev;
    int frame_id;

    cv::Size smallsize;
    cv::Mat frameL16;
    cv::Mat frameL_prev16;

    cv::Mat Qf;
    cv::Mat uncertainty_map,threshL,diffL16,diffL,diffL_small;

    void init(cv::Mat new_Qf, cv::Mat new_frameL, cv::Mat new_frameR);
    void close() {
        std::ofstream os(settingsFile, std::ios::binary);
        cereal::BinaryOutputArchive archive( os );
        archive( settings );
    }
    void update(cv::Mat new_frameL, cv::Mat new_frameR, float time, int new_frame_id);
    void init_avg_prev_frame(void);
    void collect_avg_prev_frame(cv::Mat frame);
    void collect_no_drone_frames(cv::Mat diff);
    float get_uncertainty_background() {return settings.uncertainty_background / 255.0;}
};

#endif // VIZDAT_H
