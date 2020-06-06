#pragma once
#include <fstream>
#include <vector>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/imgproc.hpp>

#include "cereal/types/unordered_map.hpp"
#include "cereal/types/memory.hpp"
#include "cereal/archives/binary.hpp"
#include <fstream>

#include <condition_variable>

#include "realsense.h"
#include "common.h"

class VisionData {
public:
    struct delete_spot {
        cv::Point pt = {0};
        int disparity = 0;
        int r = 0;
        int cnt_active;
    };
private:

    int motion_update_iterator = 0;
    bool _calibrating_background = false;
    double calibrating_background_end_time = 0;

    class VisionParameters: public xmls::Serializable
    {
    public:
        xmls::xInt motion_update_iterator_max;
        xmls::xFloat brightness_event_tresh,brightness_check_period;

        VisionParameters() {
            // Set the XML class name.
            // This name can differ from the C++ class name
            setClassName("VisionParameters");

            // Set class version
            setVersion("1.0");

            // Register members. Like the class name, member names can differ from their xml depandants
            Register("motion_update_iterator_max",&motion_update_iterator_max);
            Register("brightness_event_tresh",&brightness_event_tresh);
            Register("brightness_check_period",&brightness_check_period);
        }
    };

    int motion_update_iterator_max;
    float brightness_event_tresh;
    float brightness_check_period;

    string settings_file = "../../xml/vision.xml";
    void deserialize_settings();
    void serialize_settings();

    std::string motion_noise_map_wfn = "max_motion_noise.png";

    cv::Mat diffL16,frameL16,diffR16,frameR16;
    double _current_frame_time = 0;

    double prev_time_brightness_check = 0;
    float prev_brightness;
    bool _reset_motion_integration = false;
    delete_spot motion_spot_to_be_deleted;
    delete_spot motion_spot_to_be_reset;

    cv::Point exclude_drone_from_motion_fading_spot_L = {-1};
    cv::Point exclude_drone_from_motion_fading_spot_R = {-1};
    int exclude_drone_from_motion_fading_radius = 0;

    bool enable_viz_diff = false;

    bool initialized = false;

    void maintain_motion_noise_map();
    void track_avg_brightness(cv::Mat frame, double time);

    void fade(cv::Mat diff16, cv::Point exclude_drone_spot);

public:
    cv::Mat frameL,frameR;
    std::vector<cv::Mat> motion_noise_bufferL,motion_noise_bufferR;
    cv::Mat motion_noise_mapL,motion_noise_mapL_small;
    cv::Mat motion_noise_mapR,motion_noise_mapR_small;
    cv::Mat diffL,diffR,diffL_small,diffR_small;
    cv::Mat overexposed_mapL;

    cv::Mat viz_frame;

    unsigned long long  frame_id;
    cv::Size smallsize;
    cv::Mat Qf,Qfi;
    float camera_angle;
    float camera_gain,camera_exposure;
    cv::Mat depth_background;
    cv::Mat disparity_background;
    cv::Mat depth_background_mm;

    bool disable_fading = false;
    bool enable_collect_no_drone_frames = true;

    double current_time() {return _current_frame_time;}

    void init(cv::Mat new_Qf, cv::Mat new_frameL, cv::Mat new_frameR, float new_camera_angle, float new_camera_gain, float new_camera_exposure, cv::Mat new_depth_background_mm);
    void close();
    void update(cv::Mat new_frameL, cv::Mat new_frameR, double time, unsigned long long new_frame_id);
    void reset_motion_integration() {
        _reset_motion_integration = true;
    }
    void enable_background_motion_map_calibration(float duration);
    bool calibrating_background() {return _calibrating_background;}
    void create_overexposed_removal_mask(cv::Point2f drone_im_location,float blink_size);

    void delete_from_motion_map(cv::Point p, int disparity, int radius, int duration);
    void reset_spot_on_motion_map(cv::Point p, int disparity, int radius, int duration);
    void exclude_drone_from_motion_fading(cv::Point3f p, int radius);

    bool is_in_overexposed_area(cv::Point p) {
        if(overexposed_mapL.cols)
            return overexposed_mapL.at<uint8_t>(p*pparams.imscalef) == 0;
        else
            return true;
    }
};
