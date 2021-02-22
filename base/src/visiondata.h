#pragma once
#include <fstream>
#include <vector>
#include <cmath>
#include <condition_variable>

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/imgproc.hpp>

#include "cam.h"
#include "common.h"

class VisionData {
public:
    struct DeleteSpot {
        cv::Point pt = {0};
        int disparity = 0;
        int r = 0;
        int cnt_active;
    };
private:
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

    bool initialized = false;
    bool enable_viz_motion = false;
    Cam * _cam;
    int motion_update_iterator = 0;
    bool _calibrating_background = false;
    double calibrating_background_end_time = 0;

    int motion_update_iterator_max;
    float brightness_event_tresh;
    string settings_file = "../xml/vision.xml";
    std::string motion_noise_mapL_wfn = "max_motion_noiseL.png";
    std::string motion_noise_mapR_wfn = "max_motion_noiseR.png";
    std::string overexposed_mapL_wfn = "overexposedL.png";
    std::string overexposed_mapR_wfn = "overexposedR.png";

    cv::Mat diffL16,frameL16,diffR16,frameR16;
    double _current_frame_time = 0;
    cv::UMat dilate_element;

    float prev_brightness = -1;
    bool _reset_motion_integration = false;
    double _large_brightness_change_event_time = 0;
    DeleteSpot motion_spot_to_be_deleted;
    DeleteSpot motion_spot_to_be_reset;

    cv::Point exclude_drone_from_motion_fading_spot_L = {-1};
    cv::Point exclude_drone_from_motion_fading_spot_R = {-1};
    int exclude_drone_from_motion_fading_radius = 0;
    std::vector<cv::Mat> motion_noise_bufferL,motion_noise_bufferR;

    void deserialize_settings();
    void serialize_settings();
    void maintain_motion_noise_map();
    void track_avg_brightness(cv::Mat frame, double time);
    void fade(cv::Mat diff16, cv::Point exclude_drone_spot);

public:
    unsigned long long  frame_id;
    cv::Size smallsize;
    cv::Mat Qf,Qfi;
    float camera_angle;
    float camera_gain,camera_exposure;
    bool disable_fading = false;
    bool enable_collect_no_drone_frames = true;
    cv::Mat frameL,frameR;
    cv::Mat depth_background;
    cv::Mat disparity_background;
    cv::Mat depth_background_mm;
    cv::Mat frameL_background;
    cv::Mat motion_noise_mapL,motion_noise_mapL_small;
    cv::Mat motion_noise_mapR,motion_noise_mapR_small;
    cv::Mat diffL,diffR,diffL_small,diffR_small;
    cv::Mat overexposed_mapL,overexposed_mapL_small;
    cv::Mat overexposed_mapR,overexposed_mapR_small;
    cv::Mat viz_frame;

    void init(Cam * cam);
    void read_motion_and_overexposed_from_image(std::string replay_dir);
    void close();
    void update(StereoPair * current);
    void enable_background_motion_map_calibration(float duration);
    bool calibrating_background() {return _calibrating_background;}
    void create_overexposed_removal_mask(cv::Point3f drone_im_location,float blink_size);
    void delete_from_motion_map(cv::Point p, int disparity, int radius, int duration);
    void reset_spot_on_motion_map(cv::Point p, int disparity, int radius, int duration);
    void exclude_drone_from_motion_fading(cv::Point3f p, int radius);
    bool overexposed(cv::Point blob_pt);

    bool no_recent_large_brightness_events(double time) {
        return static_cast<float>(time-_large_brightness_change_event_time)> 3;
    }
    float average_brightness() { return prev_brightness; }
    double current_time() {return _current_frame_time;}
    void reset_motion_integration() {_reset_motion_integration = true;}
};
