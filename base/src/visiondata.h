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
        int cnt_active = 0;
    };
private:

    bool initialized = false;
    bool enable_viz_motion = false;
    Cam *_cam;
    uint32_t  frame_cnt = 0;
    int motion_update_iterator = 0;
    bool _calibrating_motion_noise_map = false;
    bool _motion_filtered_noise_initialized = false;
    double calibrating_noise_map_end_time = 0;
    const float motion_buf_size_target = 60;
    int save_every_nth_frame_during_motion_calib = 3;
    const uint default_motion_update_iterator_max = 6;
    uint motion_update_iterator_max = default_motion_update_iterator_max;

    string settings_file = "../xml/vision.xml";

    cv::Mat diffL16, frameL16, diffR16, frameR16;
    double _current_frame_time = 0;
    cv::UMat dilate_element;

    float brightness_large_event_tresh = 5;
    float brightness_small_event_tresh = 1;
    float brightness_prev = -1;
    bool _reset_motion_integration = false;
    const float large_brightness_change_timeout = 0.3;
    double large_brightness_event_time = -large_brightness_change_timeout; // minus to not trigger a brightness warning at startup
    DeleteSpot motion_spot_to_be_deleted;
    DeleteSpot motion_spot_to_be_reset;
    std::vector<DeleteSpot> motion_start_spots_to_be_reset;

    cv::Point exclude_drone_from_motion_fading_spot_L = {-1};
    cv::Point exclude_drone_from_motion_fading_spot_R = {-1};
    int exclude_drone_from_motion_fading_radius = 0;
    std::vector<cv::Mat> motion_noise_bufferL, motion_noise_bufferR;
    cv::Mat overexposed_bufferL16, overexposed_bufferR16;

    void deserialize_settings();
    void serialize_settings();
    void track_avg_brightness(cv::Mat frameL_new, double time);
    void fade(cv::Mat diff16, cv::Point exclude_drone_spot);

public:
    unsigned long long  frame_id;

    cv::Size smallsize;
    cv::Mat Qf, Qfi;
    float camera_roll() {return _cam->camera_roll();}
    float camera_pitch() {return _cam->camera_pitch();}
    float camera_gain() {return _cam->measured_gain();}
    float camera_exposure() {return _cam->measured_exposure();}
    float light_level() {return calc_light_level(_cam->measured_exposure(), _cam->measured_gain(), average_brightness());}
    bool disable_fading = false;
    cv::Mat frameL, frameR;
    cv::Mat depth_background;
    cv::Mat disparity_background;
    cv::Mat depth_background_mm;
    cv::Mat motion_filtered_noise_mapL, motion_filtered_noise_mapL_small, motion_max_noise_mapL;
    cv::Mat motion_filtered_noise_mapR;
    cv::Mat diffL, diffR, diffL_small, diffR_small;
    cv::Mat overexposed_mapL;
    cv::Mat overexposed_mapR;
    cv::Mat viz_frame;
    bool disable_cloud_rejection = true;

    void init(Cam *cam);
    void close();
    void update(StereoPair *current);
    void maintain_noise_maps();
    void enable_noise_map_calibration();
    void enable_noise_map_calibration(float duration);
    bool calibrating_background() {return _calibrating_motion_noise_map;}
    bool motion_filtered_noise_initialized() {return _motion_filtered_noise_initialized;}
    void create_overexposed_removal_mask(bool filter_drone, cv::Point3f drone_im_location, float blink_size);
    void delete_from_motion_map(cv::Point p, int disparity, int radius, int duration);
    void reset_spot_on_motion_map(cv::Point p, int disparity, int radius, int duration);
    void add_start_reset_spot_on_motion_map(cv::Point p, int radius);
    void exclude_drone_from_motion_fading(cv::Point3f p, int radius);
    int max_noise(cv::Point blob_pt);
    bool overexposed(cv::Point blob_pt);
    bool first_frame() { return frame_cnt == 2;}

    bool no_recent_brightness_events(double time) {
        return static_cast<float>(time - large_brightness_event_time) > large_brightness_change_timeout;
    }
    bool brightness_change_event(double time) {
        return static_cast<float>(time - large_brightness_event_time) < 3.f / pparams.fps;
    }
    float average_brightness() { return brightness_prev; }
    double current_time() {return _current_frame_time;}
    void reset_motion_integration() {_reset_motion_integration = true;}
    void init_replay(std::string folder);
    void init_replay(std::string folder, int id);
    void save_maps(std::string folder);
    void save_maps(int id, std::string folder);
    void save_maps_worker(int id, std::string folder, cv::Mat overexposed_mapL, cv::Mat overexposed_mapR, cv::Mat motion_filtered_noise_mapL, cv::Mat motion_filtered_noise_mapR, cv::Mat motion_filtered_noise_mapL_small, cv::Mat motion_max_noise_mapL);
};
