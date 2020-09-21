#pragma once
#include "cam.h"

#include <condition_variable>
#include <deque>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

class Realsense : public Cam {

public:

    static std::string playback_filename() { return "record.bag"; }
    Realsense() {
        from_recorded_bag = false;
    }
    Realsense(string dir) {
        replay_dir = dir;
        from_recorded_bag = true;
        set_file_paths(replay_dir);
        bag_fn = replay_dir + '/' + playback_filename();
    }
    void connect_and_check();
    void init() {
        if (from_recorded_bag)
            init_playback();
        else
            init_real();
    }

    void close();
    void reset();

    void stop_watchdog() {exit_watchdog_thread = true;}

    void back_one_sec() {
        seek(_frame_time -3);
    }

    void update();

    std::tuple<float,float,cv::Mat,cv::Mat,float> measure_auto_exposure();
    std::tuple<float,float,double,cv::Mat> measure_angle();



private:

    float _camera_angle_y_measured_from_depth = 30;
    bool hasIMU = false;
    double _frame_time_start = -1;
    string replay_dir;


    enum auto_exposure_enum {disabled = 0, enabled = 1, only_at_startup=2};
    const auto_exposure_enum enable_auto_exposure = enabled;

    int exposure = 11000; //84*(31250/256); // >11000 -> 60fps, >15500 -> 30fps, < 20 = crash
    int gain = 0;
    bool from_recorded_bag;

    bool exit_watchdog_thread = false;
    bool watchdog = true;
    std::thread thread_watchdog;
    void watchdog_thread(void);

    uint playback_buffer_size_max = 100;

    std::mutex lock_newframe;
    std::mutex lock_frame_data;
    rs2::device dev;
    rs2::pipeline cam;
    bool dev_initialized = false;

    std::string bag_fn;

    void seek(double time);
    void calibration(rs2::stream_profile infrared1,rs2::stream_profile infrared2);
    void init_real();
    void init_playback();
    void update_real();
    void update_playback();
    void rs_callback(rs2::frame f);
    void rs_callback_playback(rs2::frame f);

    void check_light_level();

    void calib_pose(bool also_do_depth);


    bool new_frame1 = false;
    bool new_frame2 = false;

    rs2::frame rs_frameL_cbtmp,rs_frameR_cbtmp;
    rs2::frame rs_frameL,rs_frameR;

};
