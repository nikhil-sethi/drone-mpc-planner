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
        set_write_file_paths(data_output_dir);
        bag_fn = "./logging/record" + std::to_string(_id) + ".bag";
    }
    Realsense(string dir) {
        replay_dir = dir;
        from_recorded_bag = true;
        set_read_file_paths(replay_dir);
        set_write_file_paths(data_output_dir);
        bag_fn = replay_dir + '/' + playback_filename();
    }
    void connect_and_check(string ser_nr,int id);
    void init() {
        if (pparams.fps == 90) {
            max_auto_exposure = 10000;
        } else if (pparams.fps == 60) {
            max_auto_exposure = 15500;
        } else if (pparams.fps == 30) {
            max_auto_exposure = 32200;
        } else {
            std::cout << "Error: unknown fps, not implemented" << std::endl;
            exit(1);
        }
        if (from_recorded_bag)
            init_playback();
        else
            init_real();
    }
    void close();
    void reset();
    void stop_watchdog() {exit_watchdog_thread = true;}
    StereoPair * update();
    std::tuple<float,float,cv::Mat,cv::Mat,cv::Mat,float> measure_auto_exposure();
    std::tuple<float,float,double,cv::Mat> measure_angle();
    bool master() {return !_id;}

protected:
    void delete_old_frames(bool skipping_frames);
    void delete_all_frames();
private:
    bool dev_initialized = false;
    bool exposure_initialized = false;
    bool angle_initialized = false;

    bool new_frame1 = false;
    bool new_frame2 = false;
    rs2::frame rs_frameL_cbtmp,rs_frameR_cbtmp;
    uint last_sync_id = 0;
    struct RSStereoPair {
        RSStereoPair(rs2::frame left_,rs2::frame right_) {
            left = left_;
            right = right_;
        }
        rs2::frame left,right;
    };
    std::map<unsigned long long,RSStereoPair * > rs_buf;

    bool isD455 = false;
    double _frame_time_start = -1;
    string replay_dir;
    bool from_recorded_bag;
    string bag_fn;
    string serial_nr_str;
    string serial_nr;

    int max_auto_exposure;

    std::mutex lock_newframe;
    rs2::device dev;
    rs2::pipeline cam_playback;
    static inline rs2::context const ctx;

    bool exit_watchdog_thread = false;
    bool watchdog = true,watchdog_attempt_to_continue = false;
    std::thread thread_watchdog;
    void watchdog_thread(void);

    void seek(double time);
    void calibration(rs2::stream_profile infrared1,rs2::stream_profile infrared2);
    void init_real();
    void init_playback();
    void update_real();
    void update_playback();
    void rs_callback(rs2::frame f);
    void rs_callback_playback(rs2::frame f);
    void calib_depth_background();
};
