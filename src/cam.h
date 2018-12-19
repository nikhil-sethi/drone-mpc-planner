#ifndef CAM_H
#define CAM_H

#include "defines.h"
#include "stopwatch.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <iomanip>
#include <unistd.h>
#include "common.h"

#include <condition_variable>
#include <deque>

#include "opencv2/highgui/highgui.hpp"

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

class Cam{

public:

    void init(std::ofstream *logger);
    void sense_light_level();
    void calib_pose();
    void init(int argc, char **argv);
    void close();
    void reset();

    void skip_one_sec() {
        requested_id_in += VIDEOFPS*2;
    }
    void back_one_sec() {
        pause();
        usleep(1000);
        lock_frame_data.lock();
        playback_bufferR.clear();
        playback_bufferL.clear();
        requested_id_in -= VIDEOFPS*2;
        lock_frame_data.unlock();
    }
    bool frame_by_frame;
    bool turbo;

    void update(void);


    int frame_number() {return _frame_number;}
    float frame_time() {return _frame_time;}
    cv::Mat Qf;

    cv::Mat frameL,frameR;

    cv::Mat depth_background;

    float _camera_angle_x,_camera_angle_y;
    float camera_angle(){
        return _camera_angle_y;
    }
    float measured_exposure(){
        return _measured_exposure;
    }

    enum auto_exposure_enum{disabled = 0, enabled = 1, only_at_startup=2};
    const auto_exposure_enum enable_auto_exposure = only_at_startup;

    struct stereo_frame{
         cv::Mat frameL,frameR;
         uint id;
         float time;
    };

private:

    uint requested_id_in =0;
    int _frame_number;
    float _frame_time = 0;
    float _frame_time_start = -1;

    float _measured_exposure = -1; // measured from sense_light_level
    int exposure = 15500; //84*(31250/256); // >11000 -> 60fps, >15500 -> 30fps, < 20 = crash
    int gain = 0;
    bool fromfile;

    std::mutex lock_flags;
    std::mutex lock_frame_data;
    std::condition_variable wait_for_image;
    std::mutex m;
    rs2::device dev;

    bool _paused;

    void pause();
    void resume();
    void seek(float time);
    void set_calibration(rs2::stream_profile infared1,rs2::stream_profile infared2);
    void update_real(void);
    void update_playback(void);
    void rs_callback(rs2::frame f);
    void rs_callback_playback(rs2::frame f);

    rs2::sensor depth_sensor;

    bool new_frame1 = false;
    bool new_frame2 = false;

    rs2::frame rs_frameL_cbtmp,rs_frameR_cbtmp;
    rs2::frame rs_frameL,rs_frameR;

    struct frame_data{
        cv::Mat frame;
        uint id;
        float time;
    };
    std::deque<frame_data> playback_bufferL;
    std::deque<frame_data> playback_bufferR;

protected:
    std::ofstream *_logger;


};

#endif // CAM_H
