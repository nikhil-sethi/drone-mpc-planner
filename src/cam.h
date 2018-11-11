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

    void init(int argc, char **argv);
    void close();

    void pause();
    void resume();
    void seek(float time);

    void skip_one_sec() {
        requested_id_in += VIDEOFPS*2;
    }
    void back_one_sec() {
        pause();
        usleep(1000);
        g_lockFrameData.lock();
        playback_bufferR.clear();
        playback_bufferL.clear();
        requested_id_in -= VIDEOFPS*2;
        g_lockFrameData.unlock();
    }
    bool frame_by_frame;


    void update(void);


    int frame_number() {return _frame_number;}
    float frame_time() {return _frame_time;}
    cv::Mat Qf;

    cv::Mat frameL,frameR;

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

    int exposure = 15500; //84*(31250/256); // >11000 -> 60fps, >15500 -> 30fps, < 20 = crash
    int gain = 20;
    bool fromfile;
    bool real_time_playback = false;
    float real_time_playback_speed = 1;

    std::mutex g_lockFlags;
    std::mutex g_lockFrameData;
    std::condition_variable g_waitforimage;
    std::mutex m;
    rs2::device dev;

    bool _paused;

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


};

#endif // CAM_H
