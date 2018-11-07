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

#include <thread>
#include <mutex>
#include <condition_variable>

#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#if CV_MAJOR_VERSION==2
#include <opencv2/contrib/contrib.hpp>
#endif

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API


class Cam{

public:

    void init(int argc, char **argv);
    void close();

    void pause();
    void resume();

    void update(void);



    int get_frame_id() {return frame_id;}
    float get_frame_time() {return frame_time;}
    cv::Mat Qf;

    cv::Mat frameL,frameR;

    enum auto_exposure_enum{disabled = 0, enabled = 1, only_at_startup=2};
    const auto_exposure_enum enable_auto_exposure = disabled;

private:

    int frame_id;
    float frame_time;

    int exposure = 15500; //84*(31250/256); // >11000 -> 60fps, >15500 -> 30fps, < 20 = crash
    int gain = 20;
    bool fromfile;
    bool ready;

    std::mutex g_lockData;
    std::condition_variable g_waitforimage;
    std::mutex m;
    rs2::device dev;

    void rs_callback(rs2::frame f);

    rs2::sensor depth_sensor;
    bool new_frameL = false;
    bool new_frameR = false;

    rs2::frame rs_frameL,rs_frameR;


};

#endif // CAM_H
