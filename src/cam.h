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

#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API



class Cam{

public:

    void init(int argc, char **argv);
    void close();

    void pause();
    void resume();

    void update(void);

    std::mutex g_lockData;
    std::mutex g_waitforimage;

    cv::Mat Qf;
    int frame_number;
    float frame_time;
    cv::Mat frameL,frameR;

    enum auto_exposure_enum{disabled = 0, enabled = 1, only_at_startup=2};
    const auto_exposure_enum enable_auto_exposure = enabled;

private:

    rs2::pipeline_profile selection;
    int exposure = 20; // >11000 -> 60fps, >16500 -> 30fps, < 20 = crash
    int gain = 16;
    bool fromfile;
    bool ready;

    float frame_time_tmp;
    int frame_number_tmp;
    cv::Mat frameL_tmp,frameR_tmp;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline cam;
    rs2::device pd;

#define IR_ID_LEFT 1 //as seen from the camera itself
#define IR_ID_RIGHT 2

    std::thread thread_cam;
    bool exitCamThread = false;
    void workerThread(void);


};

#endif // CAM_H
