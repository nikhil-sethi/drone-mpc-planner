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

    std::mutex g_lockData;
    std::condition_variable g_waitforimage;
    std::mutex m;


    int get_frame_id() {return frame_id;}
    float get_frame_time() {return frame_time;}
    cv::Mat Qf;

    cv::Mat frameL,frameR;

    enum auto_exposure_enum{disabled = 0, enabled = 1, only_at_startup=2};
    const auto_exposure_enum enable_auto_exposure = disabled;

private:

    int frame_id;
    float frame_time;
    rs2::pipeline_profile selection;
    int exposure = 8000; //84*(31250/256); // >11000 -> 60fps, >16500 -> 30fps, < 20 = crash
    int gain = 0;
    bool fromfile;
    bool ready;

    float frame_time_tmp;
    int frame_id_tmp;
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
