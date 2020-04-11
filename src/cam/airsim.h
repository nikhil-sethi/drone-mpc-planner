#pragma once
#include "defines.h"
#if FALSE
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <unistd.h>       //usleep

#include "stopwatch.h"

#include <thread>
#include <mutex>
#include <condition_variable>



#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <iostream>
#include <chrono>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


class Airsim {

public:

    void init(int argc, char **argv);
    void close();

    void pause();
    void resume();

    void update(void);

    int frame_id;
    float frame_time;
    cv::Mat frameL,frameR,frameD,frameRGB;
    cv::Mat Qf;

    enum cam_mode_enum {cam_mode_stopped = 0, cam_mode_disabled = 1, cam_mode_color = 2, cam_mode_stereo=3 };

    int get_frame_id() {return frame_id;}
    float get_frame_time() {return frame_time;}
    void switch_mode(cam_mode_enum mode);

    void go_go_go() {
        g_wait.notify_all();
    }

    cam_mode_enum get_mode() {return _mode;}
private:

    msr::airlib::MultirotorRpcLibClient client;

    std::mutex m;
    std::condition_variable g_wait;

    stopwatch_c sw;
    cam_mode_enum _mode = cam_mode_stopped;

    std::thread thread_cam;
    bool exitCamThread = false;
    void workerThread(void);
    void go_stereo();
    void go_color();
    void go_disabled();

};
#endif //CAMMODE_AIRSIM
