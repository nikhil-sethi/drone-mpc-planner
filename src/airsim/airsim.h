#pragma once
#include <iostream>
#include <unistd.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "common.h"

// if vs code grays out code in the #if section, disable dimInactiveRegions in the settings
#if HAS_AIRSIM
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

typedef ImageCaptureBase::ImageRequest ImageRequest;
typedef ImageCaptureBase::ImageResponse ImageResponse;
typedef ImageCaptureBase::ImageType ImageType;
typedef msr::airlib::RpcLibClientBase::ConnectionState ConnectionState;
typedef msr::airlib::CameraInfo CameraInfo;
typedef msr::airlib::TTimePoint TTimePoint;
#endif

class AirSim {
private:
#if HAS_AIRSIM
    msr::airlib::MultirotorRpcLibClient client;
    std::vector<ImageRequest> request;
    std::string client_name;
    TTimePoint start_timestamp = 0;
    RCData rc_data;
    std::ofstream airsim_log_file;
    bool logging_initialized = false;
    float current_led = 0;
    unsigned long long _frame_number = 0;
#endif

public:
    void init(std::string name);
    void pause(bool pause);
    void close();
    void rc_data_valid(bool valid);
    void set_led(float led);
    void move_by_rc(float throttle, float yaw, float pitch, float roll);
    void arm(bool arm);
    void load_environment(std::string level_name);
    StereoPair* new_frame(double desiredFrameTime);
    float cam_fov();
    cv::Mat depth_background();
    void init_logging();
    void log(unsigned long long frame_id);
};