#pragma once
#include "common.h"
#include <opencv2/core/types.hpp>

class PropwashHandler{
private:
    // Parameter:
    const float propwash_safety_angle = 4.f/5.f*static_cast<float>(M_PI); // The higher the value the higher the chance of propwash

    // States:
    bool _propwash = false;

public:
    bool prop_wash(cv::Point3f drone_velocity, cv::Point3f des_acc_drone);
    cv::Point3f update(cv::Point3f drone_velocity, cv::Point3f desired_acc, float thrust);

    bool propwash() {return _propwash;};
};
