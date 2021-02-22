#pragma once
#include <iostream>
#include <vector>
#include <cmath>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "cam.h"
#include "stopwatch.h"
#include "airsim.h"

class AirSimCam : public Cam {
private:
    AirSim sim;
    const std::string camera_name = "Basestation";
    const std::string calib_airsim_cam = "../xml/cam_calib_airsim.xml";
    std::string airsim_map = "Greenhouse"; // default map

    void calibration();

public:
    AirSimCam(std::string as_map) {
        if(as_map != "")
            airsim_map = as_map;
    }
    void init();
    void close();
    void update();
    void back_one_sec() {}
};
