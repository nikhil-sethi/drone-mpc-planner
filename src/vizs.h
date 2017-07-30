#ifndef VIZS_H
#define VIZS_H


#include <fstream>
#include <vector>
#include <math.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "kalamosfilecam.h"
#include "dronecontroller.h"

class Visualizer{

private:
    void plot(cv::Mat data1,cv::Mat data2, std::string name);
    void addSample(void);
    KalamosFileCam *cam;
    DroneController *dctrl;

public:

    Visualizer(void){
        roll_joystick = cv::Mat(1,1,CV_32FC1);
        roll_calculated = cv::Mat (1,1,CV_32FC1);
        throttle_joystick = cv::Mat(1,1,CV_32FC1);
        throttle_calculated = cv::Mat (1,1,CV_32FC1);

        roll_joystick.pop_back();
        roll_calculated.pop_back();
        throttle_joystick.pop_back();
        throttle_calculated.pop_back();
    }

    cv::Mat throttle_joystick;
    cv::Mat throttle_calculated;
    cv::Mat roll_joystick;
    cv::Mat roll_calculated;
    cv::Mat pitch_joystick;
    cv::Mat pitch_calculated;
    cv::Mat yaw_joystick;
    cv::Mat yaw_calculated;

    void plot(void);
    void init(KalamosFileCam *cam, DroneController *dctrl){
        this->dctrl = dctrl;
        this->cam = cam;
    }

};

#endif // VIZS_H
