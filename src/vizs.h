#ifndef VIZS_H
#define VIZS_H


#include <fstream>
#include <vector>
#include <math.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "dronecontroller.h"

class Visualizer{

private:
    void plot(cv::Mat data1,cv::Mat data2, std::string name);
    void plot(cv::Mat data1,cv::Mat data2, cv::Mat *frame, std::string name);
    void plotxy(cv::Mat datax, cv::Mat datay, cv::Mat *frame, Point setpoint, std::string name);
    void addSample(void);
    DroneController *dctrl;
    DroneTracker *dtrkr;

    const int bufsize = 200;

public:

    Visualizer(void){
        roll_joystick = cv::Mat(1,1,CV_32FC1);
        roll_calculated = cv::Mat (1,1,CV_32FC1);
        pitch_joystick = cv::Mat(1,1,CV_32FC1);
        pitch_calculated = cv::Mat (1,1,CV_32FC1);
        yaw_joystick = cv::Mat(1,1,CV_32FC1);
        yaw_calculated = cv::Mat (1,1,CV_32FC1);
        throttle_joystick = cv::Mat(1,1,CV_32FC1);
        throttle_calculated = cv::Mat (1,1,CV_32FC1);
        posX = cv::Mat (1,1,CV_32FC1);
        posY = cv::Mat (1,1,CV_32FC1);
        posZ = cv::Mat (1,1,CV_32FC1);

        roll_joystick.pop_back();
        roll_calculated.pop_back();
        pitch_joystick.pop_back();
        pitch_calculated.pop_back();
        yaw_joystick.pop_back();
        yaw_calculated.pop_back();
        throttle_joystick.pop_back();
        throttle_calculated.pop_back();
        posX.pop_back();
        posY.pop_back();
        posZ.pop_back();
    }

    cv::Mat throttle_joystick;
    cv::Mat throttle_calculated;
    cv::Mat roll_joystick;
    cv::Mat roll_calculated;
    cv::Mat pitch_joystick;
    cv::Mat pitch_calculated;
    cv::Mat yaw_joystick;
    cv::Mat yaw_calculated;

    cv::Mat posX;
    cv::Mat posY;
    cv::Mat posZ;

    void plot(void);

    void init(DroneController *dctrl, DroneTracker *dtrkr){
        this->dctrl = dctrl;
        this->dtrkr = dtrkr;
    }

};

#endif // VIZS_H
