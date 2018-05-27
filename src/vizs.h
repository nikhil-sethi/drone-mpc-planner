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
    Mat plot(std::vector<Mat> data, std::string name);
    void plot(std::vector<Mat> data, cv::Mat *frame, std::string name);
    cv::Mat plotxy(cv::Mat datax, cv::Mat datay, Point setpoint, std::string name, Point minaxis, Point maxaxis);
    cv::Mat plot_xyd(void);
    cv::Mat plot_all_control(void);
    cv::Mat plot_all_velocity(void);
    cv::Mat plot_all_position(void);

    DroneController *dctrl;
    DroneTracker *dtrkr;

    const int bufsize = 300;

    std::mutex g_lockData;
    std::thread thread_viz;
    bool exitVizThread = false;
    void workerThread(void);
    void plot(void);

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
        throttle_hover = cv::Mat (1,1,CV_32FC1);
        posX = cv::Mat (1,1,CV_32FC1);
        posY = cv::Mat (1,1,CV_32FC1);
        posZ = cv::Mat (1,1,CV_32FC1);
        disparity = cv::Mat (1,1,CV_32FC1);
        sdisparity = cv::Mat (1,1,CV_32FC1);

        roll_joystick.pop_back();
        roll_calculated.pop_back();
        pitch_joystick.pop_back();
        pitch_calculated.pop_back();
        yaw_joystick.pop_back();
        yaw_calculated.pop_back();
        throttle_joystick.pop_back();
        throttle_calculated.pop_back();
        throttle_hover.pop_back();
        posX.pop_back();
        posY.pop_back();
        posZ.pop_back();
        disparity.pop_back();
        sdisparity.pop_back();
    }

    cv::Mat throttle_joystick;
    cv::Mat throttle_calculated;
    cv::Mat throttle_hover;
    cv::Mat roll_joystick;
    cv::Mat roll_calculated;
    cv::Mat pitch_joystick;
    cv::Mat pitch_calculated;
    cv::Mat yaw_joystick;
    cv::Mat yaw_calculated;

    cv::Mat posX;
    cv::Mat posY;
    cv::Mat posZ;
    cv::Mat disparity;
    cv::Mat sdisparity;

    cv::Mat sposX;
    cv::Mat sposY;
    cv::Mat sposZ;

    cv::Mat setposX;
    cv::Mat setposY;
    cv::Mat setposZ;

    cv::Mat velX;
    cv::Mat velY;
    cv::Mat velZ;

    cv::Mat svelX;
    cv::Mat svelY;
    cv::Mat svelZ;

    cv::Mat autotakeoff_velY_thresh;

    void addSample(void);
    void init(DroneController *dctrl, DroneTracker *dtrkr){
        this->dctrl = dctrl;
        this->dtrkr = dtrkr;
        thread_viz = std::thread(&Visualizer::workerThread,this);
    }
    void close();

};

#endif // VIZS_H
