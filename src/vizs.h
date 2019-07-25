#ifndef VIZS_H
#define VIZS_H


#include <fstream>
#include <vector>
#include <cmath>
#include <mutex>
#include <condition_variable>

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "dronecontroller.h"
#include "dronetracker.h"
#include "insecttracker.h"
#include "itemtracker.h"
#include "dronenavigation.h"
#include "dronepredictor.h"

class Visualizer{

private:
    cv::Mat plot(std::vector<cv::Mat> data, std::string name);
    void plot(std::vector<cv::Mat> data, cv::Mat *frame, std::string name);
    cv::Mat plotxy(cv::Mat datax, cv::Mat datay, cv::Point setpoint, std::string name, cv::Point minaxis, cv::Point maxaxis);
    cv::Mat plot_xyd(void);
    cv::Mat plot_all_control(void);
    cv::Mat plot_all_acceleration(void);
    cv::Mat plot_all_velocity(void);
    cv::Mat plot_all_position(void);
    void draw_target_text(cv::Mat resFrame, double time, float dis, float min_dis);
    cv::Mat draw_sub_tracking_viz(cv::Mat frameL_small, cv::Size vizsizeL, cv::Point3d setpoint, std::vector<ItemTracker::WorldItem> path, std::vector<ItemTracker::ImagePredictItem> predicted_path);
    void draw_tracker_viz();

    VisionData * _visdat;
    DroneController *_dctrl;
    DroneTracker *_dtrkr;
    InsectTracker *_itrkr;
    ItemManager * _trackers;
    DroneNavigation *_dnav;
    MultiModule *_rc;
    DronePredictor *_dprdct;

    const int bufsize = 600;

    const int fsizex = 925/4;
    const int fsizey = 535/3;
    const int line_width = 1;
    const float text_size = 0.3;

    std::mutex lock_plot_data;
    std::mutex lock_frame_data;
    std::thread thread_viz;
    std::mutex m;
    std::condition_variable newdata;
    bool new_tracker_viz_data_requested = true;
    bool exitVizThread = false;
    bool initialized = false;
    void workerThread(void);
    void plot(void);

    bool _fromfile;
    float _res_mult;

    cv::Mat cir8,dif8,approx;


    struct Tracker_viz_base_data{
        cv::Mat frameL;
        cv::Point3d setpoint;
        double time;
        float dis;
        float min_dis;

        std::vector<ItemTracker::WorldItem> drn_path;
        std::vector<ItemTracker::ImagePredictItem> drn_predicted_path;
        std::vector<ItemTracker::WorldItem> ins_path;
        std::vector<ItemTracker::ImagePredictItem> ins_predicted_path;
    };
    Tracker_viz_base_data tracker_viz_base_data;

public:
    cv::Mat trackframe;
    bool request_trackframe_paint;
    cv::Mat plotframe;
    bool request_plotframe_paint;
    double first_take_off_time = 0;

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
        dt = cv::Mat(1,1,CV_32FC1);
        dt_target = cv::Mat(1,1,CV_32FC1);

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
        dt.pop_back();
        dt_target.pop_back();

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

    cv::Mat dt;
    cv::Mat dt_target;
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

    cv::Mat svelX;
    cv::Mat svelY;
    cv::Mat svelZ;

    cv::Mat saccX;
    cv::Mat saccY;
    cv::Mat saccZ;

    cv::Mat autotakeoff_velY_thresh;


    cv::Size viz_frame_size() {
        return cv::Size(IMG_W*_res_mult + IMG_W,IMG_H*_res_mult+IMG_H*_res_mult/4);
    }

    void paint();
    void addPlotSample(void);
    void update_tracker_data(cv::Mat frameL, cv::Point3d setpoint, double time);
    void init(VisionData * visdat, ItemManager *imngr, DroneController *dctrl, DroneNavigation *dnav, MultiModule *rc, bool fromfile, DronePredictor *dprdct);
    void close();

};

#endif // VIZS_H
