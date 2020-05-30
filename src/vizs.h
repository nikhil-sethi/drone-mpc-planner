#pragma once
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
#include "generatorcam.h"


class Visualizer {

private:
    cv::Mat plot(std::vector<cv::Mat> data_drone, std::string name);
    void plot(std::vector<cv::Mat> data_drone, cv::Mat *frame, std::string name);
    cv::Mat plotxy(cv::Mat data1x, cv::Mat data1y, cv::Mat data2x, cv::Mat data2y, cv::Point setpoint, std::string name, cv::Point minaxis, cv::Point maxaxis);
    cv::Mat plot_xyd(void);
    cv::Mat plot_all_im_drone_pos(void);
    cv::Mat plot_all_control(void);
    cv::Mat plot_all_acceleration(void);
    cv::Mat plot_all_velocity(void);
    cv::Mat plot_all_position(void);
    void draw_target_text(cv::Mat resFrame, double time, float dis, float min_dis);
    cv::Mat draw_sub_tracking_viz(cv::Mat frameL_small, cv::Size vizsizeL, cv::Point3d setpoint, std::vector<tracking::WorldItem> path, std::vector<tracking::ImagePredictItem> predicted_path);
    void draw_tracker_viz();

    VisionData * _visdat;
    DroneController *_dctrl;
    tracking::DroneTracker *_dtrkr;
    tracking::InsectTracker *_itrkr;
    tracking::TrackerManager * _trackers;
    navigation::DroneNavigation *_dnav;
    MultiModule *_rc;
    GeneratorCam * generator_cam;
    bool generator_cam_set = false;

    bool enable_plots = false;

    const int bufsize = 600;

    const int fsizex = 800/2;
    const int fsizey = 800/3;
    const int line_width = 2;
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
    float closest_dist;

    struct Tracker_viz_base_data {
        cv::Mat frameL;
        cv::Point3f setpoint;
        double time;
        float dis;
        float min_dis;

        std::vector<tracking::WorldItem> drn_path;
        std::vector<tracking::ImagePredictItem> drn_predicted_path;
        std::vector<tracking::WorldItem> ins_path;
        std::vector<tracking::ImagePredictItem> ins_predicted_path;
    };
    Tracker_viz_base_data tracker_viz_base_data;

public:
    cv::Mat trackframe;
    bool request_trackframe_paint;
    cv::Mat plotframe;
    bool request_plotframe_paint;
    double first_take_off_time = 0;

    Visualizer(void) {
        roll_joystick = cv::Mat(1,1,CV_32FC1);
        roll_calculated = cv::Mat (1,1,CV_32FC1);
        pitch_joystick = cv::Mat(1,1,CV_32FC1);
        pitch_calculated = cv::Mat (1,1,CV_32FC1);
        yaw_joystick = cv::Mat(1,1,CV_32FC1);
        yaw_calculated = cv::Mat (1,1,CV_32FC1);
        throttle_joystick = cv::Mat(1,1,CV_32FC1);
        throttle_calculated = cv::Mat (1,1,CV_32FC1);
        throttle_min_bound = cv::Mat (1,1,CV_32FC1);
        throttle_max_bound = cv::Mat (1,1,CV_32FC1);
        posX_drone = cv::Mat (1,1,CV_32FC1);
        posY_drone = cv::Mat (1,1,CV_32FC1);
        posZ_drone = cv::Mat (1,1,CV_32FC1);
        im_posX_drone = cv::Mat (1,1,CV_32FC1);
        im_posY_drone = cv::Mat (1,1,CV_32FC1);
        im_disp_drone = cv::Mat (1,1,CV_32FC1);
        im_size_drone = cv::Mat (1,1,CV_32FC1);
        dt = cv::Mat(1,1,CV_32FC1);
        dt_target = cv::Mat(1,1,CV_32FC1);

        gen_posY_drone = cv::Mat (1,1,CV_32FC1);
        gen_posX_drone = cv::Mat (1,1,CV_32FC1);
        gen_posZ_drone = cv::Mat (1,1,CV_32FC1);
        gen_im_posX_drone = cv::Mat (1,1,CV_32FC1);
        gen_im_posY_drone = cv::Mat (1,1,CV_32FC1);
        gen_im_disp_drone = cv::Mat (1,1,CV_32FC1);
        gen_im_size_drone = cv::Mat (1,1,CV_32FC1);

        roll_joystick.pop_back();
        roll_calculated.pop_back();
        pitch_joystick.pop_back();
        pitch_calculated.pop_back();
        yaw_joystick.pop_back();
        yaw_calculated.pop_back();
        throttle_joystick.pop_back();
        throttle_calculated.pop_back();
        throttle_min_bound.pop_back();
        throttle_max_bound.pop_back();
        posX_drone.pop_back();
        posY_drone.pop_back();
        posZ_drone.pop_back();
        im_posX_drone.pop_back();
        im_posY_drone.pop_back();
        im_disp_drone.pop_back();
        im_size_drone.pop_back();
        dt.pop_back();
        dt_target.pop_back();

        gen_posY_drone.pop_back();
        gen_posX_drone.pop_back();
        gen_posZ_drone.pop_back();
        gen_im_posX_drone.pop_back();
        gen_im_posY_drone.pop_back();
        gen_im_disp_drone.pop_back();
        gen_im_size_drone.pop_back();

    }
    ~Visualizer() {
        roll_joystick.release();
        roll_calculated.release();
        pitch_joystick.release();
        pitch_calculated.release();
        yaw_joystick.release();
        yaw_calculated.release();
        throttle_joystick.release();
        throttle_calculated.release();
        throttle_min_bound.release();
        throttle_max_bound.release();
        posX_drone.release();
        posY_drone.release();
        posZ_drone.release();
        im_posX_drone.release();
        im_posY_drone.release();
        im_disp_drone.release();
        im_size_drone.release();
        dt.release();
        dt_target.release();
    }

    cv::Mat throttle_joystick;
    cv::Mat throttle_calculated;
    cv::Mat throttle_min_bound;
    cv::Mat throttle_max_bound;
    cv::Mat roll_joystick;
    cv::Mat roll_calculated;
    cv::Mat pitch_joystick;
    cv::Mat pitch_calculated;
    cv::Mat yaw_joystick;
    cv::Mat yaw_calculated;

    cv::Mat dt;
    cv::Mat dt_target;
    cv::Mat posX_drone;
    cv::Mat posY_drone;
    cv::Mat posZ_drone;

    cv::Mat posX_target;
    cv::Mat posY_target;
    cv::Mat posZ_target;

    cv::Mat im_posX_drone;
    cv::Mat im_posY_drone;
    cv::Mat im_disp_drone;
    cv::Mat im_size_drone;

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

    cv::Mat gen_posX_drone;
    cv::Mat gen_posY_drone;
    cv::Mat gen_posZ_drone;

    cv::Mat gen_im_posX_drone;
    cv::Mat gen_im_posY_drone;
    cv::Mat gen_im_disp_drone;
    cv::Mat gen_im_size_drone;

    cv::Size viz_frame_size() {
        return cv::Size(IMG_W*_res_mult + IMG_W,IMG_H*_res_mult+IMG_H*_res_mult/4);
    }

    void paint();
    void add_plot_sample(void);
    void set_generator_cam(GeneratorCam * cam) {generator_cam = cam; generator_cam_set=true;}
    void update_tracker_data(cv::Mat frameL, cv::Point3f setpoint, double time, bool draw_plots, tracking::InsectTracker *itrkr);
    void init(VisionData * visdat, tracking::TrackerManager *imngr, DroneController *dctrl, navigation::DroneNavigation *dnav, MultiModule *rc, bool fromfile);
    void close();

};
