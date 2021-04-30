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
#include "interceptor.h"


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
    cv::Mat draw_sub_tracking_viz(cv::Mat frameL_small, cv::Size vizsizeL, cv::Point3d setpoint, std::vector<tracking::TrackData> path);
    void draw_tracker_viz();

    VisionData * _visdat;
    DroneController *_dctrl;
    tracking::TrackerManager * _trackers;
    navigation::DroneNavigation *_dnav;
    Rc *_rc;
    GeneratorCam * generator_cam;
    Interceptor *_iceptor;
    bool generator_cam_set = false;
    bool _tracking_viz_initialized = false;

    bool enable_plots = false;

    const int bufsize = 600;

    const int fsizex = 800/2;
    const int fsizey = 800/3;
    const int line_width = 2;
    const float text_size = 0.3;

    float ground_y = -1.92f; // guestimated ground level

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
    };
    Tracker_viz_base_data tracker_viz_base_data;

public:
    cv::Mat trackframe;
    bool request_trackframe_paint;
    cv::Mat plotframe;
    bool request_plotframe_paint;
    double first_take_off_time = 0;

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

    cv::Mat velX;
    cv::Mat velY;
    cv::Mat velZ;

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
    bool tracking_viz_initialized() { return _tracking_viz_initialized; }

    void paint();
    void render();
    void add_plot_sample(void);
    void set_generator_cam(GeneratorCam * cam) {generator_cam = cam; generator_cam_set=true;}
    void update_tracker_data(cv::Mat frameL, cv::Point3f setpoint, double time, bool draw_plots);
    void init(VisionData * visdat, tracking::TrackerManager *imngr, DroneController *dctrl, navigation::DroneNavigation *dnav, Rc *rc, bool fromfile, Interceptor *iceptor);
    void close();

};
