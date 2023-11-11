#pragma once
#include <fstream>
#include <vector>
#include <cmath>
#include <mutex>
#include <condition_variable>

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "dronetracker.h"
#include "insecttracker.h"
#include "itemtracker.h"
#include "pats.h"
#include "generatorcam.h"
#include "logging.h"


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
    cv::Mat plot_all_kiv_accelerations(void);
    void draw_target_text(cv::Mat resFrame, double time, float dis, float min_dis);
    cv::Mat draw_sub_tracking_viz(cv::Mat frameL_small, cv::Size vizsizeL, cv::Point3d setpoint, std::vector<tracking::TrackData> path);
    void draw_tracker_viz();

    VisionData *_visdat;
    GeneratorCam *generator_cam;
    Patser *_patser;
    bool generator_cam_set = false;
    bool _tracking_viz_initialized = false;
    bool _viz_noise_initialized = false;
    bool _viz_exposure_initialized = false;

    bool enable_plots = false;
    bool enable_optimization_drawing = false;

    const int bufsize = 600;

    const int fsizex = 800 / 2;
    const int fsizey = 800 / 3;
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

    cv::Mat cir8, dif8, approx;
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
    cv::Mat pos_x_drone;
    cv::Mat pos_y_drone;
    cv::Mat pos_z_drone;

    bool pos_log_drone_valid = false;
    cv::Point pos_log_drone;

    cv::Mat pos_x_insect;
    cv::Mat pos_y_insect;
    cv::Mat pos_z_insect;

    cv::Mat setpoint_pos_x;
    cv::Mat setpoint_pos_y;
    cv::Mat setpoint_pos_z;

    cv::Mat im_pos_x_drone;
    cv::Mat im_pos_y_drone;
    cv::Mat im_disp_drone;
    cv::Mat im_size_drone;

    cv::Mat smoothed_pos_x_drone;
    cv::Mat smoothed_pos_y_drone;
    cv::Mat smoothed_pos_z_drone;

    cv::Mat vel_x_drone;
    cv::Mat vel_y_drone;
    cv::Mat vel_z_drone;

    cv::Mat smooted_vel_x_drone;
    cv::Mat smoothed_vel_y_drone;
    cv::Mat smoothed_vel_z_drone;

    cv::Mat smoothed_acc_x_drone;
    cv::Mat smooted_acc_y_drone;
    cv::Mat smoothed_acc_z_drone;

    cv::Mat commanded_acc_x_drone;
    cv::Mat commanded_acc_y_drone;
    cv::Mat commanded_acc_z_drone;

    cv::Mat kiv_accX;
    cv::Mat kiv_accY;
    cv::Mat kiv_accZ;
    cv::Mat zero_mat;

    cv::Mat generated_pos_x_drone;
    cv::Mat generated_pos_y_drone;
    cv::Mat generated_pos_z_drone;

    cv::Mat generated_im_pos_x_drone;
    cv::Mat generated_im_pos_y_drone;
    cv::Mat generated_im_disparity_drone;
    cv::Mat generated_im_size_drone;

    static cv::Size viz_frame_size() {
        return cv::Size(IMG_W + IMG_W, IMG_H + IMG_H / 4);
    }
    bool tracking_viz_initialized() { return _tracking_viz_initialized; }

    void paint();
    void render();
    void add_plot_sample(void);
    void set_generator_cam(GeneratorCam *cam) {generator_cam = cam; generator_cam_set = true;}
    void update_tracker_data(cv::Mat frameL, cv::Point3f setpoint, double time, bool draw_plots, bool draw_optimization);
    void init(VisionData *visdat, Patser *patser, bool fromfile);
    void close();

    void enable_draw_noise_viz() {
        _viz_noise_initialized = true;
        cv::namedWindow("noise map", cv::WINDOW_OPENGL | cv::WINDOW_AUTOSIZE);
        cv::namedWindow("filtered noise map", cv::WINDOW_OPENGL | cv::WINDOW_AUTOSIZE);
    }
    void enable_draw_exposure_viz() {
        _viz_exposure_initialized = true;
        cv::namedWindow("exposure map", cv::WINDOW_OPENGL | cv::WINDOW_AUTOSIZE);
    }
    void draw_drone_from_log(logging::LogEntryDrone entry);

};
