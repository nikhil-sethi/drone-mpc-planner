#ifndef CAM_H
#define CAM_H

#include "defines.h"
#include "stopwatch.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <unistd.h>
#include "common.h"

#include <condition_variable>
#include <deque>

#include "opencv2/highgui/highgui.hpp"

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "tinyxml/XMLSerialization.h"

class Cam{

private:
    class CamCalibrationData: public xmls::Serializable
    {
    public:

        xmls::xFloat Angle_X;
        xmls::xFloat Angle_Y;
        xmls::xFloat Angle_Y_Measured_From_Depthmap;
        xmls::xFloat Exposure;
        xmls::xFloat Gain;

        CamCalibrationData():
            Angle_X(0),
            Angle_Y(30),
            Angle_Y_Measured_From_Depthmap(30),
            Exposure(0),
            Gain(0)
        {
            // Set the XML class name.
            // This name can differ from the C++ class name
            setClassName("CamCalibrationData");

            // Set class version
            setVersion("1.2");

            // Register members. Like the class name, member names can differ from their xml depandants
            Register("Angle_X", &Angle_X);
            Register("Angle_Y", &Angle_Y);
            Register("Angle_Y_Measured_From_Depthmap", &Angle_Y_Measured_From_Depthmap);
            Register("Exposure", &Exposure);
            Register("Gain", &Gain);
        };
    };

public:

    void init();
    void init(int argc, char **argv);
    void close();
    void reset();

    void skip_one_sec() {
        requested_id_in += VIDEOFPS*2;
    }
    void back_one_sec() {
        pause();
        usleep(1000);
        lock_frame_data.lock();
        playback_bufferR.clear();
        playback_bufferL.clear();
        requested_id_in -= VIDEOFPS*2;
        lock_frame_data.unlock();
    }
    bool frame_by_frame;
    bool turbo;

    void update();


    int frame_number() {return _frame_number;}
    float frame_time() {return _frame_time;}
    cv::Mat Qf;
    rs2_intrinsics * intr;

    cv::Mat frameL,frameR;

    float depth_scale;
    cv::Mat depth_background;
    cv::Mat depth_background_3mm;
    cv::Mat depth_background_mm;
    cv::Mat disparity_background;

    float _camera_angle_x = 0;
    float _camera_angle_y = 30;
    float _camera_angle_y_measured_from_depth = 30;
    float camera_angle(){
        return _camera_angle_y;
    }
    float measured_exposure(){
        return _measured_exposure;
    }

    enum auto_exposure_enum{disabled = 0, enabled = 1, only_at_startup=2};
    const auto_exposure_enum enable_auto_exposure = only_at_startup;

    struct stereo_frame{
        cv::Mat frameL,frameR;
        uint id;
        float time;
    };

private:

    bool hasIMU = false;
    uint requested_id_in =0;
    int _frame_number;
    float _frame_time = 0;
    float _frame_time_start = -1;

    float _measured_exposure = 15400; // measured from sense_light_level
    int _measured_gain = 0;
    int exposure = 11000; //84*(31250/256); // >11000 -> 60fps, >15500 -> 30fps, < 20 = crash
    int gain = 0;
    bool fromfile;

    std::mutex lock_flags;
    std::mutex lock_frame_data;
    std::condition_variable wait_for_image;
    std::mutex m;
    rs2::device dev;

    bool _paused;

    const std::string calib_log_fn = "./logging/cam_calib.xml";
    const std::string calib_template_fn = "../cam_calib.xml";
    const std::string depth_map_fn = "./logging/depth_filtered.png";
    const std::string depth_unfiltered_map_fn = "./logging/depth.png";
    const std::string disparity_map_fn = "./logging/disparity.png";


    void pause();
    void resume();
    void seek(float time);
    void set_calibration(rs2::stream_profile infared1,rs2::stream_profile infared2);
    void update_real();
    void update_playback();
    void rs_callback(rs2::frame f);
    void rs_callback_playback(rs2::frame f);

    void sense_light_level();
    void calib_pose();
    cv::Point3f rotate_point(cv::Point3f point);
    void serialize_calib();
    void deserialize_calib(string file);
    void convert_depth_background_to_world();

    rs2::sensor depth_sensor;

    bool new_frame1 = false;
    bool new_frame2 = false;

    rs2::frame rs_frameL_cbtmp,rs_frameR_cbtmp;
    rs2::frame rs_frameL,rs_frameR;

    struct frame_data{
        cv::Mat frame;
        uint id;
        float time;
    };
    std::deque<frame_data> playback_bufferL;
    std::deque<frame_data> playback_bufferR;

};

#endif // CAM_H
