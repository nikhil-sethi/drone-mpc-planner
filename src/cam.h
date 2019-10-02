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
#include <thread>

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

    void stop_watchdog() {exit_watchdog_thread = true;}

    void skip(float duration) {
        replay_skip_n_frames+=pparams.fps*duration;
    }
    void skip_one_sec() {
        replay_skip_n_frames+=pparams.fps;
    }
    void back_one_sec() {
        seek(_frame_time -3);
    }
    bool frame_by_frame;
    bool turbo;

    void update();

    float measure_auto_exposure();

    unsigned long long frame_number() {return _frame_number;}
    double frame_time() {return _frame_time;}
    cv::Mat Qf;
    rs2_intrinsics * intr;

    cv::Mat frameL,frameR;

    float depth_scale;
    cv::Mat depth_background;
    cv::Mat depth_background_3mm;
    cv::Mat depth_background_3mm_world;
    cv::Mat depth_background_mm;
    cv::Mat disparity_background;
    CameraVolume camera_volume;

    float _camera_angle_x = 0;
    float _camera_angle_y = 30;
    float _camera_angle_y_measured_from_depth = 30;
    float camera_angle(){
        return _camera_angle_y;
    }
    float measured_exposure(){
        return _measured_exposure;
    }
    float measured_gain(){
        return _measured_gain;
    }

    enum auto_exposure_enum{disabled = 0, enabled = 1, only_at_startup=2};
    const auto_exposure_enum enable_auto_exposure = enabled;

    struct stereo_frame{
        cv::Mat frameL,frameR;
        uint id;
        double time;
    };

private:

    bool hasIMU = false;
    unsigned long long _frame_number;
    double _frame_time = 0;
    double _frame_time_start = -1;
    uint replay_skip_n_frames = 0;

    float _measured_exposure = 15400; // measured from sense_light_level
    int _measured_gain = 0;
    int exposure = 11000; //84*(31250/256); // >11000 -> 60fps, >15500 -> 30fps, < 20 = crash
    int gain = 0;
    bool fromfile;
    bool initialized = false;

    bool watchdog = true;
    std::thread thread_watchdog;
    bool exit_watchdog_thread = false;
    void watchdog_thread(void);

    uint playback_buffer_size_max = 100;

    std::mutex lock_newframe;
    std::mutex lock_frame_data;
    rs2::device dev;
    rs2::pipeline cam;
    bool dev_initialized = false;

    bool _paused;

    std::string bag_fn = "record.bag";

    //read file names
    std::string calib_rfn = "cam_calib.xml";
    const std::string calib_template_rfn = "../../xml/" + calib_rfn;
    std::string depth_map_rfn = "depth_filtered.png";
    std::string depth_unfiltered_map_rfn = "depth.png";
    std::string disparity_map_rfn = "disparity.png";
    std::string brightness_map_rfn = "brightness.png";

    //write file names:
    std::string calib_wfn;
    std::string depth_map_wfn;
    std::string depth_unfiltered_map_wfn;
    std::string disparity_map_wfn;
    std::string brightness_map_wfn;

    CameraVolume def_volume();
    cv::Point3f get_SlopesOfPixel(uint x, uint y);
    void pause();
    void resume();
    void seek(double time);
    void calibration(rs2::stream_profile infared1,rs2::stream_profile infared2);
    void update_real();
    void update_playback();
    void rs_callback(rs2::frame f);
    void rs_callback_playback(rs2::frame f);

    void check_light_level();

    void calib_pose(bool also_do_depth);
    void serialize_calib();
    void deserialize_calib(string file);
    void convert_depth_background_to_world();

    bool new_frame1 = false;
    bool new_frame2 = false;

    rs2::frame rs_frameL_cbtmp,rs_frameR_cbtmp;
    rs2::frame rs_frameL,rs_frameR;




};

#endif // CAM_H
