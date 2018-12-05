#include "cam.h"
#include <sys/stat.h>

#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

#ifdef HASSCREEN
//#define TUNING
#endif

stopwatch_c swc;

void Cam::update(void) {
    if (fromfile)
        update_playback();
    else
        update_real();
}

int last_1_id =-1;
int last_2_id =-1;
float incremented_playback_frametime = (1.f/VIDEOFPS)/2.f;
void Cam::update_playback(void) {
    frame_data fL,fR;
    bool foundL=false,foundR = false;
    while (true) {
        lock_frame_data.lock();
        std::deque<frame_data> playback_bufferL_cleaned;
        std::deque<frame_data> playback_bufferR_cleaned;
        for (uint i = 0 ; i <playback_bufferR.size();i++) {
            if (    playback_bufferR.at(i).id >= requested_id_in &&
                    playback_bufferR.at(i).id < requested_id_in+100)
                playback_bufferR_cleaned.push_back(playback_bufferR.at(i));
        }
        for (uint i = 0 ; i <playback_bufferL.size();i++) {
            if (    playback_bufferL.at(i).id >= requested_id_in &&
                    playback_bufferL.at(i).id < requested_id_in+100)
                playback_bufferL_cleaned.push_back(playback_bufferL.at(i));
        }

        playback_bufferR = playback_bufferR_cleaned;
        playback_bufferL = playback_bufferL_cleaned;
        lock_frame_data.unlock();

        for (uint i = 0 ; i <playback_bufferL_cleaned.size();i++) {
            if (playback_bufferL_cleaned.at(i).id == requested_id_in){
                fL = playback_bufferL_cleaned.at(i);
                foundL = true;
                break;
            }
        }
        if (foundL)
            for (uint i = 0 ; i <playback_bufferR_cleaned.size();i++) {
                if (playback_bufferR_cleaned.at(i).id == requested_id_in){
                    fR = playback_bufferR_cleaned.at(i);
                    foundR = true;
                    break;
                }
            }

        if (_paused && playback_bufferR.size() < 3 ){ // 3 to start buffering 3 frames before the buffer runs empty
            incremented_playback_frametime = (requested_id_in-2)*(1.f/VIDEOFPS) - (1.f/VIDEOFPS)*0.1f;  //requested_id_in-2 -> -2 seems to be necessary because the RS api skips a frame of either the left or right camera after resuming
            if (incremented_playback_frametime < 0)
                incremented_playback_frametime = 0;
            //            std::cout << "Resuming RS: " << incremented_playback_frametime << std::endl;
            seek(incremented_playback_frametime);
            resume();
        }

        if (foundL && foundR){
            break;
        }

        std::unique_lock<std::mutex> lk(m);
        wait_for_image.wait(lk,[this](){return new_frame1 && new_frame2;});
        new_frame1 = false;
        new_frame2 = false;
    }
    requested_id_in++;


    if (!turbo) {
        while(swc.Read() < (1.f/VIDEOFPS)*1e3f){
            usleep(10);
        }
        swc.Restart();
    }

    if (playback_bufferR.size() >= 10 && !_paused) {
        pause();
        //        std::cout << "Pausing playback_bufferL/R size:" << playback_bufferL.size() << " / " << playback_bufferR.size() << std::endl;
    }

    while(frame_by_frame){
        unsigned char k = cv::waitKey(1);
        if (k== 'f' || k == 27)
            break;
        else if (k== ' '){
            frame_by_frame = false;
            break;
        }
    }

    frameL = fL.frame.clone();
    frameR = fR.frame.clone();
    _frame_number = fL.id;
    _frame_time = fL.time;

    //std::cout << "-------------frame id: " << frame_id << " seek time: " << incremented_playback_frametime << std::endl;

}

void Cam::rs_callback_playback(rs2::frame f) {

    lock_frame_data.lock();
    //    if (f.get_profile().stream_index() == 1 )
    //        std::cout << "Received id "         << f.get_frame_number() << ":" << ((float)f.get_timestamp()-frame_time_start)/1e3f << "@" << f.get_profile().stream_index() << "         Last: " << last_1_id << "@1 and " << last_2_id << "@2 and requested id & time:" << requested_id_in << " & " << incremented_playback_frametime << " bufsize: " << playback_bufferL.size() << std::endl;
    //    if (f.get_profile().stream_index() == 2 )
    //        std::cout << "Received id         " << f.get_frame_number() << ":" << ((float)f.get_timestamp()-frame_time_start)/1e3f << "@" << f.get_profile().stream_index() << " Last: " << last_1_id << "@1 and " << last_2_id << "@2 and requested id & time:" << requested_id_in << " & " << incremented_playback_frametime << " bufsize: " << playback_bufferL.size() << std::endl;

    if (f.get_profile().stream_index() == 1 && f.get_frame_number() >= requested_id_in && playback_bufferL.size() < 100) {
        frame_data fL;
        fL.frame = Mat(Size(848, 480), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP).clone();
        fL.id= f.get_frame_number();
        if (_frame_time_start <0)
            _frame_time_start = f.get_timestamp();
        fL.time = ((float)f.get_timestamp() -_frame_time_start)/1000.f;

        playback_bufferL.push_back(fL);
        new_frame1 = true;
        last_1_id = fL.id;
    } else if (f.get_profile().stream_index() == 2 && f.get_frame_number() >= requested_id_in && playback_bufferR.size() < 100) {
        frame_data fR;
        fR.frame = Mat(Size(848, 480), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP).clone();
        fR.id= f.get_frame_number();
        if (_frame_time_start <0)
            _frame_time_start = f.get_timestamp();
        fR.time = ((float)f.get_timestamp() -_frame_time_start)/1000.f;
        playback_bufferR.push_back(fR);
        new_frame2 = true;
        last_2_id = fR.id;
    }
    lock_frame_data.unlock();
    if (new_frame1 && new_frame2) {
        wait_for_image.notify_all();
    }

}

void Cam::update_real(void) {
    std::unique_lock<std::mutex> lk(m);
    wait_for_image.wait(lk,[this](){return new_frame1 && new_frame2;});

    lock_frame_data.lock();
    frameL = Mat(Size(848, 480), CV_8UC1, (void*)rs_frameL.get_data(), Mat::AUTO_STEP);
    frameR = Mat(Size(848, 480), CV_8UC1, (void*)rs_frameR.get_data(), Mat::AUTO_STEP);
    _frame_number = rs_frameL.get_frame_number();
    if (_frame_time_start <0)
        _frame_time_start = rs_frameL.get_timestamp();
    _frame_time = ((float)rs_frameL.get_timestamp() -_frame_time_start)/1000.f;
    //std::cout << "-------------frame id: " << frame_id << " seek time: " << incremented_playback_frametime << std::endl;
    lock_frame_data.unlock();

    new_frame1 = false;
    new_frame2 = false;

}

uint last_sync_id = 0;
void Cam::rs_callback(rs2::frame f) {

    if (f.get_profile().stream_index() == 1 && !new_frame1 && f.get_frame_number() >= last_sync_id) {
        rs_frameL_cbtmp  = f;
        new_frame1 = true;
    } else if (f.get_profile().stream_index() == 2 && !new_frame2 && f.get_frame_number() >= last_sync_id) {
        rs_frameR_cbtmp = f;
        new_frame2 = true;
    }
    if (new_frame1 && new_frame2) {
        if (rs_frameL_cbtmp.get_frame_number() == rs_frameR_cbtmp.get_frame_number()) {
            lock_frame_data.lock();
            rs_frameL = rs_frameL_cbtmp;
            rs_frameR = rs_frameR_cbtmp;
            lock_frame_data.unlock();
            wait_for_image.notify_all();
            last_sync_id = rs_frameL.get_frame_number();
        }
        else { // somehow frames are not in sync, resync
            if (f.get_profile().stream_index() == 1)
                new_frame2 = false;
            else
                new_frame1 = false;
            std::cout << "Warning: frames not in sync" << std::endl;
        }
    }
}

void Cam::set_calibration(rs2::stream_profile infared1,rs2::stream_profile infared2) {
    // Obtain focal length and principal point (from intrinsics)
    auto depth_stream = infared1.as<rs2::video_stream_profile>();
    auto i = depth_stream.get_intrinsics();
    float focal_length = i.fx; // same as fy
    float cx = i.ppx; // same for both cameras
    float cy = i.ppy;

    // Obtain baseline (from extrinsics)
    rs2_extrinsics e = infared2.get_extrinsics_to(infared1);
    float baseline = e.translation[0];

    //init Qf: https://stackoverflow.com/questions/27374970/q-matrix-for-the-reprojectimageto3d-function-in-opencv
    Qf = (Mat_<double>(4, 4) << 1.0, 0.0, 0.0, -cx, 0.0, 1.0, 0.0, -cy, 0.0, 0.0, 0.0, focal_length, 0.0, 0.0, 1/baseline, 0.0);

}

void Cam::init() {

    std::cout << "Initializing cam" << std::endl;

    calib_pose();

    rs2::stream_profile infared1,infared2;
    rs2::context ctx; // The context represents the current platform with respect to connected devices
    rs2::device_list devices = ctx.query_devices();
    if (devices.size() == 0) {
        std::cerr << "No device connected, please connect a RealSense device" << std::endl;
        exit(1);
    } else if (devices.size() > 1) {
        std::cerr << "More than one device connected...." << std::endl;
        exit(1);
    } else {
        dev = devices[0];

        //record
        mkdir("./logging", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#if VIDEORAWLR == VIDEOMODE_BAG
        dev = rs2::recorder("./logging/test.bag",dev);
#endif

        std::cout << "Found the following device:\n" << std::endl;

        // Each device provides some information on itself, such as name:
        std::string name = "Unknown Device";
        if (dev.supports(RS2_CAMERA_INFO_NAME))
            name = dev.get_info(RS2_CAMERA_INFO_NAME);

        // and the serial number of the device:
        std::string sn = "########";
        if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
            sn = std::string("#") + dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

        std::cout << name << std::endl;

    }

    std::vector<rs2::sensor> sensors = dev.query_sensors();
    depth_sensor = sensors[0]; // 0 = depth module

    std::cout << depth_sensor.get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME) << std::endl;
    std::cout << depth_sensor.get_info(rs2_camera_info::RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl;
    std::cout << depth_sensor.get_info(rs2_camera_info::RS2_CAMERA_INFO_PRODUCT_ID) << std::endl;
    std::cout << depth_sensor.get_info(rs2_camera_info::RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION) << std::endl;

    std::string current_firmware_version = depth_sensor.get_info(rs2_camera_info::RS2_CAMERA_INFO_FIRMWARE_VERSION);
    std::string required_firmwar_version = "05.10.06.00";
    if (current_firmware_version.compare(required_firmwar_version)) {
        std::cout << "Detected wrong realsense firmware version!" << std::endl;
        exit(1);
    }

    depth_sensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 0);

    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);
    }
    //        if (depth_sensor.supports(RS2_OPTION_LASER_POWER)) {
    //            auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
    //            depth_sensor.set_option(RS2_OPTION_LASER_POWER, (range.max - range.min)/2 + range.min);
    //        }
    if (enable_auto_exposure == only_at_startup || enable_auto_exposure == enabled) {
        if (depth_sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
            depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE,1.0);
        }
    } else {
        if (depth_sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
            depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE,0.0);
        }
        if (depth_sensor.supports(RS2_OPTION_EXPOSURE)) {
            // auto range = depth_sensor.get_option_range(RS2_OPTION_EXPOSURE);
            depth_sensor.set_option(RS2_OPTION_EXPOSURE, exposure);
        }
        if (depth_sensor.supports(RS2_OPTION_GAIN)) {
            // auto range = depth_sensor.get_option_range(RS2_OPTION_GAIN);
            depth_sensor.set_option(RS2_OPTION_GAIN, gain); // increasing this causes noise
        }
    }

    std::vector<rs2::stream_profile> stream_profiles = depth_sensor.get_stream_profiles();
    infared1 = stream_profiles[17]; // infared 1 864x480 60fps
    infared2 = stream_profiles[16]; // infared 2 864x480 60fps

    depth_sensor.open({infared1,infared2});
    depth_sensor.start([&](rs2::frame f) { rs_callback(f); });
#ifdef TUNING
    namedWindow("Cam tuning", WINDOW_NORMAL);
    createTrackbar("Exposure", "Cam tuning", &exposure, 32768);
    createTrackbar("Gain", "Cam tuning", &gain, 32768);
#endif

    set_calibration(infared1,infared2);

    swc.Start();
}

void Cam::calib_pose(){
    rs2::config cfg;
    cfg.disable_all_streams();
    //cfg.enable_stream(RS2_STREAM_INFRARED, 1, IMG_W, IMG_H, RS2_FORMAT_Y8, VIDEOFPS);
    cfg.enable_stream(RS2_STREAM_DEPTH, IMG_W, IMG_H, RS2_FORMAT_Z16, VIDEOFPS);
    rs2::pipeline cam;
    rs2::pipeline_profile selection = cam.start(cfg);

    //wait 2 seconds
    for (uint i = 0; i < VIDEOFPS*2; i++)
        cam.wait_for_frames();

    rs2::frameset frame = cam.wait_for_frames();
    cv::Size im_size(IMG_W, IMG_H);
    depth_background = Mat(im_size, CV_16UC1, (void *)frame.get_depth_frame().get_data(), Mat::AUTO_STEP).clone();
    //cv::Mat frameL = Mat(im_size, CV_8UC1, (void *)frame.get_infrared_frame(1).get_data(), Mat::AUTO_STEP);
    //select the lower middle quadrant of the image:
    cv::Mat ground_plane_roi = depth_background(cv::Rect(IMG_W/4,IMG_H/2,IMG_W/2,IMG_H/2));
    imwrite("./logging/background_depth.png",depth_background);
    //cv::Mat ground_plane_roiL = frameL(cv::Rect(IMG_W/4,IMG_H/2,IMG_W/2,IMG_H/2));

    //calculate differential
    cv::Mat sobelx;
    cv::Mat sobely;
    cv::Sobel(ground_plane_roi,sobelx,CV_32F,1,0,5);
    cv::Sobel(ground_plane_roi,sobely,CV_32F,0,1,5);

    cv::Scalar x3 = mean(sobelx);
    cv::Scalar y3 = mean(sobely);
    cv::Scalar z3 = mean(ground_plane_roi);
    float x = x3(0);
    float y = y3(0);
    float z = z3(0);

    const float FOV_x_d = 91.2f;
    const float FOV_y_d = 65.5f;
    const float FOVrad_x_d = (FOV_x_d / 180.f) * (float)M_PI;
    const float FOVrad_y_d = (FOV_y_d / 180.f) * (float)M_PI;

    float scale_x = (float)ground_plane_roi.cols / (float)depth_background.cols;
    float scale_y = (float)ground_plane_roi.rows / (float)depth_background.rows;

    float world_width = tanf(FOVrad_x_d/2)*z*2.f*scale_x; //  [mm]
    float world_height = tanf(FOVrad_y_d/2)*z*2.f*scale_y; // [mm]
    float alpha_x = atanf(x/world_width);
    float alpha_y = atanf(y/world_height);
    _camera_angle_x = -(alpha_x/(float)M_PI)*180.f; //        [degrees]
    _camera_angle_y =  90 + (alpha_y/(float)M_PI)*180.f; //        [degrees]

    std::cout << "Camera pose x,y [ " << to_string_with_precision(_camera_angle_x,0) << "   ,   " <<
                 to_string_with_precision(_camera_angle_y,0) << " ]" << std::endl;

    cam.stop();
}

void Cam::init(int argc __attribute__((unused)), char **argv) {

    std::cout << "Initializing cam from .bag" << std::endl;
    fromfile=true;

    rs2::stream_profile infared1,infared2;
    rs2::context ctx; // The context represents the current platform with respect to connected devices
    dev = ctx.load_device(string(argv[1]) + ".bag");

    ((rs2::playback)dev).set_real_time(false);

    std::vector<rs2::sensor> sensors = dev.query_sensors();
    depth_sensor = sensors[0]; // 0 = depth module

    std::cout << depth_sensor.get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME) << std::endl;

    std::vector<rs2::stream_profile> stream_profiles = depth_sensor.get_stream_profiles();

    for (uint i = 0; i < stream_profiles.size();i++) {
        rs2::stream_profile sp = stream_profiles.at(i);
        std::cout << sp.stream_index() << " -  " << sp.stream_name() << " ; " << sp.stream_type() << std::endl;
        if ( sp.stream_name().compare("Infrared 1") == 0)
            infared1 = sp;
        else if ( sp.stream_name().compare("Infrared 2") == 0)
            infared2 = sp;
    }
    depth_sensor.open({infared1,infared2});
    depth_sensor.start([&](rs2::frame f) { rs_callback_playback(f); });
    pause();

    set_calibration(infared1,infared2);
    swc.Start();
}

void Cam::pause(){
    if (!_paused) {
        _paused = true;
        ((rs2::playback)dev).pause();
        //        std::cout << "Paused" << std::endl;
    }
}
void Cam::resume() {
    if (_paused){
        _paused = false;
        ((rs2::playback)dev).resume();
        //        std::cout << "Resumed" << std::endl;
    }
}
void Cam::seek(float time) {
    std::chrono::nanoseconds nano((ulong)(1e9f*time));
    ((rs2::playback)dev).seek(nano);
}

void Cam::close() {
    depth_sensor.stop();
    depth_sensor.close();
}
