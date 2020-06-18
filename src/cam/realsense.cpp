#include "realsense.h"
#include <sys/stat.h>
#include "smoother.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <librealsense2/rsutil.h>
#include "third_party/stopwatch.h"

using namespace cv;
using namespace std;

static stopwatch_c swc;

void Realsense::update(void) {
    if (from_recorded_bag)
        update_playback();
    else
        update_real();
}

double incremented_playback_frametime = (1.f/pparams.fps)/2.f;
void Realsense::update_playback(void) {

    rs2::frameset fs;
    for (uint i = 0; i < replay_skip_n_frames+1; i++)
        fs = cam.wait_for_frames();
    replay_skip_n_frames = 0;
    if (_frame_time_start <0)
        _frame_time_start = fs.get_timestamp();
    _frame_number = fs.get_frame_number();
    _frame_time = (fs.get_timestamp() -_frame_time_start)/1000.;

    double duration = static_cast<double>(static_cast<rs2::playback>(dev).get_duration().count()) / 1e9;
    if (frame_time() > duration-0.1) {
        std::cout << "Video end, exiting" << std::endl;
        throw bag_video_ended();
    }
    frameL = Mat(Size(IMG_W, IMG_H), CV_8UC1, const_cast<void *>(fs.get_infrared_frame(1).get_data()), Mat::AUTO_STEP).clone();
    frameR = Mat(Size(IMG_W, IMG_H), CV_8UC1, const_cast<void *>(fs.get_infrared_frame(2).get_data()), Mat::AUTO_STEP).clone();

    if (!turbo) {
        while(swc.Read() < (1.f/pparams.fps)*1e3f) {
            usleep(10);
        }
        swc.Restart();
    }

    while(frame_by_frame) {
        unsigned char k = cv::waitKey(1);
        if (k == 'f')
            break;
        else if (k== ' ') {
            frame_by_frame = false;
            break;
        }
    }
}

void Realsense::update_real(void) {

    lock_newframe.lock(); // wait for a new frame passed by the rs callback

    lock_frame_data.lock();
    frameL = Mat(Size(IMG_W, IMG_H), CV_8UC1, const_cast<void *>(rs_frameL.get_data()), Mat::AUTO_STEP).clone();
    frameR = Mat(Size(IMG_W, IMG_H), CV_8UC1, const_cast<void *>(rs_frameR.get_data()), Mat::AUTO_STEP).clone();

    if (rs_frameL.supports_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE)) {
        camparams.measured_exposure = rs_frameL.get_frame_metadata(rs2_frame_metadata_value::RS2_FRAME_METADATA_ACTUAL_EXPOSURE);
    }

    _frame_number = rs_frameL.get_frame_number();
    if (_frame_time_start <0)
        _frame_time_start = rs_frameL.get_timestamp();
    _frame_time = (rs_frameL.get_timestamp() -_frame_time_start)/1000.;
    //    std::cout << "-------------frame id: " << _frame_number << " time: " << _frame_time << std::endl;
    lock_frame_data.unlock();

    watchdog = true;

}

uint last_sync_id = 0;
void Realsense::rs_callback(rs2::frame f) {

    //    if (f.get_profile().stream_index() == 1 )
    //        std::cout << "Received id "         << f.get_frame_number() << ":" << (static_cast<float>(f.get_timestamp()) -_frame_time_start)/1e3f << "@" << f.get_profile().stream_index() << "         Last: " << last_sync_id << std::endl;
    //    if (f.get_profile().stream_index() == 2 )
    //        std::cout << "Received id         " << f.get_frame_number() << ":" << (static_cast<float>(f.get_timestamp()) -_frame_time_start)/1e3f << "@" << f.get_profile().stream_index() << " Last: " << last_sync_id << std::endl;

    if (f.get_frame_number() < last_sync_id-50 && last_sync_id > 300) {
        std::cout << "Warning: rs frame number reset happened!!!" << std::endl;
        if (last_sync_id > 300)
            return;
        last_sync_id = f.get_frame_number();
    }

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

            lock_newframe.unlock(); // signal to processor that a new frame is ready to be processed
            new_frame1 = false;
            new_frame2 = false;
            int fl = rs_frameL.get_frame_number() - last_sync_id;
            if (fl > 3) {
                std::cout << "FRAME LOSS: " << fl << std::endl;
                _frame_loss_cnt++;
            }
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

void Realsense::calibration(rs2::stream_profile infared1,rs2::stream_profile infared2) {
    // Obtain focal length and principal point (from intrinsics)
    auto depth_stream = infared1.as<rs2::video_stream_profile>();
    auto i = depth_stream.get_intrinsics();
    float focal_length = i.fx; // same as fy
    float cx = i.ppx; // same for both cameras
    float cy = i.ppy;

    // Obtain baseline (from extrinsics)
    rs2_extrinsics e = infared1.get_extrinsics_to(infared2);
    float baseline = e.translation[0];

    baseline = fabs(baseline);
    if (baseline < 0.04f) //TODO: https://github.com/IntelRealSense/librealsense/issues/5451
        baseline = 0.05f;

    //init Qf: https://stackoverflow.com/questions/27374970/q-matrix-for-the-reprojectimageto3d-function-in-opencv
    Qf = (Mat_<double>(4, 4) << 1.0, 0.0, 0.0, -cx, 0.0, 1.0, 0.0, -cy, 0.0, 0.0, 0.0, focal_length, 0.0, 0.0, 1/baseline, 0.0);
    intr = new rs2_intrinsics(i);

    camparams.fx = intr->fx;
    camparams.fy = intr->fy;
    camparams.ppx = intr->ppx;
    camparams.ppy = intr->ppy;
    camparams.height = intr->height;
    camparams.width = intr->width;
    camparams.model = intr->model;
    camparams.coeffs[0] = intr->coeffs[0];
    camparams.coeffs[1] = intr->coeffs[1];
    camparams.coeffs[2] = intr->coeffs[2];
    camparams.coeffs[3] = intr->coeffs[3];
    camparams.coeffs[4] = intr->coeffs[4];
    camparams.baseline = e.translation[0];
}
void Realsense::connect_and_check() {

    rs2::context ctx; // The context represents the current platform with respect to connected devices
    if (!dev_initialized) {
        rs2::device_list devices = ctx.query_devices();
        if (devices.size() == 0) {
            throw my_exit("no RealSense connected");
        } else if (devices.size() > 1) {
            throw my_exit("more than one RealSense connected....");
        } else
            dev = devices[0];
    }

    std::cout << "Found the following device:";

    // Each device provides some information on itself, such as name:
    std::string name = "Unknown Device";
    if (dev.supports(RS2_CAMERA_INFO_NAME))
        name = dev.get_info(RS2_CAMERA_INFO_NAME);

    // and the serial number of the device:
    std::string sn = "########";
    if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
        sn = std::string("#") + dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

    std::size_t found = name.find("D435I");
    hasIMU = found!=std::string::npos;

    std::cout << name << " ser: " << sn << std::endl;

    rs2::depth_sensor rs_depth_sensor = dev.first<rs2::depth_sensor>();
    std::cout << rs_depth_sensor.get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME) << std::endl;
    std::cout << rs_depth_sensor.get_info(rs2_camera_info::RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl;
    std::cout << rs_depth_sensor.get_info(rs2_camera_info::RS2_CAMERA_INFO_PRODUCT_ID) << std::endl;

    std::string required_firmwar_version = "05.12.03.00";
    std::string current_firmware_version = rs_depth_sensor.get_info(rs2_camera_info::RS2_CAMERA_INFO_FIRMWARE_VERSION);
    current_firmware_version  = current_firmware_version.substr (0,required_firmwar_version.length()); //fix for what seems to be appended garbage...? 255.255.255.255 on a newline

    if (current_firmware_version != required_firmwar_version) { // wtf, string equality check is reversed!??
        std::stringstream serr;
        serr << "detected wrong RealSense firmware version! Detected: " << current_firmware_version << ". Required: "  << required_firmwar_version << ".";
        throw my_exit(serr.str());
    }


}
void Realsense::init_real() {
    lock_newframe.lock();
    std::cout << "Initializing cam" << std::endl;

    bag_fn = "./logging/" + playback_filename();

    calib_rfn = "./logging/" + calib_rfn;
    depth_map_rfn = "./logging/" + depth_map_rfn;
    depth_unfiltered_map_rfn = "./logging/" + depth_unfiltered_map_rfn;
    disparity_map_rfn = "./logging/" + disparity_map_rfn;
    brightness_map_rfn = "./logging/" + brightness_map_rfn;

    calib_wfn = calib_rfn;
    depth_map_wfn = depth_map_rfn;
    depth_unfiltered_map_wfn = depth_unfiltered_map_rfn;
    disparity_map_wfn = disparity_map_rfn;
    brightness_map_wfn = brightness_map_rfn;

    rs2::depth_sensor rs_depth_sensor = dev.first<rs2::depth_sensor>();

    if (enable_auto_exposure == only_at_startup)
        check_light_level();
    else {
        std::tie (camparams.measured_exposure,std::ignore)  = measure_auto_exposure();
        std::cout << "Measured auto exposure: " << camparams.measured_exposure << std::endl;
    }

    depth_background = imread(depth_map_rfn,cv::IMREAD_ANYDEPTH);
    camparams.depth_scale = rs_depth_sensor.get_option(RS2_OPTION_DEPTH_UNITS);

    calib_pose(true);

    rs_depth_sensor = dev.first<rs2::depth_sensor>();

    rs_depth_sensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 0);

    if (rs_depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        rs_depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);
    }

    if (enable_auto_exposure == only_at_startup ) {
        exposure = camparams.measured_exposure;
        if (pparams.fps==60 && camparams.measured_exposure>15500) //guarantee 60 FPS when requested
            exposure = 15500;
        gain = camparams.measured_gain;
    }
    if ( enable_auto_exposure == enabled) {
        if (rs_depth_sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
            rs_depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE,1.0);
        }
    } else {
        if (rs_depth_sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
            rs_depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE,0.0);
        }
        if (rs_depth_sensor.supports(RS2_OPTION_EXPOSURE)) {
            // auto range = rs_depth_sensor.get_option_range(RS2_OPTION_EXPOSURE);
            rs_depth_sensor.set_option(RS2_OPTION_EXPOSURE, exposure);
        }
        if (rs_depth_sensor.supports(RS2_OPTION_GAIN)) {
            // auto range = rs_depth_sensor.get_option_range(RS2_OPTION_GAIN);
            rs_depth_sensor.set_option(RS2_OPTION_GAIN, gain); // increasing this causes noise
        }
    }

    std::vector<rs2::stream_profile> stream_profiles = rs_depth_sensor.get_stream_profiles();
    rs2::stream_profile infared1,infared2;
    if (pparams.fps == 60) {
        infared1 = stream_profiles[17]; // infared 1 864x480 60fps
        infared2 = stream_profiles[16]; // infared 2 864x480 60fps
    } else {
        infared1 = stream_profiles[15]; // infared 1 864x480 90fps
        infared2 = stream_profiles[14]; // infared 2 864x480 90fps
    }

    //TODO: automate above with below:
    // for (uint i = 0; i < stream_profiles.size(); i++) {
    //     try {
    //         if (auto video_stream = stream_profiles[i].as<rs2::video_stream_profile>()) {
    //             rs2_intrinsics intrinsics = video_stream.get_intrinsics();
    //             std::cout << video_stream.stream_name() << " " << i << ": " << intrinsics.width << " x " << intrinsics.height << " @" << video_stream.fps() << std::endl;
    //         }
    //     } catch (const std::exception& e) {}
    // }

    rs_depth_sensor.open({infared1,infared2});
    rs_depth_sensor.start([&](rs2::frame f) { rs_callback(f); });
    if (pparams.cam_tuning) {
        namedWindow("Cam tuning", WINDOW_NORMAL);
        createTrackbar("Exposure", "Cam tuning", &exposure, 32768);
        createTrackbar("Gain", "Cam tuning", &gain, 32768);
    }

    calibration(infared1,infared2);
    camparams.serialize(calib_wfn);
    convert_depth_background_to_world();

    if (pparams.watchdog)
        thread_watchdog = std::thread(&Realsense::watchdog_thread,this);

    if (pparams.video_raw == video_bag)
        dev = rs2::recorder(bag_fn,dev);

    def_volume();

    update();
    initialized = true;
}

void Realsense::check_light_level() {

    std::cout << "Checking if scene brightness has changed" << std::endl;
    //boot the camera and set it to the same settings as the previous session
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, IMG_W, IMG_H, RS2_FORMAT_Y8, pparams.fps);
    cam.start(cfg);
    dev = cam.get_active_profile().get_device(); // after a cam start, dev is changed
    rs2::depth_sensor rs_dev = dev.first<rs2::depth_sensor>();

    rs_dev.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.0);
    rs_dev.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);

    cv::Size im_size(IMG_W, IMG_H);
    cv::Mat frameLt;

    rs2::frameset frame;
    float new_expos = 0,new_gain = 0;
    int nframes_delay = 30.f;
    for (int i = 0; i< nframes_delay; i++) // allow some time to settle
        frame = cam.wait_for_frames();

    float tmp_exposure =-1;
    float tmp_gain =-1;
    int tmp_last_exposure_frame_id = 0;
    for (int i = 0; i< 120; i++) { // check for large change in exposure
        frame = cam.wait_for_frames();
        frameLt = Mat(im_size, CV_8UC1, const_cast<void *>(frame.get_infrared_frame(1).get_data()), Mat::AUTO_STEP);
        new_gain = rs_dev.get_option(RS2_OPTION_GAIN);
        if (frame.supports_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE)) {
            new_expos = frame.get_frame_metadata(rs2_frame_metadata_value::RS2_FRAME_METADATA_ACTUAL_EXPOSURE);
            if (fabs(new_expos - tmp_exposure) > 0.5f  || fabs(tmp_gain-new_gain) > 0.5f)
                tmp_last_exposure_frame_id = i;
            if (i - tmp_last_exposure_frame_id >= 10)
                break;
            tmp_exposure = new_expos;
        }
    }
    camparams.measured_exposure = new_expos;
    camparams.measured_gain = new_gain;

    imwrite(brightness_map_wfn,frameLt);

    cam.stop();
}

std::tuple<float,cv::Mat> Realsense::measure_auto_exposure() {

    if (!dev_initialized) {
        rs2::context ctx;
        rs2::device_list devices = ctx.query_devices();
        if (devices.size() == 0) {
            throw my_exit("no RealSense connected");
        } else if (devices.size() > 1) {
            throw my_exit("more than one RealSense connected....");
        } else {
            dev = devices[0];
            dev_initialized = true;
        }
    }

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, IMG_W, IMG_H, RS2_FORMAT_Y8, pparams.fps);
    cam.start(cfg);
    dev = cam.get_active_profile().get_device(); // after a cam start, dev is changed
    rs2::depth_sensor rs_dev = dev.first<rs2::depth_sensor>();

    rs_dev.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.0);
    rs_dev.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);

    cam.wait_for_frames(); // first frame seems to be weird, ignore it

    cv::Size im_size(IMG_W, IMG_H);
    cv::Mat frameLt;

    rs2::frameset frame;
    float new_expos;

    float tmp_exposure =0;
    int tmp_last_exposure_frame_id = 0;
    int actual_exposure_was_measured = 0;
    int i =0;
    for (i= 0; i< 120; i++) { // check for large change in exposure
        frame = cam.wait_for_frames();
        frameLt = Mat(im_size, CV_8UC1, const_cast<void *>(frame.get_infrared_frame(1).get_data()), Mat::AUTO_STEP);
        if (frame.supports_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE)) {
            new_expos = frame.get_frame_metadata(rs2_frame_metadata_value::RS2_FRAME_METADATA_ACTUAL_EXPOSURE);
            actual_exposure_was_measured++;
            if (fabs(new_expos - tmp_exposure) > 0.5f )
                tmp_last_exposure_frame_id = i;
            if (i - tmp_last_exposure_frame_id >= 15)
                break;
            tmp_exposure = new_expos;
        }
    }
    cam.stop();
    if (!actual_exposure_was_measured)
        std::cout << "Warning: no exposure data could be found!!!" << std::endl;
    else if (actual_exposure_was_measured!=i+1)
        std::cout << "Not all frames contained exosure info: " << actual_exposure_was_measured << " / " << i << std::endl;

    return std::make_tuple(new_expos,frameLt);

}

std::tuple<float,float,double,cv::Mat> Realsense::measure_angle() {
    if (!dev_initialized) {
        rs2::context ctx;
        rs2::device_list devices = ctx.query_devices();
        if (devices.size() == 0) {
            throw my_exit("no RealSense connected");
        } else if (devices.size() > 1) {
            throw my_exit("more than one RealSense connected....");
        } else {
            dev = devices[0];
            dev_initialized = true;
        }
    }

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, IMG_W, IMG_H, RS2_FORMAT_Y8, pparams.fps);
    cfg.enable_stream(RS2_STREAM_ACCEL);
    cfg.enable_stream(RS2_STREAM_GYRO);
    cam.start(cfg);
    dev = cam.get_active_profile().get_device(); // after a cam start, dev is changed

    rs2::depth_sensor rs_dev = dev.first<rs2::depth_sensor>();
    cv::Size im_size(IMG_W, IMG_H);
    rs_dev.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.0);
    rs_dev.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);

    uint nframes = 30;
    filtering::Smoother smx,smy,smz;
    smx.init(static_cast<int>(nframes));
    smy.init(static_cast<int>(nframes));
    smz.init(static_cast<int>(nframes));
    float x = 0,y = 0,z = 0, roll = 0, pitch = 0;
    rs2::frameset frame;
    cv::Mat frameLt;

    for (uint i = 0; i < nframes; i++) {
        frame = cam.wait_for_frames();
        frameLt = Mat(im_size, CV_8UC1, const_cast<void *>(frame.get_infrared_frame(1).get_data()), Mat::AUTO_STEP);
        auto frame_acc = frame.first(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);

        if (frame_acc.is<rs2::motion_frame>()) {
            rs2::motion_frame mf = frame_acc.as<rs2::motion_frame>();
            rs2_vector xyz = mf.get_motion_data();
            x = smx.addSample(xyz.x);
            y = smy.addSample(xyz.y);
            z = smz.addSample(xyz.z);
            roll = atanf(-x/sqrtf(y*y + z*z)) * rad2deg;
            pitch = 90.f-atanf(y/z) * rad2deg;
        }
    }

    cam.stop();
    return std::make_tuple(roll,pitch,frame.get_timestamp(),frameLt);

}

void Realsense::calib_pose(bool also_do_depth) {

    std::cout << "Measuring pose..." << std::endl;
    auto rs_depth_sensor = dev.first<rs2::depth_sensor>();
    camparams.depth_scale = rs_depth_sensor.get_option(RS2_OPTION_DEPTH_UNITS);

    if (rs_depth_sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE))
        rs_depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.0);
    if (rs_depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
        rs_depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f);

    // Find the High-Density preset by name
    // We do this to reduce the number of black pixels
    // The hardware can perform hole-filling much better and much more power efficient then our software
    // Set the device to High Accuracy preset of the D400 stereoscopic cameras
    if (rs_depth_sensor && rs_depth_sensor.is<rs2::depth_stereo_sensor>())
    {
        rs_depth_sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
    } else {
        throw my_exit("This is not a depth sensor");
    }

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, IMG_W, IMG_H, RS2_FORMAT_Z16, pparams.fps);
    if (hasIMU) {
        cfg.enable_stream(RS2_STREAM_ACCEL);
        cfg.enable_stream(RS2_STREAM_GYRO);
    }
    cam.start(cfg);
    dev = cam.get_active_profile().get_device(); // after a cam start, dev is changed
    cv::Size im_size(IMG_W, IMG_H);

    uint nframes = 1;
    if (hasIMU)
        nframes = 10;
    filtering::Smoother smx,smy,smz;
    smx.init(static_cast<int>(nframes));
    smy.init(static_cast<int>(nframes));
    smz.init(static_cast<int>(nframes));
    float x = 0,y = 0,z = 0, roll = 0, pitch = 0;
    rs2::frameset frame;
    uint wait_cycles = 1;
    while (wait_cycles) {
        for (uint i = 0; i < nframes; i++) {
            frame = cam.wait_for_frames();

            if (hasIMU) {
                auto frame_acc = frame.first(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);

                if (frame_acc.is<rs2::motion_frame>()) {
                    rs2::motion_frame mf = frame_acc.as<rs2::motion_frame>();
                    rs2_vector xyz = mf.get_motion_data();
                    x = smx.addSample(xyz.x);
                    y = smy.addSample(xyz.y);
                    z = smz.addSample(xyz.z);
                    roll = atanf(-x/sqrtf(y*y + z*z)) * rad2deg;
                    pitch = 90.f-atanf(y/z) * rad2deg;
                }
            }
        }
        if (hasIMU)
            std::cout << "Camera roll: " << to_string_with_precision(roll,2) << "°- max: " << pparams.max_cam_roll << "°. Pitch: " << to_string_with_precision(pitch,2) << "°" << std::endl;
        wait_cycles--;
    }

    if (also_do_depth || !hasIMU) {

        // Decimation filter reduces the amount of data (while preserving best samples)
        rs2::decimation_filter dec;
        // If the demo is too slow, make sure you run in Release (-DCMAKE_BUILD_TYPE=Release)
        // but you can also increase the following parameter to decimate depth more (reducing quality)
        dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
        // Define transformations from and to Disparity domain
        rs2::disparity_transform depth2disparity;
        rs2::disparity_transform disparity2depth(false);
        // Define spatial filter (edge-preserving)
        rs2::spatial_filter spat;
        // Enable hole-filling
        // Hole filling is an aggressive heuristic and it gets the depth wrong many times
        // However, this demo is not built to handle holes
        // (the shortest-path will always prefer to "cut" through the holes since they have zero 3D distance)
        spat.set_option(RS2_OPTION_HOLES_FILL, 5); // 5 = fill all the zero pixels
        // Define temporal filter
        rs2::temporal_filter temp;
        // Spatially align all streams to depth viewport
        // We do this because:
        //   a. Usually depth has wider FOV, and we only really need depth for this demo
        //   b. We don't want to introduce new holes
        rs2::align align_to(RS2_STREAM_DEPTH);
        auto depth = dec.process(frame.get_depth_frame());
        // To make sure far-away objects are filtered proportionally
        // we try to switch to disparity domain
        depth = depth2disparity.process(depth);
        // Apply spatial filtering
        depth = spat.process(depth);
        // Apply temporal filtering
        depth = temp.process(depth);
        // If we are in disparity domain, switch back to depth
        cv::Size im_size_dec(IMG_W/2, IMG_H/2);
        disparity_background = Mat(im_size_dec, CV_32F, const_cast<void *>(depth.get_data()), Mat::AUTO_STEP).clone();
        cv::resize(disparity_background,disparity_background,im_size,0,0,INTER_CUBIC);
        depth = disparity2depth.process(depth);

        //to get a x,y distance
        //frame.get_depth_frame().get_distance()

        //original depth map:
        depth_background = Mat(im_size, CV_16UC1, const_cast<void *>(frame.get_depth_frame().get_data()), Mat::AUTO_STEP).clone();
        imwrite(depth_unfiltered_map_wfn,depth_background);
        //filtered depth map:
        depth_background = Mat(im_size_dec, CV_16UC1, const_cast<void *>(depth.get_data()), Mat::AUTO_STEP).clone();
        cv::resize(depth_background,depth_background,im_size,0,0,INTER_CUBIC);
        imwrite(depth_map_wfn,depth_background);

        imwrite(disparity_map_wfn,disparity_background);
    }
    if (hasIMU) {
        camparams.camera_angle_x = roll;
        camparams.camera_angle_y = pitch;
    } else { // determine camera angle from the depth map:

        // obtain Point Cloud
        rs2::pointcloud pc;
        rs2::points points;
        points = pc.calculate(frame.get_depth_frame());

        // init variables for plane fitting
        auto ptr = points.get_vertices();
        int pp;
        int p_smooth = 400;
        int nr_p_far = 50;
        int nr_p_close = 50;
        int ppp;

        std::vector<cv::Point3f> pointsFar;
        std::vector<cv::Point3f> pointsClose;
        cv::Point3f point_ = {0,0,0};
        cv::Point3f point_diff = {0,0,0};
        float alpha = 0;

        // obtain a set of points 'far away' (half way the image) that are averaged over image lines
        for (ppp=0; ppp<nr_p_far; ppp++) {
            auto ptr_new = ptr+points.size()-IMG_W/2-p_smooth/2-IMG_W*(nr_p_close+1+ppp);
            for (pp=0; pp<p_smooth; pp++) {
                point_.x += ptr_new->x;
                point_.y += ptr_new->y;
                point_.z += ptr_new->z;
                ptr_new++;
            }
            point_/=p_smooth;
            pointsFar.push_back(point_);
        }

        // obtain a set of points 'close by' (bottom of the image) that are averaged over image lines
        for (ppp=0; ppp<nr_p_close; ppp++) {
            auto ptr_new = ptr+points.size()-IMG_W/2-p_smooth/2-IMG_W*ppp;
            for (pp=0; pp<p_smooth; pp++) {
                point_.x += ptr_new->x;
                point_.y += ptr_new->y;
                point_.z += ptr_new->z;
                ptr_new++;
            }
            point_/=p_smooth;
            pointsClose.push_back(point_);
        }

        // calculate angle between 'far away' and 'close by' points, by avaraging over all combinations between both sets
        for (ppp=0; ppp<nr_p_close; ppp++) {
            for (pp=0; pp<nr_p_far; pp++) {
                point_diff = pointsFar.at(pp)-pointsClose.at(ppp);
                alpha += atanf(point_diff.y/point_diff.z);
            }
        }
        alpha/=(nr_p_close*nr_p_far);

        float old = _camera_angle_y_measured_from_depth;

        _camera_angle_y_measured_from_depth = -alpha*rad2deg; //[degrees]
        std::cout << "Measured angle: " << _camera_angle_y_measured_from_depth << std::endl;
        std::cout << "Estimated angle change: " << _camera_angle_y_measured_from_depth - old << std::endl;
        std::cout << "Set angle_y: " << camparams.camera_angle_y << std::endl;

        if ((fabs(_camera_angle_y_measured_from_depth - old) > 15 && file_exist(calib_rfn)) || _camera_angle_y_measured_from_depth != _camera_angle_y_measured_from_depth) {
            _camera_angle_y_measured_from_depth = 0;
            throw my_exit("camera angle change to big!");
        }
    }
    cam.stop();
}

void Realsense::init_playback() {
    std::cout << "Initializing cam from " << bag_fn << std::endl;
    if (!file_exist(bag_fn)) {
        std::stringstream serr;
        serr << "cannot not find " << bag_fn;
        throw my_exit(serr.str());
    }

    if (!file_exist(depth_map_rfn)) {
        //todo: make gui warning of this:
        std::cout << "Warning: could not find " << depth_map_rfn << std::endl;
        depth_background = cv::Mat::ones(IMG_H,IMG_W,CV_16UC1);
        depth_background = 10000; // basically disable the depth background map if it is not found
    } else {
        depth_background = imread(depth_map_rfn,cv::IMREAD_ANYDEPTH);
    }

    rs2::config cfg;
    cfg.enable_device_from_file(bag_fn);
    cam.start(cfg);
    dev = cam.get_active_profile().get_device();
    static_cast<rs2::playback>(dev).set_real_time(false);

    rs2::stream_profile infared1,infared2;
    auto rs_depth_sensor = dev.first<rs2::depth_sensor>();
    camparams.depth_scale = rs_depth_sensor.get_option(RS2_OPTION_DEPTH_UNITS);

    std::cout << rs_depth_sensor.get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME) << std::endl;

    std::vector<rs2::stream_profile> stream_profiles = rs_depth_sensor.get_stream_profiles();

    for (uint i = 0; i < stream_profiles.size(); i++) {
        rs2::stream_profile sp = stream_profiles.at(i);
        std::cout << sp.stream_index() << " -  " << sp.stream_name() << " ; " << sp.stream_type() << std::endl;
        if ( sp.stream_name().compare("Infrared 1") == 0)
            infared1 = sp;
        else if ( sp.stream_name().compare("Infrared 2") == 0)
            infared2 = sp;
    }

    calibration(infared1,infared2);
    convert_depth_background_to_world();
    camparams.serialize(calib_wfn); // tmp?

    def_volume();

    std::cout << "Awaiting first image..." << std::endl;
    swc.Start();
    update();
    initialized = true;
}

void Realsense::seek(double time) {
    if (from_recorded_bag) {
        std::chrono::nanoseconds nano(static_cast<ulong>(1e9*time));
        static_cast<rs2::playback>(dev).seek(nano);
    }
}

void Realsense::close() {
    if (initialized) {
        exit_watchdog_thread = true;
        std::cout << "Closing camera" << std::endl;
        if (!dev.as<rs2::playback>()) {
            auto rs_depth_sensor = dev.first<rs2::depth_sensor>();
            lock_newframe.unlock();
            lock_frame_data.unlock();
            rs_depth_sensor.stop();
            rs_depth_sensor.close();
            for (uint i = 0; i < 10; i++ ) {
                lock_newframe.unlock();
                lock_frame_data.unlock();
                usleep(1000);
            }
        } else {
            cam.stop();
        }
        if (pparams.watchdog && !from_recorded_bag) {
            std::cout << "Waiting for camera watchdog." << std::endl;
            thread_watchdog.join();
        }
        std::cout << "Camera closed" << std::endl;
        initialized = false;
    }
    Cam::close();
}

void Realsense::reset() {
    std::cout << "Resetting cam" << std::endl;

    rs2::stream_profile infared1,infared2;
    rs2::context ctx; // The context represents the current platform with respect to connected devices
    rs2::device_list devices = ctx.query_devices();
    if (devices.size() == 0) {
        throw my_exit("no RealSense connected");
    } else if (devices.size() > 1) {
        std::cout << "Warning detected more then one device. Resetting everything" << std::endl;
        rs2::device devt;
        for (uint i = 0; i < devices.size(); i++) {
            devt= devices[i];
            std::cout << i << std::endl;
            devt.hardware_reset();
            usleep(100000);
        }

        usleep(1000000);

        exit (0);

    } else {
        dev = devices[0];

        std::cout << "Sending hardware reset..." << std::endl;
        dev.hardware_reset();

        usleep(1000000);

        exit (0);
    }
}

void Realsense::watchdog_thread(void) {
    std::cout << "Watchdog thread started" << std::endl;
    usleep(10000000); //wait until camera is running for sure
    while (!exit_watchdog_thread) {
        usleep(pparams.wdt_timeout_us);
        if (!watchdog && !exit_watchdog_thread) {
            std::cout << "Watchdog alert! Attempting to continue" << std::endl;
            new_frame1 =true;
            new_frame2 = true;
            lock_newframe.unlock(); // wait for a new frame passed by the rs callback
            usleep(pparams.wdt_timeout_us);
            if (!watchdog) {
                std::cout << "Watchdog alert! Killing the process." << std::endl;
                auto res [[maybe_unused]] = std::system("killall -9 pats");
            }
        }
        watchdog = false;
    }
}
