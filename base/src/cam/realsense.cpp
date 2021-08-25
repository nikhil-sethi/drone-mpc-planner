#include "realsense.h"
#include <sys/stat.h>
#include "smoother.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <librealsense2/rsutil.h>
#include "third_party/stopwatch.h"

using namespace cv;
using namespace std;

static stopwatch_c swc;
bool new_frame_ready{false};

StereoPair * Realsense::update(void) {
    auto buf_copy = buf;
    auto last_iter = buf_copy.cend();
    last_iter--;
    auto new_current = last_iter;
    _last = last_iter->second;

    bool skipping_frames = false;

    if (new_current->second->rs_id == _current->rs_id) { // we need to wait for a new frame. Processing won :)
        while (last_iter->second->rs_id==_current->rs_id) {
            update_real();
            last_iter = buf.cend();
            last_iter--;
        }
        _current = last_iter->second;
    } else if (new_current->second->rs_id == _current->rs_id+1) { // next frame on top of the buffer and ready to go. Camera won :|
        _current = new_current->second;
    } else { // we are lagging behind or the rs_id has skipped one. :(
        auto current_original = _current;
        unsigned long long new_rs_id = -1;
        new_current = buf_copy.cend();
        while ((new_rs_id > current_original->rs_id && new_current != buf_copy.cbegin())) {
            new_current--;
            if (new_current ==  buf_copy.cbegin() || new_current->second->rs_id - current_original->rs_id>3) {
                if (current_original->rs_id == _current->rs_id) {
                    _current = _last;
                    if (_current->rs_id > pparams.fps*2)
                        std::cout << "Skipping frames :(" << std::endl;
                }
                break;
            } else if (current_original->rs_id == new_current->second->rs_id && _current->rs_id != current_original->rs_id) {
                break;
            } else
                _current = new_current->second;
            new_rs_id = new_current->second->rs_id;
        }
        skipping_frames = true;
        if (_current->rs_id > pparams.fps*2)
            std::cout << "Frame lag warning" << std::endl;
    }
    _current->processing = true;
    delete_old_frames(skipping_frames);
    return _current;
}

void Realsense::delete_old_frames(bool skipping_frames) {
    //At the moment we actually only need 1 frame from the past.
    //Sometimes there's some processing delay in the process_frame thread, so we have another few frames as buffer.
    //If we set this number to bigger then 7 the RS software locks up. Apparantely there is some limit of
    //keeping 7 frame callbacks in memory, even when not using the rs pipeline system.

    auto itr_buf = buf.begin();
    auto itr_rs_buf = rs_buf.begin();
    auto itr_stop = buf.cend();
    itr_stop--;
    while (itr_stop != itr_buf && buf.size()>2) {
        auto tmp1 = itr_buf->second;
        auto tmp2 = itr_rs_buf->second;
        if (!tmp1->processing) {
            auto itr_buf_tmp = itr_buf;
            auto itr_rs_buf_tmp = itr_rs_buf;
            itr_buf++;
            itr_rs_buf++;

            buf.erase(itr_buf_tmp->first);
            rs_buf.erase(itr_rs_buf_tmp->first);

            delete tmp1; delete tmp2;
        } else if (!skipping_frames) {
            break;
        } else {
            skipping_frames = false;
            itr_buf++;
            itr_rs_buf++;
        }
    }
}
void Realsense::delete_all_frames() {
    Cam::delete_all_frames();
    for (auto & sp : rs_buf) {
        delete sp.second;
    }
}

void Realsense::update_real(void) {
    std::unique_lock<std::mutex> lk(lock_newframe_mutex);
    lock_newframe.wait(lk, [] { return new_frame_ready; }); // wait for a new frame passed by the rs callback
    new_frame_ready = false;
    watchdog = true;
    if (watchdog_attempt_to_continue) {
        std::cout << "Frame processed. Buffer: ";
        for (auto fr : rs_buf) {
            std::cout << fr.first << " ";
        }
        std::cout << std::endl;
    }
}

void Realsense::rs_callback(rs2::frame f) {

    if (watchdog_attempt_to_continue) {
        if (f.get_profile().stream_index() == 1 )
            std::cout << "Received idL "         << f.get_frame_number() << ":" << f.get_timestamp()/1.e3- _frame_time_start << "@" << f.get_profile().stream_index() << "         Last: " << last_sync_id << std::endl;
        if (f.get_profile().stream_index() == 2 )
            std::cout << "Received idR         " << f.get_frame_number() << ":" << f.get_timestamp()/1.e3 -_frame_time_start << "@" << f.get_profile().stream_index() << " Last: " << last_sync_id << std::endl;
    }

    if (f.get_frame_number() < last_sync_id-50 && last_sync_id > 300) {
        std::cout << "Warning: rs frame number reset happened!!!" << std::endl;
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

            if (_frame_time_start <0)
                _frame_time_start = rs_frameL_cbtmp.get_timestamp()/1.e3;

            if (buf.size() < 5) {
                RSStereoPair * rsp = new RSStereoPair(rs_frameL_cbtmp,rs_frameR_cbtmp);
                cv::Mat frameL = Mat(Size(IMG_W, IMG_H), CV_8UC1, const_cast<void *>(rs_frameL_cbtmp.get_data()), Mat::AUTO_STEP);
                cv::Mat frameR = Mat(Size(IMG_W, IMG_H), CV_8UC1, const_cast<void *>(rs_frameR_cbtmp.get_data()), Mat::AUTO_STEP);
                StereoPair * sp = new StereoPair(frameL,frameR,rs_frameL_cbtmp.get_frame_number(),rs_frameL_cbtmp.get_timestamp()/1.e3 - _frame_time_start);
                rs_buf.insert(std::pair(rs_frameL_cbtmp.get_frame_number(),rsp));
                buf.insert(std::pair(rs_frameL_cbtmp.get_frame_number(),sp));
            } else if(last_sync_id > pparams.fps*2)
                std::cout << "BUF OVERFLOW, SKIPPING A FRAME" << std::endl;

            {   // signal to processor that a new frame is ready to be processed
                std::scoped_lock<std::mutex> lck(lock_newframe_mutex);
                new_frame_ready = true;
            }
            lock_newframe.notify_all();

            if (rs_frameL_cbtmp.supports_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE))
                camparams.measured_exposure = rs_frameL_cbtmp.get_frame_metadata(rs2_frame_metadata_value::RS2_FRAME_METADATA_ACTUAL_EXPOSURE);
            if (rs_frameL_cbtmp.supports_frame_metadata(RS2_FRAME_METADATA_GAIN_LEVEL))
                camparams.measured_gain = rs_frameL_cbtmp.get_frame_metadata(rs2_frame_metadata_value::RS2_FRAME_METADATA_GAIN_LEVEL);

            new_frame1 = false;
            new_frame2 = false;
            int fl = rs_frameL_cbtmp.get_frame_number() - last_sync_id;
            if (fl > 3 && last_sync_id> pparams.fps*2) {
                std::cout << "FRAME LOSS: " << fl << std::endl;
                _frame_loss_cnt++;
            }
            last_sync_id = rs_frameL_cbtmp.get_frame_number();
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

void Realsense::calibration(rs2::stream_profile infrared1,rs2::stream_profile infrared2) {
    // Obtain focal length and principal point (from intrinsics)
    auto depth_stream = infrared1.as<rs2::video_stream_profile>();
    auto i = depth_stream.get_intrinsics();
    float focal_length = i.fx; // same as fy
    float cx = i.ppx; // same for both cameras
    float cy = i.ppy;

    // Obtain baseline (from extrinsics)
    rs2_extrinsics e = infrared1.get_extrinsics_to(infrared2);
    float baseline = e.translation[0];

    baseline = fabs(baseline);

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
void Realsense::connect_and_check(string ser_nr,int id) {
    _id = id;
    set_write_file_paths(data_output_dir);
    bag_fn = "./logging/record" + std::to_string(_id) + ".bag";

    rs2::device_list devices = ctx.query_devices();
    if (devices.size() == 0) {
        throw NoRealsenseConnected();
    } else if (devices.size() > 1 && !ser_nr.compare("")) {
        throw MyExit("more than one RealSense connected....");
    } else if (ser_nr == "") {
        dev = devices[0];
    } else {
        bool found = false;
        for (auto d : devices) {
            string current_sn = "########";
            if (d.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
                current_sn = std::string("#") + d.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            if (current_sn == ser_nr) {
                dev = d;
                found = true;
                break;
            }
        }
        if (!found)
            throw MyExit("Could not find RealSense with sn: " + ser_nr);
    }

    std::string name = "Unknown Device";
    if (dev.supports(RS2_CAMERA_INFO_NAME))
        name = dev.get_info(RS2_CAMERA_INFO_NAME);

    serial_nr_str = "########";
    if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
        serial_nr_str = std::string("#") + dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        serial_nr = string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    } else
        throw MyExit("device does not support serial number?!");

    std::size_t found = name.find("D455");
    isD455 = found!=std::string::npos;

    rs2::depth_sensor rs_depth_sensor = dev.first<rs2::depth_sensor>();
    const std::string required_firmware1_version = "05.12.06.00";
    const std::string required_firmware2_version = "05.12.14.50";
    std::string current_firmware_version = rs_depth_sensor.get_info(rs2_camera_info::RS2_CAMERA_INFO_FIRMWARE_VERSION);
    current_firmware_version  = current_firmware_version.substr (0,required_firmware1_version.length()); //fix for what seems to be appended garbage...? 255.255.255.255 on a newline

    std::string master_or_slave_str = "master";
    if (!master())
        master_or_slave_str = "slave";
    std::cout << name << ", sn: " << serial_nr_str << ", fw: " << current_firmware_version << ", " << master_or_slave_str << std::endl;

    if (current_firmware_version != required_firmware1_version && current_firmware_version != required_firmware2_version) {
        std::stringstream serr;
        serr << "detected wrong RealSense firmware version! Detected: " << current_firmware_version << ". Required: "  << required_firmware1_version << " or " << required_firmware2_version << ".";
        throw MyExit(serr.str());
    }
    dev_initialized = true;
}
void Realsense::init_real() {
    std::cout << "Initializing cam " << serial_nr_str << std::endl;

    rs2::depth_sensor rs_depth_sensor = dev.first<rs2::depth_sensor>();

    if (!exposure_initialized && master()) {
        measure_auto_exposure();
        std::cout << "Measured auto exposure: " << camparams.measured_exposure << ", gain: " << camparams.measured_gain << ", brightness: " << camparams.measured_brightness << std::endl;
    }
    if (!angle_initialized && master()) {
        measure_angle();
        std::cout << "Camera roll: " << to_string_with_precision(camparams.camera_angle_x,2) << "°- max: " << pparams.max_cam_roll << "°. Pitch: " << to_string_with_precision(camparams.camera_angle_y,2) << "°" << std::endl;
    }

    calib_depth_background(); // this function may be merged into measure_angle or measure_auto_exposure to speed up things slightly, but it requires the laser which disturbs other systems, so I like to keep it seperate like this because then it will only be excuted once

    rs_depth_sensor = dev.first<rs2::depth_sensor>();
    rs_depth_sensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 0);
    if (rs_depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
        rs_depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);
    else
        throw MyExit("No laser emmitter option?!");
    if (rs_depth_sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE))
        rs_depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE,1.0);
    else
        throw MyExit("No auto exposure option?!");

    std::vector<rs2::stream_profile> stream_profiles = rs_depth_sensor.get_stream_profiles();
    rs2::stream_profile infrared1,infrared2;
    for (uint i = 0; i < stream_profiles.size(); i++) {
        try {
            if (auto video_stream = stream_profiles[i].as<rs2::video_stream_profile>()) {
                rs2_intrinsics intrinsics = video_stream.get_intrinsics();
                // std::cout << video_stream.stream_name() << " " << i << ": " << intrinsics.width << " x " << intrinsics.height << " @" << video_stream.fps() << std::endl;
                if (video_stream.fps() == static_cast<int>(pparams.fps) && intrinsics.width == 848 && intrinsics.height == 480) {
                    if (video_stream.stream_name().compare("Infrared 1") == 0)
                        infrared1 = video_stream;
                    if (video_stream.stream_name().compare("Infrared 2") == 0)
                        infrared2 = video_stream;
                }
            }
        } catch (const std::exception& e) {}
    }
    if (!infrared1 || !infrared2) {
        std::cout << "Error: streams not found in the realsense?!?!" << std::endl;
        exit(1);
    }



    calibration(infrared1,infrared2);
    camparams.serialize(calib_wfn);
    convert_depth_background_to_world();

    if (pparams.watchdog)
        thread_watchdog = std::thread(&Realsense::watchdog_thread,this);

    if (pparams.video_raw == video_bag)
        dev = rs2::recorder(bag_fn,dev);

    rs_depth_sensor.open({infrared1,infrared2});
    rs_depth_sensor.start([&](rs2::frame f) { rs_callback(f); });
    update_real();
    auto p = buf.cend();
    p--;
    _current = p->second;
    _current->processing = true;
    initialized = true;
}

std::tuple<float,float,cv::Mat,cv::Mat,cv::Mat,float> Realsense::measure_auto_exposure() {
    if (!dev_initialized) {
        rs2::device_list devices = ctx.query_devices();
        if (devices.size() == 0) {
            throw NoRealsenseConnected();
        } else if (devices.size() > 1) {
            throw MyExit("more than one RealSense connected....");
        } else {
            dev = devices[0];
            dev_initialized = true;
        }
    }

    rs2::config cfg;
    cfg.enable_device(serial_nr);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, IMG_W, IMG_H, RS2_FORMAT_Y8, pparams.fps);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, IMG_W, IMG_H, RS2_FORMAT_Y8, pparams.fps);
    if (isD455)
        cfg.enable_stream(RS2_STREAM_COLOR, 1280, 800, RS2_FORMAT_BGR8, 30);
    else
        cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_BGR8, 30);

    rs2::pipeline cam(ctx);
    cam.start(cfg);
    dev = cam.get_active_profile().get_device(); // after a cam start, dev is changed
    rs2::depth_sensor rs_dev = dev.first<rs2::depth_sensor>();

    rs_dev.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.0);
    rs_dev.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);

    cam.wait_for_frames(); // first frame seems to be weird, ignore it

    cv::Size im_size(IMG_W, IMG_H);
    cv::Mat frameLt,frameRt;

    rs2::frameset frame;
    float new_expos = -1;
    float new_gain = -1;

    float tmp_exposure =0, tmp_gain = 0;
    int tmp_last_ae_change_frame_id = 0;
    uint actual_exposure_was_measured = 0;
    uint i =0;

    for (i= 0; i< 2*pparams.fps; i++) { // check for large change in exposure
        frame = cam.wait_for_frames();

        if (frame.supports_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE)) {
            new_expos = frame.get_frame_metadata(rs2_frame_metadata_value::RS2_FRAME_METADATA_ACTUAL_EXPOSURE);
            new_gain = frame.get_frame_metadata(rs2_frame_metadata_value::RS2_FRAME_METADATA_GAIN_LEVEL);
            actual_exposure_was_measured++;
            if (fabs(new_expos - tmp_exposure) > 0.5f  || fabs(new_gain - tmp_gain) > 0.5f)
                tmp_last_ae_change_frame_id = i;
            if (i - tmp_last_ae_change_frame_id >= 15)
                break;
            tmp_exposure = new_expos;
        }
    }
    cam.stop();

    frameLt = Mat(im_size, CV_8UC1, const_cast<void *>(frame.get_infrared_frame(1).get_data()), Mat::AUTO_STEP).clone();
    frameRt = Mat(im_size, CV_8UC1, const_cast<void *>(frame.get_infrared_frame(2).get_data()), Mat::AUTO_STEP).clone();
    cv::Mat frame_top = frameLt(cv::Rect(frameLt.cols/3,0,frameLt.cols/3*2,frameLt.rows/3));
    float brightness = static_cast<float>(mean( frame_top )[0]);

    if (!actual_exposure_was_measured)
        std::cout << "Warning: no exposure data could be found!!!" << std::endl;
    else if (actual_exposure_was_measured!=i)
        std::cout << "Not all frames contained exosure info: " << actual_exposure_was_measured << " / " << i << std::endl;

    auto framergb = frame.get_color_frame();
    cv::Mat frame_bgr;
    if (isD455)
        frame_bgr = cv::Mat(cv::Size(1280,800), CV_8UC3, const_cast<void *>(framergb.get_data()), Mat::AUTO_STEP).clone();
    else
        frame_bgr = cv::Mat(cv::Size(1920,1080), CV_8UC3, const_cast<void *>(framergb.get_data()), Mat::AUTO_STEP).clone();

    camparams.measured_exposure = new_expos;
    camparams.measured_gain = new_gain;
    camparams.measured_brightness = brightness;
    exposure_initialized=true;
    return std::make_tuple(new_expos,new_gain,frameLt,frameRt,frame_bgr,brightness);

}

std::tuple<float,float,double,cv::Mat> Realsense::measure_angle() {
    if (!dev_initialized) {
        rs2::device_list devices = ctx.query_devices();
        if (devices.size() == 0) {
            throw NoRealsenseConnected();
        } else if (devices.size() > 1) {
            throw MyExit("more than one RealSense connected....");
        } else {
            dev = devices[0];
            dev_initialized = true;
        }
    }

    rs2::config cfg;
    cfg.enable_device(serial_nr);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, IMG_W, IMG_H, RS2_FORMAT_Y8, pparams.fps);
    cfg.enable_stream(RS2_STREAM_ACCEL);
    cfg.enable_stream(RS2_STREAM_GYRO);
    rs2::pipeline cam(ctx);
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
            roll = atan2f(-x, sqrtf(y*y + z*z)) * rad2deg;
            pitch = -(atan2f(y, z) * rad2deg+90);
        }
    }

    cam.stop();
    camparams.camera_angle_x = roll;
    camparams.camera_angle_y = pitch;
    angle_initialized = true;
    return std::make_tuple(roll,pitch,frame.get_timestamp(),frameLt);

}

void Realsense::calib_depth_background() {

    std::cout << "Measuring depth background..." << std::endl;
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
        throw MyExit("This is not a depth sensor");
    }

    rs2::config cfg;
    cfg.enable_device(serial_nr);
    cfg.enable_stream(RS2_STREAM_DEPTH, IMG_W, IMG_H, RS2_FORMAT_Z16, pparams.fps);
    rs2::pipeline cam(ctx);
    cam.start(cfg);
    dev = cam.get_active_profile().get_device(); // after a cam start, dev is changed
    cv::Size im_size(IMG_W, IMG_H);

    uint nframes = 10;
    rs2::frameset frame;
    for (uint i = 0; i < nframes; i++) {
        frame = cam.wait_for_frames();
    }

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

    cam.stop();
}

void Realsense::close() {
    if (initialized) {
        exit_watchdog_thread = true;
        std::cout << "Closing camera" << std::endl;
        if (!dev.as<rs2::playback>()) {
            auto rs_depth_sensor = dev.first<rs2::depth_sensor>();
            lock_newframe.notify_all();
            rs_depth_sensor.stop();
            rs_depth_sensor.close();
            for (uint i = 0; i < 10; i++ ) {
                new_frame_ready  = true;
                lock_newframe.notify_all();
                usleep(1000);
            }
        } else {
            cam_playback.stop();
        }
        if (pparams.watchdog) {
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
    rs2::stream_profile infrared1,infrared2;
    rs2::device_list devices = ctx.query_devices();
    if (devices.size() == 0) {
        throw NoRealsenseConnected();
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
    std::cout << "Realsense watchdog thread started" << std::endl;
    usleep(10000000); //wait until camera is running for sure
    while (!exit_watchdog_thread) {
        usleep(pparams.wdt_timeout_us);
        if (!watchdog && !exit_watchdog_thread) {
            std::cout << "Realsense  watchdog buf alert! Attempting to continue" << std::endl;
            watchdog_attempt_to_continue = true;
            new_frame1 =true;
            new_frame2 = true;
            std::cout << "RS frames in buf: ";
            for (auto fr : rs_buf) {
                std::cout << fr.first << " ";
            }
            std::cout << std::endl;
            usleep(pparams.wdt_timeout_us);
            if (!watchdog) {
                std::cout << "Realsense  watchdog alert! Killing the process." << std::endl;
                pid_t pid = getpid();
                std::cout << "pid: " << pid << std::endl;
                string kill_cmd = "kill -9 " + std::to_string(pid);
                auto res [[maybe_unused]] = std::system(kill_cmd.c_str());
            } else {
                std::cout << "Seems to work again." << std::endl;
            }
        }
        watchdog_attempt_to_continue = false;
        watchdog = false;
        set_external_wdt_flag();

    }
}
