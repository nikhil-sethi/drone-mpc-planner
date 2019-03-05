#include "cam.h"
#include <sys/stat.h>
#include "smoother.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <librealsense2/rsutil.h>

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
                    playback_bufferR.at(i).id < requested_id_in+playback_buffer_size_max)
                playback_bufferR_cleaned.push_back(playback_bufferR.at(i));
        }
        for (uint i = 0 ; i <playback_bufferL.size();i++) {
            if (    playback_bufferL.at(i).id >= requested_id_in &&
                    playback_bufferL.at(i).id < requested_id_in+playback_buffer_size_max)
                playback_bufferL_cleaned.push_back(playback_bufferL.at(i));
        }

        playback_bufferR = playback_bufferR_cleaned;
        playback_bufferL = playback_bufferL_cleaned;
        lock_frame_data.unlock();

        uint lowest_complete_id=0;
        uint highest_id = 0;
        float frame_time_higest_id = 0;
        for (uint i = 0 ; i <playback_bufferL_cleaned.size();i++) {
            if (playback_bufferL_cleaned.at(i).id > highest_id){
                highest_id = playback_bufferL_cleaned.at(i).id ;
                frame_time_higest_id = playback_bufferL_cleaned.at(i).time;
            }
            for (uint j = 0 ; j <playback_bufferR_cleaned.size();j++) {
                if (playback_bufferL_cleaned.at(i).id == playback_bufferR_cleaned.at(j).id ) {
                    lowest_complete_id = playback_bufferL_cleaned.at(i).id;
                    break;
                }
                if (lowest_complete_id>0)
                    break;
            }
        }

        if (lowest_complete_id > requested_id_in){
            //sometimes not all frames are saved, causing a frame jump
            std::cout << "Warning, frame jump detected. " << requested_id_in << " -> " << lowest_complete_id  << std::endl;
            requested_id_in = lowest_complete_id;
        }
        if (frame_time_higest_id < _frame_time){
            //for some reason the frame time and id counter sometimes resets in the bag files...
            std::cout << "Warning, frame reset detected. " << _frame_time << " -> " << frame_time_higest_id  << std::endl;
            funky_RS_frame_time_fixer_frame_count += requested_id_in;
            requested_id_in = lowest_complete_id;
        }

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

        auto duration = static_cast<float>(static_cast<rs2::playback>(dev).get_duration().count()) / 1e9f;
        if (frame_time() > duration-0.5f){
            std::cout << "Video end, exiting" << std::endl;
            throw my_exit(0);
        }

        if (_paused && playback_bufferR.size() < 3 ){ // 3 to start buffering 3 frames before the buffer runs empty
            incremented_playback_frametime = (requested_id_in+funky_RS_frame_time_fixer_frame_count-2)*(1.f/VIDEOFPS) - (1.f/VIDEOFPS)*0.1f;  //requested_id_in-2 -> -2 seems to be necessary because the RS api skips a frame of either the left or right camera after resuming
            if (incremented_playback_frametime < 0)
                incremented_playback_frametime = 0;
//            std::cout << "Resuming RS: " << incremented_playback_frametime << std::endl;

            if (duration-0.4f >= incremented_playback_frametime){
                seek(incremented_playback_frametime);
                resume();
            } else {
                std::cout << "Video end, exiting" << std::endl;
                throw my_exit(0);
            }
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

    if (playback_bufferR.size() >= playback_buffer_size_max*(2.f/3.f) && !_paused) {
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
    //    std::cout << "-------------frame id: " << _frame_number << " seek time: " << incremented_playback_frametime << std::endl;

}

void Cam::rs_callback_playback(rs2::frame f) {

    lock_frame_data.lock();
    //    if (f.get_profile().stream_index() == 1 )
    //        std::cout << "Received id "         << f.get_frame_number() << ":" << to_string_with_precision((static_cast<float>(f.get_timestamp())-_frame_time_start)/1e3f,2) << "@" << f.get_profile().stream_index() << "         Last: " << last_1_id << "@1 and " << last_2_id << "@2 and requested id & time:" << requested_id_in << " & " << to_string_with_precision(incremented_playback_frametime,2) << " bufsize: " << playback_bufferL.size() << std::endl;
    //    if (f.get_profile().stream_index() == 2 )
    //        std::cout << "Received id         " << f.get_frame_number() << ":" << to_string_with_precision((static_cast<float>(f.get_timestamp())-_frame_time_start)/1e3f,2) << "@" << f.get_profile().stream_index() << " Last: " << last_1_id << "@1 and " << last_2_id << "@2 and requested id & time:" << requested_id_in << " & " << to_string_with_precision(incremented_playback_frametime,2) << " bufsize: " << playback_bufferL.size() << std::endl;

    if (f.get_profile().stream_index() == 1 && f.get_frame_number() >= requested_id_in && playback_bufferL.size() < playback_buffer_size_max) {
        frame_data fL;
        fL.frame = Mat(Size(IMG_W, IMG_H), CV_8UC1, const_cast<void *>(f.get_data()), Mat::AUTO_STEP).clone();
        fL.id= f.get_frame_number();
        if (_frame_time_start <0)
            _frame_time_start = f.get_timestamp();
        fL.time = (static_cast<float>(f.get_timestamp()) -_frame_time_start)/1000.f;

        playback_bufferL.push_back(fL);
        new_frame1 = true;
        last_1_id = fL.id;
    } else if (f.get_profile().stream_index() == 2 && f.get_frame_number() >= requested_id_in && playback_bufferR.size() < 100) {
        frame_data fR;
        fR.frame = Mat(Size(IMG_W, IMG_H), CV_8UC1, const_cast<void *>(f.get_data()), Mat::AUTO_STEP).clone();
        fR.id= f.get_frame_number();
        if (_frame_time_start <0)
            _frame_time_start = f.get_timestamp();
        fR.time = (static_cast<float>(f.get_timestamp()) -_frame_time_start)/1000.f;
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
    frameL = Mat(Size(IMG_W, IMG_H), CV_8UC1, const_cast<void *>(rs_frameL.get_data()), Mat::AUTO_STEP);
    frameR = Mat(Size(IMG_W, IMG_H), CV_8UC1, const_cast<void *>(rs_frameR.get_data()), Mat::AUTO_STEP);
    _frame_number = rs_frameL.get_frame_number();
    if (_frame_time_start <0)
        _frame_time_start = rs_frameL.get_timestamp();
    _frame_time = (static_cast<float>(rs_frameL.get_timestamp()) -_frame_time_start)/1000.f;
    //    std::cout << "-------------frame id: " << _frame_number << " seek time: " << incremented_playback_frametime << std::endl;
    lock_frame_data.unlock();

    new_frame1 = false;
    new_frame2 = false;

}

uint last_sync_id = 0;
void Cam::rs_callback(rs2::frame f) {

    //    if (f.get_profile().stream_index() == 1 )
    //        std::cout << "Received id "         << f.get_frame_number() << ":" << (static_cast<float>(f.get_timestamp()) -_frame_time_start)/1e3f << "@" << f.get_profile().stream_index() << "         Last: " << last_sync_id << std::endl;
    //    if (f.get_profile().stream_index() == 2 )
    //        std::cout << "Received id         " << f.get_frame_number() << ":" << (static_cast<float>(f.get_timestamp()) -_frame_time_start)/1e3f << "@" << f.get_profile().stream_index() << " Last: " << last_sync_id << std::endl;

    if (f.get_frame_number() < last_sync_id-50) {
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

    baseline = fabs(baseline); //TODO: make an issue for this at Intel

    //init Qf: https://stackoverflow.com/questions/27374970/q-matrix-for-the-reprojectimageto3d-function-in-opencv
    Qf = (Mat_<double>(4, 4) << 1.0, 0.0, 0.0, -cx, 0.0, 1.0, 0.0, -cy, 0.0, 0.0, 0.0, focal_length, 0.0, 0.0, 1/baseline, 0.0);
    intr = new rs2_intrinsics(i);


}

void Cam::init() {

    std::cout << "Initializing cam" << std::endl;

    calib_log_fn = "./logging/" + calib_log_fn;
    depth_map_fn = "./logging/" + depth_map_fn;
    depth_unfiltered_map_fn = "./logging/" + depth_unfiltered_map_fn;
    disparity_map_fn = "./logging/" + disparity_map_fn;
    bag_fn = "./logging/" + bag_fn;
    brightness_map_fn = "./logging/" + brightness_map_fn;



    rs2::stream_profile infared1,infared2;
    rs2::context ctx; // The context represents the current platform with respect to connected devices
    rs2::device_list devices = ctx.query_devices();
    if (devices.size() == 0) {
        std::cerr << "No device connected, please connect a RealSense device" << std::endl;
        throw my_exit(1);
    } else if (devices.size() > 1) {
        std::cerr << "More than one device connected...." << std::endl;
        throw my_exit(1);
    } else {
        dev = devices[0];

        //record
        mkdir("./logging", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#if VIDEORAWLR == VIDEOMODE_BAG
        dev = rs2::recorder(bag_fn,dev);
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

        std::size_t found = name.find("D435I");
        hasIMU = found!=std::string::npos;

        std::cout << name << " ser: " << sn << std::endl;

    }

    // load xml, the values loaded may be (partially) overwritten by measurements below
    if (checkFileExist(calib_log_fn))
        deserialize_calib(calib_log_fn);
    else
        deserialize_calib(calib_template_fn);

    bool reloaded_calib;
    if (getSecondsSinceFileCreation(calib_log_fn) < 60*60 &&
            checkFileExist(depth_map_fn) &&
            checkFileExist(calib_log_fn)) {
        std::cout << "Calibration files recent, reusing..."   << std::endl;
        depth_background = imread(depth_map_fn,CV_LOAD_IMAGE_ANYDEPTH);

        depth_scale = 0.001; //FIXME, get real value!!!

        reloaded_calib= true;
    } else{
        calib_pose();
        sense_light_level();
        reloaded_calib = false;
    }

    std::vector<rs2::sensor> sensors = dev.query_sensors();
    depth_sensor = sensors[0]; // 0 = depth module

    std::cout << depth_sensor.get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME) << std::endl;
    std::cout << depth_sensor.get_info(rs2_camera_info::RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl;
    std::cout << depth_sensor.get_info(rs2_camera_info::RS2_CAMERA_INFO_PRODUCT_ID) << std::endl;

    std::string required_firmwar_version = "05.10.06.00";
    if (hasIMU)
        required_firmwar_version = "05.11.01.00";

    std::string current_firmware_version = depth_sensor.get_info(rs2_camera_info::RS2_CAMERA_INFO_FIRMWARE_VERSION);
    current_firmware_version  = current_firmware_version.substr (0,required_firmwar_version.length()); //fix for what seems to be appended garbage...? 255.255.255.255 on a newline

    if (current_firmware_version != required_firmwar_version) { // wtf, string equality check is reversed!??
        std::cout << "Detected wrong realsense firmware version!" << std::endl;
        std::cout << "Detected: " << current_firmware_version << ". Required: "  << required_firmwar_version << "." << std::endl;
        throw my_exit(1);
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
    if (enable_auto_exposure == only_at_startup ) {
        exposure = _measured_exposure;
        if (VIDEOFPS==60 && _measured_exposure>15500) //guarantee 60 FPS when requested
            exposure = 15500;
        gain = _measured_gain;
    }
    if ( enable_auto_exposure == enabled) {
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
    if (!reloaded_calib)
        serialize_calib();
    convert_depth_background_to_world();
    swc.Start();
}

//Converting the raw depths of depth_background distances to
//world coordinates in the IR camera frame
//There's probably a way to do this more efficient...
void Cam::convert_depth_background_to_world() {
    depth_background_3mm = cv::Mat::zeros(depth_background.rows,depth_background.cols,CV_32FC3);
    depth_background_mm = cv::Mat::zeros(depth_background.rows,depth_background.cols,CV_32FC1);
    for (int i = 0; i < depth_background.cols;i++)
        for (int j = 0; j < depth_background.rows;j++) {
            uint16_t back = depth_background.at<uint16_t>(j,i);
            float backf = static_cast<float>(back) * depth_scale;
            float pixel[2];
            pixel[0] = i;
            pixel[1] = j;
            float p[3];
            rs2_deproject_pixel_to_point(p, intr, pixel, backf);
            cv::Vec3f pixelColor(p[0],p[1],p[2]);
            depth_background_3mm.at<cv::Vec3f>(j,i) = pixelColor;
            depth_background_mm.at<float>(j,i) = sqrtf(powf(p[0],2)+powf(p[1],2)+powf(p[2],2));
        }
}

void Cam::deserialize_calib(std::string file) {
    std::cout << "Reading calibration from: " << file << std::endl;
    std::ifstream infile(file);

    std::string xmlData((std::istreambuf_iterator<char>(infile)),
                        std::istreambuf_iterator<char>());


    CamCalibrationData* dser=new CamCalibrationData; // Create new object
    if (xmls::Serializable::fromXML(xmlData, dser)) // perform deserialization
    { // Deserialization successful
        _camera_angle_x = dser->Angle_X.value();
        _camera_angle_y = dser->Angle_Y.value();
        _camera_angle_y_measured_from_depth= dser->Angle_Y_Measured_From_Depthmap.value();
        _measured_exposure = dser->Exposure.value();
        _measured_gain = dser->Gain.value();
    } else { // Deserialization not successful
        std::cout << "Error reading camera calibration file." << std::endl;
        throw my_exit(1);
    }
}

void Cam::serialize_calib() {
    CamCalibrationData *calib_settings=new CamCalibrationData; // Create new object
    calib_settings->Angle_X = _camera_angle_x;
    calib_settings->Angle_Y = _camera_angle_y;
    calib_settings->Exposure = _measured_exposure;
    calib_settings->Gain = _measured_gain;
    calib_settings->Angle_Y_Measured_From_Depthmap = _camera_angle_y_measured_from_depth;

    std::string xmlData = calib_settings->toXML();
    std::ofstream outfile(calib_log_fn);
    outfile << xmlData ;
    outfile.close();
}

void Cam::sense_light_level(){
    std::cout << "Measuring exposure..." << std::endl;
    rs2::config cfg;
    cfg.disable_all_streams();
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, IMG_W, IMG_H, RS2_FORMAT_Y8, VIDEOFPS);
    rs2::pipeline cam;
    rs2::pipeline_profile selection = cam.start(cfg);

    rs2::device selected_device = selection.get_device();
    auto rs_dev = selected_device.first<rs2::depth_sensor>();
    if (rs_dev.supports(RS2_OPTION_GAIN))
        rs_dev.set_option(RS2_OPTION_GAIN, 0);
    if (rs_dev.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE))
        rs_dev.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.0);
    if (rs_dev.supports(RS2_OPTION_EMITTER_ENABLED))
        rs_dev.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);

    cv::Size im_size(IMG_W, IMG_H);
    cv::Mat frameLt;

    _measured_gain = rs_dev.get_option(RS2_OPTION_GAIN);
    int search_speed=5;
    while(true) {
        //search for the best gain, keeping exposure at max 15500 (higher and the fps will decrease).
        //lower gain is better because less noise
        //this loop will do a quick search with increasing gain, and then a slower search decreasing the gain again
        //the slower search has smaller gain steps, and longer delays between reading
        //the resulting exposure (camera does not respond instantaniously)

        rs2::frameset frame;
        for (int i = 0; i< 60-search_speed;i++)
            frame = cam.wait_for_frames();

        if (frame.supports_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE)) {
            _measured_exposure = frame.get_frame_metadata(rs2_frame_metadata_value::RS2_FRAME_METADATA_ACTUAL_EXPOSURE);
            if (search_speed>0){
                if (_measured_exposure >= 15500 && _measured_gain <= 255 ) {
                    _measured_gain+=search_speed;
                    std::cout << "Measured exposure: " << _measured_exposure << " -> increasing gain: " << _measured_gain << std::endl;
                    rs_dev.set_option(RS2_OPTION_GAIN, _measured_gain);
                } else {
                    search_speed=-1;
                }
            }
            if (search_speed<0){
                if (_measured_exposure < 15500 && _measured_gain > 16 ) {
                    if (_measured_gain > 255)
                        _measured_gain = 255;
                    else
                        _measured_gain+=search_speed;
                    std::cout << "Measured exposure: " << _measured_exposure << " -> decreasing gain: " << _measured_gain << std::endl;
                    rs_dev.set_option(RS2_OPTION_GAIN, _measured_gain);
                } else {

                    std::cout << "Measured exposure: " << _measured_exposure << " gain: " << _measured_gain << std::endl;
                    frameLt = Mat(im_size, CV_8UC1, const_cast<void *>(frame.get_infrared_frame(1).get_data()), Mat::AUTO_STEP);
                    break;
                }
            }
        }

    }
    _measured_gain = rs_dev.get_option(RS2_OPTION_GAIN);

    imwrite(brightness_map_fn,frameLt);

    cam.stop();
}

void Cam::calib_pose(){

    std::cout << "Measuring pose..." << std::endl;
    rs2::config cfg;
    cfg.disable_all_streams();
    cfg.enable_stream(RS2_STREAM_DEPTH, IMG_W, IMG_H, RS2_FORMAT_Z16, VIDEOFPS);
    if (hasIMU) {
        cfg.enable_stream(RS2_STREAM_ACCEL);
        cfg.enable_stream(RS2_STREAM_GYRO);
    }
    rs2::pipeline cam;
    rs2::pipeline_profile selection = cam.start(cfg);

    rs2::device selected_device = selection.get_device();
    auto rs_dev = selected_device.first<rs2::depth_sensor>();
    if (rs_dev.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE))
        rs_dev.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.0);
    if (rs_dev.supports(RS2_OPTION_EMITTER_ENABLED))
        rs_dev.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f);

    // Find the High-Density preset by name
    // We do this to reduce the number of black pixels
    // The hardware can perform hole-filling much better and much more power efficient then our software
    auto range = rs_dev.get_option_range(RS2_OPTION_VISUAL_PRESET);
    for (auto i = range.min; i < range.max; i += range.step)
        if (std::string(rs_dev.get_option_value_description(RS2_OPTION_VISUAL_PRESET, i)) == "High Density")
            rs_dev.set_option(RS2_OPTION_VISUAL_PRESET, i);

    depth_scale = rs_dev.get_option(RS2_OPTION_DEPTH_UNITS);

    cv::Size im_size(IMG_W, IMG_H);

    uint nframes = 1;
    if (hasIMU)
        nframes = 10;
    Smoother smx,smy,smz;
    smx.init(static_cast<int>(nframes));
    smy.init(static_cast<int>(nframes));
    smz.init(static_cast<int>(nframes));
    float x = 0,y = 0,z = 0, roll = 0, pitch = 0;
    rs2::frameset frame;
    for (uint i = 0; i < nframes; i++) {
        frame = cam.wait_for_frames();

        if (hasIMU){
            auto frame_acc = frame.first(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);

            if (frame_acc.is<rs2::motion_frame>()) {
                rs2::motion_frame mf = frame_acc.as<rs2::motion_frame>();
                rs2_vector xyz = mf.get_motion_data();
                cout << frame_acc.get_profile().stream_type() << ": "
                     << xyz.x << ", " << xyz.y << ", " << xyz.z;
                cout << " roll: " << roll << " pitch: " << pitch << endl;
                x = smx.addSample(xyz.x);
                y = smy.addSample(xyz.y);
                z = smz.addSample(xyz.z);
                roll = atanf(-x/sqrtf(y*x + z*z)) * rad2deg;
                pitch = 90.f-atanf(y/z) * rad2deg;
            }
        }
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
    imwrite(depth_unfiltered_map_fn,depth_background);
    //filtered depth map:
    depth_background = Mat(im_size_dec, CV_16UC1, const_cast<void *>(depth.get_data()), Mat::AUTO_STEP).clone();
    cv::resize(depth_background,depth_background,im_size,0,0,INTER_CUBIC);
    imwrite(depth_map_fn,depth_background);

    imwrite(disparity_map_fn,disparity_background);

    if (hasIMU){
        _camera_angle_y = pitch;

        if (fabs(roll) > 5.f) {
            cout << "Camera is tilted in roll axis!" << std::endl;
            throw my_exit(1);
        }
        std::cout << "Measured pose: " << _camera_angle_y << std::endl;
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
        for (ppp=0;ppp<nr_p_far;ppp++) {
            auto ptr_new = ptr+points.size()-IMG_W/2-p_smooth/2-IMG_W*(nr_p_close+1+ppp);
            for (pp=0;pp<p_smooth;pp++) {
                point_.x += ptr_new->x;
                point_.y += ptr_new->y;
                point_.z += ptr_new->z;
                ptr_new++;
            }
            point_/=p_smooth;
            pointsFar.push_back(point_);
        }

        // obtain a set of points 'close by' (bottom of the image) that are averaged over image lines
        for (ppp=0;ppp<nr_p_close;ppp++) {
            auto ptr_new = ptr+points.size()-IMG_W/2-p_smooth/2-IMG_W*ppp;
            for (pp=0;pp<p_smooth;pp++) {
                point_.x += ptr_new->x;
                point_.y += ptr_new->y;
                point_.z += ptr_new->z;
                ptr_new++;
            }
            point_/=p_smooth;
            pointsClose.push_back(point_);
        }

        // calculate angle between 'far away' and 'close by' points, by avaraging over all combinations between both sets
        for (ppp=0;ppp<nr_p_close;ppp++) {
            for (pp=0;pp<nr_p_far;pp++) {
                point_diff = pointsFar.at(pp)-pointsClose.at(ppp);
                alpha += atanf(point_diff.y/point_diff.z);
            }
        }
        alpha/=(nr_p_close*nr_p_far);

        float old = _camera_angle_y_measured_from_depth;

        _camera_angle_y_measured_from_depth = -alpha*rad2deg; //[degrees]
        std::cout << "Measured angle: " << _camera_angle_y_measured_from_depth << std::endl;
        std::cout << "Estimated angle change: " << _camera_angle_y_measured_from_depth - old << std::endl;
        std::cout << "Set angle_y: " << _camera_angle_y << std::endl;

        if ((fabs(_camera_angle_y_measured_from_depth - old) > 15 && checkFileExist(calib_log_fn)) || _camera_angle_y_measured_from_depth != _camera_angle_y_measured_from_depth) {
            std::cout << "Warning: angle change to big!" << std::endl;
            _camera_angle_y_measured_from_depth = 0;
            throw my_exit(1);
        }
    }
    cam.stop();
}

void Cam::init(int argc __attribute__((unused)), char **argv) {

    std::string datadir = argv[1];

    fromfile=true;
    calib_log_fn = datadir + '/' + calib_log_fn;
    depth_map_fn = datadir + '/' + depth_map_fn;
    depth_unfiltered_map_fn = datadir + '/' + depth_unfiltered_map_fn;
    disparity_map_fn = datadir + '/' + disparity_map_fn;
    brightness_map_fn = datadir + '/' + brightness_map_fn;
    bag_fn = datadir + '/' + bag_fn;
    std::cout << "Initializing cam from " << bag_fn << std::endl;

    if (!checkFileExist(depth_map_fn)) { //FIXME: use full path to folder
        //todo: make gui warning of this:
        std::cout << "Warning: could not find " << depth_map_fn << std::endl;
        depth_background = cv::Mat::ones(IMG_H,IMG_W,CV_16UC1);
        depth_background = 10000; // basically disable the depth background map if it is not found
    } else {
        depth_background = imread(depth_map_fn,CV_LOAD_IMAGE_ANYDEPTH);
    }

    if (!checkFileExist(bag_fn)) {
        std::cout << "Could not find " << bag_fn << std::endl;
        throw my_exit(1);
    }
    rs2::stream_profile infared1,infared2;
    rs2::context ctx; // The context represents the current platform with respect to connected devices
    dev = ctx.load_device(bag_fn);

    static_cast<rs2::playback>(dev).set_real_time(false);

    std::vector<rs2::sensor> sensors = dev.query_sensors();
    depth_sensor = sensors[0]; // 0 = depth module
    depth_scale = depth_sensor.get_option(RS2_OPTION_DEPTH_UNITS);

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
    if (checkFileExist(calib_log_fn))
        deserialize_calib(calib_log_fn);
    else
        deserialize_calib(calib_template_fn);
    convert_depth_background_to_world();
    swc.Start();
}

void Cam::pause(){
    if (!_paused) {
        _paused = true;
        static_cast<rs2::playback>(dev).pause();
        //        std::cout << "Paused" << std::endl;
    }
}
void Cam::resume() {
    if (_paused){
        _paused = false;
        static_cast<rs2::playback>(dev).resume();
        //        std::cout << "Resumed" << std::endl;
    }
}
void Cam::seek(float time) {
    std::chrono::nanoseconds nano(static_cast<ulong>(1e9f*time));
    static_cast<rs2::playback>(dev).seek(nano);
}

void Cam::close() {
    std::cout << "Closing camera" << std::endl;
    depth_sensor.stop();
    depth_sensor.close();
    usleep(1000);
    std::cout << "Camera closed" << std::endl;
}

void Cam::reset() {
    std::cout << "Resetting cam" << std::endl;

    rs2::stream_profile infared1,infared2;
    rs2::context ctx; // The context represents the current platform with respect to connected devices
    rs2::device_list devices = ctx.query_devices();
    if (devices.size() == 0) {
        std::cerr << "No device connected, please connect a RealSense device" << std::endl;
        throw my_exit(1);
    } else if (devices.size() > 1) {
        std::cerr << "More than one device connected...." << std::endl;
        throw my_exit(1);
    } else {
        dev = devices[0];

        std::cout << "Sending hardware reset..." << std::endl;
        dev.hardware_reset();

        usleep(1000000);

        exit (0);
    }
}
