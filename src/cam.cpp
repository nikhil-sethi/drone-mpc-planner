#include "cam.h"
#include <sys/stat.h>

using namespace cv;
using namespace std;

#ifdef HASSCREEN
#define TUNING
#endif

stopwatch_c swc;


int last_1_id =-1;
int last_2_id =-1;
float incremented_playback_frametime = (1.f/VIDEOFPS)/2.f;
void Cam::update(void) {
    if( fromfile) {
        incremented_playback_frametime = requested_id_in*(1.f/VIDEOFPS) - (1.f/VIDEOFPS)*0.1f; //halfway before the next frame
        if (incremented_playback_frametime < 0)
            incremented_playback_frametime = 0;
        seek(incremented_playback_frametime);
        g_lockFlags.lock();
        new_frame1 = false;
        new_frame2 = false;
        requested_id_in++;
        g_lockFlags.unlock();
        resume();
    }

    std::unique_lock<std::mutex> lk(m);
    g_waitforimage.wait(lk);

    if( fromfile) {
        pause();
        while(swc.Read() < (1.f/VIDEOFPS)*1e3f){
            usleep(10);
        }
        while(frame_by_frame){
            unsigned char k = cv::waitKey(1);
            if (k== 'f')
                break;
            else if (k== ' '){
                frame_by_frame = false;
                break;
            }
        };
        swc.Restart();

        if (last_2_id != last_2_id)
            std::cout << "Playback sync problem" << std::endl;
    }

    g_lockFrameData.lock();
    frameL = Mat(Size(848, 480), CV_8UC1, (void*)rs_frameL.get_data(), Mat::AUTO_STEP);
    frameR = Mat(Size(848, 480), CV_8UC1, (void*)rs_frameR.get_data(), Mat::AUTO_STEP);
    frame_id = rs_frameL.get_frame_number();
    if (frame_time_start <0)
        frame_time_start = rs_frameL.get_timestamp();
    frame_time = ((float)rs_frameL.get_timestamp() -frame_time_start)/1000.f;
    //std::cout << "-------------frame id: " << frame_id << " seek time: " << incremented_playback_frametime << std::endl;
    g_lockFrameData.unlock();

    if(!fromfile) {
        g_lockFlags.lock();
        new_frame1 = false;
        new_frame2 = false;
        ready = true;
        g_lockFlags.unlock();
    }
}

void Cam::rs_callback_playback(rs2::frame f) {
    g_lockFlags.lock();

//    if (f.get_profile().stream_index() == 1 )
//        std::cout << "Received id " << f.get_frame_number()   << "@" << f.get_profile().stream_index() << "         Last: " << last_1_id << "@1 and " << last_2_id << "@2 and synced id:" << requested_id_in << " time: " << incremented_playback_frametime << std::endl;
//    if (f.get_profile().stream_index() == 2 )
//        std::cout << "Received id         " << f.get_frame_number()   << "@" << f.get_profile().stream_index() << " Last: " << last_1_id << "@1 and " << last_2_id << "@2 and synced id:" << requested_id_in << " time: " << incremented_playback_frametime << std::endl;

    if (f.get_profile().stream_index() == 1 && f.get_frame_number() >= requested_id_in) {
        rs_frameL  = f;
        new_frame1 = true;
        last_1_id = f.get_frame_number();
    } else if (f.get_profile().stream_index() == 2 && f.get_frame_number() >= requested_id_in) {
        rs_frameR = f;
        new_frame2 = true;
        last_2_id = f.get_frame_number();
    }

    if (new_frame1 && new_frame2) {
        g_waitforimage.notify_all();
        ready = false;
        //requested_id_in = f.get_frame_number();
    }

    g_lockFlags.unlock();
}

uint last_sync_id = 0;
void Cam::rs_callback(rs2::frame f) {

    g_lockFlags.lock();
    if (f.get_profile().stream_index() == 1 && !new_frame1 && f.get_frame_number() >= last_sync_id) {
        rs_frameL_cbtmp  = f;
        new_frame1 = true;
    } else if (f.get_profile().stream_index() == 2 && !new_frame2 && f.get_frame_number() >= last_sync_id) {
        rs_frameR_cbtmp = f;
        new_frame2 = true;
    }
    g_lockFlags.unlock();
    if (new_frame1 && new_frame2) {
        if (rs_frameL_cbtmp.get_frame_number() == rs_frameR_cbtmp.get_frame_number()) {
            g_lockFrameData.lock();
            rs_frameL = rs_frameL_cbtmp;
            rs_frameR = rs_frameR_cbtmp;
            g_lockFrameData.unlock();
            g_waitforimage.notify_all();
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


void Cam::init(int argc, char **argv) {

    std::cout << "Initializing cam" << std::endl;

    rs2::stream_profile infared1,infared2;

    if (argc ==2 ) {

        rs2::context ctx; // The context represents the current platform with respect to connected devices
        dev = ctx.load_device(string(argv[1]) + ".bag");
        //dev = rs2::playback(dev);

        fromfile=true;
        ((rs2::playback)dev).set_real_time(real_time_playback);
        if (real_time_playback)
            ((rs2::playback)dev).set_playback_speed(real_time_playback_speed);

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

    } else {
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
            dev = rs2::recorder("./logging/test.bag",dev);

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
        //TODO: check firmware version

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

        std::cout << "Set cam config" << std::endl;
    }

    if (fromfile)
        pause();

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

    swc.Start();
}

void Cam::pause(){
    ((rs2::playback)dev).pause();
}
void Cam::resume() {
    ((rs2::playback)dev).resume();
}
void Cam::seek(float time) {
    std::chrono::nanoseconds nano((ulong)(1e9f*time));
    ((rs2::playback)dev).seek(nano);
}

void Cam::close() {
    depth_sensor.stop();
    depth_sensor.close();
}
