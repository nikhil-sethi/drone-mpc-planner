#include "cam.h"


using namespace cv;
using namespace std;

#define TUNING

void Cam::update(void) {
    if (!ready) {
        std::cout << "Waiting for realsense..." << std::endl;
        while (!ready)
            usleep(1000);
    }
    g_waitforimage.lock();
    g_lockData.lock();
    frameL = frameL_tmp.clone();
    frameR = frameR_tmp.clone();
    frame_number = frame_number_tmp;
    frame_time = frame_time_tmp;
    g_lockData.unlock();
}

void Cam::init(int argc, char **argv) {

    std::cout << "Initializing cam\n";
    // Declare config
    rs2::config cfg;
    cfg.disable_all_streams();
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, IMG_W, IMG_H, RS2_FORMAT_Y8, VIDEOFPS);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, IMG_W, IMG_H, RS2_FORMAT_Y8, VIDEOFPS);

#if !INSECT_DATA_LOGGING_MODE
    if (argc ==2 ) {
        cfg.enable_device_from_file(string(argv[1]) + ".bag");
        fromfile=true;
    } else {
        cfg.enable_record_to_file("test");
    }
#endif

    selection = cam.start(cfg);
    std::cout << "Started cam\n";

    if (argc ==2 ) {
        pd = selection.get_device();
        ((rs2::playback)pd).set_real_time(true);
    } else {
        rs2::device selected_device = selection.get_device();
        auto depth_sensor = selected_device.first<rs2::depth_sensor>();

        if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
        {
            depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);
        }
        //        if (depth_sensor.supports(RS2_OPTION_LASER_POWER)) {
        //            auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
        //            depth_sensor.set_option(RS2_OPTION_LASER_POWER, (range.max - range.min)/2 + range.min);
        //        }
        if (enable_auto_exposure) {
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

#ifdef TUNING
        namedWindow("Cam tuning", WINDOW_NORMAL);
        createTrackbar("Exposure", "Cam tuning", &exposure, 32768);
        createTrackbar("Gain", "Cam tuning", &gain, 32768);
#endif

        std::cout << "Set cam config\n";
    }

    // Obtain focal length and principal point (from intrinsics)
    auto depth_stream = selection.get_stream(RS2_STREAM_INFRARED, 1).as<rs2::video_stream_profile>();
    auto i = depth_stream.get_intrinsics();
    float focal_length = i.fx; // same as fy
    float cx = i.ppx; // same for both cameras
    float cy = i.ppy;

    // Obtain baseline (from extrinsics)
    auto ir1_stream = selection.get_stream(RS2_STREAM_INFRARED, 1);
    auto ir2_stream = selection.get_stream(RS2_STREAM_INFRARED, 2);
    rs2_extrinsics e = ir2_stream.get_extrinsics_to(ir1_stream);
    float baseline = e.translation[0];

    //init Qf: https://stackoverflow.com/questions/27374970/q-matrix-for-the-reprojectimageto3d-function-in-opencv
    Qf = (Mat_<double>(4, 4) << 1.0, 0.0, 0.0, -cx, 0.0, 1.0, 0.0, -cy, 0.0, 0.0, 0.0, focal_length, 0.0, 0.0, 1/baseline, 0.0);


    thread_cam = std::thread(&Cam::workerThread,this);

}

void Cam::workerThread(void) {

    std::cout << "Cam thread started!" << std::endl;


    cv::Size imgsize(IMG_W, IMG_H);
    if (enable_auto_exposure) {
        usleep(1e6); // give auto exposure some time
    }

    rs2::frameset frame;
    frame = cam.wait_for_frames(); // init it with something

    rs2_timestamp_domain d = frame.get_frame_timestamp_domain();
    if ((d == RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME || d == RS2_TIMESTAMP_DOMAIN_COUNT) && !fromfile) {
        std::cout << "Error: Realsense hardware clock not working... " << std::endl;
        exit(1);
    }

    if (enable_auto_exposure && !fromfile) {
        rs2::device selected_device = selection.get_device();
        auto depth_sensor = selected_device.first<rs2::depth_sensor>();
        
        exposure = frame.get_infrared_frame(IR_ID_LEFT).get_frame_metadata(rs2_frame_metadata_value::RS2_FRAME_METADATA_ACTUAL_EXPOSURE) ;
        gain = frame.get_infrared_frame(IR_ID_LEFT).get_frame_metadata(rs2_frame_metadata_value::RS2_FRAME_METADATA_GAIN_LEVEL) ;
        std::cout << "Auto exposure = " << frame.get_infrared_frame(IR_ID_LEFT).get_frame_metadata(rs2_frame_metadata_value::RS2_FRAME_METADATA_ACTUAL_EXPOSURE) << ", ";
        std::cout << "Auto gain = " << frame.get_infrared_frame(IR_ID_LEFT).get_frame_metadata(rs2_frame_metadata_value::RS2_FRAME_METADATA_GAIN_LEVEL) << std::endl;
        
        depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE,0.0);
        depth_sensor.set_option(RS2_OPTION_EXPOSURE, exposure);
        depth_sensor.set_option(RS2_OPTION_GAIN, gain);

        frame = cam.wait_for_frames(); // init it with something
    }

    static int old_exposure = exposure;
    static int old_gain = gain;

    while (!exitCamThread) {

        int current_frame_id = frame.get_frame_number();
        while(current_frame_id == frame.get_frame_number()) {
            try {
                frame = cam.wait_for_frames();
            } catch (rs2::error) {}
        }

        g_lockData.lock();
        frame_time_tmp = frame.get_timestamp()/1000.f; //stopWatch.Read()/1000.f;
        frame_number_tmp = frame.get_frame_number();
        //std::cout << frame.get_frame_number() << ": " << frame_time_tmp << std::endl;
        frameL_tmp = Mat(imgsize, CV_8UC1, (void*)frame.get_infrared_frame(IR_ID_LEFT).get_data(), Mat::AUTO_STEP);
        frameR_tmp = Mat(imgsize, CV_8UC1, (void*)frame.get_infrared_frame(IR_ID_RIGHT).get_data(), Mat::AUTO_STEP);


        g_lockData.unlock();

        g_waitforimage.unlock();
        ready = true;

        if (!fromfile && !enable_auto_exposure) {
            if (exposure != old_exposure) {
                if (exposure < 20)
                    exposure =20;
                auto range = depth_sensor.get_option_range(RS2_OPTION_EXPOSURE);
                depth_sensor.set_option(RS2_OPTION_EXPOSURE, exposure);
            }
            if (gain != old_gain) {
                auto range = depth_sensor.get_option_range(RS2_OPTION_GAIN);
                depth_sensor.set_option(RS2_OPTION_GAIN, gain); // increasing this causes noise
            }
        }
        old_exposure = exposure;
        old_gain = gain;

    }
    usleep(1000);
}

void Cam::pause(){
    ((rs2::playback)pd).pause();
}
void Cam::resume() {
    ((rs2::playback)pd).resume();
}

void Cam::close() {
    exitCamThread = true;
    g_lockData.unlock();
    thread_cam.join();
}
