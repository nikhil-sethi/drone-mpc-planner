#include "cam.h"


using namespace cv;
using namespace std;



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
        if (depth_sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
            depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE,0.0);
        }

        if (depth_sensor.supports(RS2_OPTION_EXPOSURE)) {
            auto range = depth_sensor.get_option_range(RS2_OPTION_EXPOSURE);
            depth_sensor.set_option(RS2_OPTION_EXPOSURE, exposure); //TODO: automate this setting.
        }
        //weird with D435 this is totally unneccesary...? probably ROI related
        if (depth_sensor.supports(RS2_OPTION_GAIN)) {
            auto range = depth_sensor.get_option_range(RS2_OPTION_GAIN);
            //            depth_sensor.set_option(RS2_OPTION_GAIN, (range.max - range.min)/2 + range.min);
            depth_sensor.set_option(RS2_OPTION_GAIN, gain); // increasing this causes noise
        }

        namedWindow("Cam tuning", WINDOW_NORMAL);
        createTrackbar("Exposure", "Cam tuning", &exposure, 32768);
        createTrackbar("Gain", "Cam tuning", &gain, 32768);

        //        cam.stop();
        //        selection = cam.start(cfg);

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

    rs2::frameset frame;
    frame = cam.wait_for_frames(); // init it with something

    rs2_timestamp_domain d = frame.get_frame_timestamp_domain();
    if (d == RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME || d == RS2_TIMESTAMP_DOMAIN_COUNT) {
        std::cout << "Error: Realsense hardware clock not working... " << std::endl;
        exit(1);
    }

    static int old_exposure;
    static int old_gain;
    rs2::device selected_device = selection.get_device();
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();

    //    auto start = std::chrono::system_clock::now();
    //    std::time_t time = std::chrono::system_clock::to_time_t(start);
    //    std::cout << "Starting at " << std::ctime(&time) << std::endl; // something weird is going on with this line, it seems to crash the debugger if it is in another function...?
    //    std::cout << "Running..." << std::endl;

    while (!exitCamThread) {

        int current_frame_id = frame.get_frame_number();
        while(current_frame_id == frame.get_frame_number())
            frame = cam.wait_for_frames();

        g_lockData.lock();
        frame_time_tmp = frame.get_timestamp()/1000.f; //stopWatch.Read()/1000.f;
        frame_number_tmp = frame.get_frame_number();
        std::cout << frame.get_frame_number() << ": " << frame_time_tmp << std::endl;
        frameL_tmp = Mat(imgsize, CV_8UC1, (void*)frame.get_infrared_frame(IR_ID_LEFT).get_data(), Mat::AUTO_STEP);
        frameR_tmp = Mat(imgsize, CV_8UC1, (void*)frame.get_infrared_frame(IR_ID_RIGHT).get_data(), Mat::AUTO_STEP);


        g_lockData.unlock();

        g_waitforimage.unlock();
        ready = true;

        if (!fromfile) {
            if (exposure != old_exposure) {
                auto range = depth_sensor.get_option_range(RS2_OPTION_EXPOSURE);
                depth_sensor.set_option(RS2_OPTION_EXPOSURE, exposure); //TODO: automate this setting.
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
