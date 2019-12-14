#include "airsim.h"
#if CAMMODE == CAMMODE_AIRSIM

typedef ImageCaptureBase::ImageRequest ImageRequest;
typedef ImageCaptureBase::ImageResponse ImageResponse;
typedef ImageCaptureBase::ImageType ImageType;


void Airsim::update(void) {

    vector<ImageRequest> request = { ImageRequest("0", ImageType::Scene), ImageRequest("1", ImageType::DepthVis,true,false) };
    const vector<ImageResponse>& response = client.simGetImages(request);
    std::cout << "# of images received: " << response.size() << std::endl;

    if (response.size() > 0) {

        ImageResponse image_rgb_info = response[0];
        ImageResponse image_depth_info = response[1];

        cv::Mat im_rgb = cv::imdecode(image_rgb_info.image_data_uint8, CV_LOAD_IMAGE_COLOR);
        cv::Mat im_depth = cv::Mat(image_depth_info.height, image_depth_info.width, CV_32F);
        memcpy(im_depth.data, image_depth_info.image_data_float.data(), image_depth_info.image_data_float.size()*sizeof(CV_32F)); // change uchar to any type of data values that you want to use instead

        frame_id++;
        frame_time = sw.Read();
        frameRGB = im_rgb.clone();
        cv::cvtColor(im_rgb,frameL,CV_RGB2GRAY);
        frameD = im_depth;
    }

}

void Airsim::init(int argc __attribute__((unused)), char **argv __attribute__((unused))) {

    std::cout << "Initializing airsim" << std::endl;

    client.confirmConnection();
    client.enableApiControl(true);
    client.armDisarm(true);

    //init Qf: https://stackoverflow.com/questions/27374970/q-matrix-for-the-reprojectimageto3d-function-in-opencv
    //Qf = (cv::Mat_<double>(4, 4) << 1.0, 0.0, 0.0, -cx, 0.0, 1.0, 0.0, -cy, 0.0, 0.0, 0.0, focal_length, 0.0, 0.0, 1/baseline, 0.0);
#warning ToDo: Implement Qf

    thread_cam = std::thread(&Airsim::workerThread,this);
    sw.Start();

}

void Airsim::switch_mode(cam_mode_enum mode){
    if (mode != _mode) {
        if (mode == cam_mode_disabled )
            go_disabled();
        else if (mode == cam_mode_color)
            go_color();
        else if (mode == cam_mode_stereo)
            go_stereo();
    }
}

void Airsim::go_color() {
    _mode = cam_mode_color;
}

void Airsim::go_stereo() {
    _mode = cam_mode_stereo;
}

void Airsim::go_disabled() {
    _mode = cam_mode_disabled;
}

void Airsim::workerThread(void) {
    std::unique_lock<std::mutex> lk(m,std::defer_lock);

    while (!exitCamThread) {

        g_wait.wait(lk);

        float takeoffTimeout = 5;
        client.takeoffAsync(takeoffTimeout)->waitOnLastTask();

        // switch to explicit hover mode so that this is the fall back when
        // move* commands are finished.
        std::this_thread::sleep_for(std::chrono::duration<double>(5));
        client.hoverAsync()->waitOnLastTask();

        // moveByVelocityZ is an offboard operation, so we need to set offboard mode.
        client.enableApiControl(true);
        auto position = client.getMultirotorState().getPosition();
        float z = position.z(); // current position (NED coordinate system).
        const float speed = 3.0f;
        const float size = 10.0f;
        const float duration = size / speed;
        DrivetrainType driveTrain = DrivetrainType::ForwardOnly;
        YawMode yaw_mode(true, 0);
        std::cout << "moveByVelocityZ(" << speed << ", 0, " << z << "," << duration << ")" << std::endl;
        client.moveByVelocityZAsync(speed, 0, z, duration, driveTrain, yaw_mode);
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));
        std::cout << "moveByVelocityZ(0, " << speed << "," << z << "," << duration << ")" << std::endl;
        client.moveByVelocityZAsync(0, speed, z, duration, driveTrain, yaw_mode);
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));
        std::cout << "moveByVelocityZ(" << -speed << ", 0, " << z << "," << duration << ")" << std::endl;
        client.moveByVelocityZAsync(-speed, 0, z, duration, driveTrain, yaw_mode);
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));
        std::cout << "moveByVelocityZ(0, " << -speed << "," << z << "," << duration << ")" << std::endl;
        client.moveByVelocityZAsync(0, -speed, z, duration, driveTrain, yaw_mode);
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));

        client.hoverAsync()->waitOnLastTask();

        client.landAsync()->waitOnLastTask();
    }
}

void Airsim::pause(){

}
void Airsim::resume() {

}

void Airsim::close() {
    exitCamThread = true;
    thread_cam.join();
}

#endif //CAMMODE_AIRSIM
