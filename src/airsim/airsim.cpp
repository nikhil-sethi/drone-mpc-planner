#include "airsim.h"

#if HAS_AIRSIM

void AirSim::init(std::string name) {
    client_name = name;

    // check the state of connection every 1 sec until connected
    client.confirmConnection();

    // define the request parameters
    request = {
        ImageRequest("front_left", ImageType::Scene, false, false),
        ImageRequest("front_right", ImageType::Scene, false, false)
    };
}

void AirSim::load_environment(std::string level_name) {
    std::cout << "Simulator loading level: "  << level_name << std::endl;
    bool succes = client.simLoadLevel("/Game/Maps/" + level_name);
    if(!succes)
        std::cout << "Failed to load level, using GreenhouseEmpty" << std::endl;
}

// request DepthVis image from the simulator and return it if valid
cv::Mat AirSim::depth_background() {
    std::vector<ImageRequest> requestDepth = {
        ImageRequest("front_left", ImageType::DepthVis, false, false)
    };

    const std::vector<ImageResponse> &response = client.simGetImages(requestDepth, client_name);

    if (response.size() > 0) {
        ImageResponse depthVis = response[0];
        return cv::Mat(depthVis.height, depthVis.width, CV_8UC1, depthVis.image_data_uint8.data()).clone();
    } else {
        throw MyExit("could not get depth map");
    }
}

float AirSim::cam_fov() {
    CameraInfo cam_info = client.simGetCameraInfo("front_left", client_name);
    return cam_info.fov;
}

void AirSim::pause(bool pause) {
    client.simPause(pause);
}

StereoPair* AirSim::new_frame(double desired_frame_time) {
    // first resume the simulation for a specified amount of time, this is the duration between each frame hence desired_frame_time
    client.simContinueForTime(desired_frame_time);

    int frame_retrieve_errors = 0;

    do {
        const std::vector<ImageResponse> &response = client.simGetImages(request, client_name);

        if (response.size() > 0) {
            // calculate the actual frame time
            double frame_time = msr::airlib::ClockBase::elapsedBetween(response[0].time_stamp, start_timestamp);

            // store the first timestamp to calculate frame time
            if(start_timestamp == 0)
                start_timestamp = response[0].time_stamp;

            // detect incorrect data, limit frame time to 100 days
            if(frame_time < 8640000) {
                ImageResponse front_left = response[0];
                ImageResponse front_right  = response[1];

                // create the frames with 1 channel since we get the frames in grayscale
                cv::Mat frameL = cv::Mat(front_left.height,  front_left.width,  CV_8UC1, front_left.image_data_uint8.data()).clone();
                cv::Mat frameR = cv::Mat(front_right.height, front_right.width, CV_8UC1, front_right.image_data_uint8.data()).clone();

                StereoPair * sp = new StereoPair(frameL,frameR,++_frame_number,frame_time);

                return sp;
            }
        }
        frame_retrieve_errors++;
    } while(frame_retrieve_errors < 10);

    throw MyExit("Could net get new frame after 10 tries");
}

void AirSim::rc_data_valid(bool valid) {
    rc_data.is_initialized = rc_data.is_valid = valid;
}

void AirSim::set_led(float led) {
    // Check if led value has changed to save api calls
    if(current_led != led) {
        client.simSetLedIntensity(led, client_name);
        current_led = led;
    }
}

void AirSim::arm(bool arm) {
    client.enableApiControl(true, client_name); // to use armDisarm, enableApiControl needs to be true
    client.armDisarm(arm, client_name);
    client.enableApiControl(false, client_name); // to use move_by_rc, enableApiControl needs to be false
}

void AirSim::move_by_rc(float throttle, float yaw, float pitch, float roll) {
    rc_data.roll = roll;
    rc_data.yaw = yaw;
    rc_data.pitch = pitch;
    rc_data.throttle = throttle;
    client.moveByRC(rc_data, client_name);
}

void AirSim::init_logging() {
    airsim_log_file.open("logging/airsim_logs.csv");
    airsim_log_file << "Frame ID;"
                    << "Throttle;"
                    << "Yaw;"
                    << "Pitch;"
                    << "Roll;"
                    << "Position X;"
                    << "Position Y;"
                    << "Position Z;"
                    << "Orientation W;"
                    << "Orientation X;"
                    << "Orientation Y;"
                    << "Orientation Z;"
                    << "Linear acceleration X;"
                    << "Linear acceleration Y;"
                    << "Linear acceleration Z;"
                    << "Angular acceleration X;"
                    << "Angular acceleration Y;"
                    << "Angular acceleration Z;"
                    << "\n";

    logging_initialized = true;
}

void AirSim::log(unsigned long long frame_id) {
    MultirotorState state = client.getMultirotorState("Hammer");

    airsim_log_file <<
                    frame_id << ";" <<
                    state.rc_data.throttle << ";" <<
                    state.rc_data.yaw << ";" <<
                    state.rc_data.pitch << ";" <<
                    state.rc_data.roll << ";" <<
                    state.kinematics_estimated.pose.orientation.x() << ";" <<
                    abs(state.kinematics_estimated.pose.position.z()) << ";" << // z in unreal (negative) -> y in pats coordinate system (positive)
                    state.kinematics_estimated.pose.position.y() << ";" << // y in unreal is z in pats coordinate system
                    state.kinematics_estimated.pose.orientation.w() << ";" <<
                    state.kinematics_estimated.pose.orientation.x() << ";" <<
                    state.kinematics_estimated.pose.orientation.y() << ";" <<
                    state.kinematics_estimated.pose.orientation.z() << ";" <<
                    state.kinematics_estimated.accelerations.linear.x() << ";" <<
                    state.kinematics_estimated.accelerations.linear.y() << ";" <<
                    state.kinematics_estimated.accelerations.linear.z() << ";" <<
                    state.kinematics_estimated.accelerations.angular.x() << ";" <<
                    state.kinematics_estimated.accelerations.angular.y() << ";" <<
                    state.kinematics_estimated.accelerations.angular.z() << std::endl;
}

void AirSim::close() {
    // check if connected to simulator
    ConnectionState conn_state = client.getConnectionState();
    if(conn_state == ConnectionState::Connected) {
        // reset the simulator
        client.simPause(false);
        client.reset();
    }

    if(logging_initialized)
        airsim_log_file.close();
}

#else
// placeholders used if AirSim is not installed
void AirSim::init(std::string name) { (void)name;}
void AirSim::pause(bool pause) { (void)pause; };
void AirSim::close() {}
void AirSim::rc_data_valid(bool valid) { (void)valid; };
void AirSim::set_led(float led)  { (void)led; };
void AirSim::move_by_rc(float throttle, float yaw, float pitch, float roll) { (void)throttle; (void)yaw; (void)pitch; (void)roll;}
void AirSim::arm(bool arm) { (void)arm; }
void AirSim::load_environment(std::string level_name) { (void)level_name; }
float AirSim::cam_fov() { return 0;}
StereoPair*  AirSim::new_frame(double desired_frame_time) { (void)desired_frame_time; return nullptr;};
cv::Mat AirSim::depth_background() {return cv::Mat::ones(IMG_H,IMG_W,CV_16UC1);}
void AirSim::init_logging() {}
void AirSim::log(unsigned long long frame_id) { (void)frame_id; }
#endif
