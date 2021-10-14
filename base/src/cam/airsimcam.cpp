#include "airsimcam.h"

void AirSimCam::init()
{
    std::cout << "Initializing AirSimCam" << std::endl;

    calib_rfn = "./logging/" + calib_rfn;
    depth_map_rfn = "./logging/" + depth_map_rfn;
    calib_wfn = calib_rfn;
    depth_map_wfn = depth_map_rfn;

    sim.init(camera_name);
    sim.load_environment(airsim_map);
    sim.init_logging();

    // locate the calibration file and deserialize it
    if (file_exist(calib_airsim_cam))
        camparams.deserialize(calib_airsim_cam);
    else
        throw MyExit("cannot find " + calib_airsim_cam);
    calibration();

    // request a depth image which we then apply as depth_background
    depth_background = sim.depth_background();

    convert_depth_background_to_world();
    camparams.serialize(calib_wfn);
    imwrite(depth_map_wfn, depth_background);

    update();
}

// generate the Qf matrix with the provided camera information
void AirSimCam::calibration() {
    intr = new rs2_intrinsics();
    intr->fx = camparams.fx;
    intr->fy = camparams.fy;
    intr->ppx = camparams.ppx;
    intr->ppy = camparams.ppy;
    intr->height = camparams.height;
    intr->width = camparams.width;
    intr->model = static_cast<rs2_distortion>(camparams.model);
    intr->coeffs[0] = camparams.coeffs[0];
    intr->coeffs[1] = camparams.coeffs[1];
    intr->coeffs[2] = camparams.coeffs[2];
    intr->coeffs[3] = camparams.coeffs[3];
    intr->coeffs[4] = camparams.coeffs[4];

    // converting FOV to focal length: f = (Width/2) / tan(FOV/2)
    float focal_length = (IMG_W / 2) / tan((sim.cam_fov() / 2) * ((float)M_PI / 180));;
    float cx = camparams.ppx;
    float cy = camparams.ppy;
    float baseline = camparams.baseline;
    baseline = fabs(baseline);
    Qf = (cv::Mat_<double>(4, 4) << 1.0, 0.0, 0.0, -cx, 0.0, 1.0, 0.0, -cy, 0.0, 0.0, 0.0, focal_length, 0.0, 0.0, 1 / baseline, 0.0);
}

// refresh the camera images requested by the AirSim API
StereoPair *AirSimCam::update()
{
    /*
        to simulate 90 FPS we actually need 1/90=0.011111111111111112
        but for some (unkown) reason the continueForTime only continues for a number divisible by three
    */
    double desiredFrameTime = 0.012;

    // retrieve the frame from AirSim
    StereoPair *sp = sim.new_frame(desiredFrameTime);
    _current = sp;
    buf.insert(std::pair(sp->rs_id, sp));
    delete_old_frames();

    sim.log(sp->rs_id);
    return _current;
}

void AirSimCam::close()
{
    sim.close();
    Cam::close();
}
