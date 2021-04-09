#pragma once
#include "common.h"
#include "cameraview.h"
#include "filtering/filtering.h"
#include "tracking.h"

class KeepInViewController {
private:
    // Parameter:
    float drone_rotating_time;
    const float safety = 2.f;

    // States:
    std::array<float, N_PLANES> pos_err_kiv = {0}, vel_err_kiv = {0};
    bool enabled = true;

    // Handles:
    xmls::DroneCalibration* _dcalib;
    std::array<filtering::Tf_D_f, N_PLANES> d_pos_err_kiv, d_vel_err_kiv;
    CameraView* _camview;

public:
    // States:
    bool active = false;

    void init(CameraView* camview, xmls::DroneCalibration* dcalib);
    cv::Point3f update(tracking::TrackData data_drone, float transmission_delay_duration, bool correction_requested);
    cv::Point3f kiv_acceleration(std::array<bool, N_PLANES> violated_planes_inview, std::array<bool, N_PLANES> violated_planes_brakedistance);
    bool trajectory_in_view(std::vector<tracking::StateData> traj, CameraView::view_volume_check_mode c);
    void enable() {enabled = true;};
    void disable() {enabled = false;};
};
