#pragma once
#include "common.h"
#include "flightarea/flightareaconfig.h"
#include "filtering/filtering.h"
#include "tracking.h"

class KeepInViewController {
private:
    // Parameter:
    float drone_rotating_time;
    const float safety = 2.f;
    safety_margin_types used_safety_margin = relaxed;

    // States:
    bool enabled = true;
    std::vector<float> pos_err_kiv;
    std::vector<float>vel_err_kiv;

    // Handles:
    xmls::DroneCalibration *_drone_calib;
    std::vector<filtering::Tf_D_f> d_pos_err_kiv, d_vel_err_kiv;
    FlightAreaConfig *_flight_area_config;

    // Methods:
    void update_correction_acc(tracking::TrackData data_drone, float transmission_delay_duration, bool dctrl_requests_kiv_acc);
    std::tuple<std::vector<bool>, std::vector<float>, bool> violated_planes_brake_distance(tracking::TrackData data_drone, float transmission_delay_duration);
    void update_filter(tracking::TrackData data_drone, std::vector<float> speed_error_normal_to_plane);
    cv::Point3f kiv_acceleration(std::vector<bool> violated_planes_inview, std::vector<bool> violated_planes_brake_distance);

public:
    // States:
    bool active = false;  //indicator for the log, whether kiv is active or not

    // Methods:
    void init(FlightAreaConfig *flight_area_config, xmls::DroneCalibration *dcalib);
    cv::Point3f update(tracking::TrackData data_drone, float transmission_delay_duration, bool dctrl_requests_kiv_acc);
    void enable() {enabled = true;};
    void disable() {enabled = false;};
};
