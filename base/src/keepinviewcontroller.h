#pragma once
#include "common.h"
#include "flightarea/flightarea.h"
#include "filtering/filtering.h"
#include "tracking.h"
#include "rapid_route.h"

struct FlightAreaKIVStates {
    std::vector<float> pos_err_kiv;
    std::vector<float>vel_err_kiv;
    std::vector<filtering::Tf_D_f> d_pos_err_kiv;
    std::vector<filtering::Tf_D_f> d_vel_err_kiv;
    std::vector<bool> violated_planes_braking_distance;
    std::vector<bool> violated_planes_inview;
    bool drone_in_boundaries;
    bool enough_braking_distance_left;

    FlightAreaKIVStates() {}

    FlightAreaKIVStates(FlightAreaConfig *flightareaconfig) {
        pos_err_kiv.assign(flightareaconfig->n_planes(), 0);
        vel_err_kiv.assign(flightareaconfig->n_planes(), 0);
        violated_planes_braking_distance.assign(flightareaconfig->n_planes(), false);
        violated_planes_inview.assign(flightareaconfig->n_planes(), false);


        for (uint i = 0; i < flightareaconfig->n_planes(); i++) {
            filtering::Tf_D_f vel_filter;
            vel_filter.init(1.f / pparams.fps);
            d_vel_err_kiv.push_back(vel_filter);

            filtering::Tf_D_f pos_filter;
            pos_filter.init(1.f / pparams.fps);
            d_pos_err_kiv.push_back(pos_filter);
        }
    }
};

class KeepInViewController {
private:
    // Parameter:
    const float top_plane_correction = .2f;

    // States:
    bool enabled = true;
    double last_filter_update = -1;
    std::map<safety_margin_types, FlightAreaKIVStates> filters;

    // Handles:
    xmls::DroneCalibration *_drone_calib;
    FlightArea *_flight_area;

    // Methods:
    std::tuple<std::vector<bool>, std::vector<float>, bool> update_braking_distance_states(FlightAreaConfig *_flight_area_config, tracking::TrackData data_drone, float transmission_delay_duration);
    void update_filter(safety_margin_types safety_margin_type, tracking::TrackData data_drone, std::vector<float> speed_error_normal_to_plane);
    cv::Point3f calc_correction_acceleration(safety_margin_types safety_margin_type, tracking::TrackData drone, control_modes control_mode);
    FlightAreaKIVStates *flight_area_kiv_state(safety_margin_types safety_margin_type);

public:
    // Parameter:
    float drone_rotating_time;
    const float safety = 1.5f;

    // States:
    bool active = false;  //indicator for the log, whether kiv is active or not
    stopping_position_result current_stopping_position;

    // Methods:
    void init(FlightArea *flight_area, xmls::DroneCalibration *dcalib);
    void update(tracking::TrackData data_drone, float transmission_delay_duration, double time);
    cv::Point3f correction_acceleration(safety_margin_types safety_margin, tracking::TrackData drone, control_modes control_mode);
    void enable() {enabled = true;};
    void disable() {enabled = false;};
};
