#include "keepinviewcontroller.h"
#include "linalg.h"
#include "common.h"


void KeepInViewController::init(FlightArea *flight_area, xmls::DroneCalibration *drone_calib) {
    _flight_area = flight_area;
    _drone_calib = drone_calib;
    drone_rotating_time = dparams.drone_rotation_delay;

    for (uint i = 0; i < number_safety_margin_types; i++) {
        filters[static_cast<safety_margin_types>(i)] = FlightAreaKIVStates(flight_area->flight_area_config(static_cast<safety_margin_types>(i)));
    }

}


void KeepInViewController::update(tracking::TrackData data_drone, float transmission_delay_duration) {
    for (uint i = 0; i < number_safety_margin_types; i++) {
        safety_margin_types safety_margin_type = static_cast<safety_margin_types>(i);
        FlightAreaConfig *_flight_area_config = _flight_area->flight_area_config(safety_margin_type);
        FlightAreaKIVStates *_flight_area_kiv_state = flight_area_kiv_state(safety_margin_type);
        auto [violated_planes_braking_distance, speed_error_normal_to_plane, enough_braking_distance_left] = update_breaking_distance_states(_flight_area_config, data_drone, transmission_delay_duration);
        _flight_area_kiv_state->violated_planes_braking_distance = violated_planes_braking_distance;
        filters[safety_margin_type].enough_braking_distance_left = enough_braking_distance_left;

        update_filter(safety_margin_type, data_drone, speed_error_normal_to_plane);
        auto [drone_in_boundaries, violated_planes_inview] = _flight_area_config->find_violated_planes(data_drone.pos());
        _flight_area_kiv_state->violated_planes_inview = violated_planes_inview;
        _flight_area_kiv_state->drone_in_boundaries = drone_in_boundaries;
    }

    active = false;
}


FlightAreaKIVStates *KeepInViewController::flight_area_kiv_state(safety_margin_types safety_margin_type) {
    std::map<safety_margin_types, FlightAreaKIVStates>::iterator ret = filters.find(safety_margin_type);
    if (ret == filters.end())
        throw std::runtime_error("FlightAreaConfiguration error: configuration missing! Implementation error!");

    return &(ret->second);
}


cv::Point3f KeepInViewController::correction_acceleration(safety_margin_types safety_margin, tracking::TrackData drone, control_modes control_mode) {
    if (enabled && (!filters[safety_margin].drone_in_boundaries || !filters[safety_margin].enough_braking_distance_left)) {
        active = true;
        return calc_correction_acceleration(safety_margin, drone, control_mode);
    }

    return {0, 0, 0};
}

cv::Point3f KeepInViewController::calc_correction_acceleration(safety_margin_types safety_margin_type, tracking::TrackData drone, control_modes control_mode) {
    FlightAreaConfig *_flight_area_config = _flight_area->flight_area_config(safety_margin_type);
    FlightAreaKIVStates *_flightarea_kiv_state = flight_area_kiv_state(safety_margin_type);

    float control_mode_gain = 1.f;
    if (control_mode == acceleration_feedforward)
        control_mode_gain = 2.f;

    cv::Point3f correction_acceleration = {0};

    for (uint plane_id = 0; plane_id < _flight_area_config->n_planes(); plane_id++) {
        cv::Point3f correction_direction;
        if (control_mode == position_control)
            correction_direction = _flight_area_config->planes().at(plane_id).normal;
        else
            correction_direction = - drone.vel() / normf(drone.vel());

        if (_flightarea_kiv_state->violated_planes_inview.at(plane_id)) {
            int d_against_p_error = (sign(_flightarea_kiv_state->d_pos_err_kiv.at(plane_id).current_output()) != sign(_flightarea_kiv_state->pos_err_kiv.at(plane_id)));
            correction_acceleration += correction_direction * (control_mode_gain * dparams.kp_pos_kiv * _flightarea_kiv_state->pos_err_kiv.at(plane_id)
                                       + d_against_p_error * control_mode_gain * dparams.kd_pos_kiv * _flightarea_kiv_state->d_pos_err_kiv.at(plane_id).current_output());
        }
        if (_flightarea_kiv_state->violated_planes_braking_distance.at(plane_id)) {
            correction_acceleration += correction_direction * (control_mode_gain * dparams.kp_vel_kiv * _flightarea_kiv_state->vel_err_kiv.at(plane_id)
                                       + control_mode_gain * dparams.kd_vel_kiv * _flightarea_kiv_state->d_vel_err_kiv.at(plane_id).current_output());
        }
    }
    return correction_acceleration;
}

std::tuple<std::vector<bool>, std::vector<float>, bool> KeepInViewController::update_breaking_distance_states(FlightAreaConfig *_flight_area_config, tracking::TrackData data_drone, float transmission_delay_duration) {
    bool breaking_distance_ok = true;
    std::vector<float> speed_error_normal_to_plane;
    speed_error_normal_to_plane.assign(_flight_area_config->n_planes(), 0.);
    std::vector<bool> violated_planes_braking_distance;
    violated_planes_braking_distance.assign(_flight_area_config->n_planes(), false);

    for (uint plane_id = 0; plane_id < _flight_area_config->n_planes(); plane_id++) {
        if (_flight_area_config->plane(plane_id).is_active) {
            Plane plane = _flight_area_config->plane(plane_id);
            float current_drone_speed_normal_to_plane = data_drone.state.vel.dot(-plane.normal);
            float remaining_breaking_distance_normal_to_plane = plane.distance(data_drone.pos())
                    - current_drone_speed_normal_to_plane * (drone_rotating_time + transmission_delay_duration);
            if (remaining_breaking_distance_normal_to_plane < 0)
                remaining_breaking_distance_normal_to_plane = 0;
            float effective_acceleration = _drone_calib->max_thrust / safety + cv::Point3f(0, -GRAVITY, 0).dot(plane.normal);
            float required_breaking_time = sqrtf(2 * remaining_breaking_distance_normal_to_plane / effective_acceleration);
            if (required_breaking_time != required_breaking_time) { // if required_breaking_time is nan
                // drone max_thrust including the safety is not strong enough to compensate gravity!
                // assume drone can at least accelerate slightly against gravity:
                effective_acceleration = 2.5f;
                required_breaking_time = sqrtf(2 * remaining_breaking_distance_normal_to_plane / effective_acceleration);
            }
            float allowed_velocity_normal_to_plane = required_breaking_time * effective_acceleration;
            speed_error_normal_to_plane.at(plane_id) = current_drone_speed_normal_to_plane - allowed_velocity_normal_to_plane;
            if (speed_error_normal_to_plane.at(plane_id) > 0) {
                violated_planes_braking_distance.at(plane_id) = true;
                breaking_distance_ok = false;
            }
        }
    }

    return std::tuple<std::vector<bool>, std::vector<float>, bool>(violated_planes_braking_distance, speed_error_normal_to_plane, breaking_distance_ok);
}

void KeepInViewController::update_filter(safety_margin_types safety_margin_type, tracking::TrackData data_drone, std::vector<float> speed_error_normal_to_plane) {
    FlightAreaConfig *_flight_area_config = _flight_area->flight_area_config(safety_margin_type);
    FlightAreaKIVStates *_flight_area_kiv_state = flight_area_kiv_state(safety_margin_type);
    for (uint plane_id = 0; plane_id < _flight_area_config->n_planes(); plane_id++) {
        Plane plane = _flight_area_config->plane(plane_id);
        _flight_area_kiv_state->pos_err_kiv.at(plane_id) = -plane.distance(data_drone.pos());
        filters[safety_margin_type].d_pos_err_kiv.at(plane_id).new_sample(filters[safety_margin_type].pos_err_kiv.at(plane_id));
        if (data_drone.vel_valid) {
            _flight_area_kiv_state->vel_err_kiv.at(plane_id) = speed_error_normal_to_plane.at(plane_id);
            _flight_area_kiv_state->d_vel_err_kiv.at(plane_id).new_sample(filters[safety_margin_type].vel_err_kiv.at(plane_id));
        }
    }
}
