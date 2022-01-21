#include "keepinviewcontroller.h"
#include "linalg.h"
#include "common.h"


void KeepInViewController::init(FlightAreaConfig *flight_area_config, xmls::DroneCalibration *drone_calib) {
    _flight_area_config = flight_area_config;
    _drone_calib = drone_calib;
    drone_rotating_time = 6.f / pparams.fps; // Est. time to rotate the drone around 180 deg. see betaflight

    pos_err_kiv.assign(_flight_area_config->n_planes(), 0);
    vel_err_kiv.assign(_flight_area_config->n_planes(), 0);

    for (uint i = 0; i < _flight_area_config->n_planes(); i++) {
        filtering::Tf_D_f vel_filter;
        vel_filter.init(1.f / pparams.fps);
        d_vel_err_kiv.push_back(vel_filter);

        filtering::Tf_D_f pos_filter;
        pos_filter.init(1.f / pparams.fps);
        d_pos_err_kiv.push_back(pos_filter);
    }
}

cv::Point3f KeepInViewController::update(tracking::TrackData data_drone, float transmission_delay_duration, bool dctrl_requests_kiv_acc) {
    if (enabled) {
        auto [violated_planes_braking_distance, speed_error_normal_to_plane, enough_braking_distance_left] = violated_planes_brake_distance(data_drone, transmission_delay_duration);
        update_filter(data_drone, speed_error_normal_to_plane);
        auto [drone_in_boundaries, violated_planes_inview] = _flight_area_config->find_violated_planes(data_drone.pos());

        if (dctrl_requests_kiv_acc && (!drone_in_boundaries || !enough_braking_distance_left)) {
            active = true;
            return kiv_acceleration(violated_planes_inview, violated_planes_braking_distance);
        }
    }
    active = false;
    return {0};
}

cv::Point3f KeepInViewController::kiv_acceleration(std::vector<bool> violated_planes_inview, std::vector<bool> violated_planes_brake_distance) {
#if CAMERA_VIEW_DEBUGGING
    _camview->cout_plane_violation(violated_planes_inview, violated_planes_brake_distance);
#endif
    cv::Point3f correction_acceleration = {0};
    for (uint plane_id = 0; plane_id < _flight_area_config->n_planes(); plane_id++) {
        if (violated_planes_inview.at(plane_id)) {
            int d_against_p_error = (sign(d_pos_err_kiv.at(plane_id).current_output()) != sign(pos_err_kiv.at(plane_id)));
            correction_acceleration += _flight_area_config->plane(plane_id).normal * (kp_pos_kiv * pos_err_kiv.at(plane_id)
                                       + d_against_p_error * kd_pos_kiv * d_pos_err_kiv.at(plane_id).current_output());
        }
        if (violated_planes_brake_distance.at(plane_id))
            correction_acceleration += _flight_area_config->plane(plane_id).normal * (kp_vel_kiv * vel_err_kiv.at(plane_id)
                                       + kd_vel_kiv * d_vel_err_kiv.at(plane_id).current_output());
    }
    return correction_acceleration;
}

std::tuple<std::vector<bool>, std::vector<float>, bool> KeepInViewController::violated_planes_brake_distance(tracking::TrackData data_drone, float transmission_delay_duration) {
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

void KeepInViewController::update_filter(tracking::TrackData data_drone, std::vector<float> speed_error_normal_to_plane) {
    for (uint plane_id = 0; plane_id < _flight_area_config->n_planes(); plane_id++) {
        Plane plane = _flight_area_config->plane(plane_id);
        pos_err_kiv.at(plane_id) = -plane.distance(data_drone.pos());
        d_pos_err_kiv.at(plane_id).new_sample(pos_err_kiv.at(plane_id));
        if (data_drone.vel_valid) {
            vel_err_kiv.at(plane_id) = speed_error_normal_to_plane.at(plane_id);
            d_vel_err_kiv.at(plane_id).new_sample(vel_err_kiv.at(plane_id));
        }
    }
}
