#include "keepinviewcontroller.h"
#include "linalg.h"
#include "common.h"


void KeepInViewController::init(FlightArea *flight_area, xmls::DroneCalibration *drone_calib) {
    _flight_area = flight_area;
    _drone_calib = drone_calib;
    drone_rotating_time = dparams.drone_rotation_delay;
}

cv::Point3f KeepInViewController::correction_acceleration(tracking::TrackData data_drone, float transmission_delay_duration, safety_margin_types safety_margin_type) {
    FlightAreaConfig *_flight_area_config = _flight_area->flight_area_config(safety_margin_type);
    rapid_route_result current_state;
    current_state.velocity_at_intercept = data_drone.vel();
    current_state.position_to_intercept = data_drone.pos() + data_drone.vel() * transmission_delay_duration;
    RapidRouteInterface stopping_position_rapid_route_interface;
    float thrust = _drone_calib->max_thrust;
    stopping_position_rapid_route_interface.init(&thrust, 1.f, _flight_area_config, transmission_delay_duration);
    current_stopping_position = stopping_position_rapid_route_interface.find_stopping_position(current_state, safety);
    if (enabled) {
        if (!_flight_area_config->inside(data_drone.pos())) {
            // oh no! we are outside the flight area!
            active = true;
            auto [drone_in_boundaries, violated_planes_inview] = _flight_area_config->find_violated_planes(data_drone.pos());
            cv::Point3f correction_acceleration = {0, 0, 0};
            for (uint plane_id = 0; plane_id < _flight_area_config->n_planes(); plane_id++) {
                cv::Point3f plane_normal = _flight_area_config->planes().at(plane_id).normal;
                float _distance_to_plane = abs(_flight_area_config->planes().at(plane_id).distance(data_drone.pos()));
                float _velocity_normal_to_plane = data_drone.vel().dot(plane_normal);
                if (violated_planes_inview.at(plane_id)) {
                    correction_acceleration += plane_normal * (dparams.kp_pos_kiv * _distance_to_plane + dparams.kd_pos_kiv * _velocity_normal_to_plane);
                }
            }
            return correction_acceleration;

        } else if (!_flight_area_config->inside(current_stopping_position.position)) {
            // oh no! we have to brake!
            active = true;
            return current_stopping_position.acceleration;
        }
    }
    // we are safe
    active = false;
    return {0, 0, 0};
}
