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
    stopping_position_rapid_route_interface.init(&thrust, 1.f, _flight_area_config);
    current_stopping_position = stopping_position_rapid_route_interface.find_stopping_position(current_state, safety);
    if (enabled && (!_flight_area_config->inside(current_stopping_position.position) || !_flight_area_config->inside(data_drone.pos()))) {
        // oh no! we have to brake!
        active = true;
        return current_stopping_position.acceleration;
    }
    else {
        // we are safe
        active = false;
        return {0, 0, 0};
    }
}
