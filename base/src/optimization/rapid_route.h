#pragma once
#include "tracking.h"
#include "flightareaconfig.h"

struct rapid_route_result {
    rapid_route_result() {
        valid = false;
        via = false;
        time_to_intercept = 0;
        position_to_intercept = {0, 0, 0};
        acceleration_to_intercept = {0, 0, 0};
        stopping_position = {0, 0, 0};
    };
    bool valid;
    bool via;
    float time_to_intercept;
    cv::Point3f position_to_intercept;
    cv::Point3f acceleration_to_intercept;
    cv::Point3f velocity_at_intercept;
    cv::Point3f stopping_position;
    cv::Point3f intermediate_position;
};

class RapidRouteInterface {
public:
    void init(float *thrust, float thrust_factor, FlightAreaConfig *flight_area_config);
    rapid_route_result find_interception_direct(tracking::TrackData track_data_drone, tracking::TrackData track_data_insect, float delay, const float stopping_safety_factor);
    rapid_route_result find_interception_via(tracking::TrackData drone, tracking::TrackData target, float delay, const float stopping_safety_factor);
private:
    float *_thrust;
    float _thrust_factor;
    cv::Point3f _gravity;
    FlightAreaConfig _flight_area_config;
    rapid_route_result update_initial_guess(tracking::TrackData track_data_drone, tracking::TrackData track_data_insect, rapid_route_result result);
    bool feasible_solution(rapid_route_result result, tracking::TrackData track_data_drone);
    cv::Point3f find_stopping_position(rapid_route_result interception_result, tracking::TrackData drone, const float safety_factor);
    std::vector<Plane> _sorted_planes;
    std::vector<Plane> _resorted_planes;
};
