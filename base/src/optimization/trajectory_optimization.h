#pragma once
#include "tracking.h"
#include "flightarea.h"

struct trajectory_optimization_result {
    trajectory_optimization_result() {
        valid = false;
        via = false;
        time_to_intercept = 0;
        position_to_intercept = {0, 0, 0};
        velocity_at_intercept = {0, 0, 0};
        acceleration_to_intercept = {0, 0, 0};
        stopping_position = {0, 0, 0};
        intermediate_position = {0, 0, 0};
        time_to_intermediate = 0;
        interception_position_in_flightarea = false;
        stopping_position_in_flightarea = false;
    };
    bool valid;
    bool via;
    float time_to_intercept;
    cv::Point3f position_to_intercept;
    cv::Point3f acceleration_to_intercept;
    cv::Point3f velocity_at_intercept;
    cv::Point3f stopping_position;
    cv::Point3f intermediate_position;
    float time_to_intermediate;
    bool interception_position_in_flightarea;
    bool stopping_position_in_flightarea;
};

struct stopping_position_result {
    stopping_position_result() {
        valid = false;
        position = {0, 0, 0};
        time = 0;
        acceleration = {0, 0, 0};
    };
    bool valid;
    cv::Point3f position;
    float time;
    cv::Point3f acceleration;
};

class TrajectoryOptimizer {
public:
    void init(float *thrust, float thrust_factor, FlightArea *flight_area, safety_margin_types safety_margin_type, float transmission_delay);
    trajectory_optimization_result find_interception_direct(tracking::TrackData track_data_drone, tracking::TrackData track_data_insect, float delay, const float stopping_safety_factor);
    trajectory_optimization_result find_interception_via(tracking::TrackData drone, tracking::TrackData target, float delay, const float stopping_safety_factor, trajectory_optimization_result previous_result);
    trajectory_optimization_result find_interception(tracking::TrackData drone, tracking::TrackData target, float delay, const float stopping_safety_factor);
    stopping_position_result find_stopping_position(trajectory_optimization_result interception_result, const float safety_factor);
private:
    float *_thrust;
    float _thrust_factor;
    float _transmission_delay;
    cv::Point3f _gravity;
    FlightArea _flight_area;
    FlightAreaConfig _flight_area_config;
    safety_margin_types _safety_margin_type;
    trajectory_optimization_result update_initial_guess(tracking::TrackData track_data_drone, tracking::TrackData track_data_insect, trajectory_optimization_result result);
    bool feasible_solution(trajectory_optimization_result result, cv::Point3f drone_pos);
    std::vector<Plane> _sorted_planes;
    std::vector<Plane> _resorted_planes;
};
