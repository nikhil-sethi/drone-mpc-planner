#pragma once
#include "tracking.h"

struct rapid_route_result {
    rapid_route_result() {
        valid = false;
        time_to_intercept = 0;
        position_to_intercept = {0, 0, 0};
        acceleration_to_intercept = {0, 0, 0};
    };
    bool valid;
    float time_to_intercept;
    cv::Point3f position_to_intercept;
    cv::Point3f acceleration_to_intercept;
};

class RapidRouteInterface {
public:
    void init(float *thrust, float thrust_factor);
    rapid_route_result find_best_interception(tracking::TrackData track_data_drone, tracking::TrackData track_data_insect);
private:
    float *_thrust;
    float _thrust_factor;
    cv::Point3f _gravity;
    rapid_route_result update_initial_guess(tracking::TrackData track_data_drone, tracking::TrackData track_data_insect, rapid_route_result result);
    bool feasible_solution(rapid_route_result result, tracking::TrackData track_data_drone);
};
