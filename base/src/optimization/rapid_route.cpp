#include "rapid_route.h"
#include <cmath>

using namespace std;

float largestRoot(float a, float b, float c) {
    // a*x^2 + b*x + c = 0
    float discriminant = b * b - 4 * a * c;

    if (discriminant < 0) {
        return 0;
    }

    float root1 = (-b + sqrt(discriminant)) / (2 * a);
    float root2 = (-b - sqrt(discriminant)) / (2 * a);

    if (root1 > root2) {
        return root1;
    } else {
        return root2;
    }
}

void RapidRouteInterface::init(float *thrust, float thrust_factor) {
    _thrust = thrust;
    _thrust_factor = thrust_factor;
    _gravity = {0, -GRAVITY, 0};
}

rapid_route_result RapidRouteInterface::update_initial_guess(tracking::TrackData drone, tracking::TrackData target, rapid_route_result result) {
    float _position_error = norm(drone.pos() - target.pos());
    float _velocity_error = norm(drone.vel() - target.vel());
    float _estimated_time_to_intercept = largestRoot(1.f / 2.f * _thrust_factor * *_thrust, _velocity_error, -_position_error);

    cv::Point3f _estimated_target_position = target.pos() + target.vel() * _estimated_time_to_intercept;

    cv::Point3f _insect_direction = drone.pos() - _estimated_target_position;
    _insect_direction = _insect_direction / norm(_insect_direction);
    cv::Point3f _estimated_drone_acceleration = -1 * _insect_direction * _thrust_factor * *_thrust;

    result.time_to_intercept = _estimated_time_to_intercept;
    result.position_to_intercept = _estimated_target_position;
    result.acceleration_to_intercept = _estimated_drone_acceleration;
    return result;
}

rapid_route_result RapidRouteInterface::find_best_interception(tracking::TrackData drone, tracking::TrackData target) {
    rapid_route_result result;
    result = update_initial_guess(drone, target, result);

    int _iteration = 0;
    float _lower_bound = 0;
    float _upper_bound = result.time_to_intercept * 100;
    while (_iteration < 100 && !(_iteration > 1 && norm(result.acceleration_to_intercept) > 0.99999 * static_cast<double>(_thrust_factor * *_thrust) && norm(result.acceleration_to_intercept) < static_cast<double>(_thrust_factor * *_thrust))) {
        result.time_to_intercept = (_lower_bound + _upper_bound) / 2;
        result.position_to_intercept = target.pos() + target.vel() * result.time_to_intercept; //todo: consider including target acceleration
        cv::Point3f _position_error = result.position_to_intercept - drone.pos();
        result.acceleration_to_intercept = 2 * (_position_error - drone.vel()  * result.time_to_intercept) / pow(result.time_to_intercept, 2) - _gravity;
        if (norm(result.acceleration_to_intercept) < static_cast<double>(_thrust_factor * *_thrust)) {
            _upper_bound = result.time_to_intercept;
        } else {
            _lower_bound = result.time_to_intercept;
        }
        _iteration++;
    }
    if (feasible_solution(result, drone))
        result.valid = true;
    return result;
}

bool RapidRouteInterface::feasible_solution(rapid_route_result result, tracking::TrackData track_data_drone) {
    if (result.time_to_intercept < 0)
        return false;

    if (normf(result.acceleration_to_intercept) > _thrust_factor * *_thrust) {
        // std::cout << "acceleration too high: " << normf(result.acceleration_to_intercept) << std::endl;
        return false;
    }

    cv::Point3f intercept_pos_drone = track_data_drone.pos() + track_data_drone.vel() * result.time_to_intercept + 1.f / 2.f * (result.acceleration_to_intercept + _gravity) * pow(result.time_to_intercept, 2);
    float intercept_error = normf(result.position_to_intercept - intercept_pos_drone);
    if (intercept_error > 0.01f) {
        // std::cout << "intercept error(tti): " << intercept_error << std::endl;
        return false;
    }

    return true;
}
