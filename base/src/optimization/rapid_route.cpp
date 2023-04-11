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

void RapidRouteInterface::init(float *thrust, float thrust_factor, FlightAreaConfig *flight_area_config) {
    _thrust = thrust;
    _thrust_factor = thrust_factor;
    _gravity = {0, -GRAVITY, 0};
    _flight_area_config = *flight_area_config;
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

rapid_route_result RapidRouteInterface::find_interception_direct(tracking::TrackData drone, tracking::TrackData target, float delay, const float stopping_safety_factor) {
    rapid_route_result result;
    result = update_initial_guess(drone, target, result);

    int _iteration = 0;
    float _lower_bound = 0;
    float _upper_bound = abs(result.time_to_intercept * 100);
    cv::Point3f _directionality = {1, 1, 1};
    while (_iteration < 100 && !(_iteration > 1 && norm(result.acceleration_to_intercept) > 0.99999 * static_cast<double>(_thrust_factor * *_thrust) && norm(result.acceleration_to_intercept) < static_cast<double>(_thrust_factor * *_thrust))) {
        result.time_to_intercept = (_lower_bound + _upper_bound) / 2;
        result.position_to_intercept = target.pos() + target.vel() * (result.time_to_intercept + delay); //todo: consider including target acceleration
        cv::Point3f _position_error = result.position_to_intercept - drone.pos();
        result.acceleration_to_intercept = 2 * (_position_error - drone.vel()  * result.time_to_intercept) / pow(result.time_to_intercept, 2) - _gravity;
        _directionality = {_position_error.x / result.acceleration_to_intercept.x, _position_error.y / result.acceleration_to_intercept.y, _position_error.z / result.acceleration_to_intercept.z};
        if (_directionality.x < 0 && _directionality.y < 0 && _directionality.z < 0) {
            _upper_bound = result.time_to_intercept;
        }
        else {
            if (norm(result.acceleration_to_intercept) < static_cast<double>(_thrust_factor * *_thrust)) {
                _upper_bound = result.time_to_intercept;
            } else {
                _lower_bound = result.time_to_intercept;
            }
        }
        _iteration++;
    }
    result.velocity_at_intercept = drone.vel() + (result.acceleration_to_intercept + _gravity) * result.time_to_intercept;
    if (feasible_solution(result, drone))
        result.valid = true;
    result.stopping_position = find_stopping_position(result, drone, stopping_safety_factor);
    result.via = false;

    return result;
}

rapid_route_result RapidRouteInterface::find_interception_via(tracking::TrackData drone, tracking::TrackData target, float delay, const float stopping_safety_factor) {
    rapid_route_result result;
    cv::Point3f _intersection;
    cv::Point3f _target_position_after_time_to_reach;
    int _iteration_counter = 0;
    cv::Point3f _target_position = target.pos();
    while (1) {
        _sorted_planes = _flight_area_config.sort_planes_by_proximity(_target_position);
        Plane _first_plane = _sorted_planes[0];
        int _second_plane_idx = _flight_area_config.find_next_non_parallel_plane(_sorted_planes, 0);
        Plane _second_plane = _sorted_planes[_second_plane_idx];
        int _third_plane_idx = _flight_area_config.find_next_non_parallel_plane(_sorted_planes, 0, _second_plane_idx);
        Plane _third_plane = _sorted_planes[_third_plane_idx];

        _intersection = intersection_of_3_planes(&_first_plane, &_second_plane, &_third_plane);
        _intersection = _flight_area_config.move_inside(_intersection); // may still exceed the constraints of a 4+th plane
        float _time_to_reach_intersection = sqrt(4 * norm(_intersection - drone.pos()) / (_thrust_factor * *_thrust));
        _target_position_after_time_to_reach = target.pos() + target.vel() * _time_to_reach_intersection;

        _resorted_planes = _flight_area_config.sort_planes_by_proximity(_target_position_after_time_to_reach);

        if (_sorted_planes == _resorted_planes || _iteration_counter > 20)
            break;
        else
            _iteration_counter++;
    }
    tracking::TrackData _future_drone = drone;
    _future_drone.state.pos = _intersection;
    _future_drone.state.vel = {0, 0, 0};
    _future_drone.state.acc = {0, 0, 0};

    tracking::TrackData _future_target = target;
    _future_target.state.pos = _target_position_after_time_to_reach;

    result = find_interception_direct(_future_drone, _future_target, delay, stopping_safety_factor);
    result.intermediate_position = _intersection;
    result.via = true;
    // std::vector<CornerPoint> _corners = _flight_area_config.corner_points()

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

cv::Point3f RapidRouteInterface::find_stopping_position(rapid_route_result interception_result, tracking::TrackData drone, const float safety_factor) {
    cv::Point3f _velocity_at_interception = interception_result.velocity_at_intercept;
    cv::Point3f _velocity_at_interception_hat = _velocity_at_interception / norm(_velocity_at_interception);
    cv::Point3f _stopping_vector_hat = -1 * _velocity_at_interception_hat;

    float _max_thrust = _thrust_factor * *_thrust / safety_factor;
    int _iteration = 0;
    float _lower_bound = 0;
    float _upper_bound = 3 * _max_thrust;
    cv::Point3f _stopping_vector = _stopping_vector_hat * (_lower_bound + _upper_bound) / 2;
    while (_iteration < 100 && !(_iteration > 1 && norm(_stopping_vector - _gravity) > 0.99999 * static_cast<double>(_max_thrust) && norm(_stopping_vector - _gravity) < static_cast<double>(_max_thrust))) {
        float _total_acc = (_lower_bound + _upper_bound) / 2;
        _stopping_vector = _stopping_vector_hat * _total_acc;

        if (norm(_stopping_vector - _gravity) < static_cast<double>(_max_thrust)) {
            _lower_bound = _total_acc;
        } else {
            _upper_bound = _total_acc;
        }
        _iteration++;
    }
    cv::Point3f _stopping_time = {0, 0, 0};
    if (_stopping_vector.x != 0)
        _stopping_time.x = -_velocity_at_interception.x / _stopping_vector.x;
    if (_stopping_vector.y != 0)
        _stopping_time.y = -_velocity_at_interception.y / _stopping_vector.y;
    if (_stopping_vector.z != 0)
        _stopping_time.z = -_velocity_at_interception.z / _stopping_vector.z;
    float _stopping_time_max = abs(std::max(std::max(_stopping_time.x, _stopping_time.y), _stopping_time.z));
    // cv::Point3f _stopping_acceleration = _stopping_vector - _gravity;
    cv::Point3f _stopping_distance = _velocity_at_interception * _stopping_time_max + 1.f / 2.f * _stopping_vector * pow(_stopping_time_max, 2);
    cv::Point3f _stopping_position = interception_result.position_to_intercept + _stopping_distance;
    return _stopping_position;
}
