#include "trajectory_optimization.h"
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

void TrajectoryOptimizer::init(float *thrust, float thrust_factor, FlightArea *flight_area, safety_margin_types safety_margin_type, float transmission_delay) {
    _thrust = thrust;
    _thrust_factor = MAX(thrust_factor, 9.80f / *_thrust);
    _gravity = {0, -GRAVITY, 0};
    _safety_margin_type = safety_margin_type;
    _flight_area = *flight_area;
    _flight_area_config = *_flight_area.flight_area_config(_safety_margin_type);
    _transmission_delay = transmission_delay;
}

trajectory_optimization_result TrajectoryOptimizer::update_initial_guess(tracking::TrackData drone, tracking::TrackData target, trajectory_optimization_result result) {
    float _position_error = normf(drone.pos() - target.pos());
    float _velocity_error = normf(drone.vel() - target.vel());
    float _estimated_time_to_intercept = largestRoot(1.f / 2.f * _thrust_factor * *_thrust, _velocity_error, -_position_error);

    cv::Point3f _estimated_target_position = target.pos() + target.vel() * _estimated_time_to_intercept;

    cv::Point3f _insect_direction = drone.pos() - _estimated_target_position;
    _insect_direction = _insect_direction / normf(_insect_direction);
    cv::Point3f _estimated_drone_acceleration = -1 * _insect_direction * _thrust_factor * *_thrust;

    result.time_to_intercept = _estimated_time_to_intercept;
    result.position_to_intercept = _estimated_target_position;
    result.acceleration_to_intercept = _estimated_drone_acceleration;
    return result;
}

trajectory_optimization_result TrajectoryOptimizer::find_interception_direct(tracking::TrackData drone, tracking::TrackData target, float delay, const float stopping_safety_factor) {
    trajectory_optimization_result result;
    result = update_initial_guess(drone, target, result);

    int _iteration = 0;
    float _lower_bound = 0;
    float _upper_bound = abs(result.time_to_intercept * 3.f);
    while (_iteration < 100 && !(_iteration > 1 && normf(result.acceleration_to_intercept) > 0.99999f * (_thrust_factor * *_thrust) && normf(result.acceleration_to_intercept) < (_thrust_factor * *_thrust))) {
        result.time_to_intercept = (_lower_bound + _upper_bound) / 2.f;
        result.position_to_intercept = target.pos() + target.vel() * (result.time_to_intercept + delay); //todo: consider including target acceleration
        cv::Point3f _position_error = result.position_to_intercept - drone.pos();
        result.acceleration_to_intercept = 2 * (_position_error - drone.vel()  * result.time_to_intercept) / pow(result.time_to_intercept, 2) - _gravity;

        if (_position_error.dot(result.acceleration_to_intercept) < 0) {
            _lower_bound = 0;
            _upper_bound = result.time_to_intercept;
        }
        else {
            if (normf(result.acceleration_to_intercept) < (_thrust_factor * *_thrust)) {
                _upper_bound = result.time_to_intercept;
            } else {
                _lower_bound = result.time_to_intercept;
            }
        }
        _iteration++;
    }
    result.velocity_at_intercept = drone.vel() + (result.acceleration_to_intercept + _gravity) * result.time_to_intercept;
    stopping_position_result _stopping_position_result = find_stopping_position(result, stopping_safety_factor);
    result.stopping_position = _stopping_position_result.position;
    result.intermediate_position = {0.f, 0.f, 0.f};
    result.via = false;

    return result;
}

trajectory_optimization_result TrajectoryOptimizer::find_interception_via(tracking::TrackData drone, tracking::TrackData target, float delay, const float stopping_safety_factor, trajectory_optimization_result previous_result) {
    cv::Point3f _original_target_position = target.pos();
    int _intermediate_pos_cnt = 0;

    cv::Point3f _interception_position = previous_result.position_to_intercept;
    cv::Point3f _stopping_position = previous_result.stopping_position;

    cv::Point3f _constraining_point = {0.f, 0.f, 0.f};

    cv::Point3f _previous_intermediate_position = drone.pos();
    cv::Point3f _intermediate_position = drone.pos();

    Plane _previous_most_constraining_plane = Plane(0.f, 0.f, 0.f, 0.f, 0.f, 0.f, unspecified_plane);
    Plane _most_constraining_plane = Plane(0.f, 0.f, 0.f, 0.f, 0.f, 0.f, unspecified_plane);

    trajectory_optimization_result _direct_result;

    float _time_to_reach_intersection = -1;

    int MAX_ITERATIONS = 100;
    while (_intermediate_pos_cnt < MAX_ITERATIONS) {
        if (!_flight_area.inside(_interception_position, _safety_margin_type)) {
            _constraining_point = _interception_position;
        }
        else if (!_flight_area.inside(_stopping_position, _safety_margin_type)) {
            _constraining_point = _stopping_position;
        }
        else {
            // found solution
            break;
        }

        _most_constraining_plane = _flight_area_config.find_most_constraining_plane(_constraining_point);
        _intermediate_position = _flight_area.move_inside(_flight_area_config.project_towards_plane(_previous_intermediate_position, _most_constraining_plane, 0.05f), _safety_margin_type);
        _previous_intermediate_position = _intermediate_position;

        // this approximation can be improved by considering velocity in other directions
        cv::Point3f dir = (_intermediate_position - drone.pos()) / normf(_intermediate_position - drone.pos());
        float _vel_in_dir = drone.vel().dot(dir);
        float _time_to_stop = 0;
        float _distance_to_stop = 0;
        if (_vel_in_dir < 0) {
            _time_to_stop = abs(_vel_in_dir / (_thrust_factor * *_thrust));
            _distance_to_stop = abs(_vel_in_dir) * _time_to_stop - _thrust_factor * *_thrust * powf(_time_to_stop, 2) / 2.f;
        }
        float a = (_thrust_factor * *_thrust + _gravity.dot(dir)) / 2.f;
        float b = (_vel_in_dir > 0) ? _vel_in_dir : 0;
        float c = -normf(_intermediate_position - drone.pos() - dir * _distance_to_stop) / 2.f;
        float a1, a2, valid;
        std::tie(a1, a2, valid) = solve_quadratic_solution(a, b, c);
        if (valid < 0) {
            _time_to_reach_intersection = abs(sqrt(2 * (normf(_intermediate_position - drone.pos())) / _thrust_factor * *_thrust) + delay);
        }
        else {
            bool a1_valid = (0 <= a1);
            bool a2_valid = (0 <= a2);
            if (a1_valid && !a2_valid) {
                _time_to_reach_intersection = abs(a1) + delay;
            } else if (!a1_valid && a2_valid) {
                _time_to_reach_intersection = abs(a2) + delay;
            } else if (a1_valid && a2_valid) {
                if (a1 >= a2) {
                    _time_to_reach_intersection = abs(a1) + delay;
                } else {
                    _time_to_reach_intersection = abs(a2) + delay;
                }
            } else {
                _time_to_reach_intersection = abs(sqrt(2 * (normf(_intermediate_position - drone.pos())) / _thrust_factor * *_thrust) + delay);
            }
        }
        _time_to_reach_intersection += _time_to_stop;

        tracking::TrackData _future_drone = drone;
        _future_drone.state.pos = _intermediate_position;
        _future_drone.state.vel = {0, 0, 0};
        _future_drone.state.acc = {0, 0, 0};

        tracking::TrackData _future_target = target;
        _future_target.state.pos = _original_target_position + target.vel() * _time_to_reach_intersection;

        _direct_result = find_interception_direct(_future_drone, _future_target, 0.f, stopping_safety_factor);

        _interception_position = _direct_result.position_to_intercept;
        _stopping_position = _direct_result.stopping_position;

        _previous_most_constraining_plane = _most_constraining_plane;
        _intermediate_pos_cnt += 1;
    }

    _direct_result.via = true;
    _direct_result.time_to_intermediate = _time_to_reach_intersection;
    _direct_result.intermediate_position = _intermediate_position;

    return _direct_result;
}

trajectory_optimization_result TrajectoryOptimizer::find_interception(tracking::TrackData drone, tracking::TrackData target, float delay, const float stopping_safety_factor) {
    trajectory_optimization_result _result;
    _result = find_interception_direct(drone, target, delay, stopping_safety_factor);
    _result.valid = feasible_solution(_result, drone.pos());
    if (!(_flight_area.inside(_result.position_to_intercept, _safety_margin_type) && _flight_area.inside(_result.stopping_position, _safety_margin_type)) || !_result.valid) {
        _result = find_interception_via(drone, target, delay, stopping_safety_factor, _result);
        _result.valid = feasible_solution(_result, _result.intermediate_position);
    }
    return _result;
}

bool TrajectoryOptimizer::feasible_solution(trajectory_optimization_result result, cv::Point3f drone_pos) {
    if (result.time_to_intercept < 0) {
        return false;
    }

    if (normf(result.acceleration_to_intercept) > _thrust_factor * *_thrust) {
        return false;
    }
    cv::Point3f _position_error = result.position_to_intercept - drone_pos;
    if (_position_error.dot(result.acceleration_to_intercept) < 0) {
        return false;
    }

    if (result.position_to_intercept != result.position_to_intercept || result.acceleration_to_intercept != result.acceleration_to_intercept || result.velocity_at_intercept != result.velocity_at_intercept || result.stopping_position != result.stopping_position || result.intermediate_position != result.intermediate_position) {
        // NaN
        return false;
    }

    return true;
}

stopping_position_result TrajectoryOptimizer::find_stopping_position(trajectory_optimization_result interception_result, const float safety_factor) {
    cv::Point3f _velocity_at_interception = interception_result.velocity_at_intercept;
    cv::Point3f _velocity_at_interception_hat = _velocity_at_interception / normf(_velocity_at_interception);
    cv::Point3f _stopping_vector_hat = -1 * _velocity_at_interception_hat;

    float _max_thrust = *_thrust / safety_factor;
    if (_max_thrust < abs(GRAVITY)) {
        _max_thrust  = abs(GRAVITY) + .1f; // asume that drone thrust is at least gravity,
    }
    int _iteration = 0;
    float _lower_bound = 0;
    float _upper_bound = 3 * _max_thrust;
    cv::Point3f _stopping_vector = _stopping_vector_hat * (_lower_bound + _upper_bound) / 2;
    cv::Point3f _stopping_vector_gravity_compensated = _stopping_vector - _gravity;
    while (_iteration < 100 && !(_iteration > 1 && normf(_stopping_vector_gravity_compensated) > 0.99999f * _max_thrust && normf(_stopping_vector_gravity_compensated) < _max_thrust)) {
        float _total_acc = (_lower_bound + _upper_bound) / 2;
        _stopping_vector = _stopping_vector_hat * _total_acc;
        _stopping_vector_gravity_compensated = _stopping_vector - _gravity;

        if (normf(_stopping_vector_gravity_compensated) < _max_thrust) {
            _lower_bound = _total_acc;
        } else {
            _upper_bound = _total_acc;
        }
        _iteration++;
    }
    cv::Point3f _stopping_time = {0, 0, 0};
    _stopping_vector = _stopping_vector_gravity_compensated + _gravity;
    if (_stopping_vector.x != 0)
        _stopping_time.x = -_velocity_at_interception.x / _stopping_vector.x;
    if (_stopping_vector.y != 0)
        _stopping_time.y = -_velocity_at_interception.y / _stopping_vector.y;
    if (_stopping_vector.z != 0)
        _stopping_time.z = -_velocity_at_interception.z / _stopping_vector.z;
    float _stopping_time_max = abs(std::max(std::max(_stopping_time.x, _stopping_time.y), _stopping_time.z));
#ifdef UNIT_TESTING // cannot access dparams from unit tests
    cv::Point3f _stopping_distance = _velocity_at_interception * (_stopping_time_max + 0.033215252f + _transmission_delay) + 1.f / 2.f * _stopping_vector * pow(_stopping_time_max, 2);
#else
    cv::Point3f _stopping_distance = _velocity_at_interception * (_stopping_time_max + static_cast<float>(dparams.drone_rotation_delay) + _transmission_delay) + 1.f / 2.f * _stopping_vector * pow(_stopping_time_max, 2);
#endif
    cv::Point3f _stopping_position = interception_result.position_to_intercept + _stopping_distance;
    stopping_position_result _result;
    _result.position = _stopping_position;
    _result.time = _stopping_time_max;
    _result.acceleration = _stopping_vector;
    return _result;
}
