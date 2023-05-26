#pragma once
#include <mutex>
#include <condition_variable>
#include <thread>
#include "flightarea/flightarea.h"
#include "flightareaconfig.h"
#include "insecttracker.h"
#include "dronetracker.h"
#include "trackermanager.h"
#include "visiondata.h"
#include "rapid_route.h"
#include "drone.h"
#ifdef OPTI_ROSVIS
#include "rosvisualizerinterface.h"
#endif

static const char *interceptor_state_names[] = { "is_init",
                                                 "is_await_target",
                                                 "is_await_reach_zone",
                                                 "is_lurking",
                                                 "is_intercepting"
                                               };
class Drone;

class Interceptor {

private:
    enum interceptor_states {
        is_init = 0,
        is_waiting_for_target,
        is_waiting_in_reach_zone,
        is_lurking,
        is_intercepting
    };

    enum intercepting_states {
        is_approaching,
        is_intercept_maneuvering
    };

    tracking::TrackerManager *_trackers;
    VisionData *_visdat;
    Drone *_drone;
    FlightArea *_flight_area;
    FlightAreaConfig *_flight_area_config;

    bool initialized = false;
    interceptor_states _interceptor_state = is_init;
    intercepting_states _intercepting_state = is_approaching;
    double _time = -1;

    cv::Point3f interception_center;
    control_modes _control_mode = position_control;
    cv::Point3f _aim_pos;
    cv::Point3f _aim_acc;

    float interception_max_thrust;

    rapid_route_result _rapid_route_result;
    bool interception_position_in_flightarea = false;
    bool stopping_position_in_flightarea = false;

    uint _n_frames_aim_not_in_range = 0;
    uint _n_frames_aim_in_range = 0;
    float n_frames_target_cleared_timeout;
    tracking::InsectTracker *_target_insecttracker = NULL;

    float hunt_error;
    float _best_hunt_error = INFINITY;
    double _time_best_hunt_error = INFINITY;
    cv::Point3f _pos_best_hunt_error  = {INFINITY, INFINITY, INFINITY};
    cv::Point3f _vel_best_hunt_error  = {INFINITY, INFINITY, INFINITY};
    cv::Point3f _acc_best_hunt_error  = {INFINITY, INFINITY, INFINITY};
    double _tti = -1;
    const double duration_intercept_maneuver = 0.2;
    double time_start_intercept_maneuver = -1;

    double optimization_time = 0.008; // @ 90fps optimization time max is 0.011

    float tti_running_avg = -1;

    bool realtime_check = true;
    std::ofstream *_logger;

    tracking::InsectTracker *update_target_insecttracker(float delay);

    void update_aim_in_flightarea(rapid_route_result rapid_route_res);
    rapid_route_result update_aim_and_target_in_flightarea(bool drone_at_base, tracking::TrackData target, float delay);

    void update_hunt_strategy(bool drone_at_base, tracking::TrackData target, double time);
    void update_hunt_distance(bool drone_at_base, cv::Point3f drone_pos, cv::Point3f target_pos, double time);
    bool delay_takeoff_for_better_interception();
    void enter_is_intercept_maneuvering(double time, tracking::TrackData drone) {
        _intercepting_state = is_intercept_maneuvering;
        time_start_intercept_maneuver = time;
        _aim_pos += 1.f * (_aim_pos - drone.pos()) / normf(_aim_pos - drone.pos());
        _control_mode = position_control;
    };
    bool exit_is_intercept_maneuvering(double time) {
        return (time - time_start_intercept_maneuver) > duration_intercept_maneuver;
    };

public:
    RapidRouteInterface rapid_route;

    void init(tracking::TrackerManager *trackers, VisionData *visdat, FlightArea *flight_area, Drone *drone, FlightAreaConfig *flight_area_config);
    void close();
    void init_flight(std::ofstream *logger);
    void log(std::ostream *logger);
    void update(bool drone_at_base, double time);
    void abort_flight();
    void max_optimization_time(double max_time) {optimization_time = max_time;};

    tracking::TrackData target_last_trackdata();

    tracking::InsectTracker *target_insecttracker() {return _target_insecttracker;}
    int insect_id() {
        if (!_target_insecttracker)
            return -1;
        else
            return _target_insecttracker->insect_trkr_id();
    }

    bool target_acquired(double time) { return target_detected(time) && !_n_frames_aim_not_in_range; }

    bool intercepting() {return _interceptor_state == is_intercepting;}

    bool target_detected(double time) {
        if (!_target_insecttracker)
            return false;
        if (_target_insecttracker->type() == tracking::tt_insect && pparams.disable_real_hunts)
            return false;
        return !_n_frames_aim_not_in_range
               && _visdat->no_recent_brightness_events(time)
               && !_trackers->monster_alert()
               && !_target_insecttracker->false_positive()
               && _target_insecttracker->properly_tracking();
    }

    bool target_cleared() {return _n_frames_aim_not_in_range > n_frames_target_cleared_timeout;}

    control_modes control_mode() {return _control_mode;}
    cv::Point3f aim_pos() {return _aim_pos;}
    cv::Point3f aim_acc() {return _aim_acc;}

    cv::Point3f interception_pos() {return _rapid_route_result.position_to_intercept;}
    cv::Point3f stopping_pos() {return _rapid_route_result.stopping_position;}
    bool via() {return _rapid_route_result.via;}
    cv::Point3f intermediate_pos() {return _rapid_route_result.intermediate_position;}
    double time_to_intercept() {return _tti;}

    float best_distance() {return _best_hunt_error;}
    float time_best_distance() {return _time_best_hunt_error;}
    cv::Point3f pos_best_distance() {return _pos_best_hunt_error;}
    cv::Point3f vel_best_distance() {return _vel_best_hunt_error;}
    cv::Point3f acc_best_distance() {return _acc_best_hunt_error;}
    void reset_hunt_error() {_best_hunt_error = INFINITY;}

    double tti() {return _tti;};
    void target_is_hunted(int hunt_id) {
        if (_target_insecttracker)
            _target_insecttracker->hunt_id(hunt_id);
    }

    std::string Interceptor_State() {return interceptor_state_names[_interceptor_state];}

    bool insect_in_pad_area();
};
