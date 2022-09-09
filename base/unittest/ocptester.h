#pragma once
#include "tracking.h"
#include "plane.h"
#include "flightarea.h"
#include "tti_optimizer_interface.h"
#include "intercept_in_planes_optimizer_interface.h"

enum optimizer_test {
    time_to_intercept,
    intercept_in_planes,
};

struct range_stats {
    double average_optimizing_time_us;
    double sigma2_optimizing_time_ms;
    double max_optimizing_time_us;
    float valid_optimizing_timings_percent;

    uint invalid_optimization_results; // aka found flights
    double average_interception_time_s;
    double timing_variance_us;

    range_stats(): average_optimizing_time_us(0), sigma2_optimizing_time_ms(0), max_optimizing_time_us(0), valid_optimizing_timings_percent(0),
        invalid_optimization_results(0), average_interception_time_s(0), timing_variance_us(0) {};
};

struct range_data {
    uint invalid_optimization_results;
    uint interceptions_found;
    uint n_optimizer_tests;
    uint invalid_timings;
    double max_optimizing_time_us;
    double accumulated_interception_attitude;
    double accumulated_interception_time_s;
    double accumulated_optimizing_time_us;

    std::vector<double> timings_us;

    range_data(): invalid_optimization_results(0), interceptions_found(0), n_optimizer_tests(0), invalid_timings(0), max_optimizing_time_us(0),
        accumulated_interception_attitude(0), accumulated_interception_time_s(0), accumulated_optimizing_time_us(0) {
        timings_us = {};
    }
};

class OcpTester {
public:
    float thrust = 25;
    const float eps = 0.0001; // accepted error (rounding errors)
    const double realtime_boundary_ms = 1. / 90 * 1000; //[ms]

    /*In some configutation the worst case estimation of the tti is also providing the best possible solution.
      In this situation the worst estimation is likely to be better than the optimizer
      because the optimizers only runs tills it matches some convergence cretiria.
      `accpt_tti_error` is holding the maximum violation of the estimated tti.*/
    float accpt_tti_error = 0.1;

    bool is_tti_good(float optimizer_tti, float estimated_tt);
    bool is_optimizer_results_good(tracking::TrackData drone, tracking::TrackData insect, bool res_valid, float res_tti);


    std::vector<Plane> cube_planes(float cube_size, uint n_planes);

    range_stats exec_range_test(optimizer_test optimizer_select, bool use_casadi, sqp_solver_configuration sqp_config);
    void find_parameter(optimizer_test optimizer_select, bool use_casadi);
    std::tuple<tracking::TrackData, tracking::TrackData> case_test();
    std::tuple<tracking::TrackData, tracking::TrackData, std::vector<Plane>> application_test();

private:
    TTIOptimizerInterface tti;
    InterceptInPlanesOptimizerInterface iip;

    range_stats eval_range_data(range_data data);
    void cout_header(optimizer_test optimizer_select);
    void cout_setup(optimizer_test optimizer_select, bool use_casadi, sqp_solver_configuration sqp_config, bool enable_stress, bool disable_time_ensurance);
    void cout_optmization_stats(optimizer_test optimizer_select, range_data range_dat, range_stats stats);
    void cout_configuration_results(std::vector<sqp_solver_configuration> sqp_conf, std::vector<range_stats> stats);
};
