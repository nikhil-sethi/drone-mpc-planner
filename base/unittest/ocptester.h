#pragma once
#include "tracking.h"
#include "plane.h"
#include "flightarea.h"
#include "tti_optimizer_interface.h"
#include "intercept_in_planes_optimizer_interface.h"
#include "rapid_route.h"

enum optimizer_test {
    time_to_intercept,
    intercept_in_planes,
    rapid_route,
};

struct range_stats {
    double average_optimizing_time_us;
    double sigma2_optimizing_time_ms;
    double max_optimizing_time_us;
    float valid_optimizing_timings_percent;

    uint invalid_optimization_results; //[1]
    float valid_optimization_results; //[%]
    double average_interception_time_s;
    double timing_variance_us;

    range_stats(): average_optimizing_time_us(0), sigma2_optimizing_time_ms(0), max_optimizing_time_us(0), valid_optimizing_timings_percent(0),
        invalid_optimization_results(0), valid_optimization_results(0), average_interception_time_s(0), timing_variance_us(0) {};
};

std::ostream &operator<<(std::ostream &os, range_stats &rng_stats);

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

struct range_test_configuration {
    optimizer_test optimizer_select;
    bool use_casadi;
    sqp_solver_configuration sqp_config;
    QPSettings qp_settings;

    range_test_configuration(): optimizer_select(time_to_intercept), use_casadi(false) {
        sqp_config = sqp_solver_configuration();
        qp_settings = QPSettings();
    }
    range_test_configuration(optimizer_test os, bool uc, sqp_solver_configuration sqp_c, QPSettings qp_s): optimizer_select(os), use_casadi(uc), sqp_config(sqp_c), qp_settings(qp_s) {};
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


    void  init_range_test(optimizer_test optimizer_select, bool use_casadi, sqp_solver_configuration sqp_config, QPSettings *qp_settings);
    range_stats exec_range_test();
    void check_range_optimality(optimizer_test optimizer_select, range_stats range_stats);

    void find_parameter(optimizer_test optimizer_select);
    std::tuple<tracking::TrackData, tracking::TrackData> case_test();
    std::tuple<tracking::TrackData, tracking::TrackData, std::vector<Plane>> application_test();

private:
    TTIOptimizerInterface tti;
    InterceptInPlanesOptimizerInterface iip;
    RapidRouteInterface rr;
    FlightArea flightarea;

    std::vector<sqp_solver_configuration> generate_sqp_configs();
    std::vector<QPSettings> generate_qp_configs();

    bool enable_stress = false;
    bool disable_time_ensurance = false;
    range_test_configuration range_test_config;
    range_stats eval_range_data(range_data data);
    void cout_header(optimizer_test optimizer_select);
    void cout_setup(optimizer_test optimizer_select, bool use_casadi, sqp_solver_configuration sqp_config);
    void cout_optmization_stats(range_data range_dat, range_stats stats);

    void cout_configuration_results(std::vector<QPSettings> qp_confs, std::vector<sqp_solver_configuration> sqp_conf, std::vector<range_stats> stats);
};
