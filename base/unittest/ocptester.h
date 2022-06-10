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

struct range_res {
    double average_opt_time;
    double max_optimizing_time;
    uint invalid_optimization_results;
};

class OcpTester {
public:
    float thrust = 25;
    float eps = 0.0001; // accepted error (rounding errors)

    /*In some configutation the worst case estimation of the tti is also providing the best possible solution.
      In this situation the worst estimation is likely to be better than the optimizer
      because the optimizers only runs tills it matches some convergence cretiria.
      `accpt_tti_error` is holding the maximum violation of the estimated tti.*/
    float accpt_tti_error = 0.1;

    bool is_tti_good(float optimizer_tti, float estimated_tt);
    bool is_optimizer_results_good(tracking::TrackData drone, tracking::TrackData insect, bool res_valid, float res_tti);

    range_res call_range_tests();

    std::vector<Plane> cube_planes(float cube_size, uint n_planes);

    std::tuple<bool, bool, bool> exec_range_test(optimizer_test optimizer_select, bool use_casadi, sqp_solver_configuration sqp_config);
    std::tuple<tracking::TrackData, tracking::TrackData> case_test();
    std::tuple<tracking::TrackData, tracking::TrackData, std::vector<Plane>> application_test();

private:
    TTIOptimizerInterface tti;
    InterceptInPlanesOptimizerInterface iip;
};
