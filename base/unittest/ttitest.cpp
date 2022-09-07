#include "ocptester.h"
#include <chrono>
#include "tti_optimizer_interface.h"

#include <CppUTest/TestHarness.h> //include at last!

TEST_GROUP(TimeToIntercept) {
    TTIOptimizerInterface opti;
    OcpTester ocptester;
};

// TEST(TimeToIntercept, debug) {
//     opti.init(&(ocptester.thrust));
//     tracking::TrackData drone;
//     drone.state.pos = {1, 1, 1};
//     drone.state.vel = {1, 1, 1};
//     tracking::TrackData insect;
//     insect.state.pos = {0, 0, 0};
//     insect.state.vel = {0, 0, 0};
//     auto opti_res = opti.find_best_interception(drone, insect);

//     CHECK(opti_res.valid);
// }

// TEST(TimeToIntercept, debug_casadi) {
//     opti.init(&(ocptester.thrust));
//     opti.init_casadi("../../ocp_design/casadi/tti/tti_optimizer.so");
//     tracking::TrackData drone;
//     drone.state.pos = {1, 1, 1};
//     drone.state.vel = {1, 1, 1};
//     tracking::TrackData insect;
//     insect.state.pos = {0, 0, 0};
//     insect.state.vel = {0, 0, 0};
//     auto opti_res = opti.find_best_interception(drone, insect);

//     CHECK(opti_res.valid);
// }

TEST(TimeToIntercept, overall_behavior) {
    bool average_timing_ok, max_timing_ok, invalid_results_ok;
    std::tie(average_timing_ok, max_timing_ok, invalid_results_ok) = ocptester.exec_range_test(time_to_intercept, false, sqp_solver_configuration());
    CHECK(average_timing_ok);
    CHECK(max_timing_ok);
    CHECK(invalid_results_ok);

}

// TEST(TimeToIntercept, overall_behavior_casadi) {
//     bool average_timing_ok, max_timing_ok, invalid_results_ok;
//     std::tie(average_timing_ok, max_timing_ok, invalid_results_ok) = ocptester.exec_range_test(time_to_intercept, true, sqp_solver_configuration());
//     CHECK(average_timing_ok);
//     CHECK(max_timing_ok);
//     CHECK(invalid_results_ok);
// }
