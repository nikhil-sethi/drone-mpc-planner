#include "ocptester.h"
#include <chrono>
#include "tti_optimizer_interface.h"

#include <CppUTest/TestHarness.h> //include at last!

TEST_GROUP(TimeToIntercept) {
    TTIOptimizerInterface opti;
    OcpTester ocptester;

    tracking::TrackData drone;
    tracking::TrackData insect;
};

// TEST(TimeToIntercept, application_test) {
//     opti.init(&(ocptester.thrust));
//     std::tie(drone, insect, std::ignore) = ocptester.application_test();
//     std::chrono::_V2::system_clock::time_point tic = std::chrono::high_resolution_clock::now();

//     auto opti_res = opti.find_best_interception(drone, insect);

//     std::chrono::_V2::system_clock::time_point toc = std::chrono::high_resolution_clock::now();
//     double cpu_time = std::chrono::duration_cast<std::chrono::seconds>(toc - tic).count();
//     CHECK(opti_res.valid);
//     CHECK(cpu_time < 1. / 90);
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
    QPSettings qpsettings = QPSettings();
    ocptester.init_range_test(time_to_intercept, true, sqp_solver_configuration(), &qpsettings);
    auto stats = ocptester.exec_range_test();
    CHECK(stats.average_optimizing_time_us < ocptester.realtime_boundary_ms * 1000);
    CHECK(stats.max_optimizing_time_us < ocptester.realtime_boundary_ms * 1000);
    CHECK(stats.invalid_optimization_results == 0);
}

// TEST(TimeToIntercept, overall_behavior_casadi) {
//     auto stats = ocptester.exec_range_test(time_to_intercept, true, sqp_solver_configuration());
//     CHECK(stats.average_optimizing_time_us < ocptester.realtime_boundary_ms);
//     CHECK(stats.max_optimizing_time_us < ocptester.realtime_boundary_ms);
//     CHECK(stats.invalid_optimization_results == 0);
// }

TEST(TimeToIntercept, find_parameters) {
    ocptester.find_parameter(time_to_intercept);
}
