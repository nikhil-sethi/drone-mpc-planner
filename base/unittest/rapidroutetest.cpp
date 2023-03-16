#include "ocptester.h"
#include <chrono>
#include "rapid_route.h"

#include <CppUTest/TestHarness.h> //include at last!

TEST_GROUP(RapidRoute) {
    RapidRouteInterface opti;
    OcpTester ocptester;

    tracking::TrackData drone;
    tracking::TrackData insect;
};

TEST(RapidRoute, overall_behavior) {
    QPSettings qpsettings = QPSettings();
    ocptester.init_range_test(rapid_route, false, sqp_solver_configuration(), &qpsettings);
    auto stats = ocptester.exec_range_test();
    ocptester.check_range_optimality(time_to_intercept, stats);
    CHECK(stats.average_optimizing_time_us < ocptester.realtime_boundary_ms * 1000);
    CHECK(stats.max_optimizing_time_us < ocptester.realtime_boundary_ms * 1000);
    CHECK(stats.invalid_optimization_results == 0);
}