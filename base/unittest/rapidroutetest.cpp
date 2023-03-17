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

TEST(RapidRoute, stopping_distance) {
    rapid_route_result result;
    RapidRouteInterface rr;
    float _thrust = 20.f;
    rr.init(&_thrust, 0.8f);

    tracking::TrackData rr_drone;
    rr_drone.pos_valid = true;
    rr_drone.vel_valid = true;
    rr_drone.state.pos = {0, 0, 0};
    rr_drone.state.vel = {0, 0, 0};
    rr_drone.state.acc = {0, 0, 0};

    tracking::TrackData rr_insect;
    rr_insect.pos_valid = true;
    rr_insect.vel_valid = true;
    rr_insect.state.pos = {1.32f, 1.32f, 2.3f};
    rr_insect.state.vel = {0.1f, 0.2f, 0.3f};
    rr_insect.state.acc = {0.f, 0.f, 0.f};

    result = rr.find_best_interception(rr_drone, rr_insect);
    CHECK(result.valid);
    std::cout << "Stopping position: " << result.stopping_position << std::endl;

}