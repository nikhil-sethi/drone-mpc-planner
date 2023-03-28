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
    float _thrust = 24.5f;
    rr.init(&_thrust, 1.f);

    tracking::TrackData rr_drone;
    rr_drone.pos_valid = true;
    rr_drone.vel_valid = true;
    rr_drone.state.pos = {0, 0, 0};
    rr_drone.state.vel = {0, 0, 0};
    rr_drone.state.acc = {0, 0, 0};

    tracking::TrackData rr_insect;
    rr_insect.pos_valid = true;
    rr_insect.vel_valid = true;
    rr_insect.state.pos = {-0.470907f, 0.831266f, -0.47854f};
    rr_insect.state.vel = {0.0104176f, -0.32662f, -0.36589f};
    rr_insect.state.acc = {0.f, 0.f, 0.f};

    result = rr.find_best_interception(rr_drone, rr_insect, 0.0f, 1.f);
    CHECK(result.valid);
    std::cout << "Interception position: " << result.position_to_intercept << std::endl;
    std::cout << "Interception acceleration: " << result.acceleration_to_intercept << std::endl;
    std::cout << "Interception velocity: " << result.velocity_at_intercept << std::endl;
    std::cout << "Time to interception: " << result.time_to_intercept << std::endl;
    std::cout << "Stopping position: " << result.stopping_position << std::endl;
}