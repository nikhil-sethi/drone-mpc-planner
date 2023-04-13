#include "ocptester.h"
#include <chrono>
#include "rapid_route.h"
#include "flightarea.h"

#include <CppUTest/TestHarness.h> //include at last!

TEST_GROUP(RapidRoute) {
    RapidRouteInterface opti;
    OcpTester ocptester;

    tracking::TrackData drone;
    tracking::TrackData insect;
    FlightArea flightarea;
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
    std::vector<Plane> _planes;
    _planes.push_back(Plane(-0.00117206, -0.149722, 0.00905782, -0.00781371, -0.998145, 0.0603854, unspecified_plane));
    _planes.push_back(Plane(-0.125613, -0.0569893, -0.0589382, -0.837417, -0.379929, -0.392921, unspecified_plane));
    _planes.push_back(Plane(0, -0.544917, -0.83849, 0, -0.544917, -0.83849, unspecified_plane));
    _planes.push_back(Plane(0.85, 0, 0, -1, 0, 0, unspecified_plane));
    _planes.push_back(Plane(0.125414, -0.0585367, -0.0578341, 0.836093, -0.390245, -0.38556, unspecified_plane));
    _planes.push_back(Plane(0, 0, -1.95, 0, 0, 1, unspecified_plane));
    _planes.push_back(Plane(0, 0, -0.895112, 0, 0, -1, unspecified_plane));
    _planes.push_back(Plane(0, -0.90269, 0, 0, 1, 0, unspecified_plane));
    _planes.push_back(Plane(-0.85, 0, 0, 1, 0, 0, unspecified_plane));
    flightarea.init(_planes);
    FlightAreaConfig *_config = flightarea.flight_area_config(bare);
    rapid_route_result result;
    RapidRouteInterface rr;
    float _thrust = 24.5f;
    rr.init(&_thrust, 1.f, _config);

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

    result = rr.find_interception(rr_drone, rr_insect, 0.0f, 1.f);
    CHECK(result.valid);
    std::cout << "Interception position: " << result.position_to_intercept << std::endl;
    std::cout << "Interception acceleration: " << result.acceleration_to_intercept << std::endl;
    std::cout << "Interception velocity: " << result.velocity_at_intercept << std::endl;
    std::cout << "Time to interception: " << result.time_to_intercept << std::endl;
    std::cout << "Stopping position: " << result.stopping_position << std::endl;
}