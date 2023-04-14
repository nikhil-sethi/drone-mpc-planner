#include <chrono>
#include "rapid_route.h"
#include "flightarea.h"

#include <CppUTest/TestHarness.h> //include at last!


std::vector<Plane> return_plane_box(float size) {
    std::vector<Plane> _planes;
    _planes.clear();
    _planes.push_back(Plane(-size, 0, 0, 1, 0, 0, unspecified_plane));
    _planes.push_back(Plane(size, 0, 0, -1, 0, 0, unspecified_plane));
    _planes.push_back(Plane(0, -size, 0, 0, 1, 0, unspecified_plane));
    _planes.push_back(Plane(0, size, 0, 0, -1, 0, unspecified_plane));
    _planes.push_back(Plane(0, 0, -size, 0, 0, 1, unspecified_plane));
    _planes.push_back(Plane(0, 0, size, 0, 0, -1, unspecified_plane));
    return _planes;
}

TEST_GROUP(RapidRoute) {
    RapidRouteInterface opti;

    std::vector<Plane> _planes;
    FlightArea flightarea;
    FlightAreaConfig *_config;
    tracking::TrackData drone;
    tracking::TrackData insect;

    rapid_route_result result;
    RapidRouteInterface rr;

    TEST_SETUP() {
        _planes.clear();
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
        _config = flightarea.flight_area_config(bare);

        drone.pos_valid = true;
        drone.vel_valid = true;
        drone.state.pos = {0, 0, 0};
        drone.state.vel = {0, 0, 0};
        drone.state.acc = {0, 0, 0};

        insect.pos_valid = true;
        insect.vel_valid = true;
        insect.state.pos = {-0.470907f, 0.831266f, -0.47854f};
        insect.state.vel = {0.0104176f, -0.32662f, -0.36589f};
        insect.state.acc = {0.f, 0.f, 0.f};
    }
};

TEST(RapidRoute, takesDirectRouteIfPossible) {
    flightarea.init(return_plane_box(1e2f));
    _config = flightarea.flight_area_config(bare);

    float _thrust = 20.5f;
    rr.init(&_thrust, 0.8f, _config);

    insect.state.pos = {-1e-3f, 1e-3f, -1e-3f}; // 1mm away from drone in all directions
    result = rr.find_interception(drone, insect, 0.0f, 1.f);
    CHECK(result.valid);
    CHECK(!result.via);

    insect.state.pos = {1e-2f, 1e-2f, 1e-2f}; // 1cm away from drone in all directions
    result = rr.find_interception(drone, insect, 0.0f, 1.f);
    CHECK(result.valid);
    CHECK(!result.via);

    insect.state.pos = {-1e-1f, 1e-1f, 1e-1f}; // 1dm away from drone in all directions
    result = rr.find_interception(drone, insect, 0.0f, 1.f);
    CHECK(result.valid);
    CHECK(!result.via);

    insect.state.pos = {-1e0f, -1e0f, -1e0f}; // 1m away from drone in all directions
    result = rr.find_interception(drone, insect, 0.0f, 1.f);
    CHECK(result.valid);
    CHECK(!result.via);

    insect.state.pos = {1e1f, 1e1f, -1e1f}; // 10m away from drone in all directions
    result = rr.find_interception(drone, insect, 0.0f, 1.f);
    CHECK(result.valid);
    CHECK(!result.via);
}

TEST(RapidRoute, triesToFindViaRouteIfUnreachable) {
    flightarea.init(return_plane_box(1e2f)); // 2*100m width, height, depth box
    _config = flightarea.flight_area_config(bare);

    float _thrust = 10.5f;
    rr.init(&_thrust, 1.f, _config);

    insect.state.pos = {1e2f, 1e2f, 1e2f}; // 100m away from drone in all directions, on corner of box
    result = rr.find_interception(drone, insect, 0.0f, 1.f);
    CHECK(result.valid);
    CHECK(result.via);
}

TEST(RapidRoute, returnInvalidWhenThrustTooLowToReachTargetOverGravity) {
    flightarea.init(return_plane_box(1e2f));
    _config = flightarea.flight_area_config(bare);

    float _thrust = 0.5f;
    rr.init(&_thrust, 1.f, _config);

    insect.state.pos = {1e-1f, 1e-1f, 1e-1f}; // 1dm away from drone in all directions
    result = rr.find_interception(drone, insect, 0.0f, 1.f);
    CHECK(!result.valid);
}

TEST (RapidRoute, ensureStoppingDistanceIsEqualToHuntDistance) {
    flightarea.init(return_plane_box(1e2f));
    _config = flightarea.flight_area_config(bare);

    float _thrust = 24.5f;
    rr.init(&_thrust, 1.f, _config);

    insect.state.pos = {1e1f, 0, 0}; // 10m away from drone
    insect.state.vel = {0, 0, 0};
    result = rr.find_interception(drone, insect, 0.0f, 1.f);
    CHECK(!result.via);
    std::cout << result.stopping_position << std::endl;
    std::cout << result.position_to_intercept << std::endl;
    CHECK(abs(norm(result.position_to_intercept) - norm(result.stopping_position - result.position_to_intercept)) < 1e-3); // 1mm error margin
}

TEST (RapidRoute, ensureStoppingDistanceIsLongerThanHuntIfSafetyFactorIsUsed) {
    flightarea.init(return_plane_box(1e2f));
    _config = flightarea.flight_area_config(bare);

    float _thrust = 24.5f;
    rr.init(&_thrust, 1.f, _config);

    insect.state.pos = {1e1f, 0, 0}; // 10m away from drone
    insect.state.vel = {0, 0, 0};
    result = rr.find_interception(drone, insect, 0.0f, 2.f);
    CHECK(!result.via);
    std::cout << result.stopping_position << std::endl;
    std::cout << result.position_to_intercept << std::endl;
    CHECK(abs(norm(result.position_to_intercept) / norm(result.stopping_position - result.position_to_intercept)) < 0.5); // stopping distance should be more than 2x hunt distance
}

TEST (RapidRoute, ensureIntermediatePointIsACorner) {
    float _thrust = 24.5f;
    rr.init(&_thrust, 1.f, _config);

    result = rr.find_interception_via(drone, insect, 0.0f, 1.f);

    CHECK(result.via);
    std::cout << result.intermediate_position << std::endl;
    // CHECK(result.intermediate_position.x == )
}

TEST(RapidRoute, stopping_distance) {
    float _thrust = 24.5f;
    rr.init(&_thrust, 1.f, _config);

    result = rr.find_interception(drone, insect, 0.0f, 1.f);
    CHECK(result.valid);
}
