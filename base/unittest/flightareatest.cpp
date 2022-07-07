
#include "plane.h"
#include "flightarea.h"
#include <iostream>
#include <CppUTest/TestHarness.h> //include at last!

TEST_GROUP(Flightarea) {
    FlightArea flightarea;
};


TEST(Flightarea, app1) {
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

    cv::Point3f test_point = {0.0237142, -0.7, 99.};
    auto moved_point = flightarea.move_inside(test_point, relaxed);
    CHECK(flightarea.inside(moved_point, relaxed));

    test_point = {0.0237142, -0.7, 999.};
    moved_point = flightarea.move_inside(test_point, relaxed);
    CHECK(flightarea.inside(moved_point, relaxed));

}
