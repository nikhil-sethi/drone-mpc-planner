
#include <iostream>
#include "plane.h"
#include <CppUTest/TestHarness.h> //include at last!

TEST_GROUP(Plane) {
};


TEST(Plane, intersection) {
    Plane plane1 = Plane(0, 0, 0, 1, 0, 0, unspecified_plane);
    Plane plane2 = Plane(0, 0, 0, 0, 1, 0, unspecified_plane);
    Plane plane3 = Plane(0, 0, 0.1, 0, 0, 1, unspecified_plane);

    auto interception = intersection_of_3_planes(&plane1, &plane2, &plane3);
    CHECK(norm(interception - cv::Point3f(0, 0, 0.1)) < 0.001);
}

TEST(Plane, p2) {
    Plane plane1 = Plane(0, 0, 0, 0.0137185, 0.447843, -0.894007, unspecified_plane);
    Plane plane2 = Plane(0, 0, 0, -0.0136497, -0.996675, 0.080329, unspecified_plane);
    Plane plane3 = Plane(0, -0.478452, -0.702555, 0, -0.562884, -0.826536, unspecified_plane);

    auto interception = intersection_of_3_planes(&plane1, &plane2, &plane3);
    CHECK(std::isnan(interception.x));
}


TEST(Plane, parallel_planes) {
    Plane plane1 = Plane(0, 0,   0, 1, 0,  0, unspecified_plane);
    Plane plane2 = Plane(0, 0,   0, 0, 0, -1, unspecified_plane);
    Plane plane3 = Plane(0, 0, 0.1, 0, 0,  1, unspecified_plane);

    auto interception = intersection_of_3_planes(&plane1, &plane2, &plane3);
    CHECK(std::isnan(interception.x));
}
