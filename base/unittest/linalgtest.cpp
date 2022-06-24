#include <iostream>
#include "linalg.h"
#include <CppUTest/TestHarness.h> //include at last!

TEST_GROUP(Linalg) {
};


TEST(Linalg, lowest_direction_to_horizontal) {
    float angle = 45.f / 180.f * static_cast<float>(M_PI);
    auto ret = lowest_direction_to_horizontal({0.3, -1, 0}, angle);
    CHECK(ret.y > 0);
    ret = lowest_direction_to_horizontal({0.3, 1, 0}, angle);
    CHECK(ret.y > 0);
    ret = lowest_direction_to_horizontal({1, 0, 0}, angle);
    CHECK(ret.y > 0);
    ret = lowest_direction_to_horizontal({0, -1, 0}, angle);
    CHECK(ret.y > 0);
}
