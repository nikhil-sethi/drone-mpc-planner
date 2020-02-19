#include <iostream>
#include "common.h"
#include <CppUTest/TestHarness.h> //include at last!

TEST_GROUP(Common) {
};

TEST(Common, pats2beta){
  cv::Point3f patsvec(1,2,3);
  cv::Point3f betavec = pats_to_betaflight_coord(patsvec);
  CHECK( betavec==cv::Point3f(-3,-1,-2) );
}

TEST(Common, beta2pats){
  cv::Point3f bfvec(1,2,3);
  cv::Point3f patsvec = betaflight_to_pats_coord(bfvec);
  CHECK( patsvec==cv::Point3f(-2,-3,-1) );
}
