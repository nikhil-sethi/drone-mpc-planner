#include <iostream>
#include "cameraview.h"
#include <CppUTest/TestHarness.h> //include at last!

TEST_GROUP(Cameraview) {
  cv::Point3f point_left_top, point_right_top, point_left_bottom, point_right_bottom;
  float b_depth, b_height, camera_pitch_deg;

  void setup() {
    // Preset test variables - variables are reseted after every test
    point_left_top = {21.1859, -2.14396, -24.3737};
    point_right_top = {-21.1278, -2.14396, -24.3737};
    point_left_bottom = {21.1859, -21.6774, -10.5512};
    point_right_bottom = {-21.1278, -21.6774, -10.5512};
    b_depth = -3.53615;
    b_height = -1.72678;
    camera_pitch_deg = 35.2846;
    // CameraView camview; // For some reaoson that must be done in the tests itself
    // camview.init(point_left_top, point_right_top, point_left_bottom, point_left_bottom, point_right_bottom, b_depth, b_height, camera_pitch_deg);
  }
};

TEST(Cameraview, inview) {
  CameraView camview;
  camview.init(point_left_top, point_right_top, point_left_bottom, point_right_bottom, b_depth, b_height, camera_pitch_deg);
  bool inview;
  std::array<bool, N_PLANES> plane_violations;
  std::tie(inview, plane_violations) = camview.in_view({0,-1.,-2}, CameraView::relaxed);
  CHECK_TRUE(inview);
}

TEST(Cameraview, notinview) {
  CameraView camview;
  camview.init(point_left_top, point_right_top, point_left_bottom, point_right_bottom, b_depth, b_height, camera_pitch_deg);
  bool inview;
  std::array<bool, N_PLANES> plane_violations;
  std::tie(inview, plane_violations) = camview.in_view({0,-1.73,-2}, CameraView::relaxed);
  CHECK_FALSE(inview);
}

TEST(Cameraview, project_in_view) {
  CameraView camview;
  camview.init(point_left_top, point_right_top, point_left_bottom, point_right_bottom, b_depth, b_height, camera_pitch_deg);
  cv::Point3f original = {-4,-1.73,-2};
  cv::Point3f corrected;
  bool inview;
  std::array<bool, N_PLANES> plane_violations;
  for (float x=-10.0f; x<10.f; x+=0.2f) {
    for (float y=-4.0f; y<=0.f; y+=0.2f) {
      for (float z=-13.0f; z<=0.f; z+=0.2f) {
        std::tie(inview, plane_violations) = camview.in_view(original, CameraView::relaxed);
        corrected = camview.project_into_camera_volume(original, CameraView::relaxed, plane_violations);
        std::tie(inview, plane_violations) = camview.in_view(corrected, CameraView::relaxed);
        CHECK_TRUE(inview);
      }
    }
  }
}
