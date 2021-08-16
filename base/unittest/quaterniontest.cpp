#include <iostream>
#include "common.h"
#include "quaternion.h"
#include <CppUTest/TestHarness.h> //include at last!

TEST_GROUP(Quaternions) {
    float eps = 0.0001; // accepted error (rounding errors)
};

TEST(Quaternions, mult) {
    Quaternion q1(1,2,3,4);
    Quaternion q2(-5,6,-7,8);
    Quaternion q3(-28, 48, -14, -44);
    CHECK(q1*q2==q3);
}

TEST(Quaternions, calc_rot_quat1) {
    cv::Point3f bf_hover = cv::Point3f(0, 0, -1);
    cv::Point3f acc = cv::Point3f(1, 1, 1);
    Quaternion qrot = rot_quat(bf_hover, acc);
    bool check = norm(rotate(bf_hover, qrot) - acc/norm(acc))<eps;
    CHECK(check);
    CHECK(abs(normq(qrot)-1)<eps);
}

TEST(Quaternions, calc_rot_quat2) {
    cv::Point3f bf_hover = cv::Point3f(0, 0, -1);
    cv::Point3f acc = cv::Point3f(0, 0, 1);
    Quaternion qrot = rot_quat(bf_hover, acc);
    bool check = norm(rotate(bf_hover, qrot) - acc/norm(acc))<eps;
    CHECK(check);
    CHECK(abs(normq(qrot)-1)<eps);
}

TEST(Quaternions, calc_rot_quat3) {
    cv::Point3f bf_hover = cv::Point3f(0, 0, -1);
    cv::Point3f acc = cv::Point3f(0, 0, -1);
    Quaternion qrot = rot_quat(bf_hover, acc);
    bool check = norm(rotate(bf_hover, qrot) - acc/norm(acc))<eps;
    CHECK(check);
    CHECK(abs(normq(qrot)-1)<eps);
}

TEST(Quaternions, restore_quat) {
    cv::Point3f bf_hover = cv::Point3f(0, 0, -1);
    cv::Point3f acc = cv::Point3f(1, 1, 1);
    Quaternion qrot = rot_quat(bf_hover, acc);
    float qs = sqrtf(1-powf(qrot.v.x, 2) - powf(qrot.v.y, 2));
    Quaternion qrot2(qs, qrot.v.x, qrot.v.y, 0);
    CHECK(abs(normq(qrot2)-1)<eps);
    CHECK(normq(qrot-qrot2)<eps);
}

TEST(Quaternions, restore_direction) {
    cv::Point3f bf_hover = cv::Point3f(0, 0, -1);
    cv::Point3f acc = cv::Point3f(-0.58, 0.762, 0.288);
    cv::Point3f acc_bf = pats_to_betaflight_coord(acc);
    Quaternion qrot = rot_quat(bf_hover, acc_bf);
    cv::Point3f res_dir = restore_direction(qrot.v.x, qrot.v.y);
    res_dir = betaflight_to_pats_coord(res_dir);
    CHECK(norm(acc/norm(acc)-res_dir/norm(res_dir))<eps);
}
