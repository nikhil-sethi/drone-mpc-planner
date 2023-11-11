#pragma once
#include "common.h"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

struct Quaternion {
    float s;
    cv::Point3f v;

    Quaternion() {
        s = 0;
        v = cv::Point3f(0, 0, 0);
    }

    Quaternion(float theta, cv::Point3f rot_axis) { // theta is the transformation angle; not the rotation angle!
        s = cosf(-theta / 2);
        v = rot_axis * sinf(-theta / 2);
    }

    Quaternion(float qs, float qx, float qy, float qz) {
        s = qs;
        v = cv::Point3f(qx, qy, qz);
    }

    Quaternion operator *(Quaternion q2) {
        Quaternion rt;
        rt.s = this->s * q2.s - this->v.dot(q2.v);
        rt.v = this->s * q2.v + q2.s * this->v + this->v.cross(q2.v);
        return rt;
    }

    Quaternion operator -(Quaternion q) {
        Quaternion rt;
        rt.s = this->s - q.s;
        rt.v = this->v - q.v;
        return rt;
    }

    Quaternion operator /(float scale) {
        this->s /= scale;
        this->v /= scale;

        return *this;
    }

    Quaternion operator /=(float scale) {
        this->s /= scale;
        this->v /= scale;

        return *this;
    }

    bool operator ==(Quaternion q2) {
        if (this->s == q2.s && this->v == q2.v)
            return true;
        return false;
    }

};

std::ostream &operator <<(std::ostream &os, const Quaternion &q);

float normq(Quaternion q);
std::tuple<float, cv::Point3f> qcharacteristics(Quaternion q);
Quaternion conjq(Quaternion q);
Quaternion invq(Quaternion q);
Quaternion rot_quat(cv::Point3f vec0, cv::Point3f vec1);
cv::Point3f rotate(cv::Point3f vec0, Quaternion rot_quat);
cv::Point3f restore_direction(float qx, float qy);