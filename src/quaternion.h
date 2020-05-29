#pragma once
#include "common.h"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

struct quaternion {
    float s;
    cv::Point3f v;

    quaternion() {
        s = 0;
        v = cv::Point3f(0,0,0);
    }

    quaternion(float theta, cv::Point3f rot_axis) { // theta is the transformation angle; not the rotation angle!
        s = cos(-theta/2);
        v = rot_axis*sin(-theta/2);
    }

    quaternion(float qs, float qx, float qy, float qz) {
        s = qs;
        v = cv::Point3f(qx, qy, qz);
    }

    quaternion operator *(quaternion q2) {
        quaternion rt;
        rt.s = this->s*q2.s - this->v.dot(q2.v);
        rt.v = this->s*q2.v + q2.s*this->v + this->v.cross(q2.v);
        return rt;
    }

    quaternion operator -(quaternion q) {
        quaternion rt;
        rt.s = this->s - q.s;
        rt.v = this->v - q.v;
        return rt;
    }

    quaternion operator /(float scale) {
        this->s /= scale;
        this->v /= scale;

        return *this;
    }

    quaternion operator /=(float scale) {
        this->s /= scale;
        this->v /= scale;

        return *this;
    }

    bool operator ==(quaternion q2) {
        if(this->s == q2.s && this->v==q2.v)
            return true;
        return false;
    }

};

std::ostream& operator <<(std::ostream& os, const quaternion& q);

float normq(quaternion q);
std::tuple<float, cv::Point3f> qcharacteristics(quaternion q);
quaternion conjq(quaternion q);
quaternion invq(quaternion q);
quaternion rot_quat(cv::Point3f vec0, cv::Point3f vec1);
cv::Point3f rotate(cv::Point3f vec0, quaternion rot_quat);
cv::Point3f restore_direction(float qx, float qy);