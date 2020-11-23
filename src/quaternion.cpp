#include "quaternion.h"

std::ostream& operator <<(std::ostream& os, const Quaternion& q) {
    os << "[" << q.s << ", " << q.v << "]";
    return os;
}

float normq(Quaternion q) {
    return sqrt(powf(q.s, 2) + powf(q.v.x, 2) + powf(q.v.y, 2) + powf(q.v.z, 2));
}

std::tuple<float, cv::Point3f> qcharacteristics(Quaternion q) {
    float theta;
    cv::Point3f axis;
    q /= normq(q);

    theta = -2*acos(q.s);
    axis = q.v/sin(-theta/2);

    return std::make_tuple(theta, axis);
}

Quaternion conjq(Quaternion q) {
    q.v *= -1;
    return q;
};

Quaternion invq(Quaternion q) {
    return conjq(q)/normq(q);
}

Quaternion rot_quat(cv::Point3f vec0, cv::Point3f vec1) {
    // https://github.com/toji/gl-matrix/blob/f0583ef53e94bc7e78b78c8a24f09ed5e2f7a20c/src/gl-matrix/quat.js#L54
    Quaternion rt;
    vec0 /= norm(vec0);
    vec1 /= norm(vec1);
    double dot = vec0.dot(vec1);
    if(dot<-0.999999)
        return Quaternion(0,0,-1,0);
    rt.s = 1 + dot;
    rt.v = vec0.cross(vec1);
    rt /= normq(rt);
    return rt;
}

cv::Point3f rotate(cv::Point3f vec0, Quaternion rot_quat) {
    Quaternion vec0q;
    vec0q.s = 0;
    vec0q.v = vec0;
    Quaternion qrt = rot_quat*vec0q*invq(rot_quat);
    return qrt.v;
}

cv::Point3f restore_direction(float qx, float qy) {
    float qs = sqrt(1-powf(qx, 2) - powf(qy, 2));
    Quaternion qrot(qs, qx, qy, 0);
    cv::Point3f bf_hover = {0,0,-1};
    return rotate(bf_hover, qrot);
}