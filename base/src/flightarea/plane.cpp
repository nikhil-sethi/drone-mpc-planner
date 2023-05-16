#include "plane.h"
#include "common.h"
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <cmath>

// Generates the plane normal vector based on 3 points. The third point is always asumed to be (0,0,0).
cv::Point3f Plane::create_plane_normal_vector(cv::Point3f cam_corner_1_, cv::Point3f cam_corner_2_) {
    cv::Point3f n = cam_corner_1_.cross(cam_corner_2_);
    return n / normf(n);
}
float Plane::distance(cv::Point3f pnt) {
    // Return negavtive distanes if pnt is on the opposite side in which plane_normal points.
    cv::Point3f plane_support_2_pnt = pnt - support;
    normal = normal / normf(normal);
    return plane_support_2_pnt.dot(normal);
}
bool Plane::on_normal_side(cv::Point3f p) {
    // Returns true if the point is on the point the side the normal vector is looking to, else false is returned.
    auto v = p - support;
    return v.dot(normal) > 0;
}
bool Plane::on_normal_side(cv::Point3f p, float eps) {
    auto tmp_support = support - eps * normal;
    auto v = p - tmp_support;
    return v.dot(normal) > 0;
}
bool Plane::on_plane(cv::Point3f p) {
    // Returns true if the point is on the plane.
    auto v = p - support;
    return v.dot(normal) == 0;
}
std::tuple<float, cv::Point3f> Plane::hesse_normal_form() {
    cv::Point3f n0 = normal * normf(normal);
    double d = support.dot(n0);
    return std::make_tuple(d, n0);
}
cv::Point3f intersection_of_3_planes(Plane *p1, Plane *p2, Plane *p3) {
    auto [d1, n01] = p1->hesse_normal_form();
    auto [d2, n02] = p2->hesse_normal_form();
    auto [d3, n03] = p3->hesse_normal_form();

    cv::Matx31f b(d1, d2, d3);
    cv::Matx33f A(n01.x, n02.x, n03.x, n01.y, n02.y, n03.y, n01.z, n02.z, n03.z);

    if (abs(cv::determinant(A)) < 1e-2) {
        return cv::Point3f(std::nan("0"), std::nan("0"), std::nan("0"));
    }

    cv::Mat tmp = A.t().inv() * cv::Mat(b);
    return  cv::Point3f(tmp);
}

std::ostream &operator<<(std::ostream &os, Plane &plane) {
    os << "Plane: type: " << plane_types_str[plane.type] << ", plane_id: " << plane.id << ", support: " << plane.support  << ", normal: " << plane.normal << ", is_active: " << plane.is_active << std::endl;
    return os;
}
