#pragma once
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "common.h"
#include "linalg.h"

enum plane_types {
    unspecified_plane,
    bottom_plane,
    top_plane,
    back_plane,
    left_plane,
    lower_plane,
    right_plane,
    camera_protector_plane,
};

inline const char *plane_types_str[] = {
    "unspecified_plane",
    "bottom_plane",
    "top_plane",
    "back_plane",
    "left_plane",
    "lower_plane",
    "right_plane",
    "camera_protector_plane",
    "" // must be the last entry! (check in serializer)
};
struct Plane {
    cv::Point3f support;
    cv::Point3f normal;
    plane_types type; //Flag to mark specific planes which need to be addressed individual during program execution, e.g. bottom_plane
    uint id;
    bool is_active;
    cv::Point3f cam_corner_1 = {0}; // here for debugging
    cv::Point3f cam_corner_2 = {0};

    Plane(float support_x, float support_y, float support_z, float normal_x, float normal_y, float normal_z, plane_types type_) :
        support(cv::Point3f(support_x, support_y, support_z)),
        normal(cv::Point3f(normal_x, normal_y, normal_z)),
        type(type_),
        id(0),
        is_active(true) {
        normal /= normf(normal);
    }
    Plane(cv::Point3f support_, cv::Point3f normal_, plane_types type_, uint id_) :
        support(support_),
        normal(normal_),
        type(type_),
        id(id_),
        is_active(true) {
        normal = normal_ / normf(normal_);
    }
    Plane(int direction, cv::Point3f cam_corner_1_, cv::Point3f cam_corner_2_, plane_types type_, uint id_) :
        support(cv::Point3f(0, 0, 0)),
        normal(direction * create_plane_normal_vector(cam_corner_1_, cam_corner_2_)),
        type(type_),
        id(id_),
        is_active(true),
        cam_corner_1(cam_corner_1_),
        cam_corner_2(cam_corner_2_) { }

    Plane(int direction, cv::Point3f point1, cv::Point3f point2, cv::Point3f point3, plane_types type_) :
        support(point1),
        normal(direction * create_plane_normal_vector(point2 - point1, point3 - point1)),
        type(type_),
        id(0),
        is_active(true) {
        if (normf(normal) == 0)
            throw std::runtime_error("Plane configuration invalid: Normal vector has no direction");
    }

    Plane(float distance, float roll_deg, float pitch_deg, plane_types type_) :
        support(cv::Point3f(0, 0, -distance)),
        normal(cv::Point3f(0, 1, 0)),
        type(type_),
        id(0),
        is_active(true) {
        normal = rotate_vector_around_z_axis(normal, roll_deg * deg2rad);
        normal = rotate_vector_around_x_axis(normal, pitch_deg * deg2rad);
    }

    cv::Point3f create_plane_normal_vector(cv::Point3f cam_corner_1_, cv::Point3f cam_corner_2_);
    float distance(cv::Point3f pnt);
    bool on_normal_side(cv::Point3f p) ;
    bool on_normal_side(cv::Point3f p, float eps);
    std::tuple<float, cv::Point3f> hesse_normal_form();
};

std::ostream &operator<<(std::ostream &os, Plane &plane);
cv::Point3f intersection_of_3_planes(Plane p1, Plane p2, Plane p3);
