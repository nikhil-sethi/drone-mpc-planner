#include "linalg.h"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

float distance_to_plane_along_vec(cv::Mat vec, cv::Mat plane) {
    cv::Mat b = vec.col(0) - plane.col(0);
    cv::Mat n = plane.col(1) / norm(plane.col(1));
    cv::Mat e1 = orthogonal_vector(n);
    cv::Mat e2 = n.cross(e1);

    cv::Mat A(3, 3, CV_32F);
    e1.copyTo(A.col(0));
    e2.copyTo(A.col(1));
    cv::Mat tmp = (cv::Mat_<float>(3, 1) << -vec.at<float>(0, 1), -vec.at<float>(1, 1), -vec.at<float>(2, 1));
    tmp.copyTo(A.col(2));

    if (abs(cv::determinant(A)) < 0.05) {
        // the plane and the vector are (almost) parallel to each other.
        // distance is infinity
        return std::numeric_limits<float>::max();
    }

    cv::Mat params = A.inv() * b;
    float sgn_param3 = 1 - 2 * (params.at<float>(2, 0) < 0);
    return sgn_param3 * static_cast<float>(norm(params.at<float>(2, 0) * vec.col(1)));
}

cv::Mat orthogonal_vector(cv::Mat vec) {
    cv::Mat rt;
    if (vec.at<float>(2, 0) != 0) {
        rt = (cv::Mat_<float>(3, 1) << 1, 1, (-vec.at<float>(0, 0) - vec.at<float>(1, 0)) / vec.at<float>(2, 0));
    } else if (vec.at<float>(1, 0) != 0) {
        rt = (cv::Mat_<float>(3, 1) << 1, (-vec.at<float>(0, 0) - vec.at<float>(2, 0)) / vec.at<float>(1, 0), 1);
    } else if (vec.at<float>(0, 0) != 0) {
        rt = (cv::Mat_<float>(3, 1) << (-vec.at<float>(1, 0) - vec.at<float>(2, 0)) / vec.at<float>(0, 0), 1, 1);
    }
    return rt;
}

std::tuple<cv::Mat, cv::Mat> orthogonal_vectors(cv::Mat vec) {
    cv::Mat orth1 = orthogonal_vector(vec);
    cv::Mat orth2 = vec.cross(orth1);
    return std::tuple<cv::Mat, cv::Mat>(orth1, orth2);
}

std::tuple<cv::Mat, cv::Mat, cv::Mat> split_vector_to_basis_vectors(cv::Mat vec, cv::Mat b1, cv::Mat b2, cv::Mat b3) {
    cv::Mat A(3, 3, CV_32F);
    b1.copyTo(A.col(0));
    b2.copyTo(A.col(1));
    b3.copyTo(A.col(2));

    cv::Mat params(3, 1, CV_32F);
    params = A.inv() * vec;

    return std::tuple<cv::Mat, cv::Mat, cv::Mat>(params.at<float>(0, 0) * b1,
            params.at<float>(1, 0) * b2,
            params.at<float>(2, 0) * b3);
}

float projection_length_of_vec_along_dir(cv::Point3f vec, cv::Point3f dir) {
    dir = dir / norm(dir);
    return vec.dot(dir);
}

float angle_to_horizontal(cv::Point3f direction) {
    //https://onlinemschool.com/math/library/analytic_geometry/plane_line/
    float A = 0;
    float B = 1;
    float C = 0;
    float sign_direction_y = direction.y/abs(direction.y);
    if(sign_direction_y!=sign_direction_y)
        sign_direction_y = 1;

    return sign_direction_y * (fabs(A * direction.x + B * direction.y + C * direction.z) / normf({A, B, C}) / normf(direction));
}

cv::Point3f lowest_direction_to_horizontal(cv::Point3f direction, float min_angle) {
    float ath = angle_to_horizontal(direction);
    if (ath < min_angle) {
        if(normf({direction.x, direction.z})>0)
            direction.y = tan(min_angle) / normf({direction.x, direction.z});
        else
            direction = {0, 1, 0};
    }
    direction /= norm(direction);
    return direction;
}

float shortest_distance_to_line(cv::Point3f pnt, cv::Point3f line_support, cv::Point3f line_direction) {
    return norm((pnt - line_support).cross(line_direction)) / norm(line_direction);
}

cv::Point3f intersection_of_plane_and_line(cv::Point3f plane_support, cv::Point3f plane_normal, cv::Point3f line_support, cv::Point3f ln_nrm) {
    float dot = plane_normal.dot(ln_nrm);
    assert(fabs(dot) >= 0.0001f); // Line and plane are parallel and it exists no/inf intersection points.
    float d = (plane_support - line_support).dot(plane_normal) / dot;
    return line_support + d * ln_nrm;
}

cv::Point3f project_between_two_points(cv::Point3f p, cv::Point3f p1, cv::Point3f p2) {
    // https://stackoverflow.com/a/42254318/12960292
    float lsqr = (p2 - p1).dot(p2 - p1);
    float dot = (p - p1).dot(p2 - p1) / lsqr;
    float t = max(0.f, min(1.f, dot));
    return p1 + t * (p2 - p1);
}

float angle_between_points(cv::Point3f p1, cv::Point3f p2, cv::Point3f p3) {
    //https://stackoverflow.com/a/19730129/12960292
    cv::Point3f v1 = (p1 - p2) / normf(p1 - p2);
    cv::Point3f v2 = (p3 - p2) / normf(p3 - p2);
    return acosf(v1.dot(v2));
}

cv::Point3f rotate_vector_around_x_axis(cv::Point3f vector, float angle) {
    cv::Matx33f rot(1.f, 0.f, 0.f, 0.f, cosf(angle), -sinf(angle), 0.f, sinf(angle), cosf(angle));
    return rot * vector;
}

cv::Point3f rotate_vector_around_y_axis(cv::Point3f vector, float angle) {
    cv::Matx33f rot(cosf(angle), 0.f, sinf(angle), 0.f, 1.f, 0.f, -sinf(angle), 0.f, cosf(angle));
    return rot * vector;
}

cv::Point3f rotate_vector_around_z_axis(cv::Point3f vector, float angle) {
    cv::Matx33f rot(cosf(angle), -sinf(angle), 0.f, sinf(angle), cosf(angle), 0.f, 0.f, 0.f, 1.f);
    return rot * vector;
}