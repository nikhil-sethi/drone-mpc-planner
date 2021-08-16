#pragma once
#include "common.h"

float angle_to_horizontal(cv::Point3f direction);
cv::Point3f lowest_direction_to_horizontal(cv::Point3f direction, float min_angle);

/** @brief Calculates an orthogonal vector to a given vector. */
cv::Mat orthogonal_vector(cv::Mat vec);

/** @brief Calculates the two orthogonal vectors to a given vector.*/
std::tuple<cv::Mat, cv::Mat> orthogonal_vectors(cv::Mat vec);

/** @brief Splits a vector in 3 vectors which are pointing along given basis vectors. */
std::tuple<cv::Mat, cv::Mat, cv::Mat> split_vector_to_basis_vectors(cv::Mat vec, cv::Mat b1, cv::Mat b2, cv::Mat b3);

/** @brief  Calculates the length of the projection of a vector vec on the direction of dir.*/
float projection_length_of_vec_along_dir(cv::Point3f vec, cv::Point3f dir);

std::tuple<float, cv::Point3f> hesse_normal_form(cv::Point3f p0, cv::Point3f n);

/** @brief Calculates the distance along vec till the plane is intercepted.*/
float distance_to_plane_along_vec(cv::Mat vec, cv::Mat plane);

float shortest_distance_to_line(cv::Point3f pnt, cv::Point3f ln_spprt, cv::Point3f ln_nrm);

cv::Point3f intersection_of_plane_and_line(cv::Point3f plane_support, cv::Point3f plane_normal, cv::Point3f ln_spprt, cv::Point3f ln_nrm);

/** @brief Calculates the point p* which is at closest to p and on aline between p1 and p2. */
cv::Point3f project_between_two_points(cv::Point3f p, cv::Point3f p1, cv::Point3f p2);

/** @brief Calculates the angle between the 3 given points where point p2 is considered as middle point. */
float angle_between_points(cv::Point3f p1, cv::Point3f p2, cv::Point3f p3);

cv::Point3f rotate_vector_around_x_axis(cv::Point3f vector, float angle);

cv::Point3f rotate_vector_around_y_axis(cv::Point3f vector, float angle);

cv::Point3f rotate_vector_around_z_axis(cv::Point3f vector, float angle);