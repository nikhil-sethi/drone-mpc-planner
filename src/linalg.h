#pragma once

#include "defines.h"
#include "common.h"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"


float angle_to_horizontal(cv::Point3f direction);
cv::Point3f lowest_direction_to_horizontal(cv::Point3f direction, float min_angle);

/** @brief Calculates an orthogonal vector to a given vector. */
cv::Mat get_orthogonal_vector(cv::Mat vec);

/** @brief Calculates the two orthogonal vectors to a given vector.*/
std::tuple<cv::Mat, cv::Mat> get_orthogonal_vectors(cv::Mat vec);

/** @brief Splits a vector in 3 vectors which are pointing along given basis vectors. */
std::tuple<cv::Mat, cv::Mat, cv::Mat> split_vector_to_basis_vectors(cv::Mat vec, cv::Mat b1, cv::Mat b2, cv::Mat b3);

/** @brief  Generates the plane normal vector based on 3 points. The third point is always asumed to be (0,0,0)^T. */
cv::Mat get_plane_normal_vector(cv::Point3f x1, cv::Point3f x2);

/** @brief Determines on which side of the plane a point is.
* Returns true if the point is on the point the side the normal vector is looking to, else false is returned. */
bool on_normal_side(cv::Mat p0, cv::Mat n, cv::Mat p);

float distance_to_plane(cv::Mat p0, cv::Mat n, cv::Mat p);

cv::Mat intersection_of_3_planes(cv::Mat p0_1, cv::Mat n_1, cv::Mat p0_2, cv::Mat n_2, cv::Mat p0_3, cv::Mat n_3);

std::tuple<float, cv::Mat> hesse_normal_form(cv::Mat p0, cv::Mat n);

/** @brief Calculates the distance along vec till the plane is intercepted.*/
float calc_distance_to_plane(cv::Mat vec, cv::Mat plane);