#pragma once

#include "defines.h"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "common.h"

#define N_PLANES 6 //adapt this to the number of planes in plane index!
#define N_CORNER_POINTS 8 //adapt this to the number of corner points in point_index

static const char* hunt_volume_check_names[] = {
    "HV_Unknown",
    "HV_OK",
    "HV_To_High",
    "HV_To_Low",
    "HV_To_Close",
    "HV_Outside_Cone"
};

class CameraView {
public:
    enum plane_index{
        top_plane,
        front_plane,
        left_plane,
        right_plane,
        bottom_plane,
        back_plane
    };

    enum point_index{
        top_right_front,
        top_right_back,
        top_left_front,
        top_left_back,
        bottom_right_front,
        bottom_right_back,
        bottom_left_front,
        bottom_left_back,
    };

    void init(cv::Point3f point_left_top, cv::Point3f point_right_top, cv::Point3f point_left_bottom, cv::Point3f point_right_bottom,
              float depth, float height);

    enum hunt_check_result {
        HuntVolume_Unknown,
        HuntVolume_OK,
        HuntVolume_To_High,
        HuntVolume_To_Low,
        HuntVolume_To_Close,
        HuntVolume_Outside_Cone
    };

    cv::Point3f center_of_volume() {
        return cv::Point3f(0,-1,-2);
    }

    std::string convert_to_str(hunt_check_result v) {
        return hunt_volume_check_names[v];
    }

    enum view_volume_check_mode {
        strict, /**< viewable volume including a safety distance to borders */
        relaxed /**< the actual viewable volume without any safety distance to the borders */
    };

    /** @brief Checks if the point is in the viewable area.*/
    std::tuple<bool, std::array<bool, N_PLANES>> in_view(cv::Point3f p,view_volume_check_mode c);

    /** @brief Checks whether the point m (aka moth location) is in a good area for a hunt (worth to take off).
    * This area is described as cone above the drone location d. */
    hunt_check_result in_hunt_area(cv::Point3f d, cv::Point3f m);

    /** @brief Calculates the distance to the borders */
    std::tuple<bool, std::array<bool, N_PLANES>> check_distance_to_borders(track_data data_drone, float req_breaking_distance);

    float calc_shortest_distance_to_border(cv::Point3f drone_pos, uint plane_idx, view_volume_check_mode cm);

    cv::Point3f normal_vector(uint plane_idx) {return cv::Point3f(plane_normals.at(plane_idx));};
    cv::Point3f support_vector(uint plane_idx) {return cv::Point3f(plane_supports.at(plane_idx));};

    void p0_bottom_plane(float b_depth);

    cv::Point3f project_into_camera_volume(cv::Point3f pos_stepoint, std::array<bool, N_PLANES> violated_planes);
    cv::Point3f setpoint_in_cameraview(cv::Point3f pos_setpoint);

    cv::Mat corner_point(uint i) {return corner_points.at(i);};
    cv::Mat corner_point_hunt(uint i) {return corner_points_hunt.at(i);};

private:
    float relaxed_safety_margin = 0.3f;
    float strict_safetty_margin = 0.6f;

    // Define limitation planes in plane normal form:
    std::array<cv::Mat, N_PLANES> plane_normals;
    std::array<cv::Mat, N_PLANES> plane_supports;
    std::array<cv::Mat, N_PLANES> plane_normals_hunt;
    std::array<cv::Mat, N_PLANES> plane_supports_hunt;

    // Define corner points
    std::array<cv::Mat, N_CORNER_POINTS> corner_points;
    std::array<cv::Mat, N_CORNER_POINTS> corner_points_hunt;

    // Margins betweens sight-volume and hunt-volume
    double margin_top = 0.2;
    double margin_bottom = 0.2;
    double margin_front = 1.5;
    double margin_back = 0.2;
    double margin_left = 7.0;
    double margin_right = 7.0;

    float minimum_height = 0.3f; /**< Correction distance for the ground plane. */

    /** @brief Checks whether the point p is for all planes on the right side.*/
    std::tuple<bool, std::array<bool, N_PLANES>> in_view(cv::Point3f p, float hysteresis_margin);

    /** @brief Calculates the distance to the borders. */
    std::array<float, N_PLANES> calc_distance_to_borders(std::vector<cv::Point3f> p);

    void calc_corner_points(cv::Mat p0_front, cv::Mat n_front, cv::Mat p0_back, cv::Mat n_back,
                            cv::Mat p0_top, cv::Mat n_top, cv::Mat p0_bottom, cv::Mat n_bottom,
                            cv::Mat p0_left, cv::Mat n_left, cv::Mat p0_right, cv::Mat n_right);

    void calc_corner_points_hunt(cv::Mat p0_front, cv::Mat n_front, cv::Mat p0_back, cv::Mat n_back,
                                 cv::Mat p0_top, cv::Mat n_top, cv::Mat p0_bottom, cv::Mat n_bottom,
                                 cv::Mat p0_left, cv::Mat n_left, cv::Mat p0_right, cv::Mat n_right);
};