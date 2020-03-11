#pragma once

#include "defines.h"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "common.h"

#define N_PLANES 7 //adapt this to the number of planes in plane index!
#define N_PLANE_VERTICES 10 //adapt this to the number of corner points in point_index

#define CAMERA_VIEW_DEBUGGING false

static const char* hunt_volume_check_names[] = {
    "HV_Unknown",
    "HV_OK",
    "HV_To_High",
    "HV_To_Low",
    "HV_To_Close",
    "HV_Outside_Huntarea"
};

struct intersection_point {
    intersection_point(cv::Mat p, int p1, int p2, int p3) {
        pos = p;
        planes = {p1, p2, p3};
    }
    cv::Mat pos = (cv::Mat_<float>(3,1) << 0, 0, 0);
    std::array<int, 3> planes = {0};
};

class CameraView {
public:
    enum plane_index {
        top_plane,
        bottom_plane,
        back_plane,
        front_plane,
        left_plane,
        right_plane,
        camera_plane
    };

    enum point_index {
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
              float depth, float height, float camera_pitch_deg);

    enum hunt_check_result {
        HuntVolume_Unknown,
        HuntVolume_OK,
        HuntVolume_To_High,
        HuntVolume_To_Low,
        HuntVolume_To_Close,
        HuntVolume_Outside_Huntarea
    };

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

    float calc_shortest_distance_to_plane(cv::Point3f drone_pos, uint plane_idx, view_volume_check_mode cm);

    cv::Point3f normal_vector(uint plane_idx) {return cv::Point3f(plane_normals.at(plane_idx));};
    cv::Point3f support_vector(uint plane_idx) {return cv::Point3f(plane_supports.at(plane_idx));};

    void p0_bottom_plane(float b_depth);

    cv::Point3f project_into_camera_volume(cv::Point3f pos_setpoint, view_volume_check_mode cm, std::array<bool, N_PLANES> violated_planes);
    cv::Point3f setpoint_in_cameraview(cv::Point3f pos_setpoint, view_volume_check_mode cm);

    cv::Mat corner_point(uint i) {return corner_points.at(i);};
    cv::Mat corner_point_hunt(uint i) {return corner_points_hunt.at(i);};

    std::array<cv::Mat, N_PLANES> plane_normals;
    std::array<cv::Mat, N_PLANES> plane_supports;
    cv::Point3f center_of_volume = {0,-1,-2};

    void cout_plane_violation(std::array<bool, N_PLANES> inview_violations, std::array<bool, N_PLANES> breaking_violations);

    cv::Mat get_adjacency_matrix() {return adjacency_matrix;}; // For debugging only
private:

    float relaxed_safety_margin = 0.3f;
    float strict_safety_margin = 0.6f;

    // Define limitation planes in plane normal form:
    std::array<cv::Mat, N_PLANES> plane_normals_hunt;
    std::array<cv::Mat, N_PLANES> plane_supports_hunt;
    cv::Mat adjacency_matrix; // planes are vertices,
    // Define corner points
    std::vector<intersection_point> vertices;
    std::vector<intersection_point> vertices_relaxed;
    std::array<cv::Mat, N_PLANE_VERTICES> corner_points;
    std::array<cv::Mat, N_PLANE_VERTICES> corner_points_relaxed;
    std::array<cv::Mat, N_PLANE_VERTICES> corner_points_hunt;

    // Margins betweens sight-volume and hunt-volume
    double margin_top = 0.2;
    double margin_bottom = 0.2;
    double margin_front = 1.5;
    double margin_back = 0.2;
    double margin_left = 7.0;
    double margin_right = 7.0;
    double margin_camera = 0.3;

    float minimum_height = 0.3f; /**< Correction distance for the ground plane. */

    /** @brief Calculates the distance to the borders. */
    std::array<float, N_PLANES> calc_distance_to_borders(std::vector<cv::Point3f> p);

    void update_plane_vertices();
    std::vector<intersection_point> plane_vertices(uint plane_idx);
    cv::Point3f project_into_plane_segment(cv::Point3f p, std::vector<intersection_point> _plane_vertices);

    void calc_corner_points(cv::Mat p0_front, cv::Mat n_front, cv::Mat p0_back, cv::Mat n_back,
                            cv::Mat p0_top, cv::Mat n_top, cv::Mat p0_bottom, cv::Mat n_bottom,
                            cv::Mat p0_left, cv::Mat n_left, cv::Mat p0_right, cv::Mat n_right);

    void calc_corner_points_hunt(cv::Mat p0_front, cv::Mat n_front, cv::Mat p0_back, cv::Mat n_back,
                                 cv::Mat p0_top, cv::Mat n_top, cv::Mat p0_bottom, cv::Mat n_bottom,
                                 cv::Mat p0_left, cv::Mat n_left, cv::Mat p0_right, cv::Mat n_right);

    bool in_plane_segment(cv::Point3f p, std::vector<intersection_point> _plane_vertices);
    void adjacency_entry(uint val, uint p1, uint p2, uint p3);
    bool vertices_on_one_edge(intersection_point p1, intersection_point p2);

    float safety_margin(view_volume_check_mode cm) {
        if(cm==relaxed)
            return relaxed_safety_margin;
        else
            return strict_safety_margin;

    }
};

std::ostream &operator<<(std::ostream &os, const CameraView &c);