#pragma once
#include "quadraticoptimizer.h"
#include "tracking.h"
#include "flightareaconfig.h"
#include "rosvisualizerdatapublisher.h"
#include <Eigen/src/Core/Matrix.h>
#include <opencv2/core/types.hpp>

class RosVisualizerInterface {
public:
    void init();
    void drone(tracking::TrackData drone);
    void drone_input(tracking::TrackData drone, cv::Point3f acceleration);
    void update_drone_path(tracking::TrackData drone);
    void update_insect_path(tracking::TrackData insect);
    void optimized_drone_stop_pos(cv::Point3f pos);
    void insect(tracking::TrackData insect);
    void flightarea(FlightAreaConfig *flightareaconf);
    void optimization_flightarea(FlightAreaConfig *flightareaconf);
    void flightarea(FlightAreaConfig *flightareaconf, line_list_ids line_list_id);
    void drone_position_setpoint(cv::Point3f setpoint);

    void path(std::vector<Eigen::VectorXd> traj, path_ids path_id);
    void state_trajectory(std::vector<Eigen::VectorXd> traj);
    void input_trajectory(std::vector<Eigen::VectorXd> pos_traj, std::vector<Eigen::VectorXd> input_traj, arrow_list_ids arrow_list_id);

    void publish() {ros_publisher.publish();};

private:
    RosVisualizerDataPublisher ros_publisher;
    float velocity_scale = 0.1f;
    float acceleration_scale = 0.05f;

    geometry_msgs::Pose build_velocity_pose(tracking::TrackData data);
    geometry_msgs::Pose build_acceleration_pose(tracking::TrackData data, cv::Point3f acceleration);
    geometry_msgs::Pose build_pose(Eigen::VectorXd state);
    geometry_msgs::Point build_point(tracking::TrackData data);
    void add_points_to_line_list(std::vector<geometry_msgs::Point> *line_list, std::vector<cv::Point3f> points);

    std::vector<cv::Point3f> sort_corner_points(std::vector<cv::Point3f> corner_points);
    bool corner_point_combination_build_nonintersecting_polygon(std::vector<cv::Point3f> corner_points, std::vector<int> combination);
    std::vector<cv::Point3f> rebuild_corner_points(std::vector<cv::Point3f> corner_points, std::vector<int> combination);
};
