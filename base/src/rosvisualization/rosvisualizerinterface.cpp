#include "rosvisualizerinterface.h"
#include "common.h"
#include "linalg.h"
#include "flightareaconfig.h"
#include "quadraticoptimizer.h"
#include "quaternion.h"
#include "rosvisualization/rosvisualizerdatapublisher.h"
#include <cmath>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <opencv2/core/types.hpp>
#include <vector>

void RosVisualizerInterface:: init() {
    ros_publisher.init();
}


void RosVisualizerInterface::drone(tracking::TrackData drone) {
    if (drone.pos_valid) {
        geometry_msgs::Pose pose = build_velocity_pose(drone);
        ros_publisher.update_arrow_mesage(pose, velocity_scale * normf(drone.vel()), drone_vel);
        geometry_msgs::Point point = build_point(drone);
        ros_publisher.update_point_mesage(point, drone_pos);
    }
}

void RosVisualizerInterface::drone_input(tracking::TrackData drone, cv::Point3f acceleration) {
    if (drone.pos_valid) {
        geometry_msgs::Pose pose = build_acceleration_pose(drone, acceleration);
        ros_publisher.update_arrow_mesage(pose, acceleration_scale * normf(acceleration), drone_acc);
    }
}

void RosVisualizerInterface::update_drone_path(tracking::TrackData drone) {
    if (drone.pos_valid) {
        geometry_msgs::Pose pose = build_velocity_pose(drone);
        ros_publisher.add_pose_to_path(pose, drone_path, 90);
    }
}

void RosVisualizerInterface::update_insect_path(tracking::TrackData insect) {
    if (insect.pos_valid) {
        geometry_msgs::Pose pose = build_velocity_pose(insect);
        ros_publisher.add_pose_to_path(pose, insect_path, 90);
    }
}
void RosVisualizerInterface::optimized_drone_stop_pos(cv::Point3f pos) {
    geometry_msgs::Point point;
    point.x = pos.x;
    point.y = pos.y;
    point.z = pos.z;

    ros_publisher.update_point_mesage(point, opti_drone_stop_pos);
}


void RosVisualizerInterface::insect(tracking::TrackData insect) {
    geometry_msgs::Pose pose = build_velocity_pose(insect);
    ros_publisher.update_arrow_mesage(pose, normf(insect.vel()), insect_vel);
    geometry_msgs::Point point = build_point(insect);
    ros_publisher.update_point_mesage(point, insect_pos);

    // ros_publisher.add_pose_to_path(pose, insect_path, 90);
}

void RosVisualizerInterface::flightarea(FlightAreaConfig *flightareaconf) {
    flightarea(flightareaconf, flight_area);
}

void RosVisualizerInterface::optimization_flightarea(FlightAreaConfig *flightareaconf) {
    flightarea(flightareaconf, optimization_flight_area);
}

void RosVisualizerInterface::flightarea(FlightAreaConfig *flightareaconf, line_list_ids line_list_id) {
    std::vector<geometry_msgs::Point> points;

    auto planes = flightareaconf->active_planes();

    for (auto plane : planes) {
        auto corner_points = flightareaconf->find_corner_points_of_plane(plane.id);
        corner_points = sort_corner_points(corner_points);
        add_points_to_line_list(&points, corner_points);
    }

    ros_publisher.update_line_list_mesage(points, line_list_id);
}


void RosVisualizerInterface::path(std::vector<Eigen::VectorXd> traj, path_ids path_id) {
    std::vector<geometry_msgs::Pose> poses;
    for (auto sample : traj) {
        geometry_msgs::Pose pose;
        pose.position.x = sample[0];
        pose.position.y = sample[1];
        pose.position.z = sample[2];
        poses.push_back(pose);
    }
    ros_publisher.path(poses, path_id);
}

void RosVisualizerInterface::state_trajectory(std::vector<Eigen::VectorXd> traj) {
    std::vector<ArrowData> arrows = {};
    for (auto sample : traj) {
        geometry_msgs::Pose pose = build_pose(sample);
        cv::Point3f vel = cv::Point3f(sample[3], sample[4], sample[5]);
        ArrowData arrow = ArrowData(pose, velocity_scale * normf(vel));
        arrows.push_back(arrow);
    }

    ros_publisher.trajectory(arrows, drone_state_trajectory);
}

void RosVisualizerInterface::input_trajectory(std::vector<Eigen::VectorXd> pos_traj, std::vector<Eigen::VectorXd> input_traj, arrow_list_ids arrow_list_id) {
    std::vector<ArrowData> arrows = {};

    for (uint i = 0; i < input_traj.size(); i++) {
        Eigen::VectorXd sample = pos_traj.at(i);

        if (input_traj.at(i).norm() > 0) {
            sample[3] = input_traj.at(i)[0];
            sample[4] = input_traj.at(i)[1];
            sample[5] = input_traj.at(i)[2];

            cv::Point3f vec = cv::Point3f(input_traj.at(i)[0], input_traj.at(i)[1], input_traj.at(i)[2]);
            geometry_msgs::Pose pose = build_pose(sample);
            ArrowData arrow = ArrowData(pose, acceleration_scale * normf(vec));

            arrows.push_back(arrow);
        }
    }
    ros_publisher.trajectory(arrows, arrow_list_id);
}

geometry_msgs::Point RosVisualizerInterface::build_point(tracking::TrackData data) {
    geometry_msgs::Point point;
    point.x = 0;
    point.y = 0;
    point.z = 0;

    if (data.pos_valid) {
        point.x = data.pos().x;
        point.y = data.pos().y;
        point.z = data.pos().z;

    }
    return point;
}

geometry_msgs::Pose RosVisualizerInterface::build_velocity_pose(tracking::TrackData data) {
    geometry_msgs::Pose pose;

    pose.position.x = data.pos().x;
    pose.position.y = data.pos().y;
    pose.position.z = data.pos().z;

    if (normf(data.vel()) > 0) {
        cv::Point3f normal_velocity = data.vel() / normf(data.vel());
        Quaternion q = rot_quat(cv::Point3f(1, 0, 0), normal_velocity);
        pose.orientation.x = q.v.x;
        pose.orientation.y = q.v.y;
        pose.orientation.z = q.v.z;
        pose.orientation.w = q.s;
    } else {
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 0;
    }
    return pose;
}

geometry_msgs::Pose RosVisualizerInterface::build_acceleration_pose(tracking::TrackData data, cv::Point3f acceleration) {
    geometry_msgs::Pose pose;

    pose.position.x = data.pos().x;
    pose.position.y = data.pos().y;
    pose.position.z = data.pos().z;

    if (normf(acceleration) > 0) {
        cv::Point3f normal_acceleration = acceleration / normf(acceleration);
        Quaternion q = rot_quat(cv::Point3f(1, 0, 0), normal_acceleration);
        pose.orientation.x = q.v.x;
        pose.orientation.y = q.v.y;
        pose.orientation.z = q.v.z;
        pose.orientation.w = q.s;
    } else {
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 0;
    }
    return pose;

}

geometry_msgs::Pose RosVisualizerInterface::build_pose(Eigen::VectorXd state) {
    geometry_msgs::Pose pose;

    pose.position.x = state[0];
    pose.position.y = state[1];
    pose.position.z = state[2];

    cv::Point3f vec = cv::Point3f(state[3], state[4], state[5]);
    Quaternion q = rot_quat(cv::Point3f(1, 0, 0), vec / normf(vec));

    pose.orientation.x = q.v.x;
    pose.orientation.y = q.v.y;
    pose.orientation.z = q.v.z;
    pose.orientation.w = q.s;

    return pose;
}

void RosVisualizerInterface::add_points_to_line_list(std::vector<geometry_msgs::Point> *line_list, std::vector<cv::Point3f> points) {
    geometry_msgs::Point ros_p0;
    ros_p0.x = points.at(0).x;
    ros_p0.y = points.at(0).y;
    ros_p0.z = points.at(0).z;
    line_list->push_back(ros_p0);

    for (uint i = 1; i < points.size(); i++) {
        geometry_msgs::Point ros_pi;
        ros_pi.x = points.at(i).x;
        ros_pi.y = points.at(i).y;
        ros_pi.z = points.at(i).z;
        line_list->push_back(ros_pi);
        line_list->push_back(ros_pi);
    }

    line_list->push_back(ros_p0);
}


std::vector<cv::Point3f> RosVisualizerInterface::sort_corner_points(std::vector<cv::Point3f> corner_points) {
    uint N = std::tgamma(corner_points.size());
    for (uint i = 0; i < N; i++) {
        auto combination = combination_sample(corner_points.size(), i);

        if (corner_point_combination_build_nonintersecting_polygon(corner_points, combination))
            return rebuild_corner_points(corner_points, combination);

    }
    return corner_points;
}

bool RosVisualizerInterface::corner_point_combination_build_nonintersecting_polygon(std::vector<cv::Point3f> corner_points, std::vector<int> combination) {
    float polygon_angle = 0.f;
    uint n_corner_points = corner_points.size();
    for (uint i = 0; i < n_corner_points; i++) {
        uint i0 = combination.at(i);
        uint ii = combination.at((i + 1) % n_corner_points);
        uint il = combination.at((i + 2) % n_corner_points);
        polygon_angle += abs(angle_between_points(corner_points.at(i0), corner_points.at(ii), corner_points.at(il)));
    }

    if (abs(polygon_angle - M_PIf32 * (n_corner_points - 2)) < 5.f * deg2rad)
        return true;

    return false;

}

std::vector<cv::Point3f> RosVisualizerInterface::rebuild_corner_points(std::vector<cv::Point3f> corner_points, std::vector<int> combination) {
    std::vector<cv::Point3f> new_corner_points = {};

    for (uint i = 0; i < corner_points.size(); i++) {
        new_corner_points.push_back(corner_points.at(combination.at(i)));
    }

    return new_corner_points;
}
