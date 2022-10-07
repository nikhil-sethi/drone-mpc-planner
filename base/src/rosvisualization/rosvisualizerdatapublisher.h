#pragma once
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

enum point_ids {
    drone_pos,
    insect_pos,
    opti_drone_stop_pos,
    number_point_ids,
};

[[maybe_unused]] static const char *point_message_names[] = {
    "drone_pos",
    "insect_pos",
    "opti_drone_stop_pos",
};

struct PointPublish {
    ros::Publisher publisher;
    geometry_msgs::PointStamped message;
    bool subscriber_found = false;
};

enum pose_ids {
    number_pose_ids,
};

[[maybe_unused]] static const char *pose_message_names[] = {
};

struct PosePublish {
    ros::Publisher publisher;
    geometry_msgs::PoseStamped message;
    bool subscriber_found = false;
};

enum path_ids {
    drone_path,
    insect_path,
    opti_initial_guess,
    opti_current_guess,
    opti_solution,
    number_path_ids,
};

[[maybe_unused]] static const char *path_message_names[] = {
    "drone_path",
    "insect_path",
    "opti_initial_guess",
    "opti_current_guess",
    "opti_solution",
};

struct PathPublish {
    ros::Publisher publisher;
    nav_msgs::Path message;
    bool subscriber_found = false;
};

enum arrow_ids {
    drone_vel,
    drone_acc,
    insect_vel,
    number_arrow_ids,
};

[[maybe_unused]] static const char *arrow_message_names[] = {
    "drone_vel",
    "drone_acc",
    "insect_vel",
};

struct ArrowPublish {
    ros::Publisher publisher;
    visualization_msgs::Marker message;
    bool subscriber_found = false;
};

enum arrow_list_ids {
    drone_state_trajectory,
    drone_input_trajectory,
    drone_virtual_input_trajectory,
    number_arrow_list_ids,
};

[[maybe_unused]] static const char *arrow_list_message_names[] = {
    "drone_state_trajectory",
    "drone_input_trajectory",
    "drone_virtual_input_trajectory",
};

struct ArrowListPublish {
    ros::Publisher publisher;
    visualization_msgs::Marker marker_template;
    std::vector<visualization_msgs::Marker> markers;
    bool subscriber_found = false;
};

enum line_list_ids {
    flight_area,
    optimization_flight_area,
    number_line_list_ids,
};

[[maybe_unused]] static const char *line_list_message_names[] = {
    "flight_area",
    "optimization_flight_area",
};

struct LineListPublish {
    ros::Publisher publisher;
    visualization_msgs::Marker message;
    bool subscriber_found = false;
};

struct ArrowData {
    geometry_msgs::Pose pose;
    float length;

    ArrowData(geometry_msgs::Pose p, float l): pose(p), length(l) {};
};

class RosVisualizerDataPublisher {
public:
    void init();
    void update_point_mesage(geometry_msgs::Point point, point_ids point_id);
    void update_pose_mesage(geometry_msgs::Pose pose, pose_ids pose_id);
    void add_pose_to_path(geometry_msgs::Pose pose, path_ids path_id, uint path_length);
    void path(std::vector<geometry_msgs::Pose> poses, path_ids path_id);
    void trajectory(std::vector<ArrowData> arrows, arrow_list_ids arrow_list_id);

    void update_arrow_mesage(geometry_msgs::Pose pose, float length, arrow_ids arrow_id);
    void update_line_list_mesage(std::vector<geometry_msgs::Point> points, line_list_ids line_list_id);

    void publish();

private:
    ros::NodeHandle *node;
    tf2_ros::StaticTransformBroadcaster *static_broadcaster;
    void setup_markers();

    void eval_found_subscribers();

    std::map<point_ids, PointPublish> point_publishers;
    PointPublish *point_publish(point_ids point_id);

    std::map<pose_ids, PosePublish> pose_publishers;
    PosePublish *pose_publish(pose_ids pose_id);

    std::map<path_ids, PathPublish> path_publishers;
    PathPublish *path_publish(path_ids path_id);

    std::map<arrow_ids, ArrowPublish> arrow_publishers;
    ArrowPublish *arrow_publish(arrow_ids arrow_id);

    std::map<arrow_list_ids, ArrowListPublish> arrow_list_publishers;
    ArrowListPublish *arrow_list_publish(arrow_list_ids arrow_list_id);

    std::map<line_list_ids, LineListPublish> line_list_publishers;
    LineListPublish *line_list_publish(line_list_ids line_list_id);

    geometry_msgs::TransformStamped static_transformStamped;
};
