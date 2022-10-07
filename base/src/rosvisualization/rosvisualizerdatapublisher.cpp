#include "rosvisualizerdatapublisher.h"
#include "common.h"
#include <vector>

std::vector<std::string> found_subscribers = {};
static void subscriber_found_callback(const ros::SingleSubscriberPublisher &publisher) {
    std::string subscriber = publisher.getSubscriberName();
    std::cout << subscriber << " " <<  publisher.getTopic() << std::endl;
    if (subscriber.erase(5) == "/rviz") {
        std::string topic = publisher.getTopic();
        topic.erase(0, 1);
        found_subscribers.push_back(topic);
    }
}

void RosVisualizerDataPublisher::eval_found_subscribers() {
    for (auto subscriber : found_subscribers) {
        for (uint i = 0; i < number_point_ids; i++) {
            if (point_message_names[i] == subscriber) {
                PointPublish *point_pub = point_publish(static_cast<point_ids>(i));
                point_pub->subscriber_found = true;

            }
        }
        for (uint i = 0; i < number_pose_ids; i++) {
            if (pose_message_names[i] == subscriber) {
                PosePublish *pose_pub = pose_publish(static_cast<pose_ids>(i));
                pose_pub->subscriber_found = true;

            }
        }
        for (uint i = 0; i < number_path_ids; i++) {
            if (path_message_names[i] == subscriber) {
                PathPublish *path_pub = path_publish(static_cast<path_ids>(i));
                path_pub->subscriber_found = true;

            }
        }
        for (uint i = 0; i < number_arrow_ids; i++) {
            if (arrow_message_names[i] == subscriber) {
                ArrowPublish *arrow_pub = arrow_publish(static_cast<arrow_ids>(i));
                arrow_pub->subscriber_found = true;

            }
        }
        for (uint i = 0; i < number_arrow_list_ids; i++) {
            if (arrow_list_message_names[i] == subscriber) {
                ArrowListPublish *arrow_list_pub = arrow_list_publish(static_cast<arrow_list_ids>(i));
                arrow_list_pub->subscriber_found = true;

            }
        }
        for (uint i = 0; i < number_line_list_ids; i++) {
            if (line_list_message_names[i] == subscriber) {
                LineListPublish *line_list_pub = line_list_publish(static_cast<line_list_ids>(i));
                line_list_pub->subscriber_found = true;

            }
        }
    }
    found_subscribers.clear();
}


void RosVisualizerDataPublisher::init() {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "pats");
    // ros::Rate loop_rate(10); // TODO Do I want this are can this be removed?
    node = new ros::NodeHandle();
    static_broadcaster = new tf2_ros::StaticTransformBroadcaster();
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "world";
    static_transformStamped.child_frame_id = "pats";
    static_transformStamped.transform.translation.x = 0.;
    static_transformStamped.transform.translation.y = 0.;
    static_transformStamped.transform.translation.z = 0.;
    tf2::Quaternion quat;
    quat.setRPY(90 * deg2rad, 0, 0);
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();
    static_broadcaster->sendTransform(static_transformStamped);

    for (uint i = 0; i < number_point_ids; i++) {
        PointPublish point_pub;
        geometry_msgs::PointStamped point;
        point.header.frame_id = "pats";
        point.header.stamp = ros::Time::now();
        point.point.x = 0;
        point.point.y = 0;
        point.point.z = 0;
        point_pub.message = point;

        point_pub.publisher = node->advertise<geometry_msgs::PointStamped>(point_message_names[i], 1, static_cast<ros::SubscriberStatusCallback>(subscriber_found_callback));
        point_publishers[static_cast<point_ids>(i)] = point_pub;
    }

    for (uint i = 0; i < number_pose_ids; i++) {
        PosePublish pose_pub;
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "pats";
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 1;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 0;
        pose_pub.message = pose;

        pose_pub.publisher = node->advertise<geometry_msgs::PoseStamped>(pose_message_names[i], 1, static_cast<ros::SubscriberStatusCallback>(subscriber_found_callback));
        pose_publishers[static_cast<pose_ids>(i)] = pose_pub;
    }

    for (uint i = 0; i < number_path_ids; i++) {
        PathPublish path_pub;
        nav_msgs::Path path;
        path.header.frame_id = "pats";
        path.header.stamp = ros::Time::now();
        path.poses = {};
        path_pub.message = path;

        path_pub.publisher = node->advertise<nav_msgs::Path>(path_message_names[i], 1, static_cast<ros::SubscriberStatusCallback>(subscriber_found_callback));
        path_publishers[static_cast<path_ids>(i)] = path_pub;
    }

    for (uint i = 0; i < number_arrow_ids; i++) {
        ArrowPublish arrow_pub;
        visualization_msgs::Marker message;
        message.header.frame_id = "pats";
        message.header.stamp = ros::Time::now();
        message.ns = arrow_message_names[i];
        message.id = i;
        message.type = visualization_msgs::Marker::ARROW;
        // message.action = visualization_msgs::message::ADD;
        message.pose.position.x = 0;
        message.pose.position.y = 0;
        message.pose.position.z = 0;
        message.pose.orientation.x = 0.0;
        message.pose.orientation.y = 0.0;
        message.pose.orientation.z = 0.0;
        message.pose.orientation.w = 1.0;
        message.scale.x = 0.2; //arrow length
        message.scale.y = .02;
        message.scale.z = .02;
        message.color.r = 0.0f;
        message.color.g = 1.0f;
        message.color.b = 0.0f;
        message.color.a = 1.0; //visible or not

        arrow_pub.message = message;
        arrow_pub.publisher = node->advertise<visualization_msgs::Marker>(arrow_message_names[i], 1, static_cast<ros::SubscriberStatusCallback>(subscriber_found_callback));
        arrow_publishers[static_cast<arrow_ids>(i)] = arrow_pub;
    }

    for (uint i = 0; i < number_arrow_list_ids; i++) {
        ArrowListPublish arrow_list_pub;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "pats";
        marker.header.stamp = ros::Time::now();
        marker.ns = arrow_list_message_names[i];
        marker.id = i;
        marker.type = visualization_msgs::Marker::ARROW;
        // marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.0; //arrow_list length
        marker.scale.y = .02;
        marker.scale.z = .02;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0; //visible or not

        arrow_list_pub.marker_template = marker;
        arrow_list_pub.markers = {};
        arrow_list_pub.publisher = node->advertise<visualization_msgs::Marker>(arrow_list_message_names[i], 1, static_cast<ros::SubscriberStatusCallback>(subscriber_found_callback));
        arrow_list_publishers[static_cast<arrow_list_ids>(i)] = arrow_list_pub;
    }

    for (uint i = 0; i < number_line_list_ids; i++) {
        LineListPublish line_list_pub;
        visualization_msgs::Marker message;
        message.type = visualization_msgs::Marker::LINE_LIST;
        message.header.frame_id = "pats";
        message.header.stamp = ros::Time::now();
        message.ns = line_list_message_names[i];
        message.points = {};
        message.scale.x = 0.003;
        // message.scale.y = 1.;
        // message.scale.z = 1.;
        message.color.r = 1.0f;
        message.color.g = 1.0f;
        message.color.b = 1.0f;
        message.color.a = 1.0; //visible or not
        line_list_pub.message = message;
        line_list_pub.publisher = node->advertise<visualization_msgs::Marker>(line_list_message_names[i], 1, static_cast<ros::SubscriberStatusCallback>(subscriber_found_callback));
        line_list_publishers[static_cast<line_list_ids>(i)] = line_list_pub;
    }
    setup_markers();
}


void RosVisualizerDataPublisher::setup_markers() {

    for (uint i = 0; i < number_arrow_ids; i++) {
        ArrowPublish *arrow_pub = arrow_publish(static_cast<arrow_ids>(i));
        if (static_cast<arrow_ids>(i) == drone_vel) {
            arrow_pub->message.color.r = 0;
            arrow_pub->message.color.g = 1.f;
            arrow_pub->message.color.b = 0;
        } else if (static_cast<arrow_ids>(i) == drone_acc) {
            arrow_pub->message.color.r = 1.f;
            arrow_pub->message.color.g = 0;
            arrow_pub->message.color.b = 0;
        } else if (static_cast<arrow_ids>(i) == insect_vel) {
            arrow_pub->message.color.r = 1.f;
            arrow_pub->message.color.g = 1.f;
            arrow_pub->message.color.b = 1.f;
        }
    }
    for (uint i = 0; i < number_arrow_list_ids; i++) {
        ArrowListPublish *arrow_list_pub = arrow_list_publish(static_cast<arrow_list_ids>(i));
        if (static_cast<arrow_list_ids>(i) == drone_state_trajectory) {
            arrow_list_pub->marker_template.color.r = 0;
            arrow_list_pub->marker_template.color.g = 0;
            arrow_list_pub->marker_template.color.b = 1.f;
        } else if (static_cast<arrow_list_ids>(i) == drone_input_trajectory) {
            arrow_list_pub->marker_template.color.r = 0.503937008;
            arrow_list_pub->marker_template.color.g = 0.106299213;
            arrow_list_pub->marker_template.color.b = 0.956692913;
        } else if (static_cast<arrow_list_ids>(i) == drone_virtual_input_trajectory) {
            arrow_list_pub->marker_template.color.r = 1;
            arrow_list_pub->marker_template.color.g = 0.66;
            arrow_list_pub->marker_template.color.b = 0;
        }
    }

}

void RosVisualizerDataPublisher::update_point_mesage(geometry_msgs::Point point, point_ids point_id) {
    PointPublish *point_pub = point_publish(point_id);
    point_pub->message.point = point;
    point_pub->message.header.stamp = ros::Time::now();
}

void RosVisualizerDataPublisher::update_pose_mesage(geometry_msgs::Pose pose, pose_ids pose_id) {
    PosePublish *pose_pub = pose_publish(pose_id);
    pose_pub->message.pose = pose;
    pose_pub->message.header.stamp = ros::Time::now();
}

void RosVisualizerDataPublisher::add_pose_to_path(geometry_msgs::Pose pose, path_ids path_id, uint path_length) {
    PathPublish *path_pub = path_publish(path_id);
    path_pub->message.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped poses_stamped;
    poses_stamped.header.frame_id = "pats";
    poses_stamped.header.stamp =  ros::Time::now();
    poses_stamped.pose = pose;
    path_pub->message.poses.push_back(poses_stamped);

    if (path_pub->message.poses.size() > path_length) {
        uint n_erase = path_pub->message.poses.size() - path_length;
        path_pub->message.poses.erase(path_pub->message.poses.begin(),  path_pub->message.poses.begin() + n_erase);
    }
}

void RosVisualizerDataPublisher::path(std::vector<geometry_msgs::Pose> poses, path_ids path_id) {
    PathPublish *path_pub = path_publish(path_id);
    path_pub->message.header.stamp = ros::Time::now();
    path_pub->message.poses = {};

    for (auto pose : poses) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "pats";
        pose_stamped.header.stamp =  ros::Time::now();
        pose_stamped.pose = pose;
        path_pub->message.poses.push_back(pose_stamped);
    }
}

void RosVisualizerDataPublisher::trajectory(std::vector<ArrowData> arrows, arrow_list_ids arrow_list_id) {
    ArrowListPublish *arrow_list_pub = arrow_list_publish(arrow_list_id);
    arrow_list_pub->markers = {};

    uint id = 0;
    for (auto arrow : arrows) {
        visualization_msgs::Marker marker = arrow_list_pub->marker_template;
        marker.header.stamp = ros::Time::now();
        marker.id = id++;
        marker.pose = arrow.pose;
        marker.scale.x = arrow.length;
        arrow_list_pub->markers.push_back(marker);
    }
}

void RosVisualizerDataPublisher::update_arrow_mesage(geometry_msgs::Pose pose, float length, arrow_ids arrow_id) {
    ArrowPublish *arrow_pub = arrow_publish(arrow_id);
    arrow_pub->message.pose = pose;
    arrow_pub->message.scale.x = length;
    arrow_pub->message.header.stamp = ros::Time::now();
}

void RosVisualizerDataPublisher::update_line_list_mesage(std::vector<geometry_msgs::Point> points, line_list_ids line_list_id) {
    LineListPublish *line_list_pub = line_list_publish(line_list_id);
    line_list_pub->message.header.stamp = ros::Time::now();
    line_list_pub->message.points = {};
    for (uint i = 0; i < points.size(); i++) {
        line_list_pub->message.points.push_back(points.at(i));
    }
}

void RosVisualizerDataPublisher::publish() {
    for (uint i = 0; i < number_point_ids; i++) {
        PointPublish *point_pub = point_publish(static_cast<point_ids>(i));
        point_pub->publisher.publish(point_pub->message);

        while (!point_pub->subscriber_found) {
            point_pub->message.header.stamp = ros::Time::now();
            point_pub->publisher.publish(point_pub->message);
            ros::spinOnce();
            eval_found_subscribers();
        }
    }
    for (uint i = 0; i < number_pose_ids; i++) {
        PosePublish *pose_pub = pose_publish(static_cast<pose_ids>(i));
        pose_pub->publisher.publish(pose_pub->message);

        while (!pose_pub->subscriber_found) {
            pose_pub->publisher.publish(pose_pub->message);
            ros::spinOnce();
            eval_found_subscribers();
        }
    }
    for (uint i = 0; i < number_path_ids; i++) {
        PathPublish *path_pub = path_publish(static_cast<path_ids>(i));
        path_pub->publisher.publish(path_pub->message);
        while (!path_pub->subscriber_found && path_pub->message.poses.size() != 0) {
            path_pub->publisher.publish(path_pub->message);
            ros::spinOnce();
            eval_found_subscribers();
        }
    }
    for (uint i = 0; i < number_arrow_ids; i++) {
        ArrowPublish *arrow_pub = arrow_publish(static_cast<arrow_ids>(i));
        arrow_pub->publisher.publish(arrow_pub->message);
        while (!arrow_pub->subscriber_found) {
            arrow_pub->publisher.publish(arrow_pub->message);
            ros::spinOnce();
            eval_found_subscribers();
        }
    }
    for (uint i = 0; i < number_arrow_list_ids; i++) {
        ArrowListPublish *arrow_list_pub = arrow_list_publish(static_cast<arrow_list_ids>(i));
        for (auto msg : arrow_list_pub->markers) {
            arrow_list_pub->publisher.publish(msg);
        }
        while (!arrow_list_pub->subscriber_found) {
            for (auto msg : arrow_list_pub->markers) {
                arrow_list_pub->publisher.publish(msg);
            }
            ros::spinOnce();
            eval_found_subscribers();
        }
    }
    for (uint i = 0; i < number_line_list_ids; i++) {
        LineListPublish *line_list_pub = line_list_publish(static_cast<line_list_ids>(i));
        line_list_pub->publisher.publish(line_list_pub->message);
        while (!line_list_pub->subscriber_found) {
            line_list_pub->publisher.publish(line_list_pub->message);
            ros::spinOnce();
            eval_found_subscribers();
        }
    }
    ros::spinOnce();
}

PointPublish *RosVisualizerDataPublisher::point_publish(point_ids point_id) {
    std::map<point_ids, PointPublish>::iterator ret = point_publishers.find(point_id);
    if (ret == point_publishers.end())
        throw std::runtime_error("RosVisualizerDataPublisher error: PointStamped missing!");

    return &(ret->second);
}

PosePublish *RosVisualizerDataPublisher::pose_publish(pose_ids pose_id) {
    std::map<pose_ids, PosePublish>::iterator ret = pose_publishers.find(pose_id);
    if (ret == pose_publishers.end())
        throw std::runtime_error("RosVisualizerDataPublisher error: PoseStamped missing!");

    return &(ret->second);
}

PathPublish *RosVisualizerDataPublisher::path_publish(path_ids path_id) {
    std::map<path_ids, PathPublish>::iterator ret = path_publishers.find(path_id);
    if (ret == path_publishers.end())
        throw std::runtime_error("RosVisualizerDataPublisher error: PathStamped missing!");

    return &(ret->second);
}

ArrowPublish *RosVisualizerDataPublisher::arrow_publish(arrow_ids arrow_id) {
    std::map<arrow_ids, ArrowPublish>::iterator ret = arrow_publishers.find(arrow_id);
    if (ret == arrow_publishers.end())
        throw std::runtime_error("RosVisualizerDataPublisher error: ArrowStamped missing!");

    return &(ret->second);
}

ArrowListPublish *RosVisualizerDataPublisher::arrow_list_publish(arrow_list_ids arrow_list_id) {
    std::map<arrow_list_ids, ArrowListPublish>::iterator ret = arrow_list_publishers.find(arrow_list_id);
    if (ret == arrow_list_publishers.end())
        throw std::runtime_error("RosVisualizerDataPublisher error: Arrow_ListStamped missing!");

    return &(ret->second);
}

LineListPublish *RosVisualizerDataPublisher::line_list_publish(line_list_ids line_list_id) {
    std::map<line_list_ids, LineListPublish>::iterator ret = line_list_publishers.find(line_list_id);
    if (ret == line_list_publishers.end())
        throw std::runtime_error("RosVisualizerDataPublisher error: Line_ListStamped missing!");

    return &(ret->second);
}
