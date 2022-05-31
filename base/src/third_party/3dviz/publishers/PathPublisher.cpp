#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/publisher/Publisher.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/Domain.h>
#include <fastrtps/utils/eClock.h>


#include "PathPublisher.h"

#include "PoseStamped.h"
#include "Pose.h"
#include "Point.h"

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;

PathPublisher::PathPublisher() : mp_participant(nullptr), mp_publisher_proximity(nullptr) {}

PathPublisher::~PathPublisher() {   Domain::removeParticipant(mp_participant);}

bool PathPublisher::init() {

    ParticipantAttributes PParam;
    PParam.rtps.builtin.domainId = 0;
    PParam.rtps.builtin.leaseDuration = c_TimeInfinite;
    PParam.rtps.setName("path_publisher");
    mp_participant = Domain::createParticipant(PParam);

    if (mp_participant == nullptr) {
        return false;
    }

    Domain::registerType(mp_participant, static_cast<TopicDataType *>(&myType));

    PublisherAttributes Wparam;
    Wparam.topic.topicKind = NO_KEY;
    Wparam.topic.topicDataType = myType.getName();
    Wparam.topic.topicName = "rt/camera_cone";

    mp_publisher_camera_cone = Domain::createPublisher(mp_participant, Wparam, static_cast<PublisherListener *>(&m_listener));
    if (mp_publisher_camera_cone == nullptr) {
        std::cout << "Creating publisher failed" << std::endl;
        return false;
    }

    Wparam.topic.topicName = "rt/safe_zone";
    mp_publisher_safe_zone = Domain::createPublisher(mp_participant, Wparam, static_cast<PublisherListener *>(&m_listener));
    if (mp_publisher_safe_zone == nullptr) {
        std::cout << "Creating publisher failed" << std::endl;
        return false;
    }

    Wparam.topic.topicName = "rt/proximity";
    mp_publisher_proximity = Domain::createPublisher(mp_participant, Wparam, static_cast<PublisherListener *>(&m_listener));
    if (mp_publisher_proximity == nullptr) {
        std::cout << "Creating publisher failed" << std::endl;
        return false;
    }

    Wparam.topic.topicName = "rt/target";
    mp_publisher_target = Domain::createPublisher(mp_participant, Wparam, static_cast<PublisherListener *>(&m_listener));
    if (mp_publisher_target == nullptr) {
        std::cout << "Creating publisher failed" << std::endl;
        return false;
    }

    Wparam.topic.topicName = "rt/drone_path";
    mp_publisher_drone_path = Domain::createPublisher(mp_participant, Wparam, static_cast<PublisherListener *>(&m_listener));
    if (mp_publisher_drone_path == nullptr) {
        std::cout << "Creating publisher failed" << std::endl;
        return false;
    }

    Wparam.topic.topicName = "rt/insect_path";
    mp_publisher_insect_path = Domain::createPublisher(mp_participant, Wparam, static_cast<PublisherListener *>(&m_listener));
    if (mp_publisher_insect_path == nullptr) {
        std::cout << "Creating publisher failed" << std::endl;
        return false;
    }

    Wparam.topic.topicName = "rt/hunt_cone";
    mp_publisher_hunt_cone = Domain::createPublisher(mp_participant, Wparam, static_cast<PublisherListener *>(&m_listener));
    if (mp_publisher_hunt_cone == nullptr) {
        std::cout << "Creating publisher failed" << std::endl;
        return false;
    }

    Wparam.topic.topicName = "rt/flightarea";
    mp_publisher_flight_area = Domain::createPublisher(mp_participant, Wparam, static_cast<PublisherListener *>(&m_listener));
    if (mp_publisher_hunt_cone == nullptr) {
        std::cout << "Creating publisher failed" << std::endl;
        return false;
    }

    Wparam.topic.topicName = "rt/optimized_trajectory";
    mp_publisher_optimized_path = Domain::createPublisher(mp_participant, Wparam, static_cast<PublisherListener *>(&m_listener));
    if (mp_publisher_optimized_path == nullptr) {
        std::cout << "Creating publisher failed" << std::endl;
        return false;
    }

    return true;
}

void PathPublisher::PubListener::onPublicationMatched(Publisher *pub, MatchingInfo &info) {
    (void)pub;

    if (info.status == MATCHED_MATCHING) {
        n_matched++;
        std::cout << "Publisher matched" << std::endl;
    } else {
        n_matched--;
        std::cout << "Publisher unmatched" << std::endl;
    }
}


void PathPublisher::run(int64_t time) {
    nav_msgs::msg::Path st;
    nav_msgs::msg::Path st_hunt_cone;
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    std::vector<geometry_msgs::msg::PoseStamped> hunt_cone_poses;
    std_msgs::msg::Header header;
    builtin_interfaces::msg::Time t;

    t.sec(static_cast<int32_t>(time / 1e9l));
    t.nanosec(static_cast<uint32_t>(time % static_cast<int64_t>(1e9)));
    header.frame_id("pats_frame");
    header.stamp(t);

    st.poses(poses);
    st.header(header);
    mp_publisher_camera_cone->write(&st);

    st_hunt_cone.poses(hunt_cone_poses);
    st_hunt_cone.header(header);
    mp_publisher_hunt_cone->write(&st_hunt_cone);

    if (_drone_poses.size() > 200) {
        _drone_poses.erase(_drone_poses.begin(), _drone_poses.begin() + 50);
    }

    if (_insect_poses.size() > 200) {
        _insect_poses.erase(_insect_poses.begin(), _insect_poses.begin() + 50);
    }

    if (_proximity_poses.size() > 400) {
        _proximity_poses.erase(_proximity_poses.begin(), _proximity_poses.begin() + 50);
    }

    nav_msgs::msg::Path proximity_path;
    proximity_path.header(header);
    proximity_path.poses(_proximity_poses);
    mp_publisher_proximity->write(&proximity_path);

    nav_msgs::msg::Path drone_path;
    drone_path.header(header);
    drone_path.poses(_drone_poses);
    mp_publisher_drone_path->write(&drone_path);

    nav_msgs::msg::Path insect_path;
    insect_path.header(header);
    insect_path.poses(_insect_poses);
    mp_publisher_insect_path->write(&insect_path);

    nav_msgs::msg::Path target_path;
    target_path.header(header);
    target_path.poses(_target_poses);
    mp_publisher_target->write(&target_path);
    _target_poses.resize(0);

    nav_msgs::msg::Path flightarea_path;
    flightarea_path.header(header);
    flightarea_path.poses(_flightarea_poses);
    mp_publisher_flight_area->write(&flightarea_path);


    nav_msgs::msg::Path optimized_trajectory_path;
    optimized_trajectory_path.header(header);
    optimized_trajectory_path.poses(_optimized_path_poses);
    mp_publisher_optimized_path->write(&optimized_trajectory_path);
}

void PathPublisher::add_drone_pos(double x, double y, double z, int64_t time) {
    add_point(_drone_poses, x, y, z, time, "pats_frame");
    add_point(_proximity_poses, x, y, z, time, "pats_frame");
    add_point(_target_poses, x, y, z, time, "pats_frame");
}

void PathPublisher::add_insect_pos(double x, double y, double z, int64_t time) {
    add_point(_insect_poses, x, y, z, time, "pats_frame");
    add_point(_proximity_poses, x, y, z, time, "pats_frame");
}


void PathPublisher::add_point(std::vector<geometry_msgs::msg::PoseStamped> &poses, double x, double y, double z, int64_t time, std::string frame_id) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Point point;
    std_msgs::msg::Header header;
    builtin_interfaces::msg::Time t;

    point.x(x);
    point.y(y);
    point.z(z);
    pose.position(point);
    pose_stamped.pose(pose);

    t.sec(static_cast<int32_t>(time / 1e9l));
    t.nanosec(static_cast<uint32_t>(time % static_cast<int64_t>(1e9)));

    header.frame_id(frame_id);
    header.stamp(t);

    pose_stamped.header(header);
    poses.push_back(pose_stamped);
}

uint PathPublisher::add_plane(Plane plane, std::vector<CornerPoint> corner_points, int64_t time) {
    uint last_corner_point;
    for (uint corner_point_idx = 0; corner_point_idx < corner_points.size(); corner_point_idx++) {
        bool on_plane = false;
        for (auto plane_idx : corner_points.at(corner_point_idx).intersecting_planes) {
            if (plane_idx == plane.id)
                on_plane = true;
        }


        if (on_plane) {
            add_point(_flightarea_poses, static_cast<double>(corner_points.at(corner_point_idx).pos.x), static_cast<double>(corner_points.at(corner_point_idx).pos.y), static_cast<double>(corner_points.at(corner_point_idx).pos.z), time, "pats_frame");
            last_corner_point = corner_point_idx;

            for (uint corner_point_idx2 = 0; corner_point_idx2 < corner_points.size(); corner_point_idx2++) {
                bool on_plane2 = false;
                for (auto plane_idx2 : corner_points.at(corner_point_idx2).intersecting_planes) {
                    if (plane_idx2 == plane.id)
                        on_plane2 = true;
                }

                if (on_plane2) {
                    add_point(_flightarea_poses, static_cast<double>(corner_points.at(corner_point_idx2).pos.x), static_cast<double>(corner_points.at(corner_point_idx2).pos.y), static_cast<double>(corner_points.at(corner_point_idx2).pos.z), time, "pats_frame");
                }
            }
        }
    }
    add_point(_flightarea_poses, static_cast<double>(corner_points.at(last_corner_point).pos.x), static_cast<double>(corner_points.at(last_corner_point).pos.y), static_cast<double>(corner_points.at(last_corner_point).pos.z), time, "pats_frame");

    return last_corner_point;
}

// int PathPublisher::find_next_plane(Eigen::VectorXi plotted_planes, std::vector<CornerPoint> corner_points, uint corner_point_idx) {
//     for (auto plane_idx : corner_points.at(corner_point_idx).intersecting_planes) {
//         if (plotted_planes[plane_idx] == 0) {
//             return plane_idx;
//         }
//     }
//     return -1;
// }

void PathPublisher::update_flightarea(std::vector<Plane> planes, std::vector<CornerPoint> corner_points, int64_t time) {
    _flightarea_poses.clear();
    // int last_corner_point;
    // int last_plotted_plane;
    // int next_plane;
    // bool all_planes_plotted = false;
    // Eigen::VectorXi plotted_planes = Eigen::VectorXi(planes.size()).setZero();
    // plotted_planes[0] = 1;

    // last_corner_point = add_plane(planes.at(0), corner_points, time);

    for (auto plane : planes) {
        add_plane(plane, corner_points, time);

    }
    // while (!all_planes_plotted) {

    //     std::cout << plotted_planes.transpose() << std::endl;
    //     next_plane = find_next_plane(plotted_planes, corner_points, last_corner_point);

    //     if (next_plane >= 0) {
    //         plotted_planes[next_plane] = 1;
    //         last_plotted_plane = next_plane;
    //         last_corner_point = add_plane(planes.at(next_plane), corner_points, time);

    //         if (plotted_planes.sum() == planes.size())
    //             all_planes_plotted = true;
    //     } else {
    //         last_corner_point++;
    //     }
    // }
}


void PathPublisher::update_optimized_trajectory(std::vector<cv::Point3f> optimized_trajectory, int64_t time) {
    _optimized_path_poses.clear();

    // if (optimized_trajectory.size() > 0) {
    for (auto pnt : optimized_trajectory) {
        add_point(_optimized_path_poses, static_cast<double>(pnt.x), static_cast<double>(pnt.y), static_cast<double>(pnt.z), time, "pats_frame");
    }
    // }

}
