// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*! 
 * @file PathPublisher.h
 * This header file contains the declaration of the publisher functions.
 *
 * This file was generated by the tool fastcdrgen.
 */


#ifndef _NAV_MSGS_MSG_PATH_PUBLISHER_H_
#define _NAV_MSGS_MSG_PATH_PUBLISHER_H_

#include <fastrtps/fastrtps_fwd.h>
#include <fastrtps/publisher/PublisherListener.h>

#include "PathPubSubTypes.h"

#include "common.h"

class PathPublisher 
{
public:
	PathPublisher();
	virtual ~PathPublisher();
    bool init(CameraVolume *cam_volume);
    void run(int64_t time);
    void add_drone_pos(double x, double y, double z, int64_t time);
    void add_insect_pos(double x, double y, double z, int64_t time);
    void set_target(double x, double y, double z, int64 time);

private:
    void add_camera_cone(std::vector<geometry_msgs::msg::PoseStamped> &poses, int64 time);
    void add_point(std::vector<geometry_msgs::msg::PoseStamped> &poses, double x, double y, double z, int64_t time, std::string frame_id);
    void add_hunt_cone(std::vector<geometry_msgs::msg::PoseStamped> &poses, int64 time);
    void add_camera_cone(std::vector<geometry_msgs::msg::PoseStamped> &poses, cv::Mat point, int64 time, std::string frame_id);

    builtin_interfaces::msg::Time _time;

    CameraVolume *_cam_volume;

    std::vector<geometry_msgs::msg::PoseStamped> _drone_poses;
    std::vector<geometry_msgs::msg::PoseStamped> _insect_poses;
    std::vector<geometry_msgs::msg::PoseStamped> _proximity_poses;
    std::vector<geometry_msgs::msg::PoseStamped> _target_poses;

	eprosima::fastrtps::Participant *mp_participant;
        eprosima::fastrtps::Publisher *mp_publisher_proximity;
        eprosima::fastrtps::Publisher *mp_publisher_camera_cone;
        eprosima::fastrtps::Publisher *mp_publisher_safe_zone;
        eprosima::fastrtps::Publisher *mp_publisher_target;
        eprosima::fastrtps::Publisher *mp_publisher_drone_path;
        eprosima::fastrtps::Publisher *mp_publisher_insect_path;
        eprosima::fastrtps::Publisher *mp_publisher_hunt_cone;

	class PubListener : public eprosima::fastrtps::PublisherListener
	{
	public:
		PubListener() : n_matched(0){};
		~PubListener(){};
		void onPublicationMatched(eprosima::fastrtps::Publisher* pub,eprosima::fastrtps::rtps::MatchingInfo& info);
		int n_matched;
	} m_listener;
	nav_msgs::msg::PathPubSubType myType;
};

#endif // _NAV_MSGS_MSG_PATH_PUBLISHER_H_
