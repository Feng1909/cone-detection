/*
    Formula Student Driverless Project (FSD-Project).
    Copyright (c) 2019:
     - chentairan <killasipilin@gmail.com>

    FSD-Project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    FSD-Project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with FSD-Project.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include "cone_handle.hpp"
#include "register.h"
#include <chrono>

namespace ns_cone {

// Constructor
ConeHandle::ConeHandle(ros::NodeHandle &nodeHandle) :
    nodeHandle_(nodeHandle),
    cone_(nodeHandle) {
  ROS_INFO("Constructing Handle");
  loadParameters();
  subscribeToTopics();
  publishToTopics();
}

// Getters
int ConeHandle::getNodeRate() const { return node_rate_; }

// Methods
void ConeHandle::loadParameters() {
  ROS_INFO("loading handle parameters");
  if (!nodeHandle_.param<std::string>("vision_cone_detections_topic_name",
                                      vision_cone_detections_topic_name_,
                                      "/perception/vision/cone_detections")) {
    ROS_WARN_STREAM(
        "Did not load vision_cone_detections_topic_name. Standard value is: " << vision_cone_detections_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("cone_state_topic_name",
                                      cone_state_topic_name_,
                                      "/perception/vision/state")) {
    ROS_WARN_STREAM("Did not load cone_state_topic_name. Standard value is: " << cone_state_topic_name_);
  }
  if (!nodeHandle_.param("node_rate", node_rate_, 1)) {
    ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
  }
}

void ConeHandle::subscribeToTopics() {
  ROS_INFO("subscribe to topics");
  // visionConeDetectionsSubscriber_ =
  //     nodeHandle_.subscribe(vision_cone_detections_topic_name_, 1, &ConeHandle::visionConeDetectionsCallback, this);
}

void ConeHandle::publishToTopics() {
  ROS_INFO("publish to topics");
  // coneStatePublisher_ = nodeHandle_.advertise<fsd_common_msgs::ConeDetections>(cone_state_topic_name_, 1);
}

void ConeHandle::run() {
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  cone_.runAlgorithm();
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  double time_round = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
  std::cout << "time cost = " << time_round << ", frequency = " << 1 / time_round << std::endl;
  sendMsg();
}

void ConeHandle::sendMsg() {
  // coneStatePublisher_.publish(cone_.getConeDetections());
}

// void ConeHandle::visionConeDetectionsCallback(const fsd_common_msgs::ConeDetections &cones) {
//   cone_.setConeDetections(cones);
// }
}