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

#ifndef CONE_HANDLE_HPP
#define CONE_HANDLE_HPP

// #include "fsd_common_msgs/ConeDetections.h"
#include "cone.hpp"

namespace ns_cone {

class ConeHandle {

 public:
  // Constructor
  ConeHandle(ros::NodeHandle &nodeHandle);

  // Getters
  int getNodeRate() const;

  // Methods
  void loadParameters();
  void subscribeToTopics();
  void publishToTopics();
  void run();
  void sendMsg();
  // void sendVisualization();

 private:
  ros::NodeHandle nodeHandle_;
  ros::Subscriber visionConeDetectionsSubscriber_;
  ros::Publisher coneStatePublisher_;

  // void visionConeDetectionsCallback(const fsd_common_msgs::ConeDetections &cones);

  std::string vision_cone_detections_topic_name_;
  std::string cone_state_topic_name_;

  int node_rate_;

  Cone cone_;

};
}

#endif //CONE_HANDLE_HPP
