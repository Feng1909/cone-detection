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

#include "fsd_common_msgs/ConeDetections.h"
#include "cone.hpp"
// ROS
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
// OpenCv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

using TwoCameraPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>;

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

  // Initialize publisher and subscriber.
  std::string cameraTopicName1;
  std::string cameraTopicName2;
  std::string image_detection_topic_name_;
  std::string model_path_;
  std::string param_path_;

  ros::NodeHandle nodeHandle_;
  ros::Subscriber visionConeDetectionsSubscriber_;

  //! Advertise and subscribe to image topics.
  image_transport::ImageTransport imageTransport_;
  //! ROS subscriber and publisher.
  image_transport::SubscriberFilter imageSubscriber1_;
  image_transport::SubscriberFilter imageSubscriber2_;
  std::unique_ptr<message_filters::Synchronizer<TwoCameraPolicy>> sync_;

  ros::Publisher coneStatePublisher_;
  ros::Publisher ImageDetectorPublisher_;

  // void visionConeDetectionsCallback(const fsd_common_msgs::ConeDetections &cones);
  void cameraCallback(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg2);

  int node_rate_;

  //! Camera related parameters.
  int frameWidth_;
  int frameHeight_;
  cv::Mat camImageCopy_;
  std_msgs::Header imageHeader_;

  bool imageStatus_ = false;
  boost::shared_mutex mutexImageStatus_;
  boost::shared_mutex mutexImageCallback_;

  Cone cone_;

};
}

#endif //CONE_HANDLE_HPP
