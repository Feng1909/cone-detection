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
    imageTransport_(nodeHandle_),
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
  // if (!nodeHandle_.param<std::string>("vision_cone_detections_topic_name",
  //                                     vision_cone_detections_topic_name_,
  //                                     "/perception/vision/cone_detections")) {
  //   ROS_WARN_STREAM(
  //       "Did not load vision_cone_detections_topic_name. Standard value is: " << vision_cone_detections_topic_name_);
  // }
  if (!nodeHandle_.param<std::string>("camera_left_name",
                                      cameraTopicName1,
                                      "/pylon_camera_old/image_raw")) {
    ROS_WARN_STREAM(
        "Did not load camera_left_name. Standard value is: " << cameraTopicName1);
  }
  if (!nodeHandle_.param<std::string>("camera_right_name",
                                      cameraTopicName2,
                                      "/pylon_camera_old/image_raw")) {
    ROS_WARN_STREAM(
        "Did not load camera_right_name. Standard value is: " << cameraTopicName2);
  }
  if (!nodeHandle_.param<std::string>("image_detection_topic_name",
                                      image_detection_topic_name_,
                                      "/detection/receive")) {
    ROS_WARN_STREAM(
        "Did not load image_detection_topic_name. Standard value is: " << image_detection_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("model_path",
                                      model_path_,
                                      "./model/model.pdmodel")) {
    ROS_WARN_STREAM(
        "Did not load model_path. Standard value is: " << model_path_);
  }if (!nodeHandle_.param<std::string>("param_path",
                                      param_path_,
                                      "./model/model.pdiparams")) {
    ROS_WARN_STREAM(
        "Did not load param_path. Standard value is: " << param_path_);
  }
  if (!nodeHandle_.param("node_rate", node_rate_, 1)) {
    ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
  }

  cone_.set_model_file(model_path_);
  cone_.set_param_file(param_path_);
}

void ConeHandle::subscribeToTopics() {
  ROS_INFO("subscribe to topics");
  imageSubscriber1_.subscribe(imageTransport_, cameraTopicName1, 6);
  imageSubscriber2_.subscribe(imageTransport_, cameraTopicName2, 6);
  TwoCameraPolicy camera_policy(10);
  camera_policy.setMaxIntervalDuration(ros::Duration(0.2));
  sync_.reset(new message_filters::Synchronizer<TwoCameraPolicy>(
      static_cast<const TwoCameraPolicy &>(camera_policy),
      imageSubscriber1_, imageSubscriber2_));
  sync_->registerCallback(boost::bind(&ConeHandle::cameraCallback, this, _1, _2));

}

void ConeHandle::publishToTopics() {
  ROS_INFO("publish to topics");
  ImageDetectorPublisher_ = nodeHandle_.advertise<sensor_msgs::Image>(image_detection_topic_name_, 1);
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

void ConeHandle::cameraCallback(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg2) {
  ROS_DEBUG("[ConeHandle] USB image received.");
  std::cout << "\nget images1 at: " << msg1->header.stamp << std::endl;
  std::cout << "get images2 at: "<<msg2->header.stamp << std::endl;
  cv_bridge::CvImagePtr cam_image1;
  cv_bridge::CvImagePtr cam_image2;

  try {
    cam_image1 = cv_bridge::toCvCopy(msg1, sensor_msgs::image_encodings::BGR8);
    cam_image2 = cv_bridge::toCvCopy(msg2, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (cam_image1 && cam_image2) {
    {
      boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
      imageHeader_ = msg1->header;
      cv::vconcat(cam_image1->image.clone(),cam_image2->image.clone(),camImageCopy_);
      cone_.set_Image(camImageCopy_);
      cone_.flag = true;
    }
    {
      boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_);
      imageStatus_ = true;
    }
    frameWidth_ = cam_image1->image.size().width;
    frameHeight_ = cam_image1->image.size().height + cam_image2->image.size().height;
  }

  cv_bridge::CvImage cvImage;
  cvImage.header.stamp = ros::Time::now();
  cvImage.encoding = sensor_msgs::image_encodings::BGR8;
  cvImage.image = camImageCopy_;
  ImageDetectorPublisher_.publish(*cvImage.toImageMsg());

  return;
}

}