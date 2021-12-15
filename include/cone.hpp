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

#ifndef CONE_HPP
#define CONE_HPP

#include "fsd_common_msgs/ConeDetections.h"
#include "std_msgs/String.h"
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
// Paddle
#include "paddle_inference_api.h"
#include <chrono>
#include <iostream>
#include <memory>
#include <numeric>
#include <gflags/gflags.h>
#include <glog/logging.h>

using paddle_infer::Config;
using paddle_infer::Predictor;
using paddle_infer::CreatePredictor;
using paddle_infer::PrecisionType;

namespace ns_cone {

class Cone {

 public:
  // Constructor
  Cone(ros::NodeHandle &nh);

  // Getters
  // fsd_common_msgs::ConeDetections getConeDetections();

  // Setters
  // void setConeDetections(fsd_common_msgs::ConeDetections cones);
  void set_model_file(std::string msg);
  void set_param_file(std::string msg);
  void set_Image(cv::Mat msg);

  void runAlgorithm();
  bool flag;

 private:

  ros::NodeHandle &nh_;

  std::string model_file;
  std::string param_file;
  std::shared_ptr<Predictor>InitPredictor();
void run(Predictor *predictor, const std::vector<float> &input,
         const std::vector<int> &input_shape, const std::vector<float> &input_im,
         const std::vector<int> &input_im_shape, std::vector<float> *out_data);
  fsd_common_msgs::ConeDetections cone_current;

  std::shared_ptr<Predictor> predictor;
  std::vector<int> input_im_shape;
  std::vector<float> out_data;
  const int height = 1920;
  const int width = 1180;
  const int channels = 3;
  std::vector<int>input_shape;
  cv::Mat image_;

};
}

#endif //CONE_HPP
