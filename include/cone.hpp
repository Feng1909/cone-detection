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
#include "object_detector.h"
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

  // PaddleDetection::ObjectDetector objector_;

 private:

  ros::NodeHandle &nh_;

  std::string model_file;
  std::string param_file;

  PaddleDetection::ObjectDetector Detector_Init(const std::string& model_dir, 
                                                const std::string& device,
                                                bool use_mkldnn,
                                                int cpu_threads,
                                                const std::string& run_mode,
                                                const int batch_size,
                                                const int gpu_id,
                                                const int trt_min_shape,
                                                const int trt_max_shape,
                                                const int trt_opt_shape,
                                                bool trt_calib_mode);

  // const std::string& model_dir, 
  //                         const std::string& device="CPU",
  //                         bool use_mkldnn=false,
  //                         int cpu_threads=1,
  //                         const std::string& run_mode="fluid",
  //                         const int batch_size=1,
  //                         const int gpu_id=0,
  //                         const int trt_min_shape=1,
  //                         const int trt_max_shape=1280,
  //                         const int trt_opt_shape=640,
  //                         bool trt_calib_mode=false
  cv::Mat image_;

};
}

#endif //CONE_HPP
