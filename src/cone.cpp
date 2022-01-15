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
#include "cone.hpp"
#include <sstream>

namespace ns_cone {
// Constructor
Cone::Cone(ros::NodeHandle &nh) : nh_(nh) {
};

PaddleDetection::ObjectDetector Cone::Detector_Init(const std::string& model_dir, 
                                                const std::string& device="CPU",
                                                bool use_mkldnn=false,
                                                int cpu_threads=1,
                                                const std::string& run_mode="fluid",
                                                const int batch_size=1,
                                                const int gpu_id=0,
                                                const int trt_min_shape=1,
                                                const int trt_max_shape=1280,
                                                const int trt_opt_shape=640,
                                                bool trt_calib_mode=false) {
                                                PaddleDetection::ObjectDetector Detector_(model_dir, 
                                                                                              device,
                                                                                              use_mkldnn,
                                                                                              cpu_threads,
                                                                                              run_mode,
                                                                                              batch_size,
                                                                                              gpu_id,
                                                                                              trt_min_shape,
                                                                                              trt_max_shape,
                                                                                              trt_opt_shape,
                                                                                              trt_calib_mode);
                                                return Detector_;
                                                }

// Getters
// fsd_common_msgs::ConeDetections Cone::getConeDetections() { return cone_current; }

// Setters
void Cone::set_model_file(std::string msg) {model_file = msg;}
void Cone::set_param_file(std::string msg) {param_file = msg;}
void Cone::set_Image(cv::Mat msg) {image_ = msg;}



void Cone::runAlgorithm() {
    if(!flag)
    return;

}
}
