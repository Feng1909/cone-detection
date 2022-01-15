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
std::string model_dir = "/home/feng1909/back/src/cone-detection/model";
std::string device = "GPU";
bool use_mkldnn = false;
int cpu_threads = 1;
// std::string run_mode = "fluid";
std::string run_mode = "trt_fp16";
int batch_size = 1;
int gpu_id = 0;
int trt_min_shape = 1;
int trt_max_shape = 1280;
int trt_opt_shape = 480;
bool trt_calib_mode = false;

PaddleDetection::ObjectDetector objector_(  model_dir,  device, use_mkldnn, cpu_threads, run_mode,
                                            batch_size, gpu_id, trt_min_shape, trt_max_shape,
                                            trt_opt_shape, trt_calib_mode);

// Getters
// fsd_common_msgs::ConeDetections Cone::getConeDetections() { return cone_current; }

// Setters
void Cone::set_model_file(std::string msg) {model_file = msg;}
void Cone::set_param_file(std::string msg) {param_file = msg;}
void Cone::set_Image(cv::Mat msg) {image_ = msg;}



void Cone::runAlgorithm() {
    if(!flag)
    return;

    double threshold = 0.5;
    std::vector<cv::Mat> batch_imgs;
    std::vector<PaddleDetection::ObjectResult> result;
    std::vector<int> bbox_num;
    std::vector<double> det_times;
    std::cout<<"yeah"<<std::endl;
    batch_imgs.clear();
    batch_imgs.insert(batch_imgs.end(), image_);
    objector_.Predict(batch_imgs, threshold, 1, 1, &result, &bbox_num,  &det_times);
    std::cout<<"yeah end"<<std::endl;
    // objector_.
    for (int i=0; i<result.size(); i++) {
        std::cout<<result[i].rect[0]<<" "<<result[i].rect[1]<<" "<<result[i].rect[2]<<" "<<result[i].rect[3]<<std::endl;
    }
}
}
