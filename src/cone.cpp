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
    predictor = InitPredictor();
    input_im_shape = {1, 2};
    input_shape = {1, channels, height, width};
};

std::shared_ptr<Predictor> Cone::InitPredictor() {
  Config config;
  config.SetModel("/home/feng1909/cone-detection/src/cone-detection/model/model.pdmodel", "/home/feng1909/cone-detection/src/cone-detection/model/model.pdiparams");
  config.EnableUseGpu(1000, 0);
  config.EnableMemoryOptim();
  config.EnableTensorRtEngine(1 << 30, 1, 10, PrecisionType::kFloat32, false, false);
  return CreatePredictor(config);
}

// Getters
// fsd_common_msgs::ConeDetections Cone::getConeDetections() { return cone_current; }

// Setters
void Cone::set_model_file(std::string msg) {model_file = msg;}
void Cone::set_param_file(std::string msg) {param_file = msg;}
void Cone::set_Image(cv::Mat msg) {image_ = msg;}

void Cone::run(Predictor *predictor, const std::vector<float> &input,
         const std::vector<int> &input_shape, const std::vector<float> &input_im,
         const std::vector<int> &input_im_shape, std::vector<float> *out_data) {
  auto input_names = predictor->GetInputNames();
  auto im_shape_handle = predictor->GetInputHandle(input_names[0]);
  im_shape_handle->Reshape(input_im_shape);
  im_shape_handle->CopyFromCpu(input_im.data());
//   std::cout<<"1"<<std::endl;
  auto image_handle = predictor->GetInputHandle(input_names[1]);
  image_handle->Reshape(input_shape);
  image_handle->CopyFromCpu(input_im.data());
//   std::cout<<"2"<<std::endl;

  auto scale_factor_handle = predictor->GetInputHandle(input_names[2]);
  scale_factor_handle->Reshape(input_im_shape);
  scale_factor_handle->CopyFromCpu(input_im.data());
//   std::cout<<"3"<<std::endl;

//   CHECK(predictor->Run());
  try{predictor->Run();}catch (cv_bridge::Exception e) {
    return;
  }
//   std::cout<<"4"<<std::endl;

  auto output_names = predictor->GetOutputNames();
  auto output_t = predictor->GetOutputHandle(output_names[0]);
  std::vector<int> output_shape = output_t->shape();
  int out_num = std::accumulate(output_shape.begin(), output_shape.end(), 1,
                                std::multiplies<int>());

  out_data->resize(out_num);
  output_t->CopyToCpu(out_data->data());
}

void Cone::runAlgorithm() {
    if(!flag)
    return;
    std::vector<float>input_data(1 * channels * height * width);
    std::vector<float> input_im_data(1 * 2, 608);
    cv::Mat im = image_.clone();
    // cv::cvtColor(im, im, cv::COLOR_BGR2RGB);
    std::cout<<im.size()<<std::endl;
    // for(int i = 1; i<=)
    int row = im.rows;
    int col = im.cols;
    for (int t=0;t<channels;t++)
    for (int i=0;i<row;i++)
        for (int j=0;j<col;j++)
            input_data[t*row*col+j*row+i] = im.at<uchar>(i, j);
            
    for (size_t i = 0;i<input_data.size();i++){
        input_data[i] = i % 255 * 0.13f;
    }

    std::cout<<"yeah"<<std::endl;
    run(predictor.get(), input_data, input_shape, 
        input_im_data, input_im_shape, &out_data);

    std::cout<<"number: "<<out_data.size()<<std::endl;
    flag = false;

}
}
