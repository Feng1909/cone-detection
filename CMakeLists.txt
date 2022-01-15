cmake_minimum_required(VERSION 2.8.3)
project(cone)

set(CMAKE_BUILD_TYPE "Debug")
add_compile_options(-std=c++11)

set(PROJECT_DEPS
  roscpp
  std_msgs
  fsd_common_msgs
  )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  fsd_common_msgs
  image_transport
  cv_bridge
  pcl_ros
  pcl_conversions
)
find_package(OpenCV REQUIRED PATHS ${OpenCV_LIBS}/share/OpenCV NO_DEFAULT_PATH)
catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS
)
link_directories(/usr/local/lib)

include_directories(/usr/local/include/yaml-cpp)

FIND_LIBRARY(YAML_CPP_LIBRARIES yaml-cpp)


SET(PADDLE_DIR "${CMAKE_SOURCE_DIR}/paddle_inference")
SET(PADDLE_LIB_NAME "libpaddle_inference")
SET(CUDA_LIB "/usr/local/cuda/lib64")
SET(CUDNN_LIB "/usr/lib/x86_64-linux-gnu/")
set(TENSORRT_ROOT "/home/feng1909/Documents/TensorRT-7.1.3.4")
set(TENSORRT_INCLUDE_DIR ${TENSORRT_ROOT}/include)
set(TENSORRT_LIB_DIR ${TENSORRT_ROOT}/lib)
set(MKLDNN_PATH "${PADDLE_DIR}/third_party/install/mkldnn")
set(MKLDNN_LIB ${MKLDNN_PATH}/lib/libmkldnn.so.0)
set(WITH_GPU ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -o2 -fopenmp -std=c++11")
set(DEPS ${DEPS}
    ${MATH_LIB} ${MKLDNN_LIB}
    glog gflags protobuf z xxhash
    )
set(DEPS ${DEPS} ${TENSORRT_LIB_DIR}/libnvinfer${CMAKE_SHARED_LIBRARY_SUFFIX})
set(DEPS ${DEPS} ${TENSORRT_LIB_DIR}/libnvinfer_plugin${CMAKE_SHARED_LIBRARY_SUFFIX})
set(DEPS ${DEPS} ${CUDA_LIB}/libcudart${CMAKE_SHARED_LIBRARY_SUFFIX})
set(DEPS ${DEPS} ${CUDNN_LIB}/libcudnn${CMAKE_SHARED_LIBRARY_SUFFIX})
set(EXTERNAL_LIB "-ldl -lrt -lgomp -lz -lm -lpthread")
set(DEPS ${DEPS} ${EXTERNAL_LIB})
    
include_directories("${CMAKE_SOURCE_DIR}/")
include_directories("${CMAKE_SOURCE_DIR}/")
include_directories("${PADDLE_DIR}/")
include_directories("${PADDLE_DIR}/third_party/install/protobuf/include")
include_directories("${PADDLE_DIR}/third_party/install/glog/include")
include_directories("${PADDLE_DIR}/third_party/install/gflags/include")
include_directories("${PADDLE_DIR}/third_party/install/xxhash/include")
include_directories("${PADDLE_DIR}/third_party/boost")
include_directories("${PADDLE_DIR}/third_party/eigen3")
include_directories("${PADDLE_DIR}/paddle/include")

link_directories("${PADDLE_DIR}/third_party/install/protobuf/lib")
link_directories("${PADDLE_DIR}/third_party/install/glog/lib")
link_directories("${PADDLE_DIR}/third_party/install/gflags/lib")
link_directories("${PADDLE_DIR}/third_party/install/xxhash/lib")
link_directories("${PADDLE_DIR}/paddle/lib/")
link_directories("${CMAKE_CURRENT_BINARY_DIR}")
link_directories("${PADDLE_DIR}/paddle/lib")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  "${MKLDNN_PATH}/include"
)

# Each node in the package must be declared like this
add_executable(${PROJECT_NAME}
  src/cone_handle.cpp
  src/cone.cpp
  src/main.cpp
  src/preprocess_op.cc
  src/object_detector.cc
  src/picodet_postprocess.cc
  src/utils.cc
  )  

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${OpenCV_LIBS}
  ${PADDLE_DIR}/paddle/lib/${PADDLE_LIB_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX}  
  ${PADDLE_DIR}/third_party/install/mklml/lib/libiomp5${CMAKE_SHARED_LIBRARY_SUFFIX}
  ${MKLDNN_PATH}/lib/libmkldnn.so.0
  ${YAML_CPP_LIBRARIES}
)