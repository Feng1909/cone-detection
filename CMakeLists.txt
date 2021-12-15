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
find_package(OpenCV REQUIRED)
catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS
)

SET(PADDLE_DIR "/home/feng1909/cone-detection/src/cone-detection/paddle_inference")
message("PADDLE_DIR IS:" ${PADDLE_DIR})
SET(OPENCV_DIR ${OpenCV_LIBS})
SET(PADDLE_LIB_NAME "libpaddle_inference")
SET(CUDA_LIB "/usr/local/cuda/lib64")
SET(CUDNN_LIB "/usr/lib/x86_64-linux-gnu/")
SET(TENSORRT_INC_DIR "" CACHE PATH "Compile demo with TensorRT")
SET(TENSORRT_LIB_DIR "" CACHE PATH "Compile demo with TensorRT")
set(MKLDNN_PATH "${PADDLE_DIR}/third_party/install/mkldnn")
set(MKLDNN_LIB ${MKLDNN_PATH}/lib/libmkldnn.so.0)
set(WITH_GPU ON)

# Each node in the package must be declared like this
add_executable(${PROJECT_NAME}
  src/cone_handle.cpp
  src/cone.cpp
  src/main.cpp
  )    
  # set(SRCS src/preprocess_op.cc src/object_detector.cc src/picodet_postprocess.cc src/utils.cc)



include(cmake/yaml-cpp.cmake)

include_directories("${CMAKE_SOURCE_DIR}/")
include_directories("${CMAKE_CURRENT_BINARY_DIR}/ext/yaml-cpp/src/ext-yaml-cpp/include")
link_directories("${CMAKE_CURRENT_BINARY_DIR}/ext/yaml-cpp/lib")

macro(safe_set_static_flag)
    foreach(flag_var
        CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
        CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO)
      if(${flag_var} MATCHES "/MD")
        string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
      endif(${flag_var} MATCHES "/MD")
    endforeach(flag_var)
endmacro()

include_directories("${CMAKE_SOURCE_DIR}/")
include_directories("${PADDLE_DIR}/")
include_directories("${PADDLE_DIR}/third_party/install/protobuf/include")
include_directories("${PADDLE_DIR}/third_party/install/glog/include")
include_directories("${PADDLE_DIR}/third_party/install/gflags/include")
include_directories("${PADDLE_DIR}/third_party/install/xxhash/include")
if (EXISTS "${PADDLE_DIR}/third_party/install/snappy/include")
    include_directories("${PADDLE_DIR}/third_party/install/snappy/include")
endif()
if(EXISTS "${PADDLE_DIR}/third_party/install/snappystream/include")
    include_directories("${PADDLE_DIR}/third_party/install/snappystream/include")
endif()
include_directories("${PADDLE_DIR}/third_party/boost")
include_directories("${PADDLE_DIR}/third_party/eigen3")

if (EXISTS "${PADDLE_DIR}/third_party/install/snappy/lib")
    link_directories("${PADDLE_DIR}/third_party/install/snappy/lib")
endif()
if(EXISTS "${PADDLE_DIR}/third_party/install/snappystream/lib")
    link_directories("${PADDLE_DIR}/third_party/install/snappystream/lib")
endif()

link_directories("${PADDLE_DIR}/third_party/install/protobuf/lib")
link_directories("${PADDLE_DIR}/third_party/install/glog/lib")
link_directories("${PADDLE_DIR}/third_party/install/gflags/lib")
link_directories("${PADDLE_DIR}/third_party/install/xxhash/lib")
link_directories("${PADDLE_DIR}/paddle/lib/")
link_directories("${CMAKE_CURRENT_BINARY_DIR}")



if (WIN32)
  include_directories("${PADDLE_DIR}/paddle/fluid/inference")
  include_directories("${PADDLE_DIR}/paddle/include")
  link_directories("${PADDLE_DIR}/paddle/fluid/inference")
  find_package(OpenCV REQUIRED PATHS ${OPENCV_DIR}/build/ NO_DEFAULT_PATH)

else ()
  find_package(OpenCV REQUIRED PATHS ${OPENCV_DIR}/share/OpenCV NO_DEFAULT_PATH)
  include_directories("${PADDLE_DIR}/paddle/include")
  link_directories("${PADDLE_DIR}/paddle/lib")
endif ()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -o2 -fopenmp -std=c++11")
set(CMAKE_STATIC_LIBRARY_PREFIX "")

if (NOT WIN32)
    set(DEPS ${DEPS}
        ${MATH_LIB} ${MKLDNN_LIB}
        glog gflags protobuf z xxhash yaml-cpp
        )
    if(EXISTS "${PADDLE_DIR}/third_party/install/snappystream/lib")
        set(DEPS ${DEPS} snappystream)
    endif()
    if (EXISTS "${PADDLE_DIR}/third_party/install/snappy/lib")
        set(DEPS ${DEPS} snappy)
    endif()
else()
    set(DEPS ${DEPS}
        ${MATH_LIB} ${MKLDNN_LIB}
        glog gflags_static libprotobuf xxhash libyaml-cppmt)
    set(DEPS ${DEPS} libcmt shlwapi)
    if (EXISTS "${PADDLE_DIR}/third_party/install/snappy/lib")
        set(DEPS ${DEPS} snappy)
    endif()
    if(EXISTS "${PADDLE_DIR}/third_party/install/snappystream/lib")
        set(DEPS ${DEPS} snappystream)
    endif()
endif(NOT WIN32)

if(WITH_GPU)
  if(NOT WIN32)
    if (WITH_TENSORRT)
	    set(DEPS ${DEPS} ${TENSORRT_LIB_DIR}/libnvinfer${CMAKE_SHARED_LIBRARY_SUFFIX})
	    set(DEPS ${DEPS} ${TENSORRT_LIB_DIR}/libnvinfer_plugin${CMAKE_SHARED_LIBRARY_SUFFIX})
    endif()
    set(DEPS ${DEPS} ${CUDA_LIB}/libcudart${CMAKE_SHARED_LIBRARY_SUFFIX})
    set(DEPS ${DEPS} ${CUDNN_LIB}/libcudnn${CMAKE_SHARED_LIBRARY_SUFFIX})
  else()
    set(DEPS ${DEPS} ${CUDA_LIB}/cudart${CMAKE_STATIC_LIBRARY_SUFFIX} )
    set(DEPS ${DEPS} ${CUDA_LIB}/cublas${CMAKE_STATIC_LIBRARY_SUFFIX} )
    set(DEPS ${DEPS} ${CUDNN_LIB}/cudnn${CMAKE_STATIC_LIBRARY_SUFFIX})
  endif()
endif()

set(EXTERNAL_LIB "-ldl -lrt -lgomp -lz -lm -lpthread")
set(DEPS ${DEPS} ${EXTERNAL_LIB})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  "${MKLDNN_PATH}/include"
)


target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${OpenCV_LIBS}
  ${PADDLE_DIR}/paddle/lib/${PADDLE_LIB_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX}  
  ${PADDLE_DIR}/third_party/install/mklml/lib/libiomp5${CMAKE_SHARED_LIBRARY_SUFFIX}
  ${MKLDNN_PATH}/lib/libmkldnn.so.0
)