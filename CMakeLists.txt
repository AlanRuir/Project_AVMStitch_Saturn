cmake_minimum_required(VERSION 3.8)
project(avmstitch LANGUAGES CUDA CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED filesystem system regex)

# find ffmpeg
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)  
find_package(FFmpeg REQUIRED COMPONENTS AVCODEC AVFORMAT AVUTIL SWSCALE) 

# find libyuv
find_package(libyuv REQUIRED)

# find cuda
find_package(CUDAToolkit REQUIRED)
enable_language(CUDA)
include(CheckLanguage)
check_language(CUDA)

# find jsoncpp
find_package(jsoncpp REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/common
  ${PROJECT_SOURCE_DIR}/include/video
  ${PROJECT_SOURCE_DIR}/include/decode
  ${PROJECT_SOURCE_DIR}/include/encode
  ${PROJECT_SOURCE_DIR}/include/stitching
  ${PROJECT_SOURCE_DIR}/include/image
  ${PROJECT_SOURCE_DIR}/include/stream
  ${FFMPEG_INCLUDE_DIRS_AVCODEC} 
  ${FFMPEG_INCLUDE_DIRS_AVFORMAT} 
  ${FFMPEG_INCLUDE_DIRS_AVUTIL} 
  ${FFMPEG_INCLUDE_DIRS_SWSCALE} 
  ${LIBYUV_INCLUDE_DIRS}

# append
  ${OpenCV_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR})

add_executable(avmstitch 
  ${PROJECT_SOURCE_DIR}/src/video/rtsp_client.cpp
  ${PROJECT_SOURCE_DIR}/src/avmstitch_node.cpp
  ${PROJECT_SOURCE_DIR}/src/decode/video_decoder_h26x_base.cpp
  ${PROJECT_SOURCE_DIR}/src/decode/video_decoder_h26x_cpu.cpp
  ${PROJECT_SOURCE_DIR}/src/decode/video_decoder_h26x_cuda.cpp
  ${PROJECT_SOURCE_DIR}/src/encode/video_encoder_h26x_base.cpp
  ${PROJECT_SOURCE_DIR}/src/encode/video_encoder_h26x_cpu.cpp
  ${PROJECT_SOURCE_DIR}/src/encode/video_encoder_h26x_cuda.cpp
  ${PROJECT_SOURCE_DIR}/src/common/image.cpp
  ${PROJECT_SOURCE_DIR}/src/stream/video_push.cpp
  ${PROJECT_SOURCE_DIR}/src/stitching/avm_stitching.cpp
  ${PROJECT_SOURCE_DIR}/src/stitching/avm_stitching_interface.cpp
  )

target_include_directories(
  avmstitch 
  PRIVATE
  ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
)

link_directories(
  /usr/local/lib
)

ament_target_dependencies(avmstitch rclcpp OpenCV sensor_msgs)
# 显式链接FFmpeg库
target_link_libraries(
  avmstitch 
  ${FFmpeg_LIBRARIES}
  CUDA::cudart
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${LIBYUV_LIBRARY}
  jsoncpp_lib)

target_link_directories(avmstitch PRIVATE ${CUDART_LIBRARY})

install(TARGETS
avmstitch

  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
