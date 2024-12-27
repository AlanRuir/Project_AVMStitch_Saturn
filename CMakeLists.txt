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

# find ffmpeg
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)  
find_package(FFmpeg REQUIRED COMPONENTS AVCODEC AVFORMAT AVUTIL SWSCALE) 

# find cuda
find_package(CUDAToolkit REQUIRED)
enable_language(CUDA)
include(CheckLanguage)
check_language(CUDA)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/common
  ${PROJECT_SOURCE_DIR}/include/video
  ${PROJECT_SOURCE_DIR}/include/decode
  ${FFMPEG_INCLUDE_DIRS_AVCODEC} 
  ${FFMPEG_INCLUDE_DIRS_AVFORMAT} 
  ${FFMPEG_INCLUDE_DIRS_AVUTIL} 
  ${FFMPEG_INCLUDE_DIRS_SWSCALE} 
  ${LIBYUV_INCLUDE_DIRS})

add_executable(avmstitch 
  ${PROJECT_SOURCE_DIR}/src/video/rtsp_client.cpp
  ${PROJECT_SOURCE_DIR}/src/avmstitch_node.cpp
  ${PROJECT_SOURCE_DIR}/src/decode/video_decoder_h26x_base.cpp
  ${PROJECT_SOURCE_DIR}/src/decode/video_decoder_h26x_cpu.cpp
  ${PROJECT_SOURCE_DIR}/src/decode/video_decoder_h26x_cuda.cpp
  )

target_include_directories(
  avmstitch 
  PRIVATE
  ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
)

link_directories(
  /usr/local/lib
)

ament_target_dependencies(avmstitch rclcpp)
# 显式链接FFmpeg库
target_link_libraries(
  avmstitch 
  ${FFmpeg_LIBRARIES}
  CUDA::cudart)

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
