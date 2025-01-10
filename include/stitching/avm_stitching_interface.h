#ifndef __AVM_STITCHING_INTERFACE_H__
#define __AVM_STITCHING_INTERFACE_H__

#include <iostream>
#include <memory>
#include <string>
#include <boost/filesystem.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <filesystem>

#include "avm_stitching.h"

class AVMStitchingInterface
{
public:
    AVMStitchingInterface();
    ~AVMStitchingInterface();
    cv::Mat AVMStitching(const std::map<std::string, cv::Mat>& image_map);

private:
    bool                                   initialized_ = true;
    std::vector<std::string>               cameraList_;
    std::string                            cameraParamFile_;
    std::string                            calibResultFile_;
    std::string                            logFolder_;
    std::string                            main_folder_;
    std::shared_ptr<AvmStitching>          stitching_;
    AvmStitchingParams                     stithcing_params_;
    std::map<std::string, CameraInfo>      camInfoMap_;
    double                                 carHeight_;
    std::map<std::string, Eigen::Matrix4d> camInCarT_;
    std::string                            data_folder_;
};

#endif // __AVM_STITCHING_INTERFACE_H__