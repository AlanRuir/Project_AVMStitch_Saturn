#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <rclcpp/rclcpp.hpp>
#include <boost/regex.hpp>
#include <filesystem>
#include <opencv2/imgproc/types_c.h>
#include <atomic>
#include <thread>

struct CameraInfo
{
    cv::Mat cameramat;
    cv::Mat distcoeff;
    int width;
    int height;
    std::string model;
};

inline Eigen::Vector3d getEulerYPRFromRotationMatrix(const Eigen::Matrix3d& R) {
    Eigen::Vector3d eulerYPR; // yaw, pitch, roll

    // Check for gimbal lock (pitch near ±90 degrees)
    if (std::abs(R(2, 0)) < 1 - 1e-6) {
        // General case
        eulerYPR(0) = std::atan2(R(1, 0), R(0, 0)); // Yaw
        eulerYPR(1) = std::asin(-R(2, 0));          // Pitch
        eulerYPR(2) = std::atan2(R(2, 1), R(2, 2)); // Roll
    } else {
        // Gimbal lock case
        eulerYPR(0) = std::atan2(-R(0, 1), R(1, 1)); // Yaw
        eulerYPR(1) = (R(2, 0) > 0) ? -M_PI_2 : M_PI_2; // Pitch
        eulerYPR(2) = 0;                              // Roll indeterminate, set to 0
    }

    return eulerYPR;
}

inline Eigen::Matrix3d getRotationMatrixFromEulerYPR(double yaw, double pitch, double roll) {
    Eigen::Matrix3d R;

    // Precompute trigonometric functions
    double cy = std::cos(yaw);
    double sy = std::sin(yaw);
    double cp = std::cos(pitch);
    double sp = std::sin(pitch);
    double cr = std::cos(roll);
    double sr = std::sin(roll);
    std::cout << cy << " " << sy << " " << cp << " " << sp << " " << cr << " " << sr << std::endl;
    // Compute the rotation matrix
    R << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp,     cp * sr,               cp * cr;

    return R;
}