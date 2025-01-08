#pragma once
#include <iostream>
#include <string>
#include <ctime>
#include <cstdio>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include "common.h"

struct AvmStitchingParams
{
    std::string                cam_back;
    std::string                cam_front;
    double                     car_width;
    double                     bev_height;
    std::map<std::string, int> camname_index;
    int                        fx;
    int                        fy;
    int                        image_width;
    int                        image_height;
};

class AvmStitching
{
public:
    AvmStitching();
    ~AvmStitching();
    void SetMainCam(std::string main_cam);
    void AddCameraInfo(std::string camera_name, const CameraInfo& cam_info);
    void AddImage(std::string camera_name, cv::Mat img);

    void    SetMainFolder(const std::string& main_folder, const std::string& result_folder);
    void    SetConfigFiles(const std::string& camera_param_file, const std::string& result_file);
    void    Init(const std::string& extrinsic_file, const std::string& intrinsic_file, const AvmStitchingParams& params);
    cv::Mat StartStitching();
    int     mCount = 0;

private:
    void SetNode(const rclcpp::Node::SharedPtr& node);
    void GetCamInCarFromFile(const std::string& extrinsic_file);
    bool GetIntrinsicFromFile(std::string intrinsic_file, std::string camera_name, CameraInfo& cam_info);
    void GenerateBevIntrinsic();
    void SpaceToPlaneFisheye(Eigen::Matrix3d K, cv::Mat D, Eigen::Vector3d pcam, Eigen::Vector2d& p);
    void GenerateBevMapping();
    void GeneratePriMask();
    void GenerateCarMask(double front_in_car_x, double front_in_car_y, cv::Mat& car_mask_white);
    void GenerateWeights(std::map<std::string, cv::Mat> bigger_bev_masks);
    void GenerateCameraPair();
    void ProcessImage(const std::string& cam_name, const cv::Mat& img, const cv::Mat& mapX, const cv::Mat& mapY,
                      const cv::Mat& weight, const cv::Mat& overlap, cv::Mat& bev_blend_img, std::atomic<int>& counter);

    rclcpp::Logger                    logger_;
    rclcpp::Node::SharedPtr           node_;
    std::string                       mMainFolder;
    std::string                       mResultFolder;
    std::string                       mResultYaml;
    std::string                       mIntrinsicsFile;
    std::map<std::string, CameraInfo> mCamInfoMap;
    std::map<std::string, cv::Mat>    mImages;
    std::map<std::string, cv::Mat>    mBevOverlapMask;
    AvmStitchingParams                mParams;
    std::map<int, std::string>        mIndexCamname;
    std::vector<std::string>          mCameraList;
    // std::map<int, cv::Point3f> mArucoInBoard;
    std::map<int, Eigen::Vector3d>         mArucoInBoard;
    std::map<std::string, Eigen::Matrix4d> mCamInWorld;
    std::map<std::string, Eigen::Matrix4d> mCamInMain;
    std::map<std::string, Eigen::Matrix4d> mGroundInCam;
    cv::Mat                                mBevCameramat;
    cv::Size                               mBevSize;
    double                                 mBevHeight;
    double                                 mCarHeight;
    std::map<int, cv::Mat>                 mWeights;
    std::map<std::string, cv::Mat>         mMapX;
    std::map<std::string, cv::Mat>         mMapY;
    std::map<std::string, Eigen::Matrix4d> mResultCamInWorldT;
    std::map<std::string, Eigen::Matrix4d> mCamInCarT;
    Eigen::Matrix4d                        mMainInCarT;
    std::vector<std::pair<int, int>>       mCamPair;
    std::string                            mainCam;
    std::map<std::string, cv::Mat>         mImgWarpList;
};