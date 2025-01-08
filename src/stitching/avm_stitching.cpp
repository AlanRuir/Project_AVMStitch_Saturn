#include <fstream>
#include <iostream>
#include "avm_stitching.h"

AvmStitching::AvmStitching()
    : logger_(rclcpp::get_logger("AvmStitching"))
{
}
AvmStitching::~AvmStitching()
{
}

void AvmStitching::SetNode(const rclcpp::Node::SharedPtr& node)
{
    node_   = node;
    logger_ = node_->get_logger();
}

void AvmStitching::AddCameraInfo(std::string camera_name, const CameraInfo& cam_info)
{
    mCamInfoMap[camera_name] = cam_info;
}

void AvmStitching::AddImage(std::string camera_name, cv::Mat img)
{
    std::cout << "add image " << camera_name << std::endl;
    mImages[camera_name] = img.clone();
    mCount++;
}

void AvmStitching::SetMainFolder(const std::string& main_folder, const std::string& result_folder)
{
    mMainFolder   = main_folder;
    mResultFolder = result_folder + "/";
}

void AvmStitching::Init(const std::string& extrinsic_file, const std::string& intrinsic_file, const AvmStitchingParams& params)
{
    mParams    = params;
    mBevHeight = mParams.bev_height;
    for (const auto& iter : mParams.camname_index)
    {
        std::string camera = iter.first;
        // cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
        // cv::Mat distor_coeffs;
        CameraInfo cam_info;
        RCLCPP_INFO(logger_, camera.c_str());
        if (GetIntrinsicFromFile(intrinsic_file, camera, cam_info))
        {
            mCamInfoMap[camera] = cam_info;
        }
        else
        {
            RCLCPP_INFO(logger_, "Cannot find %s intrinsic!!!", camera.c_str());
            return;
        }
    }
    GetCamInCarFromFile(extrinsic_file);
    for (const auto& pair : mParams.camname_index)
    {
        mIndexCamname[pair.second] = pair.first;
    }

    GenerateBevIntrinsic();
    GenerateCameraPair();
    GenerateBevMapping();
}

void AvmStitching::GetCamInCarFromFile(const std::string& extrinsic_file)
{
    cv::FileStorage fs(extrinsic_file, cv::FileStorage::READ);
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
    cv::Mat         rotation_cv, translation_cv;
    for (const auto& iter : mParams.camname_index)
    {
        std::string cam_name = iter.first;
        mCamInCarT[cam_name] = Eigen::Matrix4d::Identity();
        fs[cam_name + "_in_car_rotation"] >> rotation_cv;
        std::cout << cam_name << "_in_car_rotation: " << rotation_cv << std::endl;
        fs[cam_name + "_in_car_translation"] >> translation_cv;
        std::cout << cam_name << "_in_car_translation: " << translation_cv << std::endl;
        cv::cv2eigen(translation_cv, translation);
        cv::cv2eigen(rotation_cv, rotation);
        mCamInCarT[cam_name].block<3, 3>(0, 0) = rotation;
        mCamInCarT[cam_name].block<3, 1>(0, 3) = translation;
    }
    double car_height;
    fs["car_height"] >> car_height;
    mCarHeight = car_height;
    std::cout << "car height " << car_height << std::endl;
    fs.release();
}

bool AvmStitching::GetIntrinsicFromFile(std::string intrinsic_file, std::string camera_name, CameraInfo& cam_info)
{
    std::ifstream file(intrinsic_file); // 打开文件

    // 检查文件是否成功打开
    if (!file.is_open())
    {
        std::cerr << "cannot open file" << std::endl;
        return false;
    }
    cam_info.cameramat = cv::Mat::eye(3, 3, CV_64F);
    std::map<std::string, std::string> keyValueMap;
    std::string                        line;

    // 逐行读取文件内容
    bool find_camera_name = false;
    while (std::getline(file, line))
    {
        if (line.find(camera_name) != std::string::npos)
        {
            find_camera_name = true;
            continue;
        }

        if (find_camera_name)
        {
            size_t pos = line.find(":"); // Find the separator ":"
            if (pos != std::string::npos)
            {
                std::string key = line.substr(0, pos); // Get the key before the separator
                key.erase(0, key.find_first_not_of(" \t"));
                std::string value = line.substr(pos + 2); // Get the value after the separator (+2 to skip ": ")
                keyValueMap[key]  = value;                // Store the key-value pair in the map

                if (key == std::string("P2"))
                {
                    break;
                }
            }
        }
    }

    int         width                   = std::stoi(keyValueMap["imageWidth"]);
    int         height                  = std::stoi(keyValueMap["imageHeight"]);
    std::string model                   = keyValueMap["model"];
    double      fx                      = std::stod(keyValueMap["fx"]);
    double      fy                      = std::stod(keyValueMap["fy"]);
    double      cx                      = std::stod(keyValueMap["cx"]);
    double      cy                      = std::stod(keyValueMap["cy"]);
    double      K1                      = std::stod(keyValueMap["K1"]);
    double      K2                      = std::stod(keyValueMap["K2"]);
    double      K3                      = std::stod(keyValueMap["K3"]);
    double      K4                      = std::stod(keyValueMap["K4"]);
    double      K5                      = std::stod(keyValueMap["K5"]);
    double      K6                      = std::stod(keyValueMap["K6"]);
    double      P1                      = std::stod(keyValueMap["P1"]);
    double      P2                      = std::stod(keyValueMap["P2"]);
    cam_info.width                      = width;
    cam_info.height                     = height;
    cam_info.model                      = model;
    cam_info.cameramat.at<double>(0, 0) = fx;
    cam_info.cameramat.at<double>(0, 2) = cx;
    cam_info.cameramat.at<double>(1, 1) = fy;
    cam_info.cameramat.at<double>(1, 2) = cy;
    if (keyValueMap["model"] == "pinhole")
    {
        cam_info.distcoeff                  = cv::Mat::zeros(8, 1, CV_64FC1);
        cam_info.distcoeff.at<double>(0, 0) = K1;
        cam_info.distcoeff.at<double>(1, 0) = K2;
        cam_info.distcoeff.at<double>(2, 0) = P1;
        cam_info.distcoeff.at<double>(3, 0) = P2;
        cam_info.distcoeff.at<double>(4, 0) = K3;
        cam_info.distcoeff.at<double>(5, 0) = K4;
        cam_info.distcoeff.at<double>(6, 0) = K5;
        cam_info.distcoeff.at<double>(7, 0) = K6;
    }
    else if (keyValueMap["model"] == "fisheye")
    {
        cam_info.distcoeff                  = cv::Mat::zeros(4, 1, CV_64FC1);
        cam_info.distcoeff.at<double>(0, 0) = K1;
        cam_info.distcoeff.at<double>(1, 0) = K2;
        cam_info.distcoeff.at<double>(2, 0) = K3;
        cam_info.distcoeff.at<double>(3, 0) = K4;
    }

    RCLCPP_INFO(logger_, "%s", camera_name.c_str());
    RCLCPP_INFO(logger_, "%d %d", cam_info.width, cam_info.height);
    RCLCPP_INFO(logger_, "%s", cam_info.model.c_str());
    std::stringstream distcoeff_stream;
    distcoeff_stream << "Discoeff" << ":\n";
    for (int i = 0; i < cam_info.distcoeff.rows; ++i)
    {
        for (int j = 0; j < cam_info.distcoeff.cols; ++j)
        {
            distcoeff_stream << cam_info.distcoeff.at<double>(i, j) << " ";
        }
    }
    RCLCPP_INFO(logger_, "%s", distcoeff_stream.str().c_str());

    std::stringstream intrinsics_stream;
    intrinsics_stream.str("");
    intrinsics_stream.clear();
    intrinsics_stream << "Intrinsics" << ":\n";
    for (int i = 0; i < cam_info.cameramat.rows; ++i)
    {
        for (int j = 0; j < cam_info.cameramat.cols; ++j)
        {
            intrinsics_stream << cam_info.cameramat.at<double>(i, j) << " ";
        }
    }
    RCLCPP_INFO(logger_, "%s", intrinsics_stream.str().c_str());
    file.close();
    return true;
}

void AvmStitching::GenerateBevIntrinsic()
{
    mBevSize.width  = mParams.image_width;
    mBevSize.height = mParams.image_height;
    double fx, fy, cx, cy;
    fx = mParams.fx;
    fy = mParams.fy;
    // cx = mCamInfoMap.begin()->second.width/2;
    // cy = mCamInfoMap.begin()->second.height/2;
    cx            = mBevSize.width / 2;
    cy            = mBevSize.height / 2;
    mBevCameramat = (cv::Mat_<double>(3, 3) << fx, 0, cx,
                     0, fy, cy,
                     0, 0, 1);
}

void AvmStitching::GenerateCameraPair()
{
    mCamPair.push_back(std::make_pair(0, 2));
    mCamPair.push_back(std::make_pair(2, 3));
    mCamPair.push_back(std::make_pair(3, 1));
    mCamPair.push_back(std::make_pair(1, 0));
}

void AvmStitching::GenerateBevMapping()
{
    Eigen::Matrix3d R_bev_in_ground;
    R_bev_in_ground << 0, -1, 0,
        -1, 0, 0,
        0, 0, -1;
    Eigen::Matrix3d bevK;
    cv::cv2eigen(mBevCameramat, bevK);
    Eigen::Matrix3d bevK_inv = bevK.inverse();
    std::cout << bevK_inv << std::endl;
    cv::Mat         groundObj = cv::Mat::zeros(mBevSize.height, mBevSize.width, CV_32FC3);
    Eigen::Vector3d groundZ(0, 0, -1);
    Eigen::Vector3d bev_origin(0, 0, mBevHeight);
    for (int row = 0; row < mBevSize.height; row++)
    {
        for (int col = 0; col < mBevSize.width; col++)
        {
            ///
            // double bev_height1 = 1.5;/// 为什么这个和原来 bev 到地面的距离 1.5相差 0.6，否则匹配不上那
            Eigen::Vector3d p(col, row, 1);
            Eigen::Vector3d dir = bevK_inv * p;
            // std::cout << "dir " << dir.transpose() << std::endl;
            dir.normalize();
            Eigen::Vector3d dir_g = R_bev_in_ground * dir; // dir in ground frame
            // std::cout << "dir_g " << dir_g.transpose() << std::endl;
            float cos_angle = dir_g.dot(groundZ);
            // std::cout << "cos angle " << cos_angle << std::endl;
            Eigen::Vector3d pt = dir;
            // std::cout << "pt " << pt.transpose() << std::endl;
            float scale = mBevHeight / cos_angle;
            pt          = bev_origin + scale * dir_g;
            // std::cout << "pt " << pt.transpose() << std::endl;

            groundObj.at<cv::Vec3f>(row, col) = cv::Vec3f(pt(0), pt(1), pt(2));
        }
    }
    cv::Mat         bev_img;
    double          sensingHeight   = 0.0;
    Eigen::Vector4d groundCoeffs    = Eigen::Vector4d(0, 0, 1, sensingHeight);
    Eigen::Matrix4d T_ground_in_car = Eigen::Matrix4d::Identity();
    // T_ground_in_car.block<3, 3>(0, 0) = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0, 0, 1), groundCoeffs.segment(0, 3)).toRotationMatrix();
    T_ground_in_car.block<3, 1>(0, 3) = Eigen::Vector3d(0, 0, -mCarHeight);

    for (const auto& iter : mCamInCarT)
    {
        std::string     cam_name     = iter.first;
        Eigen::Matrix4d T_cam_in_car = iter.second;
        mGroundInCam[cam_name]       = T_cam_in_car.inverse() * T_ground_in_car;

        cv::Mat         K = mCamInfoMap[cam_name].cameramat.clone();
        cv::Mat         D = mCamInfoMap[cam_name].distcoeff.clone();
        Eigen::Matrix3d K_eigen;
        cv::cv2eigen(K, K_eigen);
        Eigen::Matrix4d T_ground_in_cam = T_cam_in_car.inverse() * T_ground_in_car;
        Eigen::Matrix3d R_ground_in_cam = T_ground_in_cam.block<3, 3>(0, 0);
        Eigen::Vector3d t_ground_in_cam = T_ground_in_cam.block<3, 1>(0, 3);
        std::cout << cam_name << std::endl;
        std::cout << mBevSize << std::endl;
        cv::Mat mapX = cv::Mat::zeros(mBevSize, CV_32FC1);
        cv::Mat mapY = cv::Mat::zeros(mBevSize, CV_32FC1);
        for (int i = 0; i < mBevSize.height; i++)
        {
            for (int j = 0; j < mBevSize.width; j++)
            {
                cv::Vec3f       ve3f = groundObj.at<cv::Vec3f>(i, j);
                Eigen::Vector3d p(ve3f[0], ve3f[1], ve3f[2]);
                Eigen::Vector3d pcam = R_ground_in_cam * p + t_ground_in_cam; // 转到相机坐标系
                // 计算和Z轴的角度
                Eigen::Vector2d pi;
                double          angle = atan2(pcam.segment(0, 2).norm(), pcam(2));
                if (pcam(2) > 0 && angle < M_PI * 0.5)
                {
                    // if(pcam(2)>0){

                    SpaceToPlaneFisheye(K_eigen, D, pcam, pi);
                }
                else
                {
                    pi = Eigen::Vector2d(-1, -1);
                }
                mapX.at<float>(i, j) = float(pi(0));
                mapY.at<float>(i, j) = float(pi(1));
            }
        }
        // std::cout << cam_name << std::endl;
        // std::cout << mapX << std::endl;
        mMapX[cam_name] = mapX.clone();
        mMapY[cam_name] = mapY.clone();
    }
    GeneratePriMask();
}

void AvmStitching::ProcessImage(const std::string& cam_name, const cv::Mat& img, const cv::Mat& mapX, const cv::Mat& mapY,
                                const cv::Mat& weight, const cv::Mat& overlap, cv::Mat& bev_blend_img, std::atomic<int>& counter)
{
    cv::Mat mask, weight_3ch, imgWarp_32F, weightedImgWarp, temp_result, overlap_mask, imgWarp_mask;

    cv::Mat imgWarp;
    cv::remap(img, imgWarp, mapX, mapY, cv::INTER_NEAREST);

    cv::merge(std::vector<cv::Mat>{weight, weight, weight}, weight_3ch);
    imgWarp.convertTo(imgWarp_32F, CV_32FC3);
    cv::multiply(imgWarp_32F, weight_3ch, weightedImgWarp);
    weightedImgWarp.convertTo(weightedImgWarp, CV_8UC3);

    // 使用 overlap 生成 mask
    cv::compare(overlap, 0, mask, cv::CMP_GT);
    mask.convertTo(mask, CV_8U);
    weightedImgWarp.copyTo(temp_result, mask);

    // 更新全局图像
    bev_blend_img += temp_result;

    // 处理 overlap_mask
    cv::compare(overlap, 0, overlap_mask, cv::CMP_EQ);
    std::vector<cv::Mat> channels(3);
    cv::split(imgWarp, channels);
    cv::compare(channels[0], 0, imgWarp_mask, cv::CMP_GT);
    cv::bitwise_and(overlap_mask, imgWarp_mask, mask);

    // 更新最终图像
    imgWarp.copyTo(bev_blend_img, mask);

    // 计数器递增
    counter.fetch_add(1, std::memory_order_relaxed);
}

cv::Mat AvmStitching::StartStitching()
{
    std::cout << "----StartStitching----" << std::endl;
    cv::Mat                  bev_blend_img = cv::Mat::zeros(mBevSize, CV_8UC3);
    std::string              path;
    std::vector<std::thread> threads;
    std::atomic<int>         counter(0);
    for (const auto& it : mImages)
    {
        std::string    cam_name = it.first;
        const cv::Mat& img      = it.second;
        const cv::Mat& mapX     = mMapX.at(cam_name);
        const cv::Mat& mapY     = mMapY.at(cam_name);
        const cv::Mat& weight   = mWeights.at(mParams.camname_index.at(cam_name));
        const cv::Mat& overlap  = mBevOverlapMask.at(cam_name);

        threads.emplace_back([this, cam_name, img, mapX, mapY, weight, overlap, &bev_blend_img, &counter]() {
            this->ProcessImage(cam_name, img, mapX, mapY, weight, overlap, bev_blend_img, counter);
        });
    }
    for (auto& t : threads)
    {
        t.join();
    }

    // path = mResultFolder + "bev_blend.jpg";
    // cv::imwrite(path, bev_blend_img);
    imshow("bev_blend_img", bev_blend_img);
    cv::waitKey(33);
    return bev_blend_img;
}

void AvmStitching::GenerateCarMask(double front_in_car_x, double back_in_car_x, cv::Mat& car_mask_white)
{
    std::map<int, cv::Mat>   bigger_prior_masks;
    std::vector<cv::Point3f> pts3d;
    std::vector<cv::Point2f> pts2d;
    cv::Scalar               color = cv::Scalar::all(255);

    pts3d.push_back(cv::Point3f(mParams.car_width / 2.0 + 0.1, -front_in_car_x, mBevHeight));
    pts3d.push_back(cv::Point3f(mParams.car_width / 2.0 + 0.1, -back_in_car_x, mBevHeight));
    pts3d.push_back(cv::Point3f(-mParams.car_width / 2.0 - 0.1, -front_in_car_x, mBevHeight));
    pts3d.push_back(cv::Point3f(-mParams.car_width / 2.0 - 0.1, -back_in_car_x, mBevHeight));
    // pts3d.push_back(cv::Point3f(car_width/2.0,0,bev_height));
    // pts3d.push_back(cv::Point3f(-car_width/2.0,0,bev_height));

    cv::Mat rev   = cv::Mat::zeros(3, 1, CV_32FC1);
    cv::Mat tev   = cv::Mat::zeros(3, 1, CV_32FC1);
    cv::Mat dcoef = cv::Mat::zeros(5, 1, CV_32FC1);

    cv::projectPoints(pts3d, rev, tev, mBevCameramat, dcoef, pts2d);

    cv::Mat car_mask = cv::Mat::zeros(mBevSize, CV_8UC3);
    car_mask_white   = cv::Mat::zeros(mBevSize, CV_8UC3);

    cv::Mat bigger_mask = car_mask.clone();

    cv::Rect black_zone = cv::Rect(cv::Point(pts2d[2].x, pts2d[2].y), cv::Point(pts2d[1].x, pts2d[1].y));
    car_mask_white      = bigger_mask.clone();
    bigger_mask(black_zone).setTo(cv::Scalar(255, 255, 255));
    std::string path = mResultFolder + "car_zone.jpg";
    cv::imwrite(path, bigger_mask);
    path = mResultFolder + "car_mask_white.jpg";
    cv::imwrite(path, car_mask_white);
}

void AvmStitching::GeneratePriMask()
{
    std::map<std::string, cv::Mat> bigger_bev_masks;

    for (const auto& iter : mCamInfoMap)
    {
        std::string cam_name = iter.first;
        cv::Size    img_size(mCamInfoMap.begin()->second.width, mCamInfoMap.begin()->second.height);
        cv::Mat     mask           = cv::Mat::zeros(img_size, CV_8UC3);
        bigger_bev_masks[cam_name] = mask.clone();

        cv::Mat mapX = mMapX[cam_name].clone();
        cv::Mat mapY = mMapY[cam_name].clone();

        mask.setTo(cv::Scalar(255, 255, 255));

        cv::Mat maskWarp;

        std::string path = mResultFolder + "mask_" + cam_name + ".jpg";
        std::cout << path << std::endl;
        cv::remap(mask, maskWarp, mapX, mapY, 0);
        cv::imwrite(path, maskWarp);
        bigger_bev_masks[cam_name] = maskWarp.clone();
    }

    Eigen::Matrix4d T_camfront_in_car = mCamInCarT[mParams.cam_front];
    Eigen::Matrix4d T_camback_in_car  = mCamInCarT[mParams.cam_back];

    double front_in_car_x = T_camfront_in_car.block<3, 1>(0, 3)(0);
    double back_in_car_x  = T_camback_in_car.block<3, 1>(0, 3)(0);

    std::cout << "front in car x " << front_in_car_x << std::endl;
    std::cout << "back in car x " << back_in_car_x << std::endl;

    cv::Mat car_mask_white;
    GenerateCarMask(front_in_car_x, back_in_car_x, car_mask_white);
    /// 根据外参实际计算
    for (auto& it : bigger_bev_masks)
    {
        std::string cam_name = it.first;
        it.second &= (~car_mask_white);
        std::string path = mResultFolder + "mask_car_" + cam_name + ".jpg";
        cv::imwrite(path, it.second);
    }
    /// 计算重叠的部分
    // std::map<int,cv::Mat> mBevOverlapMask;
    for (auto it : bigger_bev_masks)
    {
        mBevOverlapMask[it.first] = cv::Mat::zeros(mBevSize, CV_8UC1);
        mBevOverlapMask[it.first].setTo(cv::Scalar(0));
        for (auto jt : bigger_bev_masks)
        {
            if (it.first == jt.first)
            {
                continue;
            }
            cv::Mat              maskL   = it.second;
            cv::Mat              maskR   = jt.second;
            cv::Mat              overlap = maskL & maskR;
            std::vector<cv::Mat> overlab_channel;
            cv::split(overlap, overlab_channel);
            mBevOverlapMask[it.first] |= overlab_channel[0];
        }
    }
    GenerateCameraPair();
    GenerateWeights(bigger_bev_masks);
}

void AvmStitching::SpaceToPlaneFisheye(Eigen::Matrix3d K, cv::Mat D, Eigen::Vector3d pcam, Eigen::Vector2d& p)
{
    double x = pcam(0) / pcam(2);
    double y = pcam(1) / pcam(2);
    double r = std::sqrt(x * x + y * y);
    // 在归一化的平面 Z=1
    double theta  = std::atan(r);
    double theta2 = theta * theta;
    double theta4 = theta2 * theta2;
    double theta6 = theta4 * theta2;
    double theta8 = theta4 * theta4;
    ///
    double* Dparam = (double*)D.data;

    double thetad = theta * (1 + Dparam[0] * theta2 + Dparam[1] * theta4 + Dparam[2] * theta6 + Dparam[3] * theta8);
    // rd=f*thetad f=1
    double scale = (r == 0) ? 1.0 : thetad / r;
    double xd    = K(0, 0) * scale * x + K(0, 2);
    double yd    = K(1, 1) * scale * y + K(1, 2);
    p << xd, yd;
}

void AvmStitching::GenerateWeights(std::map<std::string, cv::Mat> bigger_bev_masks)
{
    /// 初始化
    for (auto it : bigger_bev_masks)
    {
        std::string cam_name = it.first;
        int         index    = mParams.camname_index[cam_name];
        mWeights[index]      = cv::Mat::zeros(it.second.size(), CV_32FC1);
    }
    ///
    for (auto it : mCamPair)
    {
        std::string cam_name = mIndexCamname[it.first];
        cv::Mat     mask_l   = bigger_bev_masks[cam_name];
        cam_name             = mIndexCamname[it.second];
        cv::Mat mask_r       = bigger_bev_masks[cam_name];

        std::vector<cv::Mat> single_mask_l, single_mask_r;
        cv::split(mask_l, single_mask_l);
        cv::split(mask_r, single_mask_r);
        cv::Mat overlap     = single_mask_l[0] & single_mask_r[0];
        cv::Mat overlab_inv = ~overlap;
        ///  将原来mask 的重叠区域 值赋为0
        single_mask_l[0] &= overlab_inv;
        single_mask_r[0] &= overlab_inv;

        cv::Mat mask_distance_l, mask_distance_r;
        // 计算每一个非零点与其最近的零点像素之间的距离，那么要计算的就是重合区域到边缘的权重
        cv::distanceTransform(cv::Scalar(255) - single_mask_l[0], mask_distance_l, cv::DIST_L2, 0);
        cv::distanceTransform(cv::Scalar(255) - single_mask_r[0], mask_distance_r, cv::DIST_L2, 0);

        std::string path_l = mResultFolder + "weight_mask_l_" + std::to_string(it.first) + ".jpg";
        cv::imwrite(path_l, mask_distance_l * 255);

        std::string path_r = mResultFolder + "weight_mask_r_" + std::to_string(it.second) + ".jpg";
        cv::imwrite(path_r, mask_distance_r * 255);

        for (int i = 0; i < overlap.rows; i++)
        {
            for (int j = 0; j < overlap.cols; j++)
            {
                if (overlap.at<uchar>(i, j))
                {
                    float d1 = mask_distance_l.at<float>(i, j);
                    float d2 = mask_distance_r.at<float>(i, j);
                    float d  = d1 + d2;
                    float w1 = 0, w2 = 0;
                    if (iszero(d))
                    {
                        w1 = 0;
                        w2 = 0;
                    }
                    else
                    {
                        w1 = d2 / d;
                        w2 = d1 / d;
                    }
                    mWeights[it.first].at<float>(i, j)  = w1;
                    mWeights[it.second].at<float>(i, j) = w2;
                }
            }
        }
    } // end

    for (auto it : mWeights)
    {
        int         index    = it.first;
        std::string cam_name = mIndexCamname[index];
        cv::Mat     mask     = bigger_bev_masks[cam_name].clone();
        cv::Mat     overlap  = mBevOverlapMask[cam_name].clone();
        for (int i = 0; i < overlap.rows; i++)
        {
            for (int j = 0; j < overlap.cols; j++)
            {
                if (overlap.at<uchar>(i, j) == 0 && mask.at<cv::Vec3b>(i, j)[0] != 0)
                {
                    mWeights[it.first].at<float>(i, j) = 1.0;
                }
            }
        }
        std::string path = mResultFolder + "weight_mask_" + cam_name + ".jpg";
        cv::imwrite(path, mWeights[it.first] * 255);
    }
}
