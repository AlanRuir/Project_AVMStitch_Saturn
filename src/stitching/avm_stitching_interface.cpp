#include "avm_stitching_interface.h"

AVMStitchingInterface::AVMStitchingInterface()
{
    std::map<std::string, int> camname_index;
    camname_index.insert(std::make_pair("surrounding_front", 0));
    camname_index.insert(std::make_pair("surrounding_left", 1));
    camname_index.insert(std::make_pair("surrounding_right", 2));
    camname_index.insert(std::make_pair("surrounding_back", 3));

    cameraParamFile_ = "/home/nyx/Apps/remotedriving-avmstitch/config/camera_param_remote.yaml";
    calibResultFile_ = "/home/nyx/Apps/remotedriving-avmstitch/config/result.yaml";

    stithcing_params_.cam_front     = "surrounding_front";
    stithcing_params_.cam_back      = "surrounding_back";
    stithcing_params_.car_width     = 1.93;
    stithcing_params_.bev_height    = 1.5;
    stithcing_params_.fx            = 150;
    stithcing_params_.fy            = 150;
    stithcing_params_.image_width   = 900;
    stithcing_params_.image_height  = 900;
    stithcing_params_.camname_index = camname_index;

    main_folder_ = "/home/nyx/Apps/remotedriving-avmstitch";
    logFolder_   = "/home/nyx/Apps/remotedriving-avmstitch/log";

    stitching_ = std::make_shared<AvmStitching>();
    stitching_->SetMainFolder(main_folder_, logFolder_);
    stitching_->Init(calibResultFile_, cameraParamFile_, stithcing_params_);
}

AVMStitchingInterface::~AVMStitchingInterface()
{
}

cv::Mat AVMStitchingInterface::AVMStitching(const std::map<std::string, cv::Mat>& image_map)
{
    // cv::Mat front_image = cv::imread("/home/nyx/Apps/remotedriving-avmstitch/avm_stitching_data/surrounding_front.png");
    // stitching_->AddImage("surrounding_front", front_image);
    // cv::Mat left_image = cv::imread("/home/nyx/Apps/remotedriving-avmstitch/avm_stitching_data/surrounding_left.png");
    // stitching_->AddImage("surrounding_left", left_image);
    // cv::Mat right_image = cv::imread("/home/nyx/Apps/remotedriving-avmstitch/avm_stitching_data/surrounding_right.png");
    // stitching_->AddImage("surrounding_right", right_image);
    // cv::Mat back_image = cv::imread("/home/nyx/Apps/remotedriving-avmstitch/avm_stitching_data/surrounding_back.png");
    // stitching_->AddImage("surrounding_back", back_image);

    for (auto& image_pair : image_map)
    {
        stitching_->AddImage(image_pair.first, image_pair.second);
    }

    if (stitching_->mCount == 4)
    {
        cv::Mat result_mat = stitching_->StartStitching();
        stitching_->mCount = 0;
        return result_mat;
    }

    // return cv::Mat();
}
