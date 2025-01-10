#ifndef __VIDEO_ENCODER_H26X_BASE_H__
#define __VIDEO_ENCODER_H26X_BASE_H__

#include <iostream>
#include <functional>
#include "common_type.hpp"

class VideoEncoderH26XBase
{
public:
    using VideoEncoderCallbackType = std::function<void(uint8_t*, uint32_t, uint64_t, bool)>;

    VideoEncoderH26XBase(uint32_t cols, uint32_t rows);
    virtual ~VideoEncoderH26XBase();
    bool         InstallCallback(const VideoEncoderCallbackType& callback);
    virtual bool Encode(uint8_t* frame, uint32_t frame_size, uint64_t timestamp) = 0;

protected:
    uint32_t                 cols_;     // 图像宽
    uint32_t                 rows_;     // 图像高
    VideoEncoderCallbackType callback_; // 回调函数
};

#endif // __VIDEO_ENCODER_H26X_BASE_H__