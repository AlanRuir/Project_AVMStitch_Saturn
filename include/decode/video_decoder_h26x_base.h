#ifndef __VIDEO_DECODER_H26X_BASE_H__
#define __VIDEO_DECODER_H26X_BASE_H__

#include <iostream>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include "common_type.hpp"

class VideoDecoderH26xBase
{
private:
    using VideoDecoderCallbackType = std::function<void(uint8_t**, int*, uint32_t, uint32_t)>;

public:
    VideoDecoderH26xBase(uint32_t cols, uint32_t rows);
    virtual ~VideoDecoderH26xBase();
    bool         SetCallback(const VideoDecoderCallbackType& callback);
    virtual bool Decode(uint8_t* packet, uint32_t packet_size, uint64_t timestamp) = 0;

protected:
    uint32_t                 cols_;
    uint32_t                 rows_;
    VideoDecoderCallbackType callback_;
    rclcpp::Logger           logger_;
};

#endif // __VIDEO_DECODER_H26X_BASE_H__