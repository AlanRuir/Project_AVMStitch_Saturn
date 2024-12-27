#include "video_decoder_h26x_base.h"

VideoDecoderH26xBase::VideoDecoderH26xBase(uint32_t cols, uint32_t rows)
    : cols_(cols)
    , rows_(rows)
    , callback_(nullptr)
    , logger_(rclcpp::get_logger("VideoDecoderH26x"))
{
}

VideoDecoderH26xBase::~VideoDecoderH26xBase()
{
}

bool VideoDecoderH26xBase::SetCallback(const VideoDecoderCallbackType& callback)
{
    if (!callback)
    {
        return false;
    }

    callback_ = callback;

    return true;
}
