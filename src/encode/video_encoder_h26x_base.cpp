#include "video_encoder_h26x_base.h"

VideoEncoderH26XBase::VideoEncoderH26XBase(uint32_t cols, uint32_t rows)
    : cols_(cols)
    , rows_(rows)
{
}

VideoEncoderH26XBase::~VideoEncoderH26XBase()
{
}

bool VideoEncoderH26XBase::InstallCallback(const VideoEncoderCallbackType& callback)
{
    if (!callback)
    {
        return false;
    }

    callback_ = callback;

    return true;
}