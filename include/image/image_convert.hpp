#ifndef __IMAGE_CONVERT_H__
#define __IMAGE_CONVERT_H__

#include <libyuv.h>

int NV12ToYUV420P(uint8_t* src_y, uint8_t* src_uv, uint8_t* dst_y, uint8_t* dst_u, uint8_t* dst_v, int width, int height)
{
    return libyuv::NV12ToI420(src_y, width, src_uv, width, dst_y, width, dst_u, width / 2, dst_v, width / 2, width, height);
}

int YUYV422ToNV12(uint8_t* src_yuyv, uint8_t* dst_y, uint8_t* dst_uv, int width, int height)
{
    return libyuv::YUY2ToNV12(src_yuyv, width * 2, dst_y, width, dst_uv, width, width, height);
}

int YUYV422ToYUV420P(uint8_t* src_yuyv, uint8_t* dst_y, uint8_t* dst_u, uint8_t* dst_v, int width, int height)
{
    return libyuv::YUY2ToI420(src_yuyv, width * 2, dst_y, width, dst_u, width / 2, dst_v, width / 2, width, height);
}

int NV12Resize(uint8_t* src_y, uint8_t* src_uv, int src_width, int src_height, uint8_t* dst_y, uint8_t* dst_uv, int dst_width, int dst_height)
{
    return libyuv::NV12Scale(src_y, src_width, src_uv, src_width,
                             src_width, src_height,
                             dst_y, dst_width, dst_uv, dst_width,
                             dst_width, dst_height,
                             libyuv::kFilterBox);
}

#endif // __IMAGE_CONVERT_H__