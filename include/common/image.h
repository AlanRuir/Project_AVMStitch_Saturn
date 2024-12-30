#ifndef __IMAGE_H__
#define __IMAGE_H__

#include <common_type.hpp>

class Image
{
public:
    Image(uint32_t width, uint32_t height, PixelFormat pixel_format, uint64_t frame_count, uint64_t timestamp);
    ~Image();

    uint8_t*    GetData();
    uint64_t    GetSize();
    uint64_t    GetWidth();
    uint64_t    GetHeight();
    PixelFormat GetPixelFormat();
    uint64_t    GetFrameCount();
    uint64_t    GetTimestamp();

    void SetTimestamp(uint64_t timestamp);

private:
    uint8_t*    data_;
    uint64_t    size_;
    uint64_t    width_;
    uint64_t    height_;
    PixelFormat pixel_format_;
    uint64_t    frame_count_;
    uint64_t    timestamp_;
};

#endif // __IMAGE_H__