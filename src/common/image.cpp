#include <image.h>

Image::Image(uint32_t width, uint32_t height, PixelFormat pixel_format, uint64_t frame_count, uint64_t timestamp)
    : width_(width)
    , height_(height)
    , pixel_format_(pixel_format)
    , frame_count_(frame_count)
    , timestamp_(timestamp)
{
    if (PixelFormat::RGB24 == pixel_format_ || PixelFormat::BGR24 == pixel_format_)
    {
        size_ = width_ * height_ * 3;
        data_ = new uint8_t[size_];
    }
    else if (PixelFormat::YUV420P == pixel_format_ || PixelFormat::NV12 == pixel_format_)
    {
        size_ = width_ * height_ * 3 / 2;
        data_ = new uint8_t[size_];
    }
    else if (PixelFormat::UYVY422 == pixel_format_ || PixelFormat::YUYV422 == pixel_format_)
    {
        size_ = width_ * height_ * 2;
        data_ = new uint8_t[size_];
    }
    else
    {
        std::cerr << "pixel format not support";
    }
}

Image::~Image()
{
    if (data_)
    {
        delete[] data_;
    }
}

uint8_t* Image::GetData()
{
    return data_;
}

uint64_t Image::GetSize()
{
    return size_;
}

uint64_t Image::GetWidth()
{
    return width_;
}

uint64_t Image::GetHeight()
{
    return height_;
}

PixelFormat Image::GetPixelFormat()
{
    return pixel_format_;
}

uint64_t Image::GetFrameCount()
{
    return frame_count_;
}

uint64_t Image::GetTimestamp()
{
    return timestamp_;
}

void Image::SetTimestamp(uint64_t timestamp)
{
    timestamp_ = timestamp;
}