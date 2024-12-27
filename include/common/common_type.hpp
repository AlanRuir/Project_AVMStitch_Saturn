#ifndef __COMMON_TYPE_HPP__
#define __COMMON_TYPE_HPP__

#include <iostream>
#include <linux/videodev2.h>

enum CodecType : uint8_t
{
    H264,
    H265,
    UNKNOWN_CODEC_TYPE
};

enum FrameType : uint8_t
{
    I,
    P,
    B,
    UNKNOWN_FRAME_TYPE
};

enum PixelFormat : uint8_t
{
    RGB24,
    BGR24,
    YUV420P,
    NV12,
    UYVY422,
    YUYV422,
    UNKNOWN_PIXEL_FORMAT
};

enum CameraViewDirection : uint8_t
{
    FRONT,
    LEFT,
    RIGHT,
    REAR,
    UNKNOWN_CAMERA_VIEW_DIRECTION
};

struct StreamBuffer // USB相机是单平面的，不需要多平面
{
    bool     init;
    void*    start;
    uint32_t capacity;
    uint32_t size;
};

struct Timestamp
{
    uint64_t tv_sec;
    uint64_t tv_usec;
};

struct StreamInfo
{
    uint32_t width;
    uint32_t height;
    uint32_t fps;
};

#endif // __COMMON_TYPE_HPP__