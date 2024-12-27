#ifndef __RTSP_CLIENT_H__
#define __RTSP_CLIENT_H__

#include <iostream>
#include <functional>
#include <string>

extern "C"
{
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
}

#include <rclcpp/rclcpp.hpp>

#include "common_type.hpp"

class RtspClient
{
private:
    using FrameCallback = std::function<void(uint8_t* data, size_t size)>;

public:
    RtspClient();
    ~RtspClient();
    bool       OpenStream(const std::string& url);
    CodecType  GetCodec();
    StreamInfo GetStreamInfo();
    bool       ReadFrame();
    bool       SetFrameCallback(const FrameCallback& callback);

private:
    AVFormatContext* format_ctx_;
    AVCodecContext*  codec_ctx_;
    int              video_stream_index_;
    FrameCallback    callback_;
    rclcpp::Logger   logger_;
};

#endif // __RTSP_CLIENT_H__