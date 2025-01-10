#ifndef __VIDEO_ENCODER_H26X_CUDA_H__
#define __VIDEO_ENCODER_H26X_CUDA_H__

#include <iostream>
#include <functional>

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

#include "video_encoder_h26x_base.h"

class VideoEncoderH26XCUDA : public VideoEncoderH26XBase
{
public:
    VideoEncoderH26XCUDA(uint32_t cols, uint32_t rows, CodecType codec = CodecType::H265);
    ~VideoEncoderH26XCUDA() noexcept;
    bool Encode(uint8_t* frame, uint32_t frame_size, uint64_t timestamp) override;

private:
    uint32_t        counter_;       // 计数
    AVCodec*        codec_;         // 编码器
    AVCodecContext* codec_context_; // 编码上下文
    AVFrame*        frame_;         // 编码帧
    AVPacket*       pkt_;           // 编码包
};

#endif // __VIDEO_ENCODER_H26X_CUDA_H__