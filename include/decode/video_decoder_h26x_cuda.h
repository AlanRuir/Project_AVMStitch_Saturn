#ifndef __VIDEO_DECODER_H26X_CUDA_H__
#define __VIDEO_DECODER_H26X_CUDA_H__

#include <iostream>

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

#include "video_decoder_h26x_base.h"

class VideoDecoderH26xCUDA : public VideoDecoderH26xBase
{
public:
    VideoDecoderH26xCUDA(uint32_t cols, uint32_t rows, CodecType codec_type = CodecType::H265);
    ~VideoDecoderH26xCUDA() noexcept override;
    bool Decode(uint8_t* packet, uint32_t packet_size, uint64_t timestamp) override;

private:
    uint32_t        counter_;       // 计数
    AVCodec*        codec_;         // 解码器
    AVCodecContext* codec_context_; // 解码上下文
    AVFrame*        frame_;         // 解码帧
    AVPacket*       pkt_;           // 解码包
};

#endif // __VIDEO_DECODER_H26X_CUDA_H__