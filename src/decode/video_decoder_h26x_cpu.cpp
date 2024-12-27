#include "video_decoder_h26x_cpu.h"

VideoDecoderH26xCPU::VideoDecoderH26xCPU(uint32_t cols, uint32_t rows, CodecType codec)
    : VideoDecoderH26xBase(cols, rows)
    , counter_(0LU)
    , codec_(nullptr)
    , codec_context_(nullptr)
    , frame_(nullptr)
    , pkt_(nullptr)
{
#if (LIBAVCODEC_VERSION_MAJOR < 4)
    av_register_all();      // 注册所有组件
    avcodec_register_all(); // 注册所有编解码器
#endif

    if (CodecType::H264 == codec)
    {
        codec_ = avcodec_find_decoder(AV_CODEC_ID_H264);
        if (!codec_)
        {
            throw std::runtime_error("failed to find h264 decoder");
        }
        else
        {
            RCLCPP_INFO(logger_, "used codec: %s", codec_->name);
        }
    }
    else if (CodecType::H265 == codec)
    {
        codec_ = avcodec_find_decoder(AV_CODEC_ID_H265);
        if (!codec_)
        {
            throw std::runtime_error("failed to find h265 decoder");
        }
        else
        {
            RCLCPP_INFO(logger_, "used codec: %s", codec_->name);
        }
    }

    codec_context_ = avcodec_alloc_context3(codec_);
    if (!codec_context_)
    {
        throw std::runtime_error("failed to allocate codec context");
    }

    pkt_ = av_packet_alloc();
    if (!pkt_)
    {
        av_packet_free(&pkt_);
        throw std::runtime_error("failed to allocate packet");
    }

    frame_ = av_frame_alloc();
    if (!frame_)
    {
        av_frame_free(&frame_);
        throw std::runtime_error("failed to allocate frame");
    }

    /* 设置多线程解码 */
    codec_context_->thread_count = 4U;
    if (codec_->capabilities | AV_CODEC_CAP_FRAME_THREADS)
    {
        codec_context_->thread_type = FF_THREAD_FRAME;
    }
    else if (codec_->capabilities | AV_CODEC_CAP_SLICE_THREADS)
    {
        codec_context_->thread_type = FF_THREAD_SLICE;
    }
    else
    {
        codec_context_->thread_count = 1;
    }

    if (avcodec_open2(codec_context_, codec_, NULL) < 0) // 打开解码器
    {
        throw std::runtime_error("failed to open avcodec");
    }
}

VideoDecoderH26xCPU::~VideoDecoderH26xCPU() noexcept
{
    if (codec_context_)
    {
        (void)avcodec_send_packet(codec_context_, NULL);
        avcodec_free_context(&codec_context_);
    }

    if (pkt_)
    {
        av_packet_free(&pkt_);
    }

    if (frame_)
    {
        av_frame_free(&frame_);
    }
}

bool VideoDecoderH26xCPU::Decode(uint8_t* packet, uint32_t packet_size, uint64_t timestamp)
{
    if (!packet || packet_size == 0)
    {
        return false;
    }

    pkt_->data = packet;
    pkt_->size = packet_size;

    int32_t result = avcodec_send_packet(codec_context_, pkt_);
    if (result < 0)
    {
        RCLCPP_ERROR(logger_, "failed to send packet to decoder: %d", result);
        pkt_->data = nullptr;
        pkt_->size = 0;
        return false;
    }

    while (result >= 0)
    {
        result = avcodec_receive_frame(codec_context_, frame_);
        if (result == AVERROR(EAGAIN) || result == AVERROR_EOF)
        {
            break;
        }
        else if (result < 0)
        {
            RCLCPP_ERROR(logger_, "failed to receive frame from decoder: %d", result);
            pkt_->data = nullptr;
            pkt_->size = 0;
            break;
        }

        if (callback_)
        {
            callback_(frame_->data, frame_->linesize, counter_, timestamp);
            ++counter_;
        }
    }

    return true;
}
