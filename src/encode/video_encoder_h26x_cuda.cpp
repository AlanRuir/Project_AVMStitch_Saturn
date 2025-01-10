#include "video_encoder_h26x_cuda.h"
#include <exception>
// #include <chrono>

VideoEncoderH26XCUDA::VideoEncoderH26XCUDA(uint32_t cols, uint32_t rows, CodecType codec)
    : VideoEncoderH26XBase(cols, rows)
    , counter_(0U)
    , codec_(nullptr)
    , codec_context_(nullptr)
    , frame_(nullptr)
    , pkt_(nullptr)
{
#if (LIBAVCODEC_VERSION_MAJOR < 4)
    av_register_all();      // 注册所有组件
    avcodec_register_all(); // 注册所有编解码器
#endif

    if (codec == CodecType::H264)
    {
        codec_ = avcodec_find_encoder_by_name("h264_nvenc"); // 查找编码器
        if (!codec_)
        {
            throw std::runtime_error("failed to find encoder");
        }
        else
        {
            std::cout << "use codec: " << codec_->name << std::endl;
        }
    }
    else if (codec == CodecType::H265)
    {
        codec_ = avcodec_find_encoder_by_name("hevc_nvenc"); // 查找编码器
        if (!codec_)
        {
            throw std::runtime_error("failed to find encoder");
        }
        else
        {
            std::cout << "use codec: " << codec_->name << std::endl;
        }
    }
    codec_context_ = avcodec_alloc_context3(codec_); // 分配编码器上下文
    if (!codec_context_)
    {
        throw std::runtime_error("failed to alloc avcodec context");
    }
    pkt_ = av_packet_alloc(); // 分配packet
    if (!pkt_)
    {
        throw std::runtime_error("failed to alloc avcodec packet");
    }

    /* 设置编码器参数 */
    codec_context_->bit_rate = 40000000; // 设置码率，单位为bps，数值越大越清晰
    codec_context_->height   = rows;
    codec_context_->width    = cols;
    AVRational rate;
    rate.num                     = 1;
    rate.den                     = 30;
    codec_context_->time_base    = rate;
    codec_context_->gop_size     = 0;
    codec_context_->max_b_frames = 0;
    codec_context_->pix_fmt      = AV_PIX_FMT_YUV420P;

    AVDictionary* dictParam = 0;
    (void)av_dict_set(&dictParam, "preset", "medium", 0);
    // (void)av_dict_set(&dictParam, "tune", "zerolatency", 0);
    (void)av_dict_set(&dictParam, "profile", "main", 0);
    // (void)av_opt_set(codec_context_->priv_data, "tune", "zerolatency", 0);
    (void)av_opt_set(codec_context_->priv_data, "x265-params", "keyint=0", 0); // 设置x265的GOP为1即全I帧

    /* 设置多线程编码 */
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

    if (avcodec_open2(codec_context_, codec_, NULL) < 0) // 打开编码器
    {
        throw std::runtime_error("failed to open avcodec context");
    }
    frame_ = av_frame_alloc(); // 分配视频帧内存
    if (!frame_)
    {
        av_frame_free(&frame_);
        throw std::runtime_error("failed to allocate video frame");
    }

    /* 设置视频帧数据 */
    frame_->format      = codec_context_->pix_fmt;
    frame_->width       = codec_context_->width;
    frame_->height      = codec_context_->height;
    frame_->linesize[0] = FFALIGN(cols, 16);
    frame_->linesize[1] = FFALIGN(cols, 16) / 2;
    frame_->linesize[2] = FFALIGN(cols, 16) / 2;

    if (av_frame_get_buffer(frame_, 0) < 0) // 分配视频帧缓冲
    {
        av_frame_free(&frame_); // 释放分配的视频帧内存
        throw std::runtime_error("failed to get frame buffer");
    }
}

VideoEncoderH26XCUDA::~VideoEncoderH26XCUDA() noexcept
{
    if (codec_context_)
    {
        (void)avcodec_send_frame(codec_context_, NULL);
        avcodec_free_context(&codec_context_);
    }
    if (frame_)
    {
        av_frame_free(&frame_);
    }
    if (pkt_)
    {
        av_packet_free(&pkt_);
    }
}

bool VideoEncoderH26XCUDA::Encode(uint8_t* frame, uint32_t frame_size, uint64_t timestamp)
{
    // auto start = std::chrono::high_resolution_clock::now();
    if (!frame_ || !frame_->data[0] || !frame_->data[1] || !frame_->data[2])
    {
        std::cerr << "av frame data is null" << std::endl;
        return false;
    }

    if (av_frame_make_writable(frame_) < 0) // 设置视频帧可写
    {
        std::cerr << "the frame data is un writable " << std::endl;
        return false;
    }

    if (!frame)
    {
        std::cerr << "the frame data is null " << std::endl;
        return false;
    }

    if (frame_size != cols_ * rows_ * 3 / 2)
    {
        std::cerr << "the frame size is invalid " << std::endl;
        return false;
    }

    try
    {
        /* 设置视频帧数据 */
        memcpy(frame_->data[0], frame, cols_ * rows_);
        memcpy(frame_->data[1], frame + cols_ * rows_, cols_ * rows_ / 4);
        memcpy(frame_->data[2], frame + cols_ * rows_ * 5 / 4, cols_ * rows_ / 4);

        frame_->pts = timestamp;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    int32_t result;

    result = avcodec_send_frame(codec_context_, frame_); // 发送视频帧
    if (result < 0)
    {
        std::cerr << "failed to send frame " << std::endl;
        return false;
    }

    while (result >= 0)
    {
        result = avcodec_receive_packet(codec_context_, pkt_); // 接收视频包
        if (result == AVERROR(EAGAIN) || result == AVERROR_EOF)
        {
            return true;
        }
        else if (result < 0)
        {
            std::cerr << "failed to receive packet " << std::endl;
            return false;
        }

        if (callback_)
        {
            // auto                                      end      = std::chrono::high_resolution_clock::now();
            // std::chrono::duration<double, std::milli> duration = end - start;
            // std::cout << "encode time: " << duration.count() << "ms" << std::endl;

            bool is_key_frame = pkt_->flags & AV_PKT_FLAG_KEY;
            callback_(pkt_->data, pkt_->size, pkt_->pts, is_key_frame); // 编码回调
        }

        av_packet_unref(pkt_); // 释放视频包
    }

    return true;
}
