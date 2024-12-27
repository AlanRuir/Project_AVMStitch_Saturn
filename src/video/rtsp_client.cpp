#include "rtsp_client.h"

RtspClient::RtspClient()
    : format_ctx_(nullptr)
    , codec_ctx_(nullptr)
    , video_stream_index_(-1)
    , callback_(nullptr)
    , logger_(rclcpp::get_logger("rtsp_client"))
{
    int ret = avformat_network_init(); // 初始化网络
    if (ret < 0)
    {
        RCLCPP_ERROR_STREAM(logger_, "Failed to initialize network.");
    }
}

RtspClient::~RtspClient()
{
    if (format_ctx_)
    {
        avformat_close_input(&format_ctx_);
    }

    if (codec_ctx_)
    {
        avcodec_free_context(&codec_ctx_);
    }

    avformat_network_deinit(); // 关闭网络
}

bool RtspClient::OpenStream(const std::string& url)
{
    format_ctx_        = avformat_alloc_context(); // 分配格式上下文
    AVDictionary* opts = nullptr;
    av_dict_set(&opts, "buffer_size", "2048000", 0);
    av_dict_set(&opts, "max_delay", "1000000", 0);
    av_dict_set(&opts, "rtsp_transport", "tcp", 0);

    RCLCPP_INFO_STREAM(logger_, "Opening stream: " << url);
    if (avformat_open_input(&format_ctx_, url.c_str(), nullptr, &opts) != 0)
    {
        RCLCPP_ERROR_STREAM(logger_, "Failed to open stream: " << url);
        return false;
    }

    if (avformat_find_stream_info(format_ctx_, nullptr) < 0)
    {
        RCLCPP_ERROR_STREAM(logger_, "Failed to find stream info.");
        return false;
    }

    // 打印流信息
    av_dump_format(format_ctx_, 0, url.c_str(), 0);

    for (uint32_t i = 0; i < format_ctx_->nb_streams; i++)
    {
        if (format_ctx_->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO)
        {
            video_stream_index_ = i;
            codec_ctx_          = avcodec_alloc_context3(nullptr);
            avcodec_parameters_to_context(codec_ctx_, format_ctx_->streams[i]->codecpar);
            AVCodec* codec = avcodec_find_decoder(codec_ctx_->codec_id);
            if (!codec || avcodec_open2(codec_ctx_, codec, nullptr) < 0)
            {
                RCLCPP_ERROR_STREAM(logger_, "Failed to open codec.");
                return false;
            }
            break;
        }
    }

    if (-1 == video_stream_index_)
    {
        RCLCPP_ERROR_STREAM(logger_, "No video stream found.");
        return false;
    }

    return true;
}

CodecType RtspClient::GetCodec()
{
    if (!codec_ctx_)
    {
        return CodecType::UNKNOWN_CODEC_TYPE;
    }

    if (AV_CODEC_ID_H264 == codec_ctx_->codec_id)
    {
        return CodecType::H264;
    }
    else if (AV_CODEC_ID_H265 == codec_ctx_->codec_id)
    {
        return CodecType::H265;
    }
    else
    {
        return CodecType::UNKNOWN_CODEC_TYPE;
    }
}

StreamInfo RtspClient::GetStreamInfo()
{
    // 如果codec_ctx_为空，直接返回默认的StreamInfo
    if (!codec_ctx_)
    {
        StreamInfo info;
        info.width  = 0;
        info.height = 0;
        info.fps    = 0; // 无法获取FPS时，设置为0
        return info;
    }

    StreamInfo info;
    info.width  = codec_ctx_->width;
    info.height = codec_ctx_->height;

    // 获取流的FPS
    if (format_ctx_ && format_ctx_->nb_streams > 0)
    {
        AVStream* stream = format_ctx_->streams[video_stream_index_];
        if (stream)
        {
            // 通过stream的avg_frame_rate获取帧率
            AVRational fps = stream->avg_frame_rate;

            // 计算帧率，防止为0的情况
            if (fps.den != 0 && fps.num != 0)
            {
                info.fps = av_q2d(fps); // av_q2d 将 AVRational 转换为浮点数
            }
            else
            {
                info.fps = 0; // 如果帧率无法计算，设置为0
            }
        }
        else
        {
            info.fps = 0; // 如果没有视频流，设置为0
        }
    }

    return info;
}

bool RtspClient::ReadFrame()
{
    if (!format_ctx_ || !codec_ctx_)
    {
        return false;
    }

    AVPacket* packet = av_packet_alloc();

    while (av_read_frame(format_ctx_, packet) >= 0)
    {
        if (packet->stream_index == video_stream_index_)
        {
            callback_(packet->data, packet->size);
        }
        av_packet_unref(packet);
    }

    av_packet_free(&packet);

    return true;
}

bool RtspClient::SetFrameCallback(const FrameCallback& callback)
{
    if (!callback)
    {
        return false;
    }

    callback_ = callback;

    return true;
}
