#include <video_push.h>
#include <json/json.h>

VideoPush::VideoPush(uint32_t width, uint32_t height, uint32_t fps, std::string rtsp_url, CodecType codec_type)
    : width_(width)
    , height_(height)
    , fps_(fps)
    , rtsp_url_(rtsp_url)
    , fmt_ctx_(nullptr)
    , codec_type_(codec_type)
    , is_connected_(false)
{
    packet_buffer_ = std::shared_ptr<uint8_t>(new uint8_t[width * height * 3 / 2](), std::default_delete<uint8_t[]>());
}

VideoPush::~VideoPush()
{
    this->disconnectRtsp();
}

bool VideoPush::connectRtsp()
{
    // Initialize FFmpeg
    av_log_set_level(AV_LOG_ERROR);
    // av_log_set_level(AV_LOG_DEBUG); // Enable FFmpeg logging
    avformat_network_init();

    int ret = avformat_alloc_output_context2(&fmt_ctx_, nullptr, "rtsp", rtsp_url_.c_str());
    if (ret < 0)
    {
        std::string error = "Unable to allocate output context: " + getFFmpegError(ret);
        std::cerr << error << std::endl;
        is_connected_ = false;
        return false;
    }

    if (!fmt_ctx_)
    {
        std::string error = "Output format context is null";
        std::cerr << error << std::endl;
        is_connected_ = false;
        return false;
    }

    // Create output stream
    out_stream_ = avformat_new_stream(fmt_ctx_, nullptr);
    if (!out_stream_)
    {
        std::string error = "Unable to create new stream";
        avformat_free_context(fmt_ctx_);
        std::cerr << error << std::endl;
        is_connected_ = false;
        return false;
    }

    AVCodec* codec = nullptr;

    if (CodecType::H264 == codec_type_)
    {
        codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    }
    else if (CodecType::H265 == codec_type_)
    {
        codec = avcodec_find_encoder(AV_CODEC_ID_HEVC);
    }
    else
    {
        std::string error = "Unsupported codec type";
        avformat_free_context(fmt_ctx_);
        std::cerr << error << std::endl;
        is_connected_ = false;
        return false;
    }

    if (!codec)
    {
        std::string error = "HEVC encoder not found";
        avformat_free_context(fmt_ctx_);
        std::cerr << error << std::endl;
        is_connected_ = false;
        return false;
    }

    codec_ctx_ = avcodec_alloc_context3(codec);
    if (!codec_ctx_)
    {
        std::string error = "Unable to allocate codec context";
        avformat_free_context(fmt_ctx_);
        std::cerr << error << std::endl;
        is_connected_ = false;
        return false;
    }

    codec_ctx_->width     = width_;
    codec_ctx_->height    = height_;
    codec_ctx_->time_base = {1, fps_};
    codec_ctx_->pix_fmt   = AV_PIX_FMT_NV12;

    // Set global header flag if needed
    if (fmt_ctx_->oformat->flags & AVFMT_GLOBALHEADER)
    {
        codec_ctx_->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    }

    out_stream_->time_base = codec_ctx_->time_base;

    if (avcodec_parameters_from_context(out_stream_->codecpar, codec_ctx_) < 0)
    {
        std::string error = "Unable to copy codec parameters to output stream";
        avformat_free_context(fmt_ctx_);
        std::cerr << error << std::endl;
        is_connected_ = false;
        return false;
    }

    std::cout << "Opening output URL: " << rtsp_url_ << std::endl;

    av_dump_format(fmt_ctx_, 0, rtsp_url_.c_str(), 1);

    // Set max packet size for RTP
    AVDictionary* opts = nullptr;
    av_dict_set(&opts, "max_packet_size", "1400", 0);
    av_dict_set(&opts, "buffer_size", "655360", 0);
    av_dict_set(&opts, "rtsp_transport", "tcp", 0);

    // Write file header
    ret = avformat_write_header(fmt_ctx_, &opts);
    av_dict_free(&opts);
    if (ret < 0)
    {
        char errbuf[128];
        av_strerror(ret, errbuf, sizeof(errbuf));
        fprintf(stderr, "avformat_write_header failed: %s\n", errbuf);
    }

    if (!fmt_ctx_)
    {
        std::string error = "Output format context is null";
        std::cerr << error << std::endl;
        is_connected_ = false;
        return false;
    }

    is_connected_ = true;
    return true;
}
bool VideoPush::disconnectRtsp()
{
    // Write file trailer
    if (fmt_ctx_)
    {
        av_write_trailer(fmt_ctx_);
        avformat_free_context(fmt_ctx_);
    }

    if (codec_ctx_)
    {
        avcodec_free_context(&codec_ctx_);
    }

    is_connected_ = false;
    return true;
}

std::string VideoPush::getFFmpegError(int errnum)
{
    char buf[AV_ERROR_MAX_STRING_SIZE];
    av_strerror(errnum, buf, sizeof(buf));
    return std::string(buf);
}

bool VideoPush::pushFrame(const uint8_t* packet_data, uint32_t packet_size, uint64_t timestamp, FrameType frame_type)
{
    AVPacket* pkt = av_packet_alloc();
    if (!pkt)
    {
        std::cerr << "Failed to allocate AVPacket" << std::endl;
        return false;
    }
    pkt->data = const_cast<uint8_t*>(packet_data); // Set packet data
    pkt->size = packet_size;

    // 设置包的标志和流索引
    pkt->stream_index = out_stream_->index;

    AVRational input_time_base = {1, 1000000}; // 假设输入时间戳是微秒
    pkt->pts                   = av_rescale_q(timestamp, input_time_base, out_stream_->time_base);
    pkt->dts                   = pkt->pts;                                                       // DTS和PTS一样
    pkt->duration              = av_rescale_q(1, codec_ctx_->time_base, out_stream_->time_base); // 持续时间为一帧

    //////////////////////////////////////////////////////////
    if (FrameType::I == frame_type)
    {
        // 添加SEI
        Json::Value jsonObject;
        jsonObject["timestamp"] = timestamp;
        Json::StreamWriterBuilder writer;
        std::string               jsonString = Json::writeString(writer, jsonObject);

        std::string   extera_data = "66666666666666666666666666666666+" + jsonString;
        AVBSFContext* bsf         = nullptr;

        std::string              meta_data = "h264_metadata";
        const AVBitStreamFilter* filter    = av_bsf_get_by_name(meta_data.data());
        if (!filter)
        {
            std::cerr << "h264_metadata bitstream filter errorn" << std::endl;
        }
        else
        {
            int ret = av_bsf_alloc(filter, &bsf);
            av_opt_set(bsf->priv_data, "sei_user_data", extera_data.data(), AV_OPT_SEARCH_CHILDREN);
            ret = avcodec_parameters_copy(bsf->par_in, out_stream_->codecpar);
            ret = av_bsf_init(bsf);
            ret = avcodec_parameters_copy(out_stream_->codecpar, bsf->par_out);
            ret = av_bsf_send_packet(bsf, pkt);

            while (!ret)
            {
                ret = av_bsf_receive_packet(bsf, pkt);
            }
            av_bsf_free(&bsf);
            // std::cout << "Add SEI: " << extera_data << std::endl;
        }
    }
    ////////////////////////////////////////////////////////

    if (!fmt_ctx_)
    {
        std::cerr << "Output format context is null" << std::endl;
        return false;
    }

    // Write frame data to output stream
    int ret;
    int retries = 5;
    while (retries > 0)
    {
        ret = av_interleaved_write_frame(fmt_ctx_, pkt);
        if (ret >= 0)
        {
            return true;
            break; // Success
        }
        else
        {
            std::cerr << "Failed to write frame data, error code: " << ret << " (" << getFFmpegError(ret) << ")" << std::endl;
            retries--;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 / fps_)); // Wait before retrying
        }
    }

    if (ret < 0)
    {
        return false;
        std::cerr << "Failed to write frame data after retries, error code: " << ret << " (" << getFFmpegError(ret) << ")" << std::endl;
    }

    av_packet_free(&pkt);
}

void VideoPush::keepAlive()
{
    if (!fmt_ctx_)
    {
        std::cerr << "Output format context is null" << std::endl;
        return;
    }

    AVPacket* pkt = av_packet_alloc();
    if (!pkt)
    {
        std::cerr << "Failed to allocate AVPacket" << std::endl;
        return;
    }

    pkt->data = nullptr;
    pkt->size = 0;

    int ret = av_interleaved_write_frame(fmt_ctx_, pkt);
    if (ret < 0)
    {
        std::cerr << "Failed to send keep-alive packet, error code: " << ret << " (" << getFFmpegError(ret) << ")" << std::endl;
    }
}

bool VideoPush::isConnectedRtsp()
{
    return is_connected_;
}