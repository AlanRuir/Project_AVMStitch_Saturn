#include <cuda_runtime.h>
#include "avmstitch_node.h"

AvmstitchNode::AvmstitchNode()
    : Node("avmstitch_node")
{
    // 检查GPU是否可用，如果可用则使用CUDA加速解码
    int         cuda_device_count = 0;
    cudaError_t cuda_status       = cudaGetDeviceCount(&cuda_device_count);

    if (cuda_status != cudaSuccess || cuda_device_count == 0)
    {
        RCLCPP_INFO(this->get_logger(), "No GPU supporting CUDA detected, CPU version decoder and encoder will be used");
        has_cuda_ = false;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Detected GPU that supports CUDA, will use CUDA version decoder and encoder");
        has_cuda_ = true;
    }

    RCLCPP_INFO(this->get_logger(), "Avmstitch node started.");

    for (size_t i = 0; i < 4; ++i)
    {
        std::shared_ptr<Instance> instance = std::make_shared<Instance>();
        instance->rtsp_client_             = std::make_shared<RtspClient>();
        instance->rtsp_url_                = std::string("rtsp://127.0.0.1:554/cam/" + std::to_string(i));
        bool result                        = instance->rtsp_client_->OpenStream(instance->rtsp_url_);
        if (!result)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open RTSP stream");
        }

        CodecType   codec_type = instance->rtsp_client_->GetCodec();
        std::string codec_name;
        switch (codec_type)
        {
        case CodecType::H264:
            codec_name = "H.264";
            break;
        case CodecType::H265:
            codec_name = "H.265";
            break;
        default:
            throw std::runtime_error("Faild to find codec");
            break;
        }
        RCLCPP_INFO(this->get_logger(), "Codec type: %s", codec_name.c_str());

        StreamInfo stream_info = instance->rtsp_client_->GetStreamInfo();
        RCLCPP_INFO(this->get_logger(), "Stream info: %dx%d@%d", stream_info.width, stream_info.height, stream_info.fps);

        // 检查GPU是否可用，如果不可用则使用CUDA加速解码
        if (!has_cuda_)
        {
            instance->video_decoder_ = std::make_shared<VideoDecoderH26xCPU>(stream_info.width, stream_info.height, codec_type); // 创建CPU解码器
        }
        else
        {
            instance->video_decoder_ = std::make_shared<VideoDecoderH26xCUDA>(stream_info.width, stream_info.height, codec_type); // 创建CUDA解码器
        }

        if (!instance->video_decoder_)
        {
            throw std::runtime_error("Failed to create video decoder");
        }

        std::function<void(uint8_t**, int*, uint32_t, uint32_t)> decoded_data_callback = std::bind(&AvmstitchNode::DecodedDataHandler, this, instance, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
        instance->video_decoder_->SetCallback(decoded_data_callback);

        std::function<void(uint8_t*, size_t)> encoded_data_callback = std::bind(&AvmstitchNode::EncodedDataHandler, this, instance, std::placeholders::_1, std::placeholders::_2);
        instance->rtsp_client_->SetFrameCallback(encoded_data_callback);

        instance->rtsp_client_thread_ = std::make_shared<std::thread>(&AvmstitchNode::ReceiveEncodedData, this, instance);

        instances_.push_back(instance);
    }
}

AvmstitchNode::~AvmstitchNode()
{
}

void AvmstitchNode::EncodedDataHandler(std::shared_ptr<Instance> instance, uint8_t* data, size_t size)
{
    instance->video_decoder_->Decode(data, size, 0LU);
}

void AvmstitchNode::DecodedDataHandler(std::shared_ptr<Instance> instance, uint8_t** data, int* size, uint32_t frame_num, uint32_t time_stamp)
{
    RCLCPP_INFO(this->get_logger(), "instance->url: %s, Decoded frame num: %d, time stamp: %d", instance->rtsp_url_.c_str(), frame_num, time_stamp);
    (void)data;
    (void)size;
    // /*将解码后的数据写入文件*/
    // FILE* file = fopen("output.yuv", "ab+");
    // fwrite(data[0], size[0] * 1080, 1, file);
    // fwrite(data[1], size[1] * 1080 / 2, 1, file);
    // fwrite(data[2], size[2] * 1080 / 2, 1, file);
    // fclose(file);
}

void AvmstitchNode::ReceiveEncodedData(std::shared_ptr<Instance> instance)
{
    while (true)
    {
        instance->rtsp_client_->ReadFrame();
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AvmstitchNode>());
    rclcpp::shutdown();
    return 0;
}