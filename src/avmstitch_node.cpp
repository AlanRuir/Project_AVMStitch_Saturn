#include <cuda_runtime.h>
#include "avmstitch_node.h"

AvmstitchNode::AvmstitchNode()
    : Node("avmstitch_node")
    , is_running_(false)
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
        instance->view_id_                 = i;
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
        instance->codec_type_ = codec_type;

        StreamInfo stream_info = instance->rtsp_client_->GetStreamInfo();
        instance->stream_info_ = stream_info;
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

        std::function<void(uint8_t**, int*, uint64_t, uint64_t)> decoded_data_callback = std::bind(&AvmstitchNode::DecodedDataHandler, this, instance, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
        instance->video_decoder_->SetCallback(decoded_data_callback);

        std::function<void(uint8_t*, size_t)> encoded_data_callback = std::bind(&AvmstitchNode::EncodedDataHandler, this, instance, std::placeholders::_1, std::placeholders::_2);
        instance->rtsp_client_->SetFrameCallback(encoded_data_callback);

        instance->rtsp_client_thread_ = std::make_shared<std::thread>(&AvmstitchNode::ReceiveEncodedData, this, instance);

        instances_.push_back(instance);
    }

    avm_stitch_thread_ = std::make_shared<std::thread>(&AvmstitchNode::AvmStitchThread, this);
    RCLCPP_INFO(this->get_logger(), "AvmStitchNode started");
    is_running_ = true;
}

AvmstitchNode::~AvmstitchNode()
{
    is_running_ = false;
}

void AvmstitchNode::EncodedDataHandler(std::shared_ptr<Instance> instance, uint8_t* data, size_t size)
{
    instance->video_decoder_->Decode(data, size, 0LU);
}

void AvmstitchNode::DecodedDataHandler(std::shared_ptr<Instance> instance, uint8_t** data, int* size, uint64_t frame_num, uint64_t time_stamp)
{
    {
        std::unique_lock<std::mutex> lock(instance->image_queue_mutex_);
        std::shared_ptr<Image>       image = std::make_shared<Image>(instance->stream_info_.width, instance->stream_info_.height, PixelFormat::YUV420P, frame_num, time_stamp);
        memcpy(image->GetData(), data[0], size[0] * instance->stream_info_.height);
        memcpy(image->GetData() + size[0] * instance->stream_info_.height, data[1], size[1] * instance->stream_info_.height / 2);
        memcpy(image->GetData() + size[0] * instance->stream_info_.height + size[1] * instance->stream_info_.height / 2, data[2], size[2] * instance->stream_info_.height / 2);
        instance->image_queue_.push(image);
    }

    avm_condition_.notify_one();
}

void AvmstitchNode::ReceiveEncodedData(std::shared_ptr<Instance> instance)
{
    while (true)
    {
        if (!is_running_)
        {
            continue;
        }

        instance->rtsp_client_->ReadFrame();
    }
}

void AvmstitchNode::AvmStitchThread()
{
    while (true)
    {
        std::unique_lock<std::mutex> lock(avm_mtx_);
        avm_condition_.wait(lock, [this]() {
            for (auto& instance : instances_)
            {
                if (instance->image_queue_.empty())
                {
                    return false;
                }
            }

            return true;
        });

        // 找到所有队列中最小的帧编号
        uint64_t min_frame = UINT_MAX;
        for (auto& instance : instances_)
        {
            if (!instance->image_queue_.empty())
            {
                min_frame = std::min(min_frame, instance->image_queue_.front()->GetFrameCount());
            }
        }

        // 检查是否有所有流的帧编号都等于最小帧编号
        bool all_synced = true;
        for (auto& instance : instances_)
        {
            if (instance->image_queue_.empty() || instance->image_queue_.front()->GetFrameCount() != min_frame)
            {
                all_synced = false;
                break;
            }
        }

        if (all_synced)
        {
            // 输出当前帧
            // RCLCPP_INFO(this->get_logger(), "[");
            // for (auto& instance : instances_)
            // {
            //     RCLCPP_INFO(this->get_logger(), "%ld, ", instance->image_queue_.front()->GetFrameCount());

            //     // FILE* file = fopen((std::to_string(instance->image_queue_.front()->GetFrameCount()) + "_" + std::to_string(instance->view_id_) + ".yuv420p").c_str(), "wb");
            //     // fwrite(instance->image_queue_.front()->GetData(), instance->image_queue_.front()->GetSize(), 1, file);
            //     // fclose(file);
            // }
            // RCLCPP_INFO(this->get_logger(), "]");
            std::stringstream ss;
            ss << "[";
            auto it = instances_.begin();
            while (it != instances_.end())
            {
                ss << (*it)->image_queue_.front()->GetFrameCount();
                ++it;
                if (it != instances_.end())
                {
                    ss << ", "; // 逗号只有在当前不是最后一个元素时才打印
                }
            }
            ss << "]";
            // 通过一次RCLCPP_INFO调用输出整个内容
            RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

            // 移除所有队列中的当前帧
            for (auto& instance : instances_)
            {
                if (!instance->image_queue_.empty())
                {
                    instance->image_queue_.pop();
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "image_queue_ is empty");
                }
                // std::cout << "frame_queues[" << i << "] size: " << frame_queues[i].size() << std::endl;
            }
        }
        else
        {
            // 如果不同步，丢弃最小帧编号之前的帧
            for (auto& instance : instances_)
            {
                while (!instance->image_queue_.empty() && instance->image_queue_.front()->GetFrameCount() < min_frame)
                {
                    instance->image_queue_.pop();
                    // std::cout << "frame_queues[" << i << "] size: " << frame_queues[i].size() << std::endl;
                }
            }
        }
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AvmstitchNode>());
    rclcpp::shutdown();
    return 0;
}