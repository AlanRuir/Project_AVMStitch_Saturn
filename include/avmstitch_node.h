#ifndef __AVMSTITCH_NODE_H__
#define __AVMSTITCH_NODE_H__

#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <rclcpp/rclcpp.hpp>

#include "rtsp_client.h"
#include "video_decoder_h26x_base.h"
#include "video_decoder_h26x_cpu.h"
#include "video_decoder_h26x_cuda.h"
#include "video_encoder_h26x_base.h"
#include "video_encoder_h26x_cpu.h"
#include "video_encoder_h26x_cuda.h"
#include "image.h"
#include "avm_stitching_interface.h"
#include "video_push.h"

class AvmstitchNode : public rclcpp::Node
{
public:
    AvmstitchNode();
    ~AvmstitchNode();

private:
    class Instance
    {
    public:
        Instance()
        {
        }

    public:
        std::shared_ptr<std::thread>          rtsp_client_thread_;
        std::shared_ptr<RtspClient>           rtsp_client_;
        std::shared_ptr<VideoDecoderH26xBase> video_decoder_;
        std::string                           rtsp_url_;
        std::queue<std::shared_ptr<Image>>    image_queue_;
        std::mutex                            image_queue_mutex_;
        struct StreamInfo                     stream_info_;
        CodecType                             codec_type_;
        int                                   view_id_;
    };

public:
    void EncodedDataHandler(std::shared_ptr<Instance> instance, uint8_t* data, size_t size);
    void ReceiveEncodedData(std::shared_ptr<Instance> instance);
    void DecodedDataHandler(std::shared_ptr<Instance> instance, uint8_t** data, int* size, uint64_t frame_num, uint64_t time_stamp);
    void AvmStitchThread();
    void EncodeHandler(uint8_t* data, uint32_t size, uint64_t timestamp, bool is_key_frame);

private:
    bool                                   has_cuda_;
    std::list<std::shared_ptr<Instance>>   instances_;
    std::shared_ptr<std::thread>           avm_stitch_thread_;
    std::condition_variable                avm_condition_;
    std::mutex                             avm_mtx_;
    bool                                   is_running_;
    std::shared_ptr<AVMStitchingInterface> avm_stitching_instance_;
    std::shared_ptr<VideoEncoderH26XBase>  video_encoder_;
    std::shared_ptr<VideoPush>             video_push_;
    // std::shared_ptr<uint8_t>               avm_data_cache_buffer_;
};

#endif // __AVMSTITCH_NODE_H__