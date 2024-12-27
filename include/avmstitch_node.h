#ifndef __AVMSTITCH_NODE_H__
#define __AVMSTITCH_NODE_H__

#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>

#include "rtsp_client.h"
#include "video_decoder_h26x_base.h"
#include "video_decoder_h26x_cpu.h"
#include "video_decoder_h26x_cuda.h"

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
    };

public:
    void EncodedDataHandler(std::shared_ptr<Instance> instance, uint8_t* data, size_t size);
    void ReceiveEncodedData(std::shared_ptr<Instance> instance);
    void DecodedDataHandler(std::shared_ptr<Instance> instance, uint8_t** data, int* size, uint32_t frame_num, uint32_t time_stamp);

private:
    bool                                 has_cuda_;
    std::list<std::shared_ptr<Instance>> instances_;
};

#endif // __AVMSTITCH_NODE_H__