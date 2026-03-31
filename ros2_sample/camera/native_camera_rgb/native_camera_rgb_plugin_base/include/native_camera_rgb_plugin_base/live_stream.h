#pragma once

#include "log/log.h"
#include "native_camera_rgb_plugin_base/endecoder.h"
#include <cstring>
#include <iostream>
#include <mutex>
#include <stdint.h>
#include <string>
#include <vector>
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

namespace controller_native {
    class LiveStreamClass {
    public:
        struct StreamConfig {
            std::string url;
            int width       = 1920;
            int height      = 1080;
            int fps         = 25;
            int64_t bitrate = 1 * 1024 * 1024;// 2Mbps
        };
        LiveStreamClass()  = default;
        ~LiveStreamClass() = default;

        bool Init(const StreamConfig &config);
        void Write(const void *buf, uint32_t len);
        void Write(AVFrame *frame);
        void Close();

        bool Init_Need_Test(const std::string &url, int32_t width, int32_t height, int32_t fps, std::string type) {
            std::lock_guard<std::mutex> lock(mutex_);
            // 初始化网络
            avformat_network_init();

            // 分配输出上下文
            if (avformat_alloc_output_context2(&av_format_context_, nullptr, GetTypeString(false, type).c_str(),
                                               url.c_str()) < 0) {
                return false;
            }

            // 创建视频流
            if (av_stream_ = avformat_new_stream(av_format_context_, nullptr); av_stream_ == nullptr) {
                CloseUnsafe();
                return false;
            }
            // 配置流参数
            av_stream_->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
            av_stream_->codecpar->codec_id   = AV_CODEC_ID_H264;
            av_stream_->codecpar->width      = width;
            av_stream_->codecpar->height     = height;
            av_stream_->codecpar->codec_tag  = 0;
            av_stream_->avg_frame_rate       = {10, 1};// 10 FPS
            av_stream_->avg_frame_rate       = {fps, 1};
            av_stream_->time_base            = {1, 90000};   // 设置时间基准,H.264 RTP 标准时间基准
            av_stream_->codecpar->bit_rate   = 80 * 1024 * 8;//设置一些H.264特定参数
            av_stream_->codecpar->format     = AV_PIX_FMT_YUV420P;

            // 打开输出
            if (avio_open(&av_format_context_->pb, url.c_str(), AVIO_FLAG_WRITE) < 0) {
                CloseUnsafe();
                return false;
            }

            // 写入文件头
            if (avformat_write_header(av_format_context_, nullptr) < 0) {
                CloseUnsafe();
                return false;
            }

            is_working_ = true;
            return true;
        }
        bool TestDecodeH264(uint8_t *h264_data, int h264_size, uint8_t *output_buffer) {
            // 初始化 FFmpeg H.264 解码器
            const AVCodec *codec = avcodec_find_decoder(AV_CODEC_ID_H264);
            if (!codec) {
                std::cerr << "H.264 decoder not found" << std::endl;
                return false;
            }

            AVCodecContext *codec_ctx = avcodec_alloc_context3(codec);
            if (!codec_ctx) {
                std::cerr << "Failed to allocate codec context" << std::endl;
                return false;
            }

            if (avcodec_open2(codec_ctx, codec, nullptr) < 0) {
                std::cerr << "Failed to open codec" << std::endl;
                avcodec_free_context(&codec_ctx);
                return false;
            }

            // 准备解码输入
            AVPacket *packet = av_packet_alloc();
            packet->data     = h264_data;
            packet->size     = h264_size;

            // 解码
            AVFrame *frame = av_frame_alloc();
            int ret        = avcodec_send_packet(codec_ctx, packet);
            if (ret < 0) {
                std::cerr << "Error sending packet" << std::endl;
                av_packet_free(&packet);
                av_frame_free(&frame);
                avcodec_free_context(&codec_ctx);
                return false;
            }

            ret = avcodec_receive_frame(codec_ctx, frame);
            if (ret < 0) {
                std::cerr << "Error receiving frame" << std::endl;
                av_packet_free(&packet);
                av_frame_free(&frame);
                avcodec_free_context(&codec_ctx);
                return false;
            }

            // 转换 YUV420P → BGR（OpenCV 格式）
            SwsContext *sws_ctx =
                    sws_getContext(frame->width, frame->height, static_cast<AVPixelFormat>(frame->format), frame->width,
                                   frame->height, AV_PIX_FMT_BGR24, SWS_BILINEAR, nullptr, nullptr, nullptr);

            if (!sws_ctx) {
                std::cerr << "Failed to create SwsContext" << std::endl;
                av_packet_free(&packet);
                av_frame_free(&frame);
                avcodec_free_context(&codec_ctx);
                return false;
            }

            // 设置输出数据指针，直接写入到用户提供的buffer
            uint8_t *dst_data[4] = {output_buffer, nullptr, nullptr, nullptr};
            int dst_linesize[4]  = {frame->width * 3, 0, 0, 0};// BGR24 format: 3 bytes per pixel

            // 执行格式转换，数据直接写入output_buffer
            sws_scale(sws_ctx, frame->data, frame->linesize, 0, frame->height, dst_data, dst_linesize);

            // 释放资源
            av_packet_free(&packet);
            av_frame_free(&frame);
            avcodec_free_context(&codec_ctx);
            sws_freeContext(sws_ctx);

            return true;
        }
        void SetLogger(std::shared_ptr<yjhlog::Logger> logger) { logger_ = logger; }

    private:
        std::shared_ptr<yjhlog::Logger> logger_ = nullptr;
        AVFormatContext *av_format_context_     = nullptr;
        AVStream *av_stream_                    = nullptr;
        // const AVCodec *av_codec_ = nullptr;
        // AVCodecContext *av_codec_context_ = nullptr;
        // AVPacket *packet_ = nullptr;
        std::mutex mutex_;
        EnDecoderClass::EnCoder encoder_;///< 编码器

        bool is_working_ = false;
        int64_t pts_cnt_ = 0;
        StreamConfig config_;

        // RTMP/FLV specifics: H264 must be AVCC + extradata(SPS/PPS) in AVCDecoderConfigurationRecord.
        bool is_flv_         = false;
        bool header_written_ = false;
        std::vector<uint8_t> sps_;
        std::vector<uint8_t> pps_;

        std::string GetTypeString(bool input_format, std::string type);
        void CloseUnsafe();

        bool InitStream(const AVCodec *av_codec, AVCodecContext *ac_codec_context);
    };
}// namespace controller_native