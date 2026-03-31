#pragma once

#include "log/log.h"
#include "native_camera_rgb_plugin_base/native_camera_rgb_struct_define.hpp"
#include "string"
#include "yjhlog/logger.h"
#include <cstdint>
#include <libavutil/pixfmt.h>
#include <memory>
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavcodec/packet.h>
#include <libavdevice/avdevice.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavutil/time.h>
#include <libswscale/swscale.h>
}

namespace controller_native {
    class EnDecoderClass {
    public:
        class SwsWrap {
        public:
            AVFrame *Transform(AVFrame *src_frame, AVPixelFormat dst_format) {
                if (!sws_ctx_) {
                    InitSwsContext((AVPixelFormat) src_frame->format, dst_format, src_frame->width, src_frame->height);
                }
                if (src_frame->format == dst_format) {
                    return src_frame;// 如果格式相同，直接返回
                }

                sws_scale(sws_ctx_, src_frame->data, src_frame->linesize, 0, src_frame->height, frame_->data,
                          frame_->linesize);
                return frame_;
            }

            void Clear() {
                if (sws_ctx_) {
                    sws_freeContext(sws_ctx_);
                    sws_ctx_ = nullptr;
                }
                if (frame_) {
                    av_frame_free(&frame_);
                    frame_ = nullptr;
                }
            }

        private:
            SwsContext *sws_ctx_ = nullptr;
            AVFrame *frame_      = nullptr;

            void InitSwsContext(AVPixelFormat src_format, AVPixelFormat dst_format, int width, int height) {
                frame_         = av_frame_alloc();
                frame_->format = dst_format;
                frame_->width  = width;
                frame_->height = height;
                av_frame_get_buffer(frame_, 0);

                sws_ctx_ = sws_getContext(width,       // 输入宽度
                                          height,      // 输入高度
                                          src_format,  // 输入格式（如 AV_PIX_FMT_YUV420P）
                                          width,       // 输出宽度（可缩放）
                                          height,      // 输出高度（可缩放）
                                          dst_format,  // 输出格式（RGB24）
                                          SWS_BILINEAR,// 缩放算法
                                          nullptr, nullptr, nullptr);
            }
        };
        class DeCoder {
        public:
            DeCoder()  = default;
            ~DeCoder() = default;

            bool Init(std::shared_ptr<yjhlog::Logger> logger, int width, int height, const AVCodecID &camera_format,
                      const AVPixelFormat &pixel_format) {
                logger_ = logger;
                LOGGER_INFO(logger_,
                            "DeCoder::Init: Initializing decoder with width: {}, height: {}, format: {}, (auto select, "
                            "not use)pixel_format: {}",
                            width, height, (int) camera_format, (int) pixel_format);
                return InitDecoder(width, height, camera_format, pixel_format);
            }

            bool Cleanup() {
                if (decode_ctx_) { avcodec_free_context(&decode_ctx_); }
                if (frame_) { av_frame_free(&frame_); }
                if (packet_) { av_packet_free(&packet_); }
                decode_ctx_ = nullptr;
                frame_      = nullptr;
                packet_     = nullptr;
                return true;
            }

            AVFrame *Decode(const void *buf, uint32_t len) {
                if (!decode_ctx_) {
                    LOGGER_ERROR(logger_, "DeCoder::Decode: Decoder is not initialized or has been closed.");
                    return nullptr;
                }

                // 将输入数据包填充到packet
                packet_->data = (uint8_t *) buf;
                packet_->size = len;

                if (!packet_->data || packet_->size <= 0) {
                    LOGGER_ERROR(logger_, "DeCoder::Decode: Invalid input data or size.");
                    return nullptr;
                }

                // 发送数据包给解码器
                int ret = 0;
                if (ret = avcodec_send_packet(decode_ctx_, packet_); ret < 0) {
                    LOGGER_ERROR(logger_, "DeCoder::Decode: Failed to send packet to decoder :{}.", ret);
                    return nullptr;
                }

                // 接收解码后的帧
                ret = avcodec_receive_frame(decode_ctx_, frame_);
                if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                    LOGGER_INFO(logger_, "DeCoder::Decode: Decoder needs more data or has reached end of stream.");
                    return nullptr;// 需要更多数据
                } else if (ret < 0) {
                    char error_buf[AV_ERROR_MAX_STRING_SIZE];
                    av_strerror(ret, error_buf, AV_ERROR_MAX_STRING_SIZE);
                    LOGGER_ERROR(logger_, "EnDecoderClass::Decode: Error receiving frame from decoder: {}", error_buf);
                    return nullptr;
                }
                // LOGGER_INFO(logger_, "DeCoder::Decode: Frame decoded successfully, width: {}, height: {}, format: {}",
                //             frame_->width, frame_->height, (int) frame_->format);
                return frame_;
            }

        private:
            std::shared_ptr<yjhlog::Logger> logger_ = nullptr;

            AVCodecContext *decode_ctx_ = nullptr;///< 解码器上下文
            const AVCodec *decoder_     = nullptr;///< 解码器
            AVFrame *frame_             = nullptr;///< 解码后的帧
            AVPacket *packet_           = nullptr;///< 数据包

            bool InitDecoder(int width, int height, const AVCodecID &camera_format, const AVPixelFormat &pixel_format) {
                decoder_ = avcodec_find_decoder(camera_format);
                if (!decoder_) {
                    LOGGER_ERROR(logger_, "DeCoder::InitDecoder: Failed to find {} decoder.", (int) camera_format);
                    return false;
                }
                decode_ctx_ = avcodec_alloc_context3(decoder_);
                if (!decode_ctx_) {
                    LOGGER_ERROR(logger_, "DeCoder::InitDecoder: Failed to allocate AVCodecContext.");
                    return false;
                }
                decode_ctx_->width  = width;
                decode_ctx_->height = height;
                // decode_ctx_->pix_fmt = pixel_format;
                if (avcodec_open2(decode_ctx_, decoder_, nullptr) < 0) {
                    LOGGER_ERROR(logger_, "DeCoder::InitDecoder: Failed to open decoder. {}x{} pixel_format: {}", width,
                                 height, (int) pixel_format);
                    return false;
                }
                frame_  = av_frame_alloc();
                packet_ = av_packet_alloc();
                if (!frame_ || !packet_) {
                    LOGGER_ERROR(logger_, "DeCoder::InitDecoder: Failed to allocate frame or packet.");
                    return false;
                }
                return true;
            }
        };

        class EnCoder {
        public:
            struct BaseInfo {
                AVCodecID dst_format = AV_CODEC_ID_NONE;///< 摄像头格式
                int64_t bit_rate     = 0;
                int fps              = 30;///< 帧率
                int width            = 0;
                int height           = 0;
            };
            EnCoder()  = default;
            ~EnCoder() = default;
            bool Init(std::shared_ptr<yjhlog::Logger> logger, BaseInfo base_info) {
                logger_    = logger;
                base_info_ = base_info;
                return true;
            }

            bool Cleanup() {
                if (encode_ctx_) { avcodec_free_context(&encode_ctx_); }
                if (packet_) { av_packet_free(&packet_); }
                encode_ctx_ = nullptr;
                packet_     = nullptr;
                return true;
            }

            AVPacket *Encode(AVFrame *frame) {
                if (!encode_ctx_) {
                    if (!InitEncoder(frame, base_info_.dst_format)) {
                        LOGGER_ERROR(logger_, "EnCoder::Encode: Encoder is not initialized or has been closed.");
                        return nullptr;
                    }
                }
                frame->pts = ++pts_counter_;
                // 发送帧给编码器
                if (avcodec_send_frame(encode_ctx_, frame) < 0) {
                    LOGGER_ERROR(logger_, "EnCoder::Encode: Failed to send frame to encoder.");
                    return nullptr;
                }

                // 接收编码后的包
                int ret = avcodec_receive_packet(encode_ctx_, packet_);
                if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                    LOGGER_INFO(logger_, "EnCoder::Encode: Encoder needs more frames or has reached end of stream.");
                    return nullptr;// 需要更多帧
                } else if (ret < 0) {
                    char error_buf[AV_ERROR_MAX_STRING_SIZE];
                    av_strerror(ret, error_buf, AV_ERROR_MAX_STRING_SIZE);
                    LOGGER_ERROR(logger_, "EnCoder::Encode: Error receiving packet from encoder: {}", error_buf);
                    return nullptr;
                }
                return packet_;
            }

            const AVCodec *GetAVCodec() { return encoder_; }

            AVCodecContext *GetAVCodecContext() { return encode_ctx_; }

        private:
            std::shared_ptr<yjhlog::Logger> logger_ = nullptr;
            AVCodecContext *encode_ctx_             = nullptr;///< 编码器上下文
            const AVCodec *encoder_                 = nullptr;///< 编码器
            AVPacket *packet_                       = nullptr;///< 数据包
            uint64_t pts_counter_                   = 0;      ///< pts计数器
            BaseInfo base_info_;                              ///< 基础信息

            bool InitEncoder(AVFrame *frame, const AVCodecID &dst_format) {
                encoder_ = avcodec_find_encoder(dst_format);
                if (!encoder_) {
                    LOGGER_ERROR(logger_, "EnCoder::InitEncoder: Failed to find {} encoder.", (int) dst_format);
                    return false;
                }
                encode_ctx_ = avcodec_alloc_context3(encoder_);
                if (!encode_ctx_) {
                    LOGGER_ERROR(logger_, "EnCoder::InitEncoder: Failed to allocate AVCodecContext.");
                    return false;
                }
                encode_ctx_->width     = base_info_.width == 0 ? frame->width : base_info_.width;
                encode_ctx_->height    = base_info_.height == 0 ? frame->height : base_info_.height;
                encode_ctx_->bit_rate  = base_info_.bit_rate;// expect bits / second,after encoding
                encode_ctx_->time_base = {1, base_info_.fps};
                encode_ctx_->framerate = {base_info_.fps, 1};
                // encode_ctx_->bit_rate = 4 * 1024 * 1024;
                encode_ctx_->pix_fmt = (AVPixelFormat) frame->format;///< 使用输入帧的像素格式

                if (dst_format == AV_CODEC_ID_H264) {
                    // H.264特定参数
                    encode_ctx_->gop_size     = base_info_.fps * 2;//关键帧间隔
                    encode_ctx_->max_b_frames = 0;                 //不使用B帧
                    av_opt_set(encode_ctx_->priv_data, "preset", "fast", 0);
                    av_opt_set(encode_ctx_->priv_data, "tune", "zerolatency", 0);
                    av_opt_set(encode_ctx_->priv_data, "profile", "baseline", 0);
                }

                if (avcodec_open2(encode_ctx_, encoder_, nullptr) < 0) {
                    LOGGER_ERROR(logger_, "EnCoder::InitEncoder: Failed to open encoder.");
                    return false;
                }
                packet_ = av_packet_alloc();
                if (!packet_) {
                    LOGGER_ERROR(logger_, "EnCoder::InitEncoder: Failed to allocate frame or packet.");
                    return false;
                }
                return true;
            }
        };

        EnDecoderClass()  = default;
        ~EnDecoderClass() = default;

        enum class CameraFormat { NONE, MJPEG, H264 };
        bool Init(int width, int height, int fps, const std::string &camera_format, int dst_width, int dst_height,
                  int bit_rate, const std::string &dst_format);
        void TakePhoto(AVFrame *frame_bgr, const std::string &file_name);
        void Close();
        void SetLogger(std::shared_ptr<yjhlog::Logger> logger) { logger_ = logger; }
        AVCodecID GetCameraFormat() const { return camera_format_; }

        AVFrame *Decode(const void *buf, uint32_t len) {
            if (!is_working_) {
                LOGGER_ERROR(logger_, "EnDecoderClass::Decode: Decoder is not initialized or has been closed.");
                return nullptr;
            }
            return decoder_.Decode(buf, len);
        }

        AVPacket *Encode(AVFrame *yuv420p) {
            if (!is_working_) {
                LOGGER_ERROR(logger_, "EnDecoderClass::Encode: Encoder is not initialized or has been closed.");
                return {};
            }
            return encoder_.Encode(yuv420p);
        }

        AVFrame *TransformToGBR24(AVFrame *src_frame) {
            if (!is_working_) {
                LOGGER_ERROR(logger_, "EnDecoderClass::TransformToRGB: Not initialized or has been closed.");
                return nullptr;
            }
            return to_rgb_.Transform(src_frame, AV_PIX_FMT_BGR24);
        }

        AVFrame *TransformToYUV420(AVFrame *src_frame) {
            if (!is_working_) {
                LOGGER_ERROR(logger_, "EnDecoderClass::TransformToRGB: Not initialized or has been closed.");
                return nullptr;
            }
            return to_yuv420_.Transform(src_frame, AV_PIX_FMT_YUV420P);
        }

    private:
        std::shared_ptr<yjhlog::Logger> logger_ = nullptr;
        EnCoder encoder_;                           ///< 编码器
        DeCoder decoder_;                           ///< 解码器
        SwsWrap to_rgb_;                            ///< RGB转换器
        SwsWrap to_yuv420_;                         ///< RGB转换器
        bool is_working_         = false;           ///< 是否正在工作
        AVCodecID camera_format_ = AV_CODEC_ID_NONE;///< 摄像头格式

        AVCodecID GetCodecID(const std::string &format);
        AVPixelFormat GetPixelFormat(const std::string &format);
        bool initializeFramesAndScaler(int width, int height);
    };
}// namespace controller_native