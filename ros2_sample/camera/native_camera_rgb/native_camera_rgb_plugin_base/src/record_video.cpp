#include "native_camera_rgb_plugin_base/record_video.h"
#include "log/log.h"
#include "native_camera_rgb_plugin_base/utils.hpp"
#include <mutex>
#include <string>

namespace controller_native {
    bool RecordVideoClass::Init(int fps, const std::string &file_dir, const std::string &src_format,
                                const std::string &dst_format) {
        (void) src_format;// Record only supports H264 format
        (void) dst_format;// Record only supports FLV format
        std::lock_guard<std::mutex> lock(mutex_);
        if (is_working_) {
            LOGGER_ERROR(logger_, "RecordVideoClass: Already initialized, cannot re-initialize.");
            return false;
        }

        fps_ = fps;
        auto tmp_src_format = GetTypeString(true, "H264");
        auto tmp_dst_format = GetTypeString(false, "FLV");

        if (auto [flag, file_name] = CameraUtils::CreateDirectoriesAndGenFileName(file_dir, "." + tmp_dst_format);
            flag) {
            file_name_ = file_name;
        } else {
            LOGGER_ERROR(logger_, "RecordVideoClass: Failed to create directories or generate file name.");
            return false;
        }

        encoder_.Init(logger_, {AV_CODEC_ID_H264, 80 * 1024 * 8, fps});
        avformat_alloc_output_context2(&av_format_ctx_, NULL, tmp_dst_format.c_str(), file_name_.c_str());
        if (!av_format_ctx_) {
            LOGGER_ERROR(logger_, "RecordVideoClass: Failed to allocate av_format_ctx_.");
            return false;
        }
        is_working_ = true;
        return true;
    }

    std::string RecordVideoClass::GetTypeString(bool input_format, std::string type) {
        std::transform(type.begin(), type.end(), type.begin(), [](unsigned char c) { return std::toupper(c); });

        if (input_format) {
            std::string dst_type = "H264";
            if (type == "H264") { dst_type = "H264"; }
            return dst_type;
        } else {
            std::string dst_type = "flv";
            if (type == "FLV") dst_type = "flv";
            return dst_type;
        }
    }

    bool RecordVideoClass::InitStream(const AVCodec *av_codec, AVCodecContext *ac_codec_context) {
        // 创建视频流
        av_stream_ = avformat_new_stream(av_format_ctx_, av_codec);
        if (!av_stream_) {
            LOGGER_ERROR(logger_, "RecordVideoClass: Failed to create new video stream.");
            return false;
        }

        if (avcodec_parameters_from_context(av_stream_->codecpar, ac_codec_context) < 0) {
            LOGGER_ERROR(logger_, "RecordVideoClass: Failed to set codec parameters from context.");
            return false;
        }
        av_stream_->time_base = {1, 1000};// 设置时间基准为毫秒

        if (avio_open(&av_format_ctx_->pb, file_name_.c_str(), AVIO_FLAG_WRITE) < 0) {
            LOGGER_ERROR(logger_, "RecordVideoClass: Failed to open output file: {}", file_name_);
            return false;
        }

        if (avformat_write_header(av_format_ctx_, NULL) < 0) {
            LOGGER_ERROR(logger_, "RecordVideoClass: Failed to write header to output file: {}", file_name_);
            return false;
        }
        return true;
    }

    void RecordVideoClass::RecordVideo(const void *buf, uint32_t len) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!is_working_) {
            LOGGER_ERROR(logger_, "RecordVideoClass: Not initialized, cannot record video.");
            return;
        }

        if (!started_) {
            start_time_ = std::chrono::steady_clock::now();
            started_ = true;
        }

        AVPacket *pkt = av_packet_alloc();

        pkt->data = (uint8_t *) buf;
        pkt->size = len;

        // pkt->pts = frame_cnt_++ * 1000 / fps_;
        pkt->pts = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time_)
                           .count();
        pkt->dts = pkt->pts;
        pkt->stream_index = 0;

        av_interleaved_write_frame(av_format_ctx_, pkt);
        av_packet_free(&pkt);
    }

    void RecordVideoClass::RecordVideo(AVFrame *yuv420p) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!is_working_) {
            LOGGER_ERROR(logger_, "RecordVideoClass: Not initialized, cannot record video.");
            return;
        }

        auto pkt = encoder_.Encode(yuv420p);
        if (!av_stream_) {
            if (!InitStream(encoder_.GetAVCodec(), encoder_.GetAVCodecContext())) {
                LOGGER_ERROR(logger_, "RecordVideoClass: Failed to initialize stream.");
                return;
            }
        }
        // pkt->pts = frame_cnt_++ * 1000 / fps_;
        if (pkt) {
            pkt->pts = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                             start_time_)
                               .count();
            pkt->dts = pkt->pts;
            pkt->stream_index = av_stream_->index;

            av_interleaved_write_frame(av_format_ctx_, pkt);
            av_packet_unref(pkt);
        }
    }

    std::string RecordVideoClass::Close() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (encoder_.GetAVCodecContext()) { encoder_.Cleanup(); }
        if (av_format_ctx_) {
            av_write_trailer(av_format_ctx_);
            avio_close(av_format_ctx_->pb);
            avformat_free_context(av_format_ctx_);
            av_stream_ = nullptr;

            av_format_ctx_ = NULL;
            frame_cnt_ = 0;
            is_working_ = false;
            fps_ = 0;
            LOGGER_INFO(logger_, "RecordVideoClass: Video file saved successfully");
            return file_name_;
        }
        return "";
    }
}// namespace controller_native