#pragma once

#include "log/log.h"
#include "native_camera_rgb_plugin_base/endecoder.h"
#include <chrono>
#include <mutex>
#include <stdint.h>
#include <string>
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/opt.h>
}

namespace controller_native {
    class RecordVideoClass {
    public:
        RecordVideoClass() = default;
        virtual ~RecordVideoClass() = default;
        bool Init(int fps, const std::string &file_dir, const std::string &src_format, const std::string &dst_format);
        void RecordVideo(AVFrame *yuv420p);
        void RecordVideo(const void *buf, uint32_t len);
        std::string Close();
        void SetLogger(std::shared_ptr<yjhlog::Logger> logger) { logger_ = logger; }

    private:
        std::shared_ptr<yjhlog::Logger> logger_ = nullptr;
        AVFormatContext *av_format_ctx_ = nullptr;
        AVStream *av_stream_ = nullptr;
        int64_t frame_cnt_ = 0;
        // const AVCodec *av_codec_ = nullptr;
        // AVCodecContext *ac_codec_context_ = nullptr;
        bool is_working_ = false;
        int fps_ = 0;
        std::mutex mutex_;
        std::string file_name_ = "";
        bool started_ = false;
        std::chrono::steady_clock::time_point start_time_;
        EnDecoderClass::EnCoder encoder_;///< 编码器

        bool InitStream(const AVCodec *av_codec, AVCodecContext *ac_codec_context);
        std::string GetTypeString(bool input_format, std::string type);
    };
};// namespace controller_native