#include "native_camera_rgb_plugin_base/endecoder.h"
#include "log/log.h"
#include "yjhlog/logger.h"
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

namespace controller_native {
    bool EnDecoderClass::Init(int camera_width, int camera_height, int fps, const std::string &camera_format,
                              int dst_width, int dst_height, int bit_rate, const std::string &dst_format) {
        (void) dst_width;
        (void) dst_height;
        (void) bit_rate;
        avdevice_register_all();

        if (!decoder_.Init(logger_, camera_width, camera_height, GetCodecID(camera_format),
                           GetPixelFormat(camera_format))) {
            Close();
            return false;
        }

        if (!encoder_.Init(logger_, {GetCodecID(dst_format), fps, bit_rate})) {
            Close();
            return false;
        }

        camera_format_ = GetCodecID(camera_format);
        is_working_    = true;
        return true;
    }

    void EnDecoderClass::TakePhoto(AVFrame *frame_bgr, const std::string &file_name) {
        int width  = frame_bgr->width;
        int height = frame_bgr->height;

        // 假设你使用的是 AV_PIX_FMT_RGB24
        if (frame_bgr->format == AV_PIX_FMT_RGB24) {
            cv::Mat mat(height, width, CV_8UC3);
            // 如果linesize[0] == width * 3，可以直接复制
            if (frame_bgr->linesize[0] == width * 3) {
                memcpy(mat.data, frame_bgr->data[0], width * height * 3);
            } else {
                // 逐行复制（处理padding）
                for (int y = 0; y < height; y++) {
                    memcpy(mat.ptr<uint8_t>(y), frame_bgr->data[0] + y * frame_bgr->linesize[0], width * 3);
                }
            }
            cv::imwrite(file_name.c_str(), mat);
        }
        // 如果使用 AV_PIX_FMT_BGR24
        else if (frame_bgr->format == AV_PIX_FMT_BGR24) {
            cv::Mat mat(height, width, CV_8UC3);
            if (frame_bgr->linesize[0] == width * 3) {
                memcpy(mat.data, frame_bgr->data[0], width * height * 3);
            } else {
                for (int y = 0; y < height; y++) {
                    memcpy(mat.ptr<uint8_t>(y), frame_bgr->data[0] + y * frame_bgr->linesize[0], width * 3);
                }
            }
            cv::imwrite(file_name.c_str(), mat);
        }
        // 如果使用 AV_PIX_FMT_RGBA
        else if (frame_bgr->format == AV_PIX_FMT_RGBA) {
            cv::Mat mat(height, width, CV_8UC4);
            if (frame_bgr->linesize[0] == width * 4) {
                memcpy(mat.data, frame_bgr->data[0], width * height * 4);
            } else {
                for (int y = 0; y < height; y++) {
                    memcpy(mat.ptr<uint8_t>(y), frame_bgr->data[0] + y * frame_bgr->linesize[0], width * 4);
                }
            }
            cv::imwrite(file_name.c_str(), mat);
        }
    }

    void EnDecoderClass::Close() {
        LOGGER_INFO(logger_, "EnDecoderClass::Close: Releasing resources...");
        decoder_.Cleanup();
        encoder_.Cleanup();
        to_rgb_.Clear();
        to_yuv420_.Clear();
        is_working_ = false;
    }

    AVCodecID EnDecoderClass::GetCodecID(const std::string &format) {
        std::string format_upper = format;
        std::transform(format_upper.begin(), format_upper.end(), format_upper.begin(),
                       [](unsigned char c) { return std::toupper(c); });
        if (format_upper == "MJPEG" || format_upper == "MJPG") {
            return AV_CODEC_ID_MJPEG;
        } else if (format_upper == "H264") {
            return AV_CODEC_ID_H264;
        } else if (format_upper == "YUYV422") {
            return AV_CODEC_ID_RAWVIDEO;
        } else {
            return AV_CODEC_ID_NONE;
        }
    }

    AVPixelFormat EnDecoderClass::GetPixelFormat(const std::string &format) {
        std::string format_upper = format;
        std::transform(format_upper.begin(), format_upper.end(), format_upper.begin(),
                       [](unsigned char c) { return std::toupper(c); });
        if (format_upper == "MJPEG" || format_upper == "MJPG") {
            return AV_PIX_FMT_YUV420P;
        } else if (format_upper == "H264") {
            return AV_PIX_FMT_YUV420P;
        } else if (format_upper == "YUYV422") {
            return AV_PIX_FMT_YUYV422;
        } else {
            return AV_PIX_FMT_NONE;
        }
    }

}// namespace controller_native