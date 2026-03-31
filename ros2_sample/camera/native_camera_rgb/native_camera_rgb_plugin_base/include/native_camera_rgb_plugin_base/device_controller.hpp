#pragma once

#include "log/log.h"
#include "native_camera_rgb_plugin_base/native_camera_rgb_struct_define.hpp"
#include "string"
#include "yjhlog/logger.h"
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <vector>
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

class DeviceController {
public:
    bool ShowAllDevice() {
        avdevice_register_all();
        // 获取视频输入设备格式（如 v4l2）
        const AVInputFormat *fmt = av_find_input_format("v4l2");
        if (!fmt) {
            LOGGER_ERROR(logger_, "NativeUSBCameraPlugin: Failed to get v4l2 input format.");
            return false;
        }

        // 枚举设备
        AVDeviceInfoList *device_list = nullptr;
        if (avdevice_list_input_sources(fmt, nullptr, nullptr, &device_list) < 0) {
            LOGGER_ERROR(logger_, "NativeUSBCameraPlugin: Failed to enumerate devices.");
            return false;
        }

        // 打印所有视频设备信息
        for (int i = 0; i < device_list->nb_devices; i++) {
            AVDeviceInfo *dev = device_list->devices[i];
            std::string msg   = "[" + std::to_string(i) + "] " + dev->device_name;
            if (dev->device_description) { msg += " (" + std::string(dev->device_description) + ")"; }
            LOGGER_INFO(logger_, "NativeUSBCameraPlugin: {}", msg);
        }

        // 释放资源
        avdevice_free_list_devices(&device_list);
        return true;
    }

    bool Init(int width, int height, const std::string &camera_format, int fps, const std::string &camera_device) {
        LOGGER_INFO(logger_, "camera_device: {}, video_size: {}x{}, pixel_format: {}, framerate: {}",
                    camera_device.c_str(), width, height, camera_format, fps);

        avdevice_register_all();
        // 分配AVFormatContext
        format_ctx_ = avformat_alloc_context();
        if (!format_ctx_) {
            LOGGER_ERROR(logger_, "cant allocate AVFormatContext");
            return false;
        }

        // 设置输入选项
        AVDictionary *options = nullptr;
        av_dict_set(&options, "video_size", (std::to_string(width) + "x" + std::to_string(height)).c_str(), 0);
        av_dict_set(&options, "framerate", std::to_string(fps).c_str(), 0);
        av_dict_set(&options, "pixel_format", camera_format.c_str(), 0);//设置usb摄像头的像素格式，比如yuyv422
        av_dict_set(&options, "buffers", "8", 0);                       // 增加缓冲区
        // av_dict_set(&options, "use_mmap", "1", 0);                      // 使用 mmap


        char resolved_path[PATH_MAX];
        if (realpath(camera_device.c_str(), resolved_path) == nullptr) {
            LOGGER_ERROR(logger_, "Failed to resolve camera device path: {}", camera_device.c_str());
            return false;
        }
        LOGGER_INFO(logger_, "Original path: {}, Resolved path: {}", camera_device.c_str(), resolved_path);

        // 2. 打开 USB 摄像头
        if (auto ret = avformat_open_input(&format_ctx_, resolved_path, nullptr, &options); ret < 0) {
            char error_buf[AV_ERROR_MAX_STRING_SIZE];
            av_strerror(ret, error_buf, AV_ERROR_MAX_STRING_SIZE);
            LOGGER_ERROR(logger_, "cant open device {}, error: {} ({})", resolved_path, error_buf, ret);
            av_dict_free(&options);
            return false;
        }
        av_dict_free(&options);
        LOGGER_INFO(logger_, "input format: {}", format_ctx_->iformat->name);

        // 查找流信息
        if (avformat_find_stream_info(format_ctx_, NULL) < 0) {
            LOGGER_ERROR(logger_, "avformat_find_stream_info fail");
            return false;
        }
        for (unsigned int i = 0; i < format_ctx_->nb_streams; i++) {
            if (format_ctx_->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
                video_stream_idx_ = i;
                break;
            }
        }
        return video_stream_idx_ != -1 ? true : false;
    }

    bool Cleanup() {
        if (format_ctx_) {
            avformat_close_input(&format_ctx_);
            format_ctx_ = nullptr;
        }
        video_stream_idx_ = -1;
        is_working_       = false;
        return true;
    }

    void StartCameraStream(std::function<void(void *buffer, int buf_len)> image_callback) {
        if (!format_ctx_ || video_stream_idx_ == -1) {
            LOGGER_ERROR(logger_, "Format context or video stream index is invalid.");
            return;
        }
        LOGGER_INFO(logger_, "StartCameraStream");
        is_working_ = true;
        thread_     = std::thread([this, image_callback = image_callback]() {
            pthread_setname_np(pthread_self(), "camera_stream");
            while (is_working_) {
                if (auto ret = av_read_frame(format_ctx_, &packet_); ret == 0) {
                    if (packet_.stream_index == video_stream_idx_) { image_callback(packet_.data, packet_.size); }
                } else {
                    LOGGER_INFO(logger_, "Error reading frame: {}", ret);
                    std::raise(SIGINT);
                    return;
                }
                av_packet_unref(&packet_);
            }
        });
    }

    bool StopCameraStream() {
        is_working_ = false;
        if (thread_.joinable()) { thread_.join(); }
        return true;
    }

    void SetLogger(std::shared_ptr<yjhlog::Logger> logger) { logger_ = logger; }

private:
    std::shared_ptr<yjhlog::Logger> logger_ = nullptr;
    AVFormatContext *format_ctx_            = nullptr;///< 输入格式上下文
    AVPacket packet_;                                 ///< 数据包
    int video_stream_idx_         = -1;               ///< 视频流索引
    std::atomic<bool> is_working_ = false;            ///< 是否正在工作
    std::thread thread_;                              ///< 工作线程
};