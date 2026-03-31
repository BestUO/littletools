#pragma once

#include "log/log.h"
#include "native_camera_rgb_plugin_base/native_camera_rgb_struct_define.hpp"
#include <cstdint>
#include <opencv2/core/mat.hpp>
#include <stdint.h>

namespace controller_native {
    class TakePhotoClass {
    public:
        TakePhotoClass() = default;
        bool Init(int32_t width, int32_t height, const std::string &file_dir, const std::string &dst_format);
        void TakePhoto(const cv::Mat &cv_mat);
        std::tuple<bool, std::string, cv::Mat>
        WaitInfo(std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));
        void Close();
        void SetLogger(std::shared_ptr<yjhlog::Logger> logger) { logger_ = logger; }

    private:
        std::shared_ptr<yjhlog::Logger> logger_ = nullptr;
        CameraRGBController::DataPtrPV<cv::Mat> camera_data_;
        int32_t width_ = 0;              ///< 图像宽度
        int32_t height_ = 0;             ///< 图像高度
        std::string dst_format_ = ".jpg";///< 图像保存格式，默认为.jpg
        std::string file_name_;          ///< 文件名

        std::string GetTypeString(bool input_format, std::string type);
    };
}// namespace controller_native