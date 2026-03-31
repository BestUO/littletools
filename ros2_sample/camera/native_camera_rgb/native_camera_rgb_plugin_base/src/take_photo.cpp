#include "native_camera_rgb_plugin_base/take_photo.h"
#include "log/log.h"
#include "native_camera_rgb_plugin_base/utils.hpp"
#include <memory>
#include <opencv2/opencv.hpp>

#include <string>
namespace controller_native {
    bool TakePhotoClass::Init(int32_t width, int32_t height, const std::string &file_dir,
                              const std::string &dst_format) {
        (void) dst_format;// TakePhotoClass does not use dst_format, it always saves as JPG
        camera_data_.ClearCache();

        width_ = width;
        height_ = height;
        dst_format_ = GetTypeString(false, "JPG");


        if (auto [flag, file_name] = CameraUtils::CreateDirectoriesAndGenFileName(file_dir, dst_format_); flag) {
            file_name_ = file_name;
        } else {
            LOGGER_ERROR(logger_, "RecordVideoClass: Failed to create directories or generate file name.");
            return false;
        }

        return true;
    }

    void TakePhotoClass::TakePhoto(const cv::Mat &cv_mat) {
        auto color_img_ptr = std::make_shared<cv::Mat>(cv_mat.clone());
        camera_data_.SetAndNotify(color_img_ptr);
    }

    std::tuple<bool, std::string, cv::Mat> TakePhotoClass::WaitInfo(std::chrono::milliseconds timeout) {
        if (auto data_descriptor_ptr = camera_data_.WaitInfo(timeout); data_descriptor_ptr) {
            cv::Mat resized_image;
            cv::resize(*data_descriptor_ptr, resized_image, cv::Size(width_, height_), 0, 0, cv::INTER_LINEAR);
            cv::imwrite(file_name_, resized_image);
            return {true, file_name_, resized_image};
        } else {
            return {false, {}, {}};
        }
    }

    void TakePhotoClass::Close() { camera_data_.Notify(); }

    std::string TakePhotoClass::GetTypeString(bool input_format, std::string type) {
        std::transform(type.begin(), type.end(), type.begin(), [](unsigned char c) { return std::toupper(c); });
        if (input_format) {
            std::string dst_type = "H264";
            if (type == "H264") dst_type = "H264";
            return dst_type;
        } else {
            std::string dst_type = ".jpg";
            if (type == "JPG") dst_type = ".jpg";
            else if (type == "PNG")
                dst_type = ".png";
            else if (type == "BMP")
                dst_type = ".bmp";
            return dst_type;
        }
    }
}// namespace controller_native