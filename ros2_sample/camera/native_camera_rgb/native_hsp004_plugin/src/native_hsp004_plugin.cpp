#include "native_hsp004_plugin/native_hsp004_plugin.h"
#include "log/log.h"
#include <functional>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <string>

namespace controller_native {

    NativeHSP004Plugin::~NativeHSP004Plugin() {
        if (device_info_) {
            hsp004_wrap_.StopCameraStream(device_info_);
            hsp004_wrap_.CloseDevice(device_info_);
            delete device_info_;
            device_info_ = nullptr;
        }
        LOGGER_INFO(logger_, "NativeHSP004Plugin: Plugin destroyed.");
    }

    bool NativeHSP004Plugin::ShowInfo() {
        LOGGER_INFO(logger_, "HSP004Plugin: This is a RGB camera plugin for HSP004.");
        hsp004_wrap_.ShowInfo();
        return true;
    }

    bool NativeHSP004Plugin::OpenDevice(std::function<void(const std::string &)> fault_report,
                                        std::function<void(const cv::Mat &)> user_image_callback) {
        if (device_info_ = OpenOneDeviceDependJsonConfig(); device_info_) {
            auto width = GetValueFromJson2<int32_t>(params_root_, "width");
            auto height = GetValueFromJson2<int32_t>(params_root_, "height");
            auto frame_rate = GetValueFromJson2<int32_t>(params_root_, "frame_rate");
            auto camera_format = GetValueFromJson2<std::string>(params_root_, "camera_format");
            if (width && height && frame_rate && camera_format) {
                camera_rgb_base_info_ptr_ = std::make_shared<CameraRGBBaseInfo>();
                camera_rgb_base_info_ptr_->width = *width;
                camera_rgb_base_info_ptr_->height = *height;
                camera_rgb_base_info_ptr_->frame_rate = *frame_rate;
                camera_rgb_base_info_ptr_->uuid = std::to_string(device_info_->uuid);
                camera_rgb_base_info_ptr_->serial_number = device_info_->serial_number;
                camera_rgb_base_info_ptr_->usb_port = device_info_->usb_port;
                camera_rgb_base_info_ptr_->camera_output_format = *camera_format;
                camera_rgb_base_info_ptr_->dev_status_callback = fault_report;
                camera_rgb_base_info_ptr_->camera_image_callback = std::bind(
                        &NativeCameraRGBPluginBase::ImageCallback, this, std::placeholders::_1, std::placeholders::_2);
                camera_rgb_base_info_ptr_->user_image_callback = user_image_callback;
                device_info_->base_info_ptr = camera_rgb_base_info_ptr_;

                return true;
            } else {
                LOGGER_ERROR(logger_, "read width, height, frame_rate in config file fail");
            }
            return true;
        } else {
            LOGGER_ERROR(logger_, "open target Device fail");
            return false;
        }
    }

    bool NativeHSP004Plugin::CloseDevice() { return hsp004_wrap_.CloseDevice(device_info_); }

    bool NativeHSP004Plugin::StartCameraStream() {
        if (camera_rgb_base_info_ptr_) {
            InitMPPDec(camera_rgb_base_info_ptr_->width, camera_rgb_base_info_ptr_->height);
            InitEnDeCoder(camera_rgb_base_info_ptr_->width, camera_rgb_base_info_ptr_->height,
                          camera_rgb_base_info_ptr_->frame_rate, camera_rgb_base_info_ptr_->camera_output_format,
                          camera_rgb_base_info_ptr_->width, camera_rgb_base_info_ptr_->height, 0, "H264");
        }
        return hsp004_wrap_.StartCameraStream(device_info_);
    }

    bool NativeHSP004Plugin::StopCameraStream() { return hsp004_wrap_.StopCameraStream(device_info_); }

    HSP004Wrap::HSP004DeviceInfo *NativeHSP004Plugin::OpenOneDeviceDependJsonConfig() {
        if (auto uuid_str = GetValueFromJson2<std::string>(params_root_, "uuid"); uuid_str) {
            return hsp004_wrap_.OpenTargetDevice(
                    [this, uuid = std::stoull(*uuid_str)](const HSP004Wrap::HSP004DeviceInfo *device_info) {
                        LOGGER_INFO(logger_, "HSP004Plugin target uuid: {} searched uuid:{}", uuid, device_info->uuid);
                        if (uuid == device_info->uuid) {
                            return true;
                        } else {
                            return false;
                        }
                    });
        }
        return nullptr;
    }

    bool NativeHSP004Plugin::SetCalibrationParam(CameraRGBCalibrateParam camera_calibrate_param) {
        return hsp004_wrap_.SetCalibrationParam(device_info_, camera_calibrate_param);
    }

    std::tuple<bool, CameraRGBBaseInfo> NativeHSP004Plugin::GetDeviceInfo() {
        if (device_info_) { return {true, *device_info_->base_info_ptr}; }
        return {false, CameraRGBBaseInfo{}};
    }
}// namespace controller_native

PLUGINLIB_EXPORT_CLASS(controller_native::NativeHSP004Plugin, controller_native::NativeCommonBase)