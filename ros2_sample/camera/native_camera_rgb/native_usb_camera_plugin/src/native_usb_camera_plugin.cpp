#include "native_usb_camera_plugin/native_usb_camera_plugin.h"
#include "log/log.h"
#include "yjhlog/logger.h"
#include <functional>
#include <jsoncpp/json/value.h>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <string>

namespace controller_native {

    NativeUSBCameraPlugin::~NativeUSBCameraPlugin() {
        LOGGER_INFO(logger_, "NativeUSBCameraPlugin: Plugin destroyed.");
    }

    bool NativeUSBCameraPlugin::ShowInfo() { return device_controller_.ShowAllDevice(); }

    bool NativeUSBCameraPlugin::OpenDevice(std::function<void(const std::string &)> fault_report,
                                           std::function<void(const cv::Mat &)> user_image_callback) {
        auto width         = GetValueFromJson2<int32_t>(params_root_, "width");
        auto height        = GetValueFromJson2<int32_t>(params_root_, "height");
        auto frame_rate    = GetValueFromJson2<int32_t>(params_root_, "frame_rate");
        auto camera_format = GetValueFromJson2<std::string>(params_root_, "camera_format");
        auto camera_device = GetValueFromJson2<std::string>(params_root_, "camera_device");
        if (width && height && frame_rate && camera_format && camera_device) {
            camera_rgb_base_info_ptr_                        = std::make_shared<CameraRGBBaseInfo>();
            camera_rgb_base_info_ptr_->width                 = *width;
            camera_rgb_base_info_ptr_->height                = *height;
            camera_rgb_base_info_ptr_->frame_rate            = *frame_rate;
            camera_rgb_base_info_ptr_->camera_output_format  = *camera_format;
            camera_rgb_base_info_ptr_->camera_device         = *camera_device;
            camera_rgb_base_info_ptr_->dev_status_callback   = fault_report;
            camera_rgb_base_info_ptr_->camera_image_callback = std::bind(
                    &NativeCameraRGBPluginBase::ImageCallback, this, std::placeholders::_1, std::placeholders::_2);
            camera_rgb_base_info_ptr_->user_image_callback = user_image_callback;

            // 从JSON中读取标定参数并填充到camera_calibrate_param_中
            if (auto calibrate_param = GetValueFromJson2<Json::Value>(params_root_, "calibrate_param");
                calibrate_param) {
                auto distortion_model = GetValueFromJson2<std::string>(*calibrate_param, "distortion_model");

                camera_calibrate_param_.width  = *width;
                camera_calibrate_param_.height = *height;
                if (distortion_model) camera_calibrate_param_.distortion_model = *distortion_model;

                // 读取相机矩阵 (3x3)
                auto camera_matrix = GetValueFromJson2<Json::Value>(*calibrate_param, "camera_matrix");
                if (camera_matrix) {
                    for (int i = 0; i < 9 && i < static_cast<int>(camera_matrix->size()); ++i) {
                        if ((*camera_matrix)[i].isDouble() || (*camera_matrix)[i].isInt()) {
                            camera_calibrate_param_.camera_matrix[i] = (*camera_matrix)[i].asFloat();
                        }
                    }
                }

                // 读取畸变系数
                auto distortion_coefficients =
                        GetValueFromJson2<Json::Value>(*calibrate_param, "distortion_coefficients");
                if (distortion_coefficients) {
                    for (int i = 0; i < 8 && i < static_cast<int>(distortion_coefficients->size()); ++i) {
                        if ((*distortion_coefficients)[i].isDouble() || (*distortion_coefficients)[i].isInt()) {
                            camera_calibrate_param_.distortion_coefficients[i] =
                                    (*distortion_coefficients)[i].asFloat();
                        }
                    }
                }

                // 读取校正矩阵 (3x3)
                auto rectification_matrix = GetValueFromJson2<Json::Value>(*calibrate_param, "rectification_matrix");
                if (rectification_matrix) {
                    for (int i = 0; i < 9 && i < static_cast<int>(rectification_matrix->size()); ++i) {
                        if ((*rectification_matrix)[i].isDouble() || (*rectification_matrix)[i].isInt()) {
                            camera_calibrate_param_.rectification_matrix[i] = (*rectification_matrix)[i].asFloat();
                        }
                    }
                }

                // 读取投影矩阵 (3x4)
                auto projection_matrix = GetValueFromJson2<Json::Value>(*calibrate_param, "projection_matrix");
                if (projection_matrix) {
                    for (int i = 0; i < 12 && i < static_cast<int>(projection_matrix->size()); ++i) {
                        if ((*projection_matrix)[i].isDouble() || (*projection_matrix)[i].isInt()) {
                            camera_calibrate_param_.projection_matrix[i] = (*projection_matrix)[i].asFloat();
                        }
                    }
                }

                LOGGER_INFO(logger_, "NativeUSBCameraPlugin: Camera calibration parameters loaded from JSON.");
            } else {
                LOGGER_WARN(logger_, "NativeUSBCameraPlugin: No calibration parameters found in JSON configuration.");
            }

            return device_controller_.Init(camera_rgb_base_info_ptr_->width, camera_rgb_base_info_ptr_->height,
                                           camera_rgb_base_info_ptr_->camera_output_format,
                                           camera_rgb_base_info_ptr_->frame_rate,
                                           camera_rgb_base_info_ptr_->camera_device);
        }
        return false;
    }

    bool NativeUSBCameraPlugin::CloseDevice() { return device_controller_.Cleanup(); }

    bool NativeUSBCameraPlugin::StartCameraStream() {
        StartWorkThread();
        if (camera_rgb_base_info_ptr_) {
            InitMPPDec(camera_rgb_base_info_ptr_->width, camera_rgb_base_info_ptr_->height);
            if (InitEnDeCoder(camera_rgb_base_info_ptr_->width, camera_rgb_base_info_ptr_->height,
                              camera_rgb_base_info_ptr_->frame_rate, camera_rgb_base_info_ptr_->camera_output_format,
                              camera_rgb_base_info_ptr_->width, camera_rgb_base_info_ptr_->height, 0, "H264")) {
                device_controller_.StartCameraStream(camera_rgb_base_info_ptr_->camera_image_callback);
                return true;
            }
        }
        LOGGER_INFO(logger_, "NativeUSBCameraPlugin: Failed to start camera stream.");
        return false;
    }

    bool NativeUSBCameraPlugin::StopCameraStream() {
        StopWorkThread();
        return device_controller_.StopCameraStream() && DeInitEnDeCoder();
    }

    CameraCalibrateParamFull NativeUSBCameraPlugin::GetCalibrationParam() { return camera_calibrate_param_; }

    bool NativeUSBCameraPlugin::SetCalibrationParam(CameraRGBCalibrateParam camera_calibrate_param) {
        (void) camera_calibrate_param;
        return false;
    }

    std::tuple<bool, CameraRGBBaseInfo> NativeUSBCameraPlugin::GetDeviceInfo() {
        if (camera_rgb_base_info_ptr_) {
            return {true, *camera_rgb_base_info_ptr_};
        } else {
            LOGGER_ERROR(logger_, "NativeUSBCameraPlugin: Camera RGB base info is not initialized.");
            return {false, CameraRGBBaseInfo{}};
        }
    }

    bool NativeUSBCameraPlugin::OpenDevice([[maybe_unused]] const CameraRGBBaseInfo &camera_info) {
        camera_rgb_base_info_ptr_                        = std::make_shared<CameraRGBBaseInfo>(camera_info);
        camera_rgb_base_info_ptr_->camera_image_callback = std::bind(&NativeCameraRGBPluginBase::ImageCallback, this,
                                                                     std::placeholders::_1, std::placeholders::_2);
        camera_calibrate_param_.width                    = camera_info.width;
        camera_calibrate_param_.height                   = camera_info.height;
        return device_controller_.Init(camera_rgb_base_info_ptr_->width, camera_rgb_base_info_ptr_->height,
                                       camera_rgb_base_info_ptr_->camera_output_format,
                                       camera_rgb_base_info_ptr_->frame_rate, camera_rgb_base_info_ptr_->camera_device);
    }
}// namespace controller_native

PLUGINLIB_EXPORT_CLASS(controller_native::NativeUSBCameraPlugin, controller_native::NativeCommonBase)