#pragma once
#include "native_camera_rgb_plugin_base/device_controller.hpp"
#include "native_camera_rgb_plugin_base/native_camera_rgb_plugin_base.h"
#include "native_camera_rgb_plugin_base/native_camera_rgb_struct_define.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace controller_native {
    class NativeUSBCameraPlugin : public NativeCameraRGBPluginBase {
    public:
        using BaseType          = NativeCameraRGBPluginBase;
        NativeUSBCameraPlugin() = default;

        virtual ~NativeUSBCameraPlugin();

        virtual bool ShowInfo() override;

        virtual bool OpenDevice(std::function<void(const std::string &)> fault_report,
                                std::function<void(const cv::Mat &)> user_image_callback) override;

        virtual bool CloseDevice() override;

        virtual bool StartCameraStream() override;

        virtual bool StopCameraStream() override;

        virtual CameraCalibrateParamFull GetCalibrationParam() override;

        virtual bool SetCalibrationParam(CameraRGBCalibrateParam camera_calibrate_param) override;

        virtual std::tuple<bool, CameraRGBBaseInfo> GetDeviceInfo() override;

        virtual void InitNaitve() override {
            device_controller_.SetLogger(logger_);
            BaseType::InitNaitve();
        }

        virtual bool OpenDevice(const CameraRGBBaseInfo &camera_info) override;

    private:
        DeviceController device_controller_;             ///< 设备控制器
        CameraCalibrateParamFull camera_calibrate_param_;///< 相机标定参数

        void LoadCameraCalibrateParam(int width, int height);
    };
}// namespace controller_native