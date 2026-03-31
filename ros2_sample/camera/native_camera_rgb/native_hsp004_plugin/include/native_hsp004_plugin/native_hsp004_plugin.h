#pragma once
#include "hsp004_wrap.h"
#include "native_camera_rgb_plugin_base/native_camera_rgb_plugin_base.h"
#include "native_camera_rgb_plugin_base/native_camera_rgb_struct_define.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace controller_native {
    class NativeHSP004Plugin : public NativeCameraRGBPluginBase {
    public:
        using BaseType = NativeCameraRGBPluginBase;
        NativeHSP004Plugin() = default;

        virtual ~NativeHSP004Plugin();

        virtual bool ShowInfo() override;

        virtual bool OpenDevice(std::function<void(const std::string &)> fault_report,
                                std::function<void(const cv::Mat &)> user_image_callback) override;

        virtual bool CloseDevice() override;

        virtual bool StartCameraStream() override;

        virtual bool StopCameraStream() override;

        virtual bool SetCalibrationParam(CameraRGBCalibrateParam camera_calibrate_param) override;

        virtual std::tuple<bool, CameraRGBBaseInfo> GetDeviceInfo() override;

        virtual void InitNaitve() override {
            hsp004_wrap_.SetLogger(logger_);
            BaseType::InitNaitve();
        }

    private:
        HSP004Wrap hsp004_wrap_;
        HSP004Wrap::HSP004DeviceInfo *device_info_ = nullptr;

        /**
         * @brief 根据JSON配置文件打开一个HSP004设备
         * @return HSP004Wrap::HSP004DeviceInfo* 返回设备信息指针，失败时返回nullptr
         */
        HSP004Wrap::HSP004DeviceInfo *OpenOneDeviceDependJsonConfig();
    };
}// namespace controller_native