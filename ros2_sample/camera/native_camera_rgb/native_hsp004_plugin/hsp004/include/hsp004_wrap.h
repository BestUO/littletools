#pragma once

#include "hsp004_dev_sdk.h"
#include "log/log.h"
#include "native_camera_rgb_plugin_base/native_camera_rgb_struct_define.hpp"
#include <functional>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
namespace controller_native {
    /**
     * @class HSP004Wrap
     * @brief HSP004相机设备包装类
     * @details 该类封装了HSP004相机设备的SDK接口，提供了设备管理、
     *          流控制和参数配置等功能的高层抽象
     */
    class HSP004Wrap {
    public:
        /**
         * @struct HSP004DeviceInfo
         * @brief HSP004设备信息结构体
         * @details 包含设备的基本信息和连接句柄
         */
        struct HSP004DeviceInfo {
            HCAM hcam = nullptr;           ///< 设备句柄
            unsigned long long uuid = 0;   ///< 设备唯一标识符
            std::string serial_number = "";///< 设备序列号
            std::string usb_port = "";     ///< USB端口信息

            std::shared_ptr<CameraRGBBaseInfo> base_info_ptr = nullptr;///< 相机基础信息指针
        };

        HSP004Wrap() = default;
        virtual ~HSP004Wrap() = default;
        /**
         * @brief 显示设备信息
         * @return bool 成功返回true，失败返回false
         * @details 显示当前系统中所有可用的HSP004设备信息
         */
        bool ShowInfo();

        /**
         * @brief 获取设备UUID列表
         * @return std::vector<HSP004DeviceInfo> 返回所有发现的设备信息列表
         * @details 扫描系统中所有连接的HSP004设备，返回设备信息向量
         */
        std::vector<HSP004DeviceInfo> GetDeviceUUIDList();

        /**
         * @brief 打开目标设备
         * @param compare_fun 设备比较函数，用于选择特定设备
         * @return HSP004Wrap::HSP004DeviceInfo* 成功返回设备信息指针，失败返回nullptr
         * @details 根据提供的比较函数选择并打开符合条件的设备
         */
        HSP004Wrap::HSP004DeviceInfo *
        OpenTargetDevice(const std::function<bool(const HSP004DeviceInfo *device_info)> compare_fun);

        /**
         * @brief 关闭设备
         * @param device_info 要关闭的设备信息指针
         * @return bool 成功返回true，失败返回false
         * @details 关闭指定的HSP004设备并释放相关资源
         */
        bool CloseDevice(HSP004DeviceInfo *device_info);

        /**
         * @brief 启动相机数据流
         * @param device_info 设备信息指针
         * @return bool 成功返回true，失败返回false
         * @details 开始从指定设备接收图像数据流
         */
        bool StartCameraStream(HSP004DeviceInfo *device_info);

        /**
         * @brief 停止相机数据流
         * @param device_info 设备信息指针
         * @return bool 成功返回true，失败返回false
         * @details 停止从指定设备接收图像数据流
         */
        bool StopCameraStream(HSP004DeviceInfo *device_info);

        /**
         * @brief 设置相机标定参数
         * @param device_info 设备信息指针
         * @param camera_calibrate_param 相机标定参数
         * @return bool 成功返回true，失败返回false
         * @details 为指定设备设置相机标定参数，用于图像校正
         */
        bool SetCalibrationParam(HSP004DeviceInfo *device_info,
                                 controller_native::CameraRGBCalibrateParam camera_calibrate_param);
        void SetLogger(std::shared_ptr<yjhlog::Logger> logger) { logger_ = logger; }

    private:
        std::shared_ptr<yjhlog::Logger> logger_ = nullptr;
        /**
         * @brief 设备状态回调函数
         * @param tofDevStatus 设备状态信息
         * @param pUserData 用户自定义数据指针
         * @details 静态回调函数，用于处理设备状态变化事件
         */
        static void DevStatusCB(stSunnyDevStatus tofDevStatus, void *pUserData);

        /**
         * @brief 数据流回调函数
         * @param buffer 图像数据缓冲区指针
         * @param buf_len 缓冲区长度
         * @param user_data 用户自定义数据指针
         * @details 静态回调函数，用于处理从设备接收到的图像数据
         */
        static void StreamCB(void *buffer, int buf_len, void *user_data);
    };
}// namespace controller_native