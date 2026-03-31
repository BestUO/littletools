#pragma once

#include "camera_msgs/srv/live_stream.hpp"
#include "camera_msgs/srv/record_video.hpp"
#include "camera_msgs/srv/start_camera.hpp"
#include "camera_msgs/srv/take_photo.hpp"
#include "native_camera_rgb_plugin_base/native_camera_rgb_plugin_base.h"
#include "ros_controller_base/ros_common_base.h"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <diagnostic_msgs/srv/self_test.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace controller_ros {
    class ROSCameraRGB : public RosCommonBase {
    public:
        explicit ROSCameraRGB(const rclcpp::NodeOptions &options);
        virtual ~ROSCameraRGB();

        /**
         * @brief 初始化ROS接口
         * @details 重写基类方法，初始化相机相关的ROS发布者、服务和动作服务器
         */
        virtual void InitRos() override;

    private:
        // 相机参数配置
        int width_;                               ///< 图像宽度（像素）
        int height_;                              ///< 图像高度（像素）
        int frame_rate_;                          ///< 帧率（FPS）
        sensor_msgs::msg::CameraInfo camera_info_;///< 相机信息消息

        std::shared_ptr<controller_native::NativeCameraRGBPluginBase> camera_rgb_plugin_;///< RGB相机插件智能指针
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;          ///< 原始图像发布者
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_publisher_;///< 压缩图像发布者
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher;///< 相机信息发布者
        rclcpp::Service<camera_msgs::srv::TakePhoto>::SharedPtr take_photo_server_;      ///< 保存图像服务
        rclcpp::Service<camera_msgs::srv::LiveStream>::SharedPtr live_stream_server_;    ///< 实时流控制服务
        rclcpp::Service<camera_msgs::srv::RecordVideo>::SharedPtr record_video_server_;///< 保存视频控制字服务
        rclcpp::Service<camera_msgs::srv::StartCamera>::SharedPtr start_camera_server_;///< 启动相机服务

        /**
         * @brief 创建相机插件实例
         * @return bool 成功返回true，失败返回false
         * @details 根据配置参数创建对应的相机插件实例
         */
        bool CastPlugin();

        /**
         * @brief 显示相机设备信息
         * @return bool 成功返回true，失败返回false
         * @details 显示当前系统中可用的相机设备信息
         */
        bool ShowInfo();

        /**
         * @brief 初始化相机RGB ROS接口
         * @return bool 成功返回true，失败返回false
         * @details 创建并配置所有ROS发布者、服务和动作服务器
         */
        bool InitCameraRGBRosInterface();

        /**
         * @brief 打开相机RGB设备
         * @return bool 成功返回true，失败返回false
         * @details 连接并初始化相机设备，准备开始图像采集
         */
        bool OpenCameraRGBDevice();

        /**
         * @brief ROS图像发布回调函数
         * @param colorImg OpenCV彩色图像矩阵
         * @details 将OpenCV图像转换为ROS消息格式并发布到相应topic
         */
        void ROSPublishCB(const cv::Mat &colorImg);

        /**
         * @brief 处理保存图像服务请求
         * @param request 保存图像请求
         * @param response 保存图像响应
         * @details 服务回调函数，处理客户端发送的图像保存请求
         */
        void HandleTakePhoto(const std::shared_ptr<camera_msgs::srv::TakePhoto::Request> request,
                             std::shared_ptr<camera_msgs::srv::TakePhoto::Response> response);

        /**
         * @brief 处理实时流服务请求
         * @param request 实时流请求
         * @param response 实时流响应
         * @details 服务回调函数，处理实时流的开启/关闭请求
         */
        void HandleLiveStream(const std::shared_ptr<camera_msgs::srv::LiveStream::Request> request,
                              std::shared_ptr<camera_msgs::srv::LiveStream::Response> response);

        void HandleRecordVideo(const std::shared_ptr<camera_msgs::srv::RecordVideo::Request> request,
                               std::shared_ptr<camera_msgs::srv::RecordVideo::Response> response);

        void HandleStartCamera(const std::shared_ptr<camera_msgs::srv::StartCamera::Request> request,
                               std::shared_ptr<camera_msgs::srv::StartCamera::Response> response);

        void SetCameraInfo();
    };
}// namespace controller_ros