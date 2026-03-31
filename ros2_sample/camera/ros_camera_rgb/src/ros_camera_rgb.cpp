#include "ros_camera_rgb/ros_camera_rgb.h"
#include "camera_msgs/srv/record_video.hpp"
#include "controller_msgs/msg/component_value.hpp"
#include "controller_msgs/msg/property_value.hpp"
#include "log/log.h"
#include "sensor_msgs/msg/camera_info.hpp"
#include <cstring>
#include <ctime>
#include <cv_bridge/cv_bridge.hpp>
#include <functional>
#include <memory>
#include <rclcpp/event.hpp>
#include <string>
#include <sys/types.h>

namespace controller_ros {

    ROSCameraRGB::ROSCameraRGB(const rclcpp::NodeOptions &options) : RosCommonBase(options) {}
    ROSCameraRGB::~ROSCameraRGB() {
        if (camera_rgb_plugin_) {
            camera_rgb_plugin_->StopCameraStream();
            camera_rgb_plugin_->CloseDevice();
        }
    }

    void ROSCameraRGB::InitRos() {
        bool ret = true;
        if (ret) { ret = CastPlugin(); }
        if (ret) {
            ret = ShowInfo();
        } else {
            LOGGER_WARN(logger_, "CastPlugin failed");
        }
        if (ret) {
            ret = InitCameraRGBRosInterface();
        } else {
            LOGGER_WARN(logger_, "ShowInfo failed");
        }
        if (ret) {
            ret = OpenCameraRGBDevice();
        } else {
            LOGGER_WARN(logger_, "InitCameraRGBRosInterface failed");
        }
        if (!ret) { LOGGER_WARN(logger_, "OpenCameraRGBDevice failed"); }
    }

    bool ROSCameraRGB::CastPlugin() {
        camera_rgb_plugin_ = std::dynamic_pointer_cast<controller_native::NativeCameraRGBPluginBase>(native_base_);
        return true;
    }

    bool ROSCameraRGB::ShowInfo() {
        if (camera_rgb_plugin_) { return camera_rgb_plugin_->ShowInfo(); }
        return false;
    }

    bool ROSCameraRGB::OpenCameraRGBDevice() {
        if (camera_rgb_plugin_) {
            if (camera_rgb_plugin_->OpenDevice(
                        [this](const std::string &msg) {
                            controller_msgs::msg::ComponentValue component_value;
                            component_value.component_name = "CameraRGB";
                            controller_msgs::msg::PropertyValue property_value;
                            property_value.property_name  = "fault";
                            property_value.property_value = msg;
                            property_value.property_type  = controller_msgs::msg::PropertyValue::PROPERTY_TYPE_STRING;
                            property_value.property_desc  = "";
                            component_value.proterty_values.push_back(property_value);
                            pub_fault_->publish(component_value);
                            LOGGER_INFO(logger_, "FaultReport: %s", msg.c_str());
                        },
                        std::function<void(const cv::Mat &)>(
                                std::bind(&ROSCameraRGB::ROSPublishCB, this, std::placeholders::_1)))) {
                SetCameraInfo();
                if (auto [flag, device_info] = camera_rgb_plugin_->GetDeviceInfo(); flag) {
                    width_      = device_info.width;
                    height_     = device_info.height;
                    frame_rate_ = device_info.frame_rate;
                    camera_rgb_plugin_->StartCameraStream();
                    return true;
                }
            } else {
                LOGGER_ERROR(logger_, "ROSCameraRGB: Camera RGB plugin OpenDevice fail.");
            }
        } else {
            LOGGER_ERROR(logger_, "ROSCameraRGB: Camera RGB plugin is not initialized.");
        }
        return false;
    }

    bool ROSCameraRGB::InitCameraRGBRosInterface() {
        // 设置订阅者连接回调
        auto options                             = rclcpp::PublisherOptions();
        options.event_callbacks.matched_callback = [this](rclcpp::MatchedInfo &info) {
            if (info.current_count > 0) {
                if (camera_rgb_plugin_) {
                    LOGGER_INFO(logger_, "has subscriber, decode image");
                    camera_rgb_plugin_->SetPublish(true);
                }
            } else if (info.current_count == 0) {
                if ((image_publisher_ && image_publisher_->get_subscription_count() <= 0) &&
                    (compressed_image_publisher_ && compressed_image_publisher_->get_subscription_count() <= 0) &&
                    camera_rgb_plugin_) {
                    LOGGER_INFO(logger_, "image_publisher_ count:{} compressed_image_publisher_ count:{}",
                                image_publisher_->get_subscription_count(),
                                compressed_image_publisher_->get_subscription_count());
                    camera_rgb_plugin_->SetPublish(false);
                }
            }
        };

        image_publisher_ = create_publisher<sensor_msgs::msg::Image>("~/image_raw", rclcpp::SensorDataQoS(), options);
        compressed_image_publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(
                "~/image_compressed", rclcpp ::SensorDataQoS(), options);
        camera_info_publisher =
                create_publisher<sensor_msgs::msg::CameraInfo>("~/camera_info", rclcpp::SensorDataQoS());
        take_photo_server_ = create_service<camera_msgs::srv::TakePhoto>(
                "~/take_photo",
                std::bind(&ROSCameraRGB::HandleTakePhoto, this, std::placeholders::_1, std::placeholders::_2));
        live_stream_server_ = create_service<camera_msgs::srv::LiveStream>(
                "~/live_stream",
                std::bind(&ROSCameraRGB::HandleLiveStream, this, std::placeholders::_1, std::placeholders::_2));
        record_video_server_ = create_service<camera_msgs::srv::RecordVideo>(
                "~/record_video",
                std::bind(&ROSCameraRGB::HandleRecordVideo, this, std::placeholders::_1, std::placeholders::_2));
        start_camera_server_ = create_service<camera_msgs::srv::StartCamera>(
                "~/start_camera",
                std::bind(&ROSCameraRGB::HandleStartCamera, this, std::placeholders::_1, std::placeholders::_2));

        return true;
    }

    void ROSCameraRGB::SetCameraInfo() {
        auto camera_calibrate_param = camera_rgb_plugin_->GetCalibrationParam();

        camera_info_.width            = camera_calibrate_param.width;
        camera_info_.height           = camera_calibrate_param.height;
        camera_info_.distortion_model = camera_calibrate_param.distortion_model;
        camera_info_.d.resize(8, 0.0);

        for (int i = 0; i < 9; ++i) { camera_info_.k[i] = camera_calibrate_param.camera_matrix[i]; }
        for (int i = 0; i < 8; ++i) { camera_info_.d[i] = camera_calibrate_param.distortion_coefficients[i]; }
        for (int i = 0; i < 9; ++i) { camera_info_.r[i] = camera_calibrate_param.rectification_matrix[i]; }
        for (int i = 0; i < 12; ++i) { camera_info_.p[i] = camera_calibrate_param.projection_matrix[i]; }
    }

    void ROSCameraRGB::HandleTakePhoto(const std::shared_ptr<camera_msgs::srv::TakePhoto::Request> request,
                                       std::shared_ptr<camera_msgs::srv::TakePhoto::Response> response) {
        LOGGER_INFO(logger_, "ROSCameraRGB: HandleTakePhoto called with format: {}, resolution: {}x{}",
                    request->save_file.format, request->save_file.resolution_width,
                    request->save_file.resolution_height);
        if (camera_rgb_plugin_) {
            if (auto [flag, file_name, cv_jpg] = camera_rgb_plugin_->TakePhoto(
                        request->save_file.resolution_width, request->save_file.resolution_height,
                        request->save_file.file_dir, request->save_file.format);
                flag) {
                sensor_msgs::msg::CompressedImage compressed_image;
                compressed_image.header.stamp    = rclcpp::Clock().now();
                compressed_image.header.frame_id = "";
                compressed_image.format          = "jpg";
                cv::imencode("." + compressed_image.format, cv_jpg, compressed_image.data);
                response->comprressed_image = compressed_image;
                response->message           = file_name;
                response->success           = true;
            } else {
                response->success = false;
                response->message = "Failed to save image, no image captured.";
                LOGGER_ERROR(logger_, "ROSCameraRGB: Failed to save image, no image captured within 1s.");
            }
        } else {
            response->success = false;
            response->message = "Camera RGB plugin is not initialized.";
            LOGGER_ERROR(logger_, "ROSCameraRGB: Camera RGB plugin is not initialized.");
        }
    }

    void ROSCameraRGB::HandleLiveStream(const std::shared_ptr<camera_msgs::srv::LiveStream::Request> request,
                                        std::shared_ptr<camera_msgs::srv::LiveStream::Response> response) {
        if (camera_rgb_plugin_) {
            if (!request->url.empty()) {
                if (camera_rgb_plugin_->OpenLiveStream(request->url, request->width, request->height, frame_rate_,
                                                       request->bitrate)) {
                    response->success = true;
                } else {
                    response->success = false;
                    response->message = "OpenLiveStream fail.";
                    LOGGER_ERROR(logger_, "ROSCameraRGB: OpenLiveStream fail.");
                }

            } else {
                camera_rgb_plugin_->CloseLiveStream();
                response->success = true;
                response->message = "Live stream Close.";
            }
        } else {
            response->success = false;
            response->message = "Camera RGB plugin is not initialized.";
            LOGGER_ERROR(logger_, "ROSCameraRGB: Camera RGB plugin is not initialized.");
        }
    }

    void ROSCameraRGB::HandleRecordVideo(const std::shared_ptr<camera_msgs::srv::RecordVideo::Request> request,
                                         std::shared_ptr<camera_msgs::srv::RecordVideo::Response> response) {
        if (camera_rgb_plugin_) {
            if (request->record) {
                if (camera_rgb_plugin_->StartRecordVideo(frame_rate_, width_, height_, request->save_file.file_dir,
                                                         request->save_file.format)) {
                    response->success = true;
                    response->message = "Video recording started.";
                    LOGGER_INFO(logger_, "ROSCameraRGB: Video recording started.");
                } else {
                    response->success = false;
                    response->message = "Failed to start video recording.";
                    LOGGER_ERROR(logger_, "ROSCameraRGB: Failed to start video recording.");
                }
            } else {
                response->success = true;
                response->message = camera_rgb_plugin_->StopRecordVideo();
                LOGGER_INFO(logger_, "ROSCameraRGB: Video recording stopped.");
            }
        }
    }

    void ROSCameraRGB::ROSPublishCB(const cv::Mat &color_img) {
        std_msgs::msg::Header header;
        header.stamp    = now();
        header.frame_id = get_name();

        cv_bridge::CvImage cv_image{header, "rgb8", color_img};
        if (camera_info_publisher->get_subscription_count() > 0) {
            camera_info_.header = header;
            camera_info_publisher->publish(camera_info_);
        }

        if (image_publisher_->get_subscription_count() > 0) { image_publisher_->publish(*cv_image.toImageMsg()); }

        if (compressed_image_publisher_->get_subscription_count() > 0) {
            // sensor_msgs::msg::CompressedImage compressed_image;
            // cv_image.toCompressedImageMsg(compressed_image);
            // compressed_image_publisher_->publish(compressed_image);
            std::vector<uint8_t> compressed_data;
            cv::imencode(".jpg", color_img, compressed_data);

            sensor_msgs::msg::CompressedImage compressed_image;
            compressed_image.header = header;
            compressed_image.format = "jpeg";
            compressed_image.data   = compressed_data;
            compressed_image_publisher_->publish(compressed_image);
        }
    }

    void ROSCameraRGB::HandleStartCamera(const std::shared_ptr<camera_msgs::srv::StartCamera::Request> request,
                                         std::shared_ptr<camera_msgs::srv::StartCamera::Response> response) {
        if (camera_rgb_plugin_) {
            camera_rgb_plugin_->StopRecordVideo();
            camera_rgb_plugin_->CloseLiveStream();
            camera_rgb_plugin_->StopCameraStream();
            camera_rgb_plugin_->CloseDevice();

            controller_native::CameraRGBBaseInfo camera_info{
                    "",
                    "",
                    "",
                    request->width,
                    request->height,
                    request->frame_rate,
                    request->camera_format,
                    request->camera_device,
                    [this](const std::string &msg) {
                        controller_msgs::msg::ComponentValue component_value;
                        component_value.component_name = "CameraRGB";
                        controller_msgs::msg::PropertyValue property_value;
                        property_value.property_name  = "fault";
                        property_value.property_value = msg;
                        property_value.property_type  = controller_msgs::msg::PropertyValue::PROPERTY_TYPE_STRING;
                        property_value.property_desc  = "";
                        component_value.proterty_values.push_back(property_value);
                        pub_fault_->publish(component_value);
                        LOGGER_INFO(logger_, "FaultReport: %s", msg.c_str());
                    },
                    nullptr,
                    std::function<void(const cv::Mat &)>(
                            std::bind(&ROSCameraRGB::ROSPublishCB, this, std::placeholders::_1))};

            if (camera_rgb_plugin_->OpenDevice(camera_info)) {
                SetCameraInfo();
                width_      = camera_info.width;
                height_     = camera_info.height;
                frame_rate_ = camera_info.frame_rate;
                if (camera_rgb_plugin_->StartCameraStream()) {
                    response->success = true;
                    response->message = "Camera stream started successfully.";
                    LOGGER_INFO(logger_, "ROSCameraRGB: Camera stream started successfully.");
                } else {
                    response->success = false;
                    response->message = "Failed to start camera stream.";
                    LOGGER_ERROR(logger_, "ROSCameraRGB: Failed to start camera stream.");
                }
            } else {
                response->success = false;
                response->message = "Failed to open camera device.";
                LOGGER_ERROR(logger_, "ROSCameraRGB: Camera RGB plugin OpenDevice fail.");
            }
        } else {
            response->success = false;
            response->message = "Camera RGB plugin is not initialized.";
            LOGGER_ERROR(logger_, "ROSCameraRGB: Camera RGB plugin is not initialized.");
        }
    }
}// namespace controller_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(controller_ros::ROSCameraRGB)