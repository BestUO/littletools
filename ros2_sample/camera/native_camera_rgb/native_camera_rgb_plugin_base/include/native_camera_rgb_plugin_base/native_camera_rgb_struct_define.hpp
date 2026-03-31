#pragma once
#include <condition_variable>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <string>

namespace controller_native {
    struct CameraRGBBaseInfo {
        std::string uuid;
        std::string serial_number;
        std::string usb_port;

        int width = 0;
        int height = 0;
        int frame_rate = 10;
        std::string camera_output_format;// e.g., "MJPG", "H264", etc.
        std::string camera_device;       // if exist e.g., "/dev/video0"

        std::function<void(const std::string &)> dev_status_callback = nullptr;
        std::function<void(void *buffer, int buf_len)> camera_image_callback = nullptr;
        std::function<void(const cv::Mat &)> user_image_callback = nullptr;
    };

    struct CameraRGBCalibrateParam {
        float Cx;
        float Cy;
        float Fx;
        float Fy;
        float K1;
        float K2;
        float K3;
        float P1;
        float P2;
    };

    struct CameraCalibrateParamFull {
        int width = 0;
        int height = 0;
        std::string distortion_model = "plumb_bob";// e.g., "plumb_bob", "rational_polynomial"
        float camera_matrix[9] = {0.0f};           // 3x3 camera matrix
        float distortion_coefficients[8] = {0.0f}; // Distortion
        float rectification_matrix[9] = {0.0f};    // 3x3 rectification matrix
        float projection_matrix[12] = {0.0f};      // 3x4
    };

    struct CameraRGBController {
        enum Model {
            IDLE = 0,
            PUBLISH = 1 << 0,
            SAVEIMG = PUBLISH << 1,
            SAVEVIDEO = SAVEIMG << 1,
            LIVESTREAM = SAVEVIDEO << 1,
        };

        struct DataDeepCopy {
            uint8_t *data = nullptr;
            size_t size = 0;

            DataDeepCopy() = default;
            DataDeepCopy(uint8_t *data_ptr, size_t size) : size(size) {
                data = new uint8_t[size];
                memcpy(this->data, data_ptr, size);
            }
            ~DataDeepCopy() {
                if (data) {
                    delete[] data;
                    data = nullptr;
                    size = 0;
                }
            };
        };

        template<typename T>
        struct DataPtrPV {
            std::mutex mutex;
            std::condition_variable cv;
            std::shared_ptr<T> data_descriptor = nullptr;

            std::shared_ptr<T> WaitInfo(std::chrono::milliseconds timeout = std::chrono::milliseconds(1000)) {
                std::unique_lock<std::mutex> lock(mutex);

                if (cv.wait_for(lock, timeout, [this]() { return data_descriptor != nullptr; })) {
                    return std::move(data_descriptor);
                }
                return nullptr;
            }

            void SetAndNotify(std::shared_ptr<T> data) {
                std::lock_guard<std::mutex> lock(mutex);

                data_descriptor = data;

                cv.notify_one();
            }

            void ClearCache() {
                std::lock_guard<std::mutex> lock(mutex);
                data_descriptor = nullptr;
            }

            void Notify() {
                std::lock_guard<std::mutex> lock(mutex);
                cv.notify_all();
            }
        };
    };
};// namespace controller_native