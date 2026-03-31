#pragma once

// 引入基础库和依赖项
#include "native_base/native_common_base.h"
// #include "native_camera_rgb_plugin_base/h264ToMp4.hpp"
#include "native_camera_rgb_plugin_base/live_stream.h"
#include "native_camera_rgb_plugin_base/native_camera_rgb_struct_define.hpp"
#include "native_camera_rgb_plugin_base/record_video.h"
// #include "native_camera_rgb_plugin_base/rockchip.hpp"
#include "native_camera_rgb_plugin_base/take_photo.h"
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace controller_native {
    class DeCoder;
    class EnCoder;
    class StreamH264Pusher;
    namespace h264_to_mp4 {
        class Mp4Muxer;

    };
    /**
     * @brief RGB相机插件基类
     * 
     * 该类为RGB相机设备提供统一的接口定义，继承自NativeCommonBase。
     * 支持相机设备的基本操作如开启/关闭设备、流媒体传输、拍照等功能。
     * 同时支持H.264硬件编解码和实时流媒体传输。
     */
    class NativeCameraRGBPluginBase : public NativeCommonBase {
    public:
        NativeCameraRGBPluginBase();
        virtual ~NativeCameraRGBPluginBase();

        /**
         * @brief 显示设备信息
         * @return bool 成功返回true，失败返回false
         */
        virtual bool ShowInfo() = 0;

        /**
         * @brief 打开相机设备
         * @param fault_report 错误报告回调函数
         * @param user_image_callback 用户图像数据回调函数
         * @return bool 成功返回true，失败返回false
         */
        virtual bool OpenDevice(std::function<void(const std::string &)> fault_report,
                                std::function<void(const cv::Mat &)> user_image_callback) = 0;

        /**
         * @brief 关闭相机设备
         * @return bool 成功返回true，失败返回false
         */
        virtual bool CloseDevice() = 0;

        /**
         * @brief 启动相机数据流
         * @return bool 成功返回true，失败返回false
         */
        virtual bool StartCameraStream() = 0;

        /**
         * @brief 停止相机数据流
         * @return bool 成功返回true，失败返回false
         */
        virtual bool StopCameraStream() = 0;

        /**
         * @brief 设置相机标定参数
         * @param camera_calibrate_param 相机标定参数结构体
         * @return bool 成功返回true，失败返回false
         */
        virtual bool SetCalibrationParam(CameraRGBCalibrateParam camera_calibrate_param) = 0;

        /**
         * @brief 获取相机标定参数
         * @return CameraCalibrateParamFull 包含相机标定参数的结构体
         */
        virtual CameraCalibrateParamFull GetCalibrationParam() { return CameraCalibrateParamFull(); }

        /**
         * @brief 获取设备基础信息
         * @return std::tuple<bool, CameraRGBBaseInfo> 包含操作结果和设备信息的元组
         */
        virtual std::tuple<bool, CameraRGBBaseInfo> GetDeviceInfo() = 0;
        virtual void InitNaitve() override;
        /**
         * @brief 图像数据回调处理函数
         * @param buffer 图像数据缓冲区指针
         * @param buf_len 缓冲区长度
         */
        void ImageCallback(void *buffer, int buf_len);

        /**
         * @brief 拍照功能
         * @param width 图像宽度
         * @param height 图像高度
         * @param should_save 是否保存图片到文件
         * @param file_dir 保存目录路径
         * @param format 图片格式
         * @return std::tuple<bool, std::string, cv::Mat> 包含操作结果、文件路径和图像数据的元组
         */
        void SetPublish(bool flag);
        std::tuple<bool, std::string, cv::Mat> TakePhoto(int32_t width, int32_t height, const std::string &file_dir,
                                                         const std::string &dst_format);

        /**
         * @brief 开启实时视频流
         * @param url 流媒体推送地址
         * @param width 视频宽度
         * @param height 视频高度
         * @param frame_rate 帧率
         * @param format 视频格式
         * @return bool 成功返回true，失败返回false
         */
        bool OpenLiveStream(const std::string &url, int width, int height, int frame_rate, int64_t bitrate);

        /**
         * @brief 关闭实时视频流
         * @return bool 成功返回true，失败返回false
         */
        bool CloseLiveStream();
        bool StartRecordVideo(int fps, int32_t width, int32_t height, const std::string &file_dir,
                              const std::string &dst_format);
        std::string StopRecordVideo();
        virtual bool OpenDevice([[maybe_unused]] const CameraRGBBaseInfo &camera_info) { return false; }
        virtual void SelfTest(StructSelfTest &response) override;

    protected:
        /// RGB相机基础信息共享指针
        std::shared_ptr<CameraRGBBaseInfo> camera_rgb_base_info_ptr_ = nullptr;
        CameraRGBController::DataPtrPV<CameraRGBController::DataDeepCopy> camera_data_pv_;
        std::thread work_thread_;///< 工作线程
        bool is_working_ = false;///< 是否正在工作

        /**
         * @brief 初始化MPP硬件解码器
         * @param width 视频宽度
         * @param height 视频高度
         * @return bool 成功返回true，失败返回false
         */
        bool InitMPPDec(int width, int height);
        bool InitEnDeCoder(int width, int height, int fps, const std::string &camera_format, int dst_width,
                           int dst_height, int bit_rate, const std::string &dst_format);
        bool DeInitEnDeCoder();
        void WorkThreadFunc();
        void StartWorkThread();
        void StopWorkThread();
        void Clear();

    private:
        // DeCoder decoder_;
        // EnCoder encoder_;
        // h264_to_mp4::Mp4Muxer mp4_muxer_;///< Debug/remux: dump H264 and/or mux MP4
        // LiveStreamClass live_stream_;    ///< H.264实时视频流处理对象
        // TakePhotoClass take_photo_;      ///< 拍照处理对象
        // std::atomic<int> model_ = 0;     ///< 相机型号/模式，使用原子变量保证线程安全
        DeCoder *decoder_ptr_                     = nullptr;///< MPP硬件解码器对象
        EnCoder *encoder_ptr_                     = nullptr;///< MPP硬件编码器对象
        h264_to_mp4::Mp4Muxer *mp4_muxer_ptr_     = nullptr;///< Debug/remux: dump H264 and/or mux MP4
        StreamH264Pusher *stream_h264_pusher_ptr_ = nullptr;///< H.264流推送器对象
        // DeCoder decoder_;                               ///< MPP硬件解码器对象
        // EnCoder encoder_;                               ///< MPP硬件编码器对象
        // h264_to_mp4::Mp4Muxer mp4_muxer_;           ///< Debug/remux: dump H264 and/or mux MP4
        EnDecoderClass endecoder_;   ///< 编解码器对象
        RecordVideoClass save_video_;///< 视频保存对象
        LiveStreamClass live_stream_;///< H.264实时视频流处理对象
        TakePhotoClass take_photo_;  ///< 拍照处理对象
        std::atomic<int> model_ = 0; ///< 相机型号/模式，使用原子变量保证线程安全
                                     // #ifdef USE_RK_MPP
                                     //         DeCoder decoder_;
                                     //         EnCoder encoder_;
        //         h264_to_mp4::Mp4Muxer mp4_muxer_;///< Debug/remux: dump H264 and/or mux MP4
        // #else
        //         EnDecoderClass endecoder_;   ///< 编解码器对象
        //         RecordVideoClass save_video_;///< 视频保存对象
        // #endif
        //         LiveStreamClass live_stream_;///< H.264实时视频流处理对象
        //         TakePhotoClass take_photo_;  ///< 拍照处理对象
        //         std::atomic<int> model_ = 0; ///< 相机型号/模式，使用原子变量保证线程安全
    };
}// namespace controller_native