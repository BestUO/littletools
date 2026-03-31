#include "native_camera_rgb_plugin_base/native_camera_rgb_plugin_base.h"
#include "native_camera_rgb_plugin_base/native_camera_rgb_struct_define.hpp"
#include "native_camera_rgb_plugin_base/utils.hpp"
#include "yjhlog/logger.h"
#include <cstdio>
#include <memory>
#include <opencv2/core/mat.hpp>
extern "C" {
#include <libavutil/frame.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}
#include "native_camera_rgb_plugin_base/live_stream.h"
#include "native_camera_rgb_plugin_base/record_video.h"
#include "native_camera_rgb_plugin_base/take_photo.h"
#ifdef USE_RK_MPP
#include "native_camera_rgb_plugin_base/h264ToMp4.hpp"
#include "native_camera_rgb_plugin_base/rockchip.hpp"
#include "native_camera_rgb_plugin_base/stream_h264_pusher.hpp"
#else
#include "native_camera_rgb_plugin_base/endecoder.h"
#endif

namespace controller_native {
    NativeCameraRGBPluginBase::NativeCameraRGBPluginBase() { StartWorkThread(); }

    NativeCameraRGBPluginBase::~NativeCameraRGBPluginBase() {
        StopWorkThread();
        Clear();
    }

    void NativeCameraRGBPluginBase::StartWorkThread() {
        if (!is_working_) {
            is_working_  = true;
            work_thread_ = std::thread(&NativeCameraRGBPluginBase::WorkThreadFunc, this);
        }
    }

    void NativeCameraRGBPluginBase::StopWorkThread() {
        is_working_ = false;
        if (work_thread_.joinable()) { work_thread_.join(); }
    }

    void NativeCameraRGBPluginBase::Clear() {// Ensure MP4 trailer is written.
        std::lock_guard<std::mutex> lock(mutex_);
        live_stream_.Close();
        take_photo_.Close();

#ifdef USE_RK_MPP
        if (decoder_ptr_) delete decoder_ptr_;
        if (encoder_ptr_) delete encoder_ptr_;
        if (mp4_muxer_ptr_) {
            mp4_muxer_ptr_->Close();
            delete mp4_muxer_ptr_;
        }
        if (stream_h264_pusher_ptr_) {
            stream_h264_pusher_ptr_->Stop();
            delete stream_h264_pusher_ptr_;
        }
#endif
    }

    void NativeCameraRGBPluginBase::ImageCallback(void *buffer, int buf_len) {
        if (buf_len > 0 && model_ != CameraRGBController::Model::IDLE) {
            auto camera_data_ptr = std::make_shared<CameraRGBController::DataDeepCopy>((uint8_t *) buffer, buf_len);
            camera_data_pv_.SetAndNotify(camera_data_ptr);
        }
    }

    void NativeCameraRGBPluginBase::WorkThreadFunc() {
        (void) pthread_setname_np(pthread_self(), "cameara_work");
        // controller_native::StreamH264Pusher sp;
        // sp.Init("rtmp://172.27.62.141:1935/camera1", 640, 80, 10);
        // sp.Start();
        auto time_now = std::chrono::steady_clock::now();
        while (is_working_) {
            if (auto camera_data_ptr = camera_data_pv_.WaitInfo(); camera_data_ptr) {
                if (auto time_now_tmp = std::chrono::steady_clock::now();
                    time_now_tmp - time_now > std::chrono::seconds(30)) {
                    time_now = time_now_tmp;
                    LOGGER_INFO(logger_, "recv camera data, model_:{}", (int) model_.load());
                }
                std::lock_guard<std::mutex> lock(mutex_);
#ifdef USE_RK_MPP
                auto frame_yuv420SP_VU =
                        decoder_ptr_->Decode((const char *) camera_data_ptr->data, camera_data_ptr->size);
                if (!frame_yuv420SP_VU) {
                    LOGGER_ERROR(logger_, "NativeCameraRGBPluginBase: Decode failed or no frame available.");
                    continue;
                }
                if (model_ & CameraRGBController::Model::SAVEIMG || model_ & CameraRGBController::Model::PUBLISH) {
                    if (cv::Mat color_img = decoder_ptr_->CVConvert(frame_yuv420SP_VU); color_img.empty()) {
                        std::cout << "CVConvert failed." << std::endl;
                        LOGGER_ERROR(logger_, "NativeCameraRGBPluginBase: CVConvert failed.");
                        continue;
                    } else {
                        // auto out_buf = encoder_ptr_->Encoder(frame_yuv420SP_VU);
                        // auto *data   = std::get<0>(out_buf);
                        // auto len     = std::get<1>(out_buf);
                        // if (!data || !len) { continue; }

                        // // Provide SPS/PPS header to RTMP pusher (some encoders don't prepend SPS/PPS to IDR).
                        // sp.setCodecHeaderAnnexB(encoder_ptr_->GetCodecHeader());
                        // // A crude keyframe hint: IDR NAL is type 5 after start code.
                        // bool key_hint = false;
                        // for (uint32_t i = 0; i + 5 < len; ++i) {
                        //     if ((uint8_t) data[i] == 0x00 && (uint8_t) data[i + 1] == 0x00 &&
                        //         (uint8_t) data[i + 2] == 0x00 && (uint8_t) data[i + 3] == 0x01) {
                        //         uint8_t nal = ((uint8_t) data[i + 4]) & 0x1f;
                        //         if (nal == 5) {
                        //             key_hint = true;
                        //             break;
                        //         }
                        //     }
                        // }

                        // sp.PushFrame((uint8_t *) data, len, key_hint);

                        // // Init dump/mp4 on first keyframe so outputs begin at IDR.
                        // if (key_hint && (!mp4_muxer_ptr_->IsOpen() || !mp4_muxer_ptr_->IsH264DumpEnabled())) {
                        //     const auto &hdr = encoder_ptr_->GetCodecHeader();
                        //     const int mp4_w = (int) mpp_frame_get_width(frame_yuv420SP_VU);
                        //     const int mp4_h = (int) mpp_frame_get_height(frame_yuv420SP_VU);
                        //     // if (!mp4_muxer_.IsH264DumpEnabled()) {
                        //     //     const bool ok = mp4_muxer_.EnableH264Dump("output.h264", hdr);
                        //     //     if (!ok) { LOGGER_ERROR(logger_, "NativeCameraRGBPluginBase: h264 dump open failed."); }
                        //     // }

                        //     if (!mp4_muxer_ptr_->IsOpen()) {
                        //         if (!hdr.empty()) {
                        //             mp4_muxer_ptr_->Init("output.mp4", 10);
                        //             const bool ok = mp4_muxer_ptr_->Init(mp4_w, mp4_h, hdr);
                        //             if (!ok) {
                        //                 LOGGER_ERROR(logger_, "NativeCameraRGBPluginBase: mp4 muxer init failed.");
                        //             }
                        //         }
                        //     }
                        // }

                        // if (mp4_muxer_ptr_->IsOpen() || mp4_muxer_ptr_->IsH264DumpEnabled()) {
                        //     (void) mp4_muxer_ptr_->WriteAnnexB((const uint8_t *) data, len, key_hint);
                        // }

                        if (model_ & CameraRGBController::Model::SAVEIMG) {
                            take_photo_.TakePhoto(color_img);
                            model_ &= ~CameraRGBController::Model::SAVEIMG;
                        }

                        if (model_ & CameraRGBController::Model::PUBLISH) {
                            camera_rgb_base_info_ptr_->user_image_callback(color_img);
                        }
                    }
                }

                if (model_ & CameraRGBController::Model::LIVESTREAM || model_ & CameraRGBController::Model::SAVEVIDEO) {
                    auto out_buf = encoder_ptr_->Encoder(frame_yuv420SP_VU);
                    auto *data   = std::get<0>(out_buf);
                    auto len     = std::get<1>(out_buf);
                    if (!data || !len) { continue; }

                    // A crude keyframe hint: IDR NAL is type 5 after start code.
                    bool key_hint = false;
                    for (uint32_t i = 0; i + 5 < len; ++i) {
                        if ((uint8_t) data[i] == 0x00 && (uint8_t) data[i + 1] == 0x00 &&
                            (uint8_t) data[i + 2] == 0x00 && (uint8_t) data[i + 3] == 0x01) {
                            uint8_t nal = ((uint8_t) data[i + 4]) & 0x1f;
                            if (nal == 5) {
                                key_hint = true;
                                break;
                            }
                        }
                    }

                    if (model_ & CameraRGBController::Model::SAVEVIDEO) {
                        // Init dump/mp4 on first keyframe so outputs begin at IDR.
                        if (key_hint && (!mp4_muxer_ptr_->IsOpen() || !mp4_muxer_ptr_->IsH264DumpEnabled())) {
                            const auto &hdr = encoder_ptr_->GetCodecHeader();
                            const int mp4_w = (int) mpp_frame_get_width(frame_yuv420SP_VU);
                            const int mp4_h = (int) mpp_frame_get_height(frame_yuv420SP_VU);
                            // if (!mp4_muxer_.IsH264DumpEnabled()) {
                            //     const bool ok = mp4_muxer_.EnableH264Dump("output.h264", hdr);
                            //     if (!ok) { LOGGER_ERROR(logger_, "NativeCameraRGBPluginBase: h264 dump open failed."); }
                            // }

                            if (!mp4_muxer_ptr_->IsOpen()) {
                                if (!hdr.empty()) {
                                    // mp4_muxer_ptr_->Init("output.mp4", 10);
                                    const bool ok = mp4_muxer_ptr_->Init(mp4_w, mp4_h, hdr);
                                    if (!ok) {
                                        LOGGER_ERROR(logger_, "NativeCameraRGBPluginBase: mp4 muxer init failed.");
                                    }
                                }
                            }
                        }

                        if (mp4_muxer_ptr_->IsOpen() || mp4_muxer_ptr_->IsH264DumpEnabled()) {
                            (void) mp4_muxer_ptr_->WriteAnnexB((const uint8_t *) data, len, key_hint);
                        }
                    }
                    if (model_ & CameraRGBController::Model::LIVESTREAM) {
                        stream_h264_pusher_ptr_->PushFrame((const uint8_t *) data, len, key_hint);
                    }
                }
#else
                AVFrame *frame = endecoder_.Decode(camera_data_ptr->data, camera_data_ptr->size);
                // AVFrame *frame_copy = av_frame_clone(frame_yuv420p);
                // auto frame_yuv420p_ptr = std::shared_ptr<AVFrame>(frame_copy, [](AVFrame *f) { av_frame_free(&f); });
                if (!frame) { continue; }
                if (model_ & CameraRGBController::Model::SAVEIMG || model_ & CameraRGBController::Model::PUBLISH) {
                    auto bgr_frame = endecoder_.TransformToGBR24(frame);
                    if (bgr_frame) {
                        cv::Mat mat_img = cv::Mat(bgr_frame->height, bgr_frame->width, CV_8UC3, bgr_frame->data[0]);
                        if (model_ & CameraRGBController::Model::SAVEIMG) {
                            take_photo_.TakePhoto(mat_img);
                            model_ &= ~CameraRGBController::Model::SAVEIMG;
                        }

                        if (model_ & CameraRGBController::Model::PUBLISH) {
                            cv::cvtColor(mat_img, mat_img, cv::COLOR_BGR2RGB);// 原地bgr2rgb转换
                            camera_rgb_base_info_ptr_->user_image_callback(mat_img);
                        }
                    }
                }

                if (model_ & CameraRGBController::Model::LIVESTREAM || model_ & CameraRGBController::Model::SAVEVIDEO) {
                    auto yuv420p_frame = endecoder_.TransformToYUV420(frame);

                    if (model_ & CameraRGBController::Model::LIVESTREAM) live_stream_.Write(yuv420p_frame);

                    if (model_ & CameraRGBController::Model::SAVEVIDEO) { save_video_.RecordVideo(yuv420p_frame); }
                }
#endif
            }
        }
    }


    void NativeCameraRGBPluginBase::SetPublish(bool flag) {
        if (flag) {
            model_ |= CameraRGBController::Model::PUBLISH;
        } else {
            model_ &= ~CameraRGBController::Model::PUBLISH;
        }
    }

    std::tuple<bool, std::string, cv::Mat> NativeCameraRGBPluginBase::TakePhoto(int32_t width, int32_t height,
                                                                                const std::string &file_dir,
                                                                                const std::string &dst_format) {
        std::lock_guard<std::mutex> lock(mutex_);
        take_photo_.Init(width, height, file_dir, dst_format);
        model_ |= CameraRGBController::Model::SAVEIMG;

        return take_photo_.WaitInfo(std::chrono::milliseconds(1000));
    }

    bool NativeCameraRGBPluginBase::OpenLiveStream(const std::string &url, int width, int height, int frame_rate,
                                                   [[maybe_unused]] int64_t bitrate) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (camera_rgb_base_info_ptr_) {
            if (!url.empty()) {
#ifdef USE_RK_MPP
                stream_h264_pusher_ptr_->Init(url, width, height, frame_rate);
                auto flag = stream_h264_pusher_ptr_->Start();
                if (flag) {
                    // Provide SPS/PPS header once per Start(); SetCodecHeaderAnnexB handles Annex-B/AVCC auto-detect.
                    stream_h264_pusher_ptr_->SetCodecHeaderAnnexB(encoder_ptr_->GetCodecHeader());
                }
#else
                auto flag = live_stream_.Init({url, width, height, frame_rate, bitrate});
#endif
                if (flag) {
                    model_ |= CameraRGBController::Model::LIVESTREAM;
                    LOGGER_INFO(logger_, "NativeCameraRGBPluginBase: Live stream initialized successfully with URL: {}",
                                url);
                    return true;
                } else {
                    LOGGER_ERROR(logger_, "NativeCameraRGBPluginBase: Failed to initialize live stream with URL: {}",
                                 url);
                    return false;
                }
            } else {
                LOGGER_ERROR(logger_, "NativeCameraRGBPluginBase: Live stream URL is empty or already open.");
                return false;
            }
        } else {
            LOGGER_ERROR(logger_, "NativeCameraRGBPluginBase: Camera RGB base info is not initialized.");
            return false;
        }
    }

    bool NativeCameraRGBPluginBase::CloseLiveStream() {
        std::lock_guard<std::mutex> lock(mutex_);
        model_ &= ~CameraRGBController::Model::LIVESTREAM;
#ifdef USE_RK_MPP
        stream_h264_pusher_ptr_->Stop();
#else
        live_stream_.Close();
#endif
        LOGGER_INFO(logger_, "NativeCameraRGBPluginBase: Live stream closed.");
        return true;
    }

    bool NativeCameraRGBPluginBase ::StartRecordVideo(int fps, int32_t width, int32_t height,
                                                      const std::string &file_dir, const std::string &dst_format) {
        std::lock_guard<std::mutex> lock(mutex_);
        (void) width;
        (void) height;
        LOGGER_INFO(logger_, "StartRecordVideo fps: {}, file_dir: {}, dst_format: {}", fps, file_dir, dst_format);
        if (camera_rgb_base_info_ptr_) {
#ifdef USE_RK_MPP
            // auto [h264_flag, h264_path] = CameraUtils::CreateDirectoriesAndGenFileName(file_dir, ".h264");
            auto [mp4_flag, mp4_path] = CameraUtils::CreateDirectoriesAndGenFileName(file_dir, ".mp4");
            auto flag                 = mp4_muxer_ptr_->Init(mp4_path, fps);
#else
            auto flag = save_video_.Init(fps, file_dir, camera_rgb_base_info_ptr_->camera_output_format, dst_format);
#endif
            if (flag) {
                model_ |= CameraRGBController::Model::SAVEVIDEO;
                LOGGER_INFO(logger_, "NativeCameraRGBPluginBase: Video save initialized successfully.");
                return true;
            } else {
                LOGGER_ERROR(logger_, "NativeCameraRGBPluginBase: Failed to initialize video save.");
                return false;
            }
        } else {
            LOGGER_ERROR(logger_, "NativeCameraRGBPluginBase: Camera RGB base info is not initialized.");
            return false;
        }
    }

    std::string NativeCameraRGBPluginBase::StopRecordVideo() {
        std::lock_guard<std::mutex> lock(mutex_);
        model_ &= ~CameraRGBController::Model::SAVEVIDEO;
        LOGGER_INFO(logger_, "StopRecordVideo");
#ifdef USE_RK_MPP
        return mp4_muxer_ptr_ ? mp4_muxer_ptr_->Close() : std::string();
#else
        return save_video_.Close();
#endif
    }

    bool NativeCameraRGBPluginBase::InitMPPDec(int width, int height) {
        (void) width; // Suppress unused parameter warning
        (void) height;// Suppress unused parameter warning
#ifdef USE_RK_MPP
        decoder_ptr_            = new DeCoder();
        encoder_ptr_            = new EnCoder();
        mp4_muxer_ptr_          = new h264_to_mp4::Mp4Muxer();
        stream_h264_pusher_ptr_ = new StreamH264Pusher();
#endif
        return true;
    }

    bool NativeCameraRGBPluginBase::InitEnDeCoder(int width, int height, [[maybe_unused]] int fps,
                                                  [[maybe_unused]] const std::string &camera_format,
                                                  [[maybe_unused]] int dst_width, [[maybe_unused]] int dst_height,
                                                  [[maybe_unused]] int bit_rate,
                                                  [[maybe_unused]] const std::string &dst_format) {
#ifdef USE_RK_MPP
        return encoder_ptr_->Init(width, height, fps, MPP_VIDEO_CodingAVC, MPP_FMT_YUV420SP_VU) &&
               decoder_ptr_->Init(width, height, MPP_VIDEO_CodingMJPEG, MPP_FMT_YUV420SP_VU);
#else
        return endecoder_.Init(width, height, fps, camera_format, dst_width, dst_height, bit_rate, dst_format);
#endif
    }

    bool NativeCameraRGBPluginBase::DeInitEnDeCoder() {
#ifdef USE_RK_MPP
        decoder_ptr_->Uninit();
        encoder_ptr_->Uninit();
#else
        endecoder_.Close();
#endif
        return true;
    }

    void NativeCameraRGBPluginBase::InitNaitve() {
#ifdef USE_RK_MPP
        if (mp4_muxer_ptr_) { mp4_muxer_ptr_->Close(); }
#endif
        endecoder_.SetLogger(logger_);
        save_video_.SetLogger(logger_);
        live_stream_.SetLogger(logger_);
        take_photo_.SetLogger(logger_);
    };

    void NativeCameraRGBPluginBase::SelfTest(StructSelfTest &response) {
        auto [flag, path, mat] = TakePhoto(320, 240, "/tmp", "JPG");
        StructSelfTestStatus status;
        status.name  = "camera status";
        status.level = StructSelfTestStatus::OK;
        if (!flag) {
            status.level   = StructSelfTestStatus::ERROR;
            status.message = "take photo failed";
        }
        response.passed = flag;
        response.status.push_back(status);
        LOGGER_INFO(logger_, "camera selftest flag:{} path:{}", flag, path);
    }
}// namespace controller_native