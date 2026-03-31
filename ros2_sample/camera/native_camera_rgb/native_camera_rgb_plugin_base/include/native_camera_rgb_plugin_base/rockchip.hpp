#pragma once

#include <cstring>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>
extern "C" {
#include "rockchip/mpi_dec_utils.h"
#include "rockchip/mpp_buffer.h"
#include "rockchip/mpp_frame.h"
#include "rockchip/mpp_log.h"
#include "rockchip/mpp_packet.h"
#include "rockchip/rk_mpi.h"
}

#define MPP_ALIGN(x, a) (((x) + (a) - 1) & ~((a) - 1))

namespace controller_native {
    class DeCoder {
    public:
        bool Init(int width, int height, MppCodingType input_format, MppFrameFormat output_format) {
            input_format_  = input_format;
            output_format_ = output_format;
            need_split_    = (input_format_ == MPP_VIDEO_CodingMJPEG) ? 0 : 1;
            simple_        = (input_format != MPP_VIDEO_CodingMJPEG) ? true : false;
            MPP_RET ret    = MPP_OK;

            if (ret == MPP_OK) {
                if (ret = dec_buf_mgr_init(&buf_mgr_); ret) { mpp_err("dec_buf_mgr_init failed\n"); }
            }

            if (ret == MPP_OK) {
                if (ret = mpp_packet_init(&packet_, NULL, 0); ret) { mpp_err("mpp_packet_init failed\n"); }
            }

            if (ret == MPP_OK) {
                if (ret = mpp_create(&ctx_, &mpi_); ret) { mpp_err("mpp_create failed\n"); }
            }

            mpp_log("%p mpi_dec_test decoder start w %d h %d type %d\n", ctx_, width, height, input_format_);

            if (ret == MPP_OK) {
                if (ret = mpi_->control(ctx_, MPP_SET_DISABLE_THREAD, NULL); ret) {
                    mpp_err("MPP_SET_DISABLE_THREAD failed\n");
                }
            }

            if (ret == MPP_OK) {
                if (ret = mpp_init(ctx_, MPP_CTX_DEC, input_format); ret) { mpp_err("mpp_init failed\n"); }
            }

            mpp_dec_cfg_init(&cfg_);

            /* get default config from decoder context */
            if (ret == MPP_OK) {
                if (ret = mpi_->control(ctx_, MPP_DEC_GET_CFG, cfg_); ret) {
                    mpp_err("%p failed to get decoder cfg ret %d\n", ctx_, ret);
                }
            }

            if (ret == MPP_OK) {
                if (ret = mpp_dec_cfg_set_u32(cfg_, "base:split_parse", need_split_); ret) {
                    mpp_err("%p failed to set split_parse ret %d\n", ctx_, ret);
                }
            }

            if (ret == MPP_OK) {
                if (ret = mpi_->control(ctx_, MPP_DEC_SET_CFG, cfg_); ret) {
                    mpp_err("%p failed to set cfg ret %d\n", ctx_, ret);
                }
            }

            if (!simple_) {
                // Set decoder output format once (safe to call multiple times).
                if (MPP_FRAME_FMT_IS_YUV(output_format_) || MPP_FRAME_FMT_IS_RGB(output_format_)) {
                    MppFrameFormat fmt = output_format_;
                    ret                = mpi_->control(ctx_, MPP_DEC_SET_OUTPUT_FORMAT, &fmt);
                    if (ret) {
                        mpp_err("Failed to set output format 0x%x ret %d\n", fmt, ret);
                        return ret;
                    }
                }
            }
            return ret == MPP_OK;
        }

        void Uninit() {
            if (packet_) {
                mpp_packet_deinit(&packet_);
                packet_ = nullptr;
            }

            if (frame_) {
                mpp_frame_deinit(&frame_);
                frame_ = nullptr;
            }

            if (ctx_) {
                mpp_destroy(ctx_);
                ctx_ = nullptr;
            }

            if (!simple_) {
                if (pkt_buf_) {
                    mpp_buffer_put(pkt_buf_);
                    pkt_buf_ = nullptr;
                }
            }

            frm_grp_ = nullptr;
            if (buf_mgr_) {
                dec_buf_mgr_deinit(buf_mgr_);
                buf_mgr_ = nullptr;
            }

            if (cfg_) {
                mpp_dec_cfg_deinit(cfg_);
                cfg_ = nullptr;
            }
        }

        MppFrame Decode(const char *data, size_t size) {
            if (!ctx_ || !mpi_ || !packet_ || !data || size == 0) return nullptr;
            if (!simple_) {
                if (!pkt_buf_ || mpp_buffer_get_size(pkt_buf_) < size) {
                    if (pkt_buf_) {
                        mpp_buffer_put(pkt_buf_);
                        pkt_buf_ = nullptr;
                    }
                    MPP_RET r = mpp_buffer_get(NULL, &pkt_buf_, size);
                    if (r) {
                        mpp_err("mpp_buffer_get for input failed size %zu ret %d\n", size, r);
                        return nullptr;
                    }
                }

                void *buf = mpp_buffer_get_ptr(pkt_buf_);
                memcpy(buf, data, size);

                mpp_packet_set_data(packet_, buf);
                mpp_packet_set_size(packet_, size);
                mpp_packet_set_pos(packet_, buf);
                mpp_packet_set_length(packet_, size);
                mpp_packet_set_buffer(packet_, pkt_buf_);
            } else {
                mpp_packet_set_data(packet_, (void *) data);
                mpp_packet_set_size(packet_, size);
                mpp_packet_set_pos(packet_, (void *) data);
                mpp_packet_set_length(packet_, size);
            }
            mpp_packet_clr_eos(packet_);

            if (frame_) {
                mpp_frame_deinit(&frame_);
                frame_ = nullptr;
            }

            constexpr int kDrainTries = 6;
            for (int i = 0; i < kDrainTries; ++i) {
                auto ret = simple_ ? DecSimple(data, size) : DecAdvanced(data, size);
                if (ret && ret != MPP_ERR_TIMEOUT && ret != MPP_ERR_BUFFER_FULL) return nullptr;
                if (frame_) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            return frame_;
        }

        cv::Mat CVConvert(MppFrame frame) {
            cv::Mat image;
            RK_U32 err_info = mpp_frame_get_errinfo(frame);
            RK_U32 discard  = mpp_frame_get_discard(frame);

            if (err_info == 0 && discard == 0) {
                RK_U32 width       = mpp_frame_get_width(frame);
                RK_U32 height      = mpp_frame_get_height(frame);
                RK_U32 hor_stride  = mpp_frame_get_hor_stride(frame);
                MppBuffer buffer   = mpp_frame_get_buffer(frame);
                MppFrameFormat fmt = mpp_frame_get_fmt(frame);

                if (buffer) {
                    void *ptr = mpp_buffer_get_ptr(buffer);
                    if (ptr) {
                        if (fmt == MPP_FMT_RGB888) {
                            cv::Mat rgb_img(height, width, CV_8UC3, ptr, hor_stride * 3);
                            cv::cvtColor(rgb_img, image, cv::COLOR_RGB2BGR);
                        } else if (fmt == MPP_FMT_BGR888) {
                            cv::Mat bgr_img(height, width, CV_8UC3, ptr, hor_stride * 3);
                            bgr_img.copyTo(image);
                        } else if (fmt == MPP_FMT_RGBA8888) {
                            cv::Mat rgba_img(height, width, CV_8UC4, ptr, hor_stride * 4);
                            cv::cvtColor(rgba_img, image, cv::COLOR_RGBA2BGR);
                        } else if (fmt == MPP_FMT_BGRA8888) {
                            cv::Mat bgra_img(height, width, CV_8UC4, ptr, hor_stride * 4);
                            cv::cvtColor(bgra_img, image, cv::COLOR_BGRA2BGR);
                        } else {
                            // Default to NV12 or other YUV formats
                            cv::Mat yuv_img(height * 3 / 2, width, CV_8UC1, ptr, hor_stride);
                            cv::Mat yuv_cropped = yuv_img(cv::Rect(0, 0, width, height * 3 / 2));
                            cv::cvtColor(yuv_cropped, image, cv::COLOR_YUV2RGB_NV12);
                        }
                    }
                }
            }
            return image;
        }

    private:
        MppCodingType input_format_;
        MppFrameFormat output_format_;
        RK_U32 need_split_ = 0;

        MppCtx ctx_  = nullptr;
        MppApi *mpi_ = nullptr;

        MppPacket packet_ = nullptr;
        MppFrame frame_   = nullptr;

        DecBufMgr buf_mgr_      = nullptr;
        MppBufferGroup frm_grp_ = nullptr;
        MppBuffer pkt_buf_      = nullptr;
        MppDecCfg cfg_          = nullptr;
        bool simple_            = false;

        int DecSimple(const char *, size_t) {
            MPP_RET ret     = MPP_OK;
            RK_U32 pkt_done = 0;

            for (int i = 0; i < 10 && !pkt_done; i++) {
                ret = mpi_->decode_put_packet(ctx_, packet_);
                if (ret == MPP_OK) {
                    pkt_done = 1;
                    break;
                }
                if (ret == MPP_ERR_BUFFER_FULL || ret == MPP_ERR_TIMEOUT) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
                mpp_err("decode_put_packet failed ret %d\n", ret);
                return ret;
            }
            if (!pkt_done) return MPP_ERR_TIMEOUT;

            for (int times = 30; times > 0; times--) {
                MppFrame frame = nullptr;
                ret            = mpi_->decode_get_frame(ctx_, &frame);
                if (ret == MPP_ERR_TIMEOUT) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
                if (ret) {
                    mpp_err("decode_get_frame failed ret %d\n", ret);
                    return ret;
                }
                if (!frame) return MPP_OK;

                if (mpp_frame_get_info_change(frame)) {
                    RK_U32 buf_size    = mpp_frame_get_buf_size(frame);
                    MppBufferGroup grp = dec_buf_mgr_setup(buf_mgr_, buf_size, 24, MPP_DEC_BUF_INTERNAL);
                    ret                = mpi_->control(ctx_, MPP_DEC_SET_EXT_BUF_GROUP, grp);
                    if (ret) {
                        mpp_err("set buffer group failed ret %d\n", ret);
                        mpp_frame_deinit(&frame);
                        return ret;
                    }
                    frm_grp_ = grp;
                    ret      = mpi_->control(ctx_, MPP_DEC_SET_INFO_CHANGE_READY, NULL);
                    mpp_frame_deinit(&frame);
                    if (ret) {
                        mpp_err("info change ready failed ret %d\n", ret);
                        return ret;
                    }
                    continue;
                }

                frame_ = frame;
                return MPP_OK;
            }

            return MPP_OK;
        }

        int DecAdvanced(const char *, size_t) {
            MPP_RET ret    = MPP_OK;
            MppFrame frame = nullptr;

            ret = mpi_->decode(ctx_, packet_, &frame);
            if (ret) {
                if (ret == MPP_ERR_TIMEOUT || ret == MPP_ERR_BUFFER_FULL) return MPP_OK;
                mpp_err("decode failed ret %d\n", ret);
                return ret;
            }

            if (!frame) return MPP_OK;

            if (mpp_frame_get_info_change(frame)) {
                RK_U32 buf_size    = mpp_frame_get_buf_size(frame);
                MppBufferGroup grp = dec_buf_mgr_setup(buf_mgr_, buf_size, 24, MPP_DEC_BUF_INTERNAL);
                ret                = mpi_->control(ctx_, MPP_DEC_SET_EXT_BUF_GROUP, grp);
                if (ret) {
                    mpp_err("set buffer group failed ret %d\n", ret);
                    mpp_frame_deinit(&frame);
                    return ret;
                }
                frm_grp_ = grp;
                ret      = mpi_->control(ctx_, MPP_DEC_SET_INFO_CHANGE_READY, NULL);
                mpp_frame_deinit(&frame);
                if (ret) {
                    mpp_err("info change ready failed ret %d\n", ret);
                    return ret;
                }
                return MPP_OK;
            }

            frame_ = frame;
            return MPP_OK;
        }
    };

    class EnCoder {
    public:
        // Returns codec header (SPS/PPS for H264 or VPS/SPS/PPS for H265) in AnnexB format.
        // This should be written once at the beginning of a raw .h264/.h265 file.
        const std::vector<uint8_t> &GetCodecHeader() const { return codec_header_; }

        bool Init(int width, int height, int fps, MppCodingType output_type, MppFrameFormat input_fmt) {
            // If re-initialized, clean up previous instance to avoid leaks / undefined behavior.
            // This also matches typical lifecycle expectations of MPP contexts.
            Uninit();

            width_       = width;
            height_      = height;
            input_fmt_   = input_fmt;
            output_type_ = output_type;
            fps_         = fps;

            MPP_RET ret = MPP_OK;

            codec_header_.clear();
            if (ret == MPP_OK) {
                ret = mpp_create(&ctx_, &mpi_);
                if (ret) { mpp_err("mpp_create failed ret %d\n", ret); }
            }

            if (ret == MPP_OK) {
                ret = mpp_init(ctx_, MPP_CTX_ENC, output_type_);
                if (ret) { mpp_err("mpp_init failed ret %d\n", ret); }
            }

            if (ret == MPP_OK) {
                ret = mpp_enc_cfg_init(&cfg_);
                if (ret) { mpp_err("mpp_enc_cfg_init failed ret %d\n", ret); }
            }

            if (ret == MPP_OK) {
                /* setup default parameter */
                int fps_in_num  = fps_;
                int fps_in_den  = 1;
                int fps_out_num = fps_;
                int fps_out_den = 1;
                int bps         = width * height / 8 * (fps_out_num / fps_out_den);
                int bps_max     = bps * 17 / 16;
                int bps_min     = bps * 15 / 16;
                int gop_len     = fps_out_num * 2;
                int qp_init     = -1;
                int qp_max      = 51;
                int qp_min      = 10;
                int qp_max_i    = 51;
                int qp_min_i    = 10;
                int qp_ip       = 2;
                int fqp_min_i   = 10;
                int fqp_max_i   = 45;
                int fqp_min_p   = 10;
                int fqp_max_p   = 45;
                int profile     = 100; /* High profile */
                int level       = 40;  /* 1080p@30fps */
                int cabac_en    = 1;
                int cabac_idc   = 0;
                int trans8x8    = 1;
                int q_factor    = 80;
                int qf_max      = 99;
                int qf_min      = 1;
                int rc_mode     = MPP_ENC_RC_MODE_CBR;
                int mirroring   = 0;
                int rotation    = 0;
                int flip        = 0;
                // int gop_mode     = 0;
                MppEncRefCfg ref = NULL;

                /* setup preprocess parameters */
                mpp_enc_cfg_set_s32(cfg_, "prep:width", width_);
                mpp_enc_cfg_set_s32(cfg_, "prep:height", height_);
                mpp_enc_cfg_set_s32(cfg_, "prep:hor_stride", width_);
                mpp_enc_cfg_set_s32(cfg_, "prep:ver_stride", height_);
                mpp_enc_cfg_set_s32(cfg_, "prep:format", input_fmt_);
                mpp_enc_cfg_set_s32(cfg_, "prep:mirroring", mirroring);
                mpp_enc_cfg_set_s32(cfg_, "prep:rotation", rotation);
                mpp_enc_cfg_set_s32(cfg_, "prep:flip", flip);

                /* setup rate control parameters */
                mpp_enc_cfg_set_s32(cfg_, "rc:mode", rc_mode);
                mpp_enc_cfg_set_u32(cfg_, "rc:max_reenc_times", 0);
                mpp_enc_cfg_set_u32(cfg_, "rc:super_mode", 0);

                /* fix input / output frame rate */
                mpp_enc_cfg_set_s32(cfg_, "rc:fps_in_flex", 0);
                mpp_enc_cfg_set_s32(cfg_, "rc:fps_in_num", fps_in_num);
                mpp_enc_cfg_set_s32(cfg_, "rc:fps_in_denom", fps_in_den);
                mpp_enc_cfg_set_s32(cfg_, "rc:fps_out_flex", 0);
                mpp_enc_cfg_set_s32(cfg_, "rc:fps_out_num", fps_out_num);
                mpp_enc_cfg_set_s32(cfg_, "rc:fps_out_denom", fps_out_den);

                /* drop frame or not when bitrate overflow */
                mpp_enc_cfg_set_u32(cfg_, "rc:drop_mode", MPP_ENC_RC_DROP_FRM_DISABLED);
                mpp_enc_cfg_set_u32(cfg_, "rc:drop_thd", 20); /* 20% of max bps */
                mpp_enc_cfg_set_u32(cfg_, "rc:drop_gap", 1);  /* Do not continuous drop frame */

                /* setup bitrate for different rc_mode */
                mpp_enc_cfg_set_s32(cfg_, "rc:bps_target", bps);
                mpp_enc_cfg_set_s32(cfg_, "rc:bps_max", bps_max);
                mpp_enc_cfg_set_s32(cfg_, "rc:bps_min", bps_min);

                /* setup qp for different codec and rc_mode */
                switch (output_type_) {
                    case MPP_VIDEO_CodingAVC:
                    case MPP_VIDEO_CodingHEVC: {
                        mpp_enc_cfg_set_s32(cfg_, "rc:qp_init", qp_init);
                        mpp_enc_cfg_set_s32(cfg_, "rc:qp_max", qp_max);
                        mpp_enc_cfg_set_s32(cfg_, "rc:qp_min", qp_min);
                        mpp_enc_cfg_set_s32(cfg_, "rc:qp_max_i", qp_max_i);
                        mpp_enc_cfg_set_s32(cfg_, "rc:qp_min_i", qp_min_i);
                        mpp_enc_cfg_set_s32(cfg_, "rc:qp_ip", qp_ip);
                        mpp_enc_cfg_set_s32(cfg_, "rc:fqp_min_i", fqp_min_i);
                        mpp_enc_cfg_set_s32(cfg_, "rc:fqp_max_i", fqp_max_i);
                        mpp_enc_cfg_set_s32(cfg_, "rc:fqp_min_p", fqp_min_p);
                        mpp_enc_cfg_set_s32(cfg_, "rc:fqp_max_p", fqp_max_p);
                    } break;
                    case MPP_VIDEO_CodingVP8: {
                        /* vp8 only setup base qp range */
                        mpp_enc_cfg_set_s32(cfg_, "rc:qp_init", 40);
                        mpp_enc_cfg_set_s32(cfg_, "rc:qp_max", 127);
                        mpp_enc_cfg_set_s32(cfg_, "rc:qp_min", 0);
                        mpp_enc_cfg_set_s32(cfg_, "rc:qp_max_i", 127);
                        mpp_enc_cfg_set_s32(cfg_, "rc:qp_min_i", 0);
                        mpp_enc_cfg_set_s32(cfg_, "rc:qp_ip", 6);
                    } break;
                    case MPP_VIDEO_CodingMJPEG: {
                        /* jpeg use special codec config to control qtable */
                        mpp_enc_cfg_set_s32(cfg_, "jpeg:q_factor", q_factor);
                        mpp_enc_cfg_set_s32(cfg_, "jpeg:qf_max", qf_max);
                        mpp_enc_cfg_set_s32(cfg_, "jpeg:qf_min", qf_min);
                    } break;
                    default: {
                    } break;
                }

                /* setup codec  */
                mpp_enc_cfg_set_s32(cfg_, "codec:type", output_type_);
                switch (output_type_) {
                    case MPP_VIDEO_CodingAVC: {
                        mpp_enc_cfg_set_s32(cfg_, "h264:profile", profile);
                        mpp_enc_cfg_set_s32(cfg_, "h264:level", level);
                        mpp_enc_cfg_set_s32(cfg_, "h264:cabac_en", cabac_en);
                        mpp_enc_cfg_set_s32(cfg_, "h264:cabac_idc", cabac_idc);
                        mpp_enc_cfg_set_s32(cfg_, "h264:trans8x8", trans8x8);
                    } break;
                    case MPP_VIDEO_CodingHEVC:
                    case MPP_VIDEO_CodingMJPEG:
                    case MPP_VIDEO_CodingVP8: {
                    } break;
                    default: {
                        mpp_err_f("unsupport encoder coding type %d\n", output_type_);
                    } break;
                }

                mpp_enc_cfg_set_s32(cfg_, "rc:gop", gop_len);

                // if (gop_mode) {
                //     mpp_enc_ref_cfg_init(&ref);
                //     if (gop_mode < 4)
                //         mpi_enc_gen_ref_cfg(ref, gop_mode);
                //     else
                //         mpi_enc_gen_smart_gop_ref_cfg(ref, gop_len, gop_len);
                //     mpp_enc_cfg_set_ptr(cfg_, "rc:ref_cfg", ref);
                // }

                ret = mpi_->control(ctx_, MPP_ENC_SET_CFG, cfg_);
                if (ret) { mpp_err("mpi control enc set cfg failed ret %d\n", ret); }

                if (ref) mpp_enc_ref_cfg_deinit(&ref);
            }

            if (ret == MPP_OK) {
                // NOTE:
                // For MPP_ENC_GET_HDR_SYNC, the packet must have a valid buffer.
                // Using mpp_packet_init(packet, NULL, 0) results in an empty packet without
                // backing storage and can crash inside mpp_packet_copy() (memcpy on invalid ptr).
                // See Rockchip sample: hw_endecoder/sample/mpi_enc_test.c
                MppPacket packet  = nullptr;
                MppBuffer pkt_buf = nullptr;
                if (output_type_ == MPP_VIDEO_CodingAVC || output_type_ == MPP_VIDEO_CodingHEVC) {
                    // Allocate a small packet buffer for codec headers (SPS/PPS/VPS).
                    // 4KB is usually enough for AVC/HEVC extra data.
                    const size_t hdr_buf_size = 4096;
                    ret                       = mpp_buffer_get(nullptr, &pkt_buf, hdr_buf_size);
                    if (ret) {
                        mpp_err("mpp_buffer_get for hdr failed ret %d\n", ret);
                    } else {
                        ret = mpp_packet_init_with_buffer(&packet, pkt_buf);
                        if (ret) {
                            mpp_err("mpp_packet_init_with_buffer failed ret %d\n", ret);
                        } else {
                            // IMPORTANT: clear output packet length before GET_HDR_SYNC
                            mpp_packet_set_length(packet, 0);

                            /* get sps/pps (and vps for hevc) */
                            ret = mpi_->control(ctx_, MPP_ENC_GET_HDR_SYNC, packet);
                            if (ret) {
                                mpp_err("mpi control enc get extra info failed\n");
                            } else {
                                void *ptr  = mpp_packet_get_pos(packet);
                                size_t len = mpp_packet_get_length(packet);
                                if (ptr && len) { codec_header_.assign((uint8_t *) ptr, (uint8_t *) ptr + len); }
                            }
                        }
                    }
                    if (ret) { mpp_err("mpi control enc get extra info failed\n"); }
                }

                if (packet) mpp_packet_deinit(&packet);
                if (pkt_buf) mpp_buffer_put(pkt_buf);
            }

            return ret == MPP_OK;
        }

        void Uninit() {
            if (buf_) {
                delete[] buf_;
                buf_ = nullptr;
            }
            buf_size_ = 0;

            // Release reusable output packet/buffer.
            if (out_packet_) {
                mpp_packet_deinit(&out_packet_);
                out_packet_ = nullptr;
            }
            if (out_pkt_buf_) {
                mpp_buffer_put(out_pkt_buf_);
                out_pkt_buf_ = nullptr;
            }
            out_pkt_buf_size_ = 0;

            if (ctx_) {
                mpp_destroy(ctx_);
                ctx_ = nullptr;
            }
            if (cfg_) {
                mpp_enc_cfg_deinit(cfg_);
                cfg_ = nullptr;
            }
            codec_header_.clear();
        }

        ~EnCoder() { Uninit(); }

        std::tuple<char *, uint32_t> Encoder(MppFrame frame) {
            if (!ctx_ || !mpi_ || !frame) return {nullptr, 0};

            // Reuse an output packet with a backing MppBuffer.
            // This avoids per-frame internal allocations that can look like a slow memory leak.
            // The packet content is copied into buf_ before returning.

            MPP_RET ret = MPP_OK;

            // Lazily allocate output packet/buffer.
            // Start at 1MB and grow if needed (rare large frames / IDR spikes).
            if (!out_packet_ || !out_pkt_buf_) {
                constexpr size_t kInitialOutPktBufSize = 1 * 1024 * 1024;
                out_pkt_buf_size_                      = kInitialOutPktBufSize;
                ret                                    = mpp_buffer_get(nullptr, &out_pkt_buf_, out_pkt_buf_size_);
                if (ret) {
                    mpp_err("mpp_buffer_get for out packet failed ret %d\n", ret);
                    out_pkt_buf_      = nullptr;
                    out_pkt_buf_size_ = 0;
                    return {nullptr, 0};
                }

                ret = mpp_packet_init_with_buffer(&out_packet_, out_pkt_buf_);
                if (ret) {
                    mpp_err("mpp_packet_init_with_buffer failed ret %d\n", ret);
                    mpp_buffer_put(out_pkt_buf_);
                    out_pkt_buf_      = nullptr;
                    out_pkt_buf_size_ = 0;
                    out_packet_       = nullptr;
                    return {nullptr, 0};
                }
            }

            // Clear packet length before encoding.
            mpp_packet_set_length(out_packet_, 0);

            ret = mpi_->encode_put_frame(ctx_, frame);
            if (ret) {
                mpp_err("encode_put_frame failed ret %d\n", ret);
                return {nullptr, 0};
            }

            // Some MPP implementations will fill the provided packet. Others may replace it.
            // We handle both cases and keep our reusable packet alive for the next frame.
            MppPacket packet = out_packet_;
            ret              = mpi_->encode_get_packet(ctx_, &packet);
            if (ret) {
                if (ret == MPP_ERR_TIMEOUT || ret == MPP_ERR_BUFFER_FULL) { return {nullptr, 0}; }
                mpp_err("encode_get_packet failed ret %d\n", ret);
                return {nullptr, 0};
            }
            if (!packet) return {nullptr, 0};

            void *ptr  = mpp_packet_get_pos(packet);
            size_t len = mpp_packet_get_length(packet);
            if (!ptr || !len) {
                // If MPP returned a different packet object, it owns it.
                if (packet != out_packet_) mpp_packet_deinit(&packet);
                return {nullptr, 0};
            }

            // Ensure our copy buffer is large enough.
            if (len > buf_size_) {
                delete[] buf_;
                buf_size_ = static_cast<uint32_t>(len);
                buf_      = new char[buf_size_];
            }
            memcpy(buf_, ptr, len);

            // If MPP returned a different packet instance, release it here.
            // If it returned our reusable packet, keep it for next frame.
            if (packet != out_packet_) { mpp_packet_deinit(&packet); }

            // If the output did not fit our backing buffer, MPP may have returned a new packet
            // and/or signaled via length; grow our reusable buffer for subsequent frames.
            if (len > out_pkt_buf_size_) {
                // Rebuild reusable packet/buffer bigger than current output.
                mpp_log("Rebuild reusable packet/buffer bigger than current output.\n");
                if (out_packet_) {
                    mpp_packet_deinit(&out_packet_);
                    out_packet_ = nullptr;
                }
                if (out_pkt_buf_) {
                    mpp_buffer_put(out_pkt_buf_);
                    out_pkt_buf_ = nullptr;
                }
                // Round up to reduce realloc frequency.
                out_pkt_buf_size_ = MPP_ALIGN(len, 4096);
                MPP_RET r         = mpp_buffer_get(nullptr, &out_pkt_buf_, out_pkt_buf_size_);
                if (!r) {
                    r = mpp_packet_init_with_buffer(&out_packet_, out_pkt_buf_);
                    if (r) {
                        mpp_err("re-init out_packet failed ret %d\n", r);
                        mpp_buffer_put(out_pkt_buf_);
                        out_pkt_buf_      = nullptr;
                        out_pkt_buf_size_ = 0;
                        out_packet_       = nullptr;
                    }
                } else {
                    mpp_err("realloc out buffer failed ret %d\n", r);
                    out_pkt_buf_      = nullptr;
                    out_pkt_buf_size_ = 0;
                    out_packet_       = nullptr;
                }
            }

            return {buf_, static_cast<uint32_t>(len)};
        }

    private:
        int width_  = 0;
        int height_ = 0;
        MppFrameFormat input_fmt_;
        MppCodingType output_type_;
        int fps_ = 10;

        MppCtx ctx_    = nullptr;
        MppApi *mpi_   = nullptr;
        MppEncCfg cfg_ = nullptr;

        char *buf_         = nullptr;
        uint32_t buf_size_ = 0;

        // Reusable output packet/buffer to avoid per-frame allocations.
        MppPacket out_packet_    = nullptr;
        MppBuffer out_pkt_buf_   = nullptr;
        size_t out_pkt_buf_size_ = 1 * 1024 * 1024;

        std::vector<uint8_t> codec_header_;
    };
}// namespace controller_native