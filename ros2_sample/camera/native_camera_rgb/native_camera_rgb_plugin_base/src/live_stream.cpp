#include "native_camera_rgb_plugin_base/live_stream.h"
#include "yjhlog/logger.h"
#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <libavutil/error.h>
#include <libavutil/rational.h>
#include <mutex>
#include <stdint.h>
#include <string>
#include <vector>

namespace controller_native {

    namespace {
        static inline bool is_start_code3(const uint8_t *p) { return p[0] == 0x00 && p[1] == 0x00 && p[2] == 0x01; }
        static inline bool is_start_code4(const uint8_t *p) {
            return p[0] == 0x00 && p[1] == 0x00 && p[2] == 0x00 && p[3] == 0x01;
        }

        static inline std::vector<std::vector<uint8_t>> split_annexb_nals(const uint8_t *data, size_t size) {
            std::vector<std::vector<uint8_t>> nals;
            if (!data || size < 4) return nals;

            auto find_start = [&](size_t from) -> size_t {
                for (size_t j = from; j + 3 < size; ++j) {
                    if (j + 4 <= size && is_start_code4(&data[j])) return j;
                    if (is_start_code3(&data[j])) return j;
                }
                return size;
            };

            size_t start = find_start(0);
            while (start < size) {
                const size_t sc_len  = (start + 4 <= size && is_start_code4(&data[start])) ? 4 : 3;
                const size_t nal_beg = start + sc_len;
                const size_t next    = find_start(nal_beg);
                size_t nal_end       = (next < size) ? next : size;
                while (nal_end > nal_beg && data[nal_end - 1] == 0x00) nal_end--;
                if (nal_end > nal_beg) nals.emplace_back(data + nal_beg, data + nal_end);
                start = next;
            }
            return nals;
        }

        static inline std::vector<uint8_t> annexb_to_avcc_sample(const uint8_t *data, size_t size) {
            std::vector<uint8_t> out;
            auto nals = split_annexb_nals(data, size);
            for (const auto &nal : nals) {
                const uint32_t len = (uint32_t) nal.size();
                out.push_back((len >> 24) & 0xff);
                out.push_back((len >> 16) & 0xff);
                out.push_back((len >> 8) & 0xff);
                out.push_back((len) & 0xff);
                out.insert(out.end(), nal.begin(), nal.end());
            }
            return out;
        }

        static inline std::vector<uint8_t> build_avcc_extradata_from_sps_pps(const std::vector<uint8_t> &sps,
                                                                             const std::vector<uint8_t> &pps) {
            std::vector<uint8_t> extra;
            if (sps.size() < 4 || pps.empty()) return extra;

            // AVCDecoderConfigurationRecord
            extra.push_back(0x01);
            extra.push_back(sps[1]);
            extra.push_back(sps[2]);
            extra.push_back(sps[3]);
            extra.push_back(0xFC | 0x03);// lengthSizeMinusOne => 4 bytes
            extra.push_back(0xE0 | 0x01);// numOfSPS
            extra.push_back((sps.size() >> 8) & 0xff);
            extra.push_back((sps.size()) & 0xff);
            extra.insert(extra.end(), sps.begin(), sps.end());
            extra.push_back(0x01);// numOfPPS
            extra.push_back((pps.size() >> 8) & 0xff);
            extra.push_back((pps.size()) & 0xff);
            extra.insert(extra.end(), pps.begin(), pps.end());
            return extra;
        }
    }// namespace

    bool LiveStreamClass::Init(const StreamConfig &config) {
        auto tmp_dst_format = GetTypeString(false, config.url);
        std::lock_guard<std::mutex> lock(mutex_);
        if (is_working_) { return false; }
        // 初始化网络
        avformat_network_init();

        // 分配输出上下文
        if (avformat_alloc_output_context2(&av_format_context_, nullptr, tmp_dst_format.c_str(), config.url.c_str()) <
            0) {
            return false;
        }

        if (tmp_dst_format == "rtsp") {
            av_opt_set(av_format_context_->priv_data, "rtsp_transport", "tcp", 0);
            // av_opt_set(av_format_context_->priv_data, "stimeout", "10000000", 0);  // 10秒超时
            // av_opt_set(av_format_context_->priv_data, "udp_timeout", "5000000", 0);// UDP超时5秒
            // av_opt_set(av_format_context_->priv_data, "reuse", "1", 0);            // 端口复用
            av_opt_set(av_format_context_->priv_data, "buffer_size", "2048000", 0);// 增大缓冲区
            // av_opt_set(av_format_context_->priv_data, "pkt_size", "1200", 0);
            av_opt_set(av_format_context_->priv_data, "max_delay", "500000", 0);// 0.5秒
        }

        is_flv_         = (tmp_dst_format == "flv");
        header_written_ = false;
        sps_.clear();
        pps_.clear();

        encoder_.Init(logger_, {AV_CODEC_ID_H264, config.bitrate, config.fps, config.width, config.height});
        config_     = config;
        pts_cnt_    = 0;
        is_working_ = true;
        return is_working_;
    }

    void LiveStreamClass::Write(AVFrame *frame) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (is_working_) {
            auto pack = encoder_.Encode(frame);
            if (!pack) { return; }
            if (!av_stream_) { InitStream(encoder_.GetAVCodec(), encoder_.GetAVCodecContext()); };

            // For RTMP/FLV: ensure header has SPS/PPS extradata and convert payload to AVCC.
            AVPacket *out_pkt = pack;
            AVPacket *tmp_pkt = nullptr;
            if (is_flv_) {
                // Parse SPS/PPS from Annex-B (common for libx264 output) and write header only after we have them.
                if (!header_written_) {
                    auto nals = split_annexb_nals(pack->data, (size_t) pack->size);
                    for (const auto &nal : nals) {
                        if (nal.empty()) continue;
                        const uint8_t t = nal[0] & 0x1f;
                        if (t == 7 && sps_.empty()) sps_ = nal;
                        if (t == 8 && pps_.empty()) pps_ = nal;
                        if (!sps_.empty() && !pps_.empty()) break;
                    }

                    if (!sps_.empty() && !pps_.empty()) {
                        auto extra = build_avcc_extradata_from_sps_pps(sps_, pps_);
                        if (!extra.empty()) {
                            av_freep(&av_stream_->codecpar->extradata);
                            av_stream_->codecpar->extradata_size = 0;
                            av_stream_->codecpar->extradata =
                                    (uint8_t *) av_mallocz(extra.size() + AV_INPUT_BUFFER_PADDING_SIZE);
                            if (av_stream_->codecpar->extradata) {
                                std::memcpy(av_stream_->codecpar->extradata, extra.data(), extra.size());
                                av_stream_->codecpar->extradata_size = (int) extra.size();

                                // Write header now (was delayed in InitStream for flv).
                                if (avformat_write_header(av_format_context_, nullptr) == 0) {
                                    header_written_ = true;
                                } else {
                                    CloseUnsafe();
                                    return;
                                }
                            }
                        }
                    }

                    // Still no header: drop frames to avoid sending garbage config.
                    if (!header_written_) {
                        av_packet_unref(pack);
                        return;
                    }
                }

                // Convert Annex-B to AVCC sample payload.
                auto payload = annexb_to_avcc_sample(pack->data, (size_t) pack->size);
                if (payload.empty()) {
                    av_packet_unref(pack);
                    return;
                }
                tmp_pkt = av_packet_alloc();
                if (!tmp_pkt) {
                    av_packet_unref(pack);
                    return;
                }
                if (av_new_packet(tmp_pkt, (int) payload.size()) < 0) {
                    av_packet_free(&tmp_pkt);
                    av_packet_unref(pack);
                    return;
                }
                std::memcpy(tmp_pkt->data, payload.data(), payload.size());
                tmp_pkt->size  = (int) payload.size();
                tmp_pkt->pts   = pack->pts;
                tmp_pkt->dts   = pack->dts;
                tmp_pkt->flags = pack->flags;
                out_pkt        = tmp_pkt;
            }

            av_packet_rescale_ts(pack, encoder_.GetAVCodecContext()->time_base, av_stream_->time_base);
            // Use out_pkt for writing (pack for timestamp rescale anchor)
            out_pkt->pts          = pack->pts;
            out_pkt->dts          = pack->dts;
            out_pkt->stream_index = av_stream_->index;

            // For flv, timestamps are still valid after assigning above.

            // 写入包到输出
            auto ret = av_interleaved_write_frame(av_format_context_, out_pkt);
            if (ret < 0) {
                char errbuf[AV_ERROR_MAX_STRING_SIZE] = {0};
                av_strerror(ret, errbuf, sizeof(errbuf));
                LOGGER_INFO(logger_, "Error writing packet: {} ({})", ret, errbuf);

                // 常见：对端断开/重置连接，继续写只会一直报错。
                if (ret == AVERROR(EPIPE) || ret == AVERROR(ECONNRESET)) { CloseUnsafe(); }
            }
            if (tmp_pkt) { av_packet_free(&tmp_pkt); }
            av_packet_unref(pack);

            // // 写入包到输出
            // ret = av_interleaved_write_frame(av_format_context_, packet_);
            // if (ret < 0) {
            //     av_log(nullptr, AV_LOG_ERROR, "Error writing packet\n");
            //     av_packet_unref(packet_);
            //     return;
            // }
            // // 发送帧给编码器
            // if (avcodec_send_frame(av_codec_context_, frame) < 0) {
            //     LOGGER_ERROR(logger_, "EnCoder::Encode: Failed to send frame to encoder.");
            //     return;
            // }

            // // 接收编码后的包
            // int ret = avcodec_receive_packet(av_codec_context_, packet_);
            // if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            //     LOGGER_INFO(logger_, "EnCoder::Encode: Encoder needs more frames or has reached end of stream.");
            //     return;// 需要更多帧
            // } else if (ret < 0) {
            //     char error_buf[AV_ERROR_MAX_STRING_SIZE];
            //     av_strerror(ret, error_buf, AV_ERROR_MAX_STRING_SIZE);
            //     LOGGER_ERROR(logger_, "EnCoder::Encode: Error receiving packet from encoder: {}", error_buf);
            //     return;
            // }

            // av_packet_rescale_ts(packet_, av_codec_context_->time_base, av_stream_->time_base);
            // packet_->stream_index = av_stream_->index;

            // // 写入包到输出
            // ret = av_interleaved_write_frame(av_format_context_, packet_);
            // if (ret < 0) {
            //     av_log(nullptr, AV_LOG_ERROR, "Error writing packet\n");
            //     av_packet_unref(packet_);
            //     return;
            // }
            // av_packet_unref(packet_);
        }
    }

    void LiveStreamClass::Write(const void *buf, uint32_t len) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (is_working_) {
            if (!av_stream_) {
                // 该重载直接写 H264 码流时，上层必须确保 InitStream() 已经完成。
                // 这里保护一下，避免空指针导致的未定义行为。
                (void) InitStream(encoder_.GetAVCodec(), encoder_.GetAVCodecContext());
                if (!av_stream_) { return; }
            }

            AVPacket *pkt = av_packet_alloc();
            if (!pkt) { return; }

            // 不能直接把外部 buf 赋给 pkt->data：av_packet_free()/unref 会尝试释放它。
            // 这里分配并拷贝一份由 FFmpeg 管理的内存。
            if (av_new_packet(pkt, static_cast<int>(len)) < 0) {
                av_packet_free(&pkt);
                return;
            }
            std::memcpy(pkt->data, buf, len);
            pkt->size = static_cast<int>(len);
            // pkt->pts = (pts_cnt_ += 9000);// pts_increment = (1 / 帧率) / (time_base.num / time_base.den)
            // av_packet_rescale_ts(pkt, encoder_ctx_->time_base, video_stream_->time_base);
            pkt->pts = av_rescale_q(pts_cnt_++, encoder_.GetAVCodecContext()->time_base, av_stream_->time_base);
            pkt->dts = pkt->pts;
            pkt->stream_index = av_stream_->index;

            const int ret = av_interleaved_write_frame(av_format_context_, pkt);
            if (ret < 0) {
                char errbuf[AV_ERROR_MAX_STRING_SIZE] = {0};
                av_strerror(ret, errbuf, sizeof(errbuf));
                LOGGER_INFO(logger_, "Error writing packet: {} ({})", ret, errbuf);
                if (ret == AVERROR(EPIPE) || ret == AVERROR(ECONNRESET)) { CloseUnsafe(); }
            }

            av_packet_free(&pkt);
        }
    }

    void LiveStreamClass::Close() {
        std::lock_guard<std::mutex> lock(mutex_);
        CloseUnsafe();
    }

    bool LiveStreamClass::InitStream(const AVCodec *av_codec, AVCodecContext *ac_codec_context) {
        av_stream_ = avformat_new_stream(av_format_context_, av_codec);
        if (av_stream_ == nullptr) {
            CloseUnsafe();
            LOGGER_ERROR(logger_, "LiveStreamClass: Failed to create new video stream.");
            return false;
        }
        av_stream_->codecpar->codec_tag = 0;
        if (avcodec_parameters_from_context(av_stream_->codecpar, ac_codec_context) < 0) {
            CloseUnsafe();
            LOGGER_ERROR(logger_, "LiveStreamClass: Failed to set codec parameters from context.");
            return false;
        }

        // av_dump_format(av_format_context_, 0, config_.url.c_str(), true);
        if (!(av_format_context_->oformat->flags & AVFMT_NOFILE)) {
            if (int ret = avio_open(&av_format_context_->pb, config_.url.c_str(), AVIO_FLAG_WRITE); ret < 0) {
                char errbuf[AV_ERROR_MAX_STRING_SIZE] = {0};
                av_strerror(ret, errbuf, sizeof(errbuf));
                CloseUnsafe();
                LOGGER_ERROR(logger_, "LiveStreamClass: Failed to open output URL: {} {}", config_.url, errbuf);
                return false;
            }
        }

        // For RTMP/FLV, delay header writing until we have SPS/PPS and can fill codecpar->extradata.
        // Otherwise many servers will reject the stream with errors like:
        //   "unable to parse H264 config: invalid size 1"
        if (!is_flv_) {
            if (avformat_write_header(av_format_context_, nullptr) < 0) {
                CloseUnsafe();
                LOGGER_ERROR(logger_, "LiveStreamClass: Failed to write header to output URL: {}", config_.url);
                return false;
            }
            header_written_ = true;
        }
        return true;
    }

    std::string LiveStreamClass::GetTypeString(bool input_format, std::string type) {
        std::transform(type.begin(), type.end(), type.begin(), [](unsigned char c) { return std::toupper(c); });
        if (input_format) {
            std::string dst_type = "H264";
            if (type == "H264") dst_type = "H264";
            return dst_type;
        } else {
            std::string dst_type = "mpegts";
            if (type.find("RTMP") != std::string::npos) dst_type = "flv";
            else if (type.find("RTP") != std::string::npos)
                dst_type = "rtp";
            else if (type.find("UDP") != std::string::npos)
                dst_type = "mpegts";
            else if (type.find("RTSP") != std::string::npos)
                dst_type = "rtsp";
            return dst_type;
        }
    }

    void LiveStreamClass::CloseUnsafe() {
        // if (av_codec_context_) {
        //     avcodec_free_context(&av_codec_context_);
        //     av_codec_context_ = nullptr;
        // }
        if (encoder_.GetAVCodecContext()) { encoder_.Cleanup(); }
        if (av_format_context_) {
            if (av_format_context_->pb) {
                av_write_trailer(av_format_context_);
                avio_close(av_format_context_->pb);
            };
            avformat_free_context(av_format_context_);
            av_format_context_ = nullptr;
        }
        // if (packet_) {
        //     av_packet_free(&packet_);
        //     packet_ = nullptr;
        // }

        av_stream_      = nullptr;
        is_working_     = false;
        pts_cnt_        = 0;
        header_written_ = false;
        is_flv_         = false;
        sps_.clear();
        pps_.clear();
    }
}// namespace controller_native