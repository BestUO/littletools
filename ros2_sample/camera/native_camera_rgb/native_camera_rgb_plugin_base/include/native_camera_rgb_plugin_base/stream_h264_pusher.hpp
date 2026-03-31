#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/opt.h>
#include <libavutil/time.h>
}

// NOTE:
// For RTMP/FLV, H264 must be sent as AVCC (length-prefixed NAL units) and the stream header must contain
// AVCDecoderConfigurationRecord (SPS/PPS) in codecpar->extradata. This file previously pushed Annex-B directly,
// which commonly triggers server errors like: "unable to parse H264 config: invalid size 1".
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

            extra.push_back(0x01);
            extra.push_back(sps[1]);
            extra.push_back(sps[2]);
            extra.push_back(sps[3]);
            extra.push_back(0xFC | 0x03);// lengthSizeMinusOne -> 4 bytes

            extra.push_back(0xE0 | 0x01);// numOfSequenceParameterSets = 1
            extra.push_back((uint8_t) ((sps.size() >> 8) & 0xff));
            extra.push_back((uint8_t) (sps.size() & 0xff));
            extra.insert(extra.end(), sps.begin(), sps.end());

            extra.push_back(0x01);// numOfPictureParameterSets = 1
            extra.push_back((uint8_t) ((pps.size() >> 8) & 0xff));
            extra.push_back((uint8_t) (pps.size() & 0xff));
            extra.insert(extra.end(), pps.begin(), pps.end());

            return extra;
        }
    }// namespace

    enum class StreamProtocol { RTSP, RTMP, RTP };

    class StreamH264Pusher {
    public:
        StreamH264Pusher()
            : format_ctx_(nullptr), video_stream_(nullptr), codec_ctx_(nullptr), is_running_(false), frame_count_(0) {}

        // Optional: provide codec header (SPS/PPS) in Annex-B format from encoder.
        // If your encoder does not prepend SPS/PPS to IDR frames, call this once after encoder init.
        void SetCodecHeaderAnnexB(const std::vector<uint8_t> &header) {
            codec_header_annexb_.clear();
            if (header.empty()) return;

            auto has_start_code = [&](const std::vector<uint8_t> &buf) -> bool {
                if (buf.size() < 4) return false;
                for (size_t i = 0; i + 3 < buf.size(); ++i) {
                    if (is_start_code3(&buf[i])) return true;
                    if (i + 4 <= buf.size() && is_start_code4(&buf[i])) return true;
                }
                return false;
            };

            // If it's already Annex-B, keep as-is.
            if (has_start_code(header)) {
                codec_header_annexb_ = header;
                fprintf(stdout, "already Annex-B\n");
                return;
            }

            // Otherwise best-effort: treat as AVCC (length-prefixed NALs) and convert to Annex-B.
            // Some encoders return SPS/PPS header in AVCC format.
            size_t off = 0;
            while (off + 4 <= header.size()) {
                uint32_t len = (uint32_t) header[off] << 24 | (uint32_t) header[off + 1] << 16 |
                               (uint32_t) header[off + 2] << 8 | (uint32_t) header[off + 3];
                off += 4;
                if (len == 0 || off + len > header.size()) break;
                // append 4-byte start code
                codec_header_annexb_.push_back(0x00);
                codec_header_annexb_.push_back(0x00);
                codec_header_annexb_.push_back(0x00);
                codec_header_annexb_.push_back(0x01);
                codec_header_annexb_.insert(codec_header_annexb_.end(), header.begin() + (long) off,
                                            header.begin() + (long) (off + len));
                off += len;
            }

            if (codec_header_annexb_.empty()) {
                // Keep it empty; PushFrame will fall back to parsing from frames.
                fprintf(stderr, "SetCodecHeaderAnnexB: header has no start code and AVCC parse failed (size=%zu)\n",
                        header.size());
            }
        }

        // 初始化推流
        bool Init(const std::string &url, int width, int height, int fps) {
            url_      = url;
            width_    = width;
            height_   = height;
            fps_      = fps;
            protocol_ = DetectProtocol(url);
            return true;
        }

        bool Start() {
            int ret;

            // 根据协议选择输出格式
            const char *format_name = nullptr;
            if (protocol_ == StreamProtocol::RTSP) {
                format_name = "rtsp";
            } else if (protocol_ == StreamProtocol::RTMP) {
                format_name = "flv";
            } else if (protocol_ == StreamProtocol::RTP) {
                format_name = "rtp";
            }

            // 分配输出格式上下文
            avformat_alloc_output_context2(&format_ctx_, nullptr, format_name, url_.c_str());
            if (!format_ctx_) {
                fprintf(stderr, "Could not create output context\n");
                return false;
            }

            // 创建视频流（仅复用 FFmpeg muxer；这里不做编码）
            video_stream_ = avformat_new_stream(format_ctx_, nullptr);
            if (!video_stream_) {
                fprintf(stderr, "Failed to create video stream\n");
                return false;
            }

            video_stream_->id = format_ctx_->nb_streams - 1;

            // 直接设置 codecpar（推送的是已编码 H264，不需要打开 encoder）
            video_stream_->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
            video_stream_->codecpar->codec_id   = AV_CODEC_ID_H264;
            video_stream_->codecpar->width      = width_;
            video_stream_->codecpar->height     = height_;
            // FLV/RTMP 要求 codec_tag = 0
            if (protocol_ == StreamProtocol::RTMP) { video_stream_->codecpar->codec_tag = 0; }

            // 设置流的时间基
            video_stream_->time_base = AVRational{1, fps_};

            // 打印输出格式信息
            av_dump_format(format_ctx_, 0, url_.c_str(), 1);

            // 设置协议选项
            AVDictionary *opts = nullptr;
            if (protocol_ == StreamProtocol::RTSP) {
                av_dict_set(&opts, "rtsp_transport", "tcp", 0);
                av_dict_set(&opts, "muxdelay", "0.1", 0);
            } else if (protocol_ == StreamProtocol::RTMP) {
                av_dict_set(&opts, "rtmp_live", "live", 0);
                av_dict_set(&opts, "rtmp_buffer", "1000", 0);
            } else if (protocol_ == StreamProtocol::RTP) {
                av_dict_set(&opts, "rtpflags", "latm", 0);
            }

            // 打开输出URL（某些格式不需要，如RTSP自己管理连接）
            if (!(format_ctx_->oformat->flags & AVFMT_NOFILE)) {
                ret = avio_open2(&format_ctx_->pb, url_.c_str(), AVIO_FLAG_WRITE, nullptr, &opts);
                if (ret < 0) {
                    char errbuf[AV_ERROR_MAX_STRING_SIZE];
                    av_strerror(ret, errbuf, sizeof(errbuf));
                    fprintf(stderr, "Could not open output URL '%s': %s\n", url_.c_str(), errbuf);
                    av_dict_free(&opts);
                    return false;
                }
            }
            av_dict_free(&opts);

            // RTMP/FLV 的 H264 需要 SPS/PPS (AVCDecoderConfigurationRecord) 写进 header。
            // 这里先不写 header，等 pushFrame() 从码流中提取到 SPS/PPS 后再写。
            if (protocol_ != StreamProtocol::RTMP) {
                ret = avformat_write_header(format_ctx_, nullptr);
                if (ret < 0) {
                    char errbuf[AV_ERROR_MAX_STRING_SIZE];
                    av_strerror(ret, errbuf, sizeof(errbuf));
                    fprintf(stderr, "Error occurred when writing header: %s\n", errbuf);
                    return false;
                }
                header_written_ = true;
            }

            is_running_ = true;
            start_time_ = av_gettime();

            printf("%s pusher initialized successfully: %s\n", protocol_ == StreamProtocol::RTSP ? "RTSP" : "RTMP",
                   url_.c_str());
            return true;
        }

        // 推送H264裸流数据
        bool PushFrame(const uint8_t *h264_data, size_t data_size, bool is_keyframe = false) {
            if (!is_running_ || !format_ctx_) {
                fprintf(stderr, "Pusher not initialized or not running\n");
                return false;
            }

            if (protocol_ == StreamProtocol::RTMP && !header_written_) {
                // Prefer encoder-provided header (Annex-B) if available.
                const uint8_t *src = h264_data;
                size_t src_size    = data_size;
                if (!codec_header_annexb_.empty()) {
                    src      = codec_header_annexb_.data();
                    src_size = codec_header_annexb_.size();
                }

                // 从 Annex-B 里解析 SPS/PPS（如果拿不到，说明输入不是 Annex-B 或编码器没输出 header）
                std::vector<uint8_t> sps;
                std::vector<uint8_t> pps;
                auto nals = split_annexb_nals(src, src_size);
                for (const auto &nal : nals) {
                    if (nal.empty()) continue;
                    const uint8_t nal_type = nal[0] & 0x1f;
                    if (nal_type == 7 && sps.empty()) sps = nal;
                    if (nal_type == 8 && pps.empty()) pps = nal;
                }

                // If still empty, try detect AVCC-style data (no start codes). This is a best-effort for some encoders.
                if (sps.empty() || pps.empty()) {
                    auto try_parse_avcc = [&](const uint8_t *p, size_t n) {
                        size_t off = 0;
                        while (off + 4 <= n) {
                            uint32_t len = (uint32_t) p[off] << 24 | (uint32_t) p[off + 1] << 16 |
                                           (uint32_t) p[off + 2] << 8 | (uint32_t) p[off + 3];
                            off += 4;
                            if (len == 0 || off + len > n) break;
                            const uint8_t nal_type = p[off] & 0x1f;
                            if (nal_type == 7 && sps.empty()) sps.assign(p + off, p + off + len);
                            if (nal_type == 8 && pps.empty()) pps.assign(p + off, p + off + len);
                            off += len;
                            if (!sps.empty() && !pps.empty()) break;
                        }
                    };
                    if (!codec_header_annexb_.empty()) {
                        // header is Annex-B, so AVCC likely not needed here
                    } else {
                        try_parse_avcc(h264_data, data_size);
                    }
                }

                if (!sps.empty() && !pps.empty()) {
                    auto extra = build_avcc_extradata_from_sps_pps(sps, pps);
                    if (!extra.empty()) {
                        av_freep(&video_stream_->codecpar->extradata);
                        video_stream_->codecpar->extradata_size = 0;
                        video_stream_->codecpar->extradata =
                                (uint8_t *) av_mallocz(extra.size() + AV_INPUT_BUFFER_PADDING_SIZE);
                        memcpy(video_stream_->codecpar->extradata, extra.data(), extra.size());
                        video_stream_->codecpar->extradata_size = (int) extra.size();

                        int ret = avformat_write_header(format_ctx_, nullptr);
                        if (ret < 0) {
                            char errbuf[AV_ERROR_MAX_STRING_SIZE];
                            av_strerror(ret, errbuf, sizeof(errbuf));
                            fprintf(stderr, "Error occurred when writing RTMP header (after SPS/PPS): %s\n", errbuf);
                            return false;
                        }
                        header_written_ = true;
                        fprintf(stderr, "RTMP header written OK. extradata=%d bytes (SPS=%zu PPS=%zu)\n",
                                video_stream_->codecpar->extradata_size, sps.size(), pps.size());
                    }
                }

                // 还没拿到 SPS/PPS：先不推任何帧，避免服务端把垃圾当 config。
                if (!header_written_) {
                    static int dropped = 0;
                    if ((dropped++ % 50) == 0) {
                        fprintf(stderr, "RTMP waiting SPS/PPS... drop frame (size=%zu key=%d hdr=%zu)\n", data_size,
                                is_keyframe ? 1 : 0, codec_header_annexb_.size());
                    }
                    return true;
                }
            }

            AVPacket *pkt = av_packet_alloc();
            if (!pkt) {
                fprintf(stderr, "Could not allocate packet\n");
                return false;
            }

            // RTMP/FLV 需要 AVCC：每个 NAL 前缀 4 字节长度；不能直接推 Annex-B 起始码。
            std::vector<uint8_t> payload;
            if (protocol_ == StreamProtocol::RTMP) {
                payload = annexb_to_avcc_sample(h264_data, data_size);
            } else {
                payload.assign(h264_data, h264_data + data_size);
            }

            /////1
            // pkt->data = (uint8_t *) av_malloc(payload.size());
            // if (!pkt->data) {
            //     av_packet_free(&pkt);
            //     return false;
            // }
            // memcpy(pkt->data, payload.data(), payload.size());

            /////2_OK
            // uint8_t *data_copy = (uint8_t *) av_malloc(payload.size());
            // if (!data_copy) {
            //     av_packet_free(&pkt);
            //     return false;
            // }
            // memcpy(data_copy, payload.data(), payload.size());
            //// 使用 av_packet_from_data，FFmpeg 会自动管理这块内存
            // int ret_data = av_packet_from_data(pkt, data_copy, payload.size());
            // if (ret_data < 0) {
            //     av_free(data_copy);
            //     av_packet_free(&pkt);
            //     return false;
            // }

            /////3
            if (auto ret = av_new_packet(pkt, payload.size()); ret < 0) {
                av_packet_free(&pkt);
                char errbuf[AV_ERROR_MAX_STRING_SIZE];
                av_strerror(ret, errbuf, sizeof(errbuf));
                fprintf(stderr, "Could not allocate packet data: %s\n", errbuf);
                return false;
            }
            memcpy(pkt->data, payload.data(), payload.size());

            pkt->size         = (int) payload.size();
            pkt->stream_index = video_stream_->index;

            // 设置关键帧标志
            if (is_keyframe) { pkt->flags |= AV_PKT_FLAG_KEY; }

            // 计算PTS和DTS
            int64_t current_time = av_gettime() - start_time_;
            int64_t pts          = av_rescale_q(current_time, AVRational{1, AV_TIME_BASE}, video_stream_->time_base);

            pkt->pts      = pts;
            pkt->dts      = pts;
            pkt->duration = av_rescale_q(1, AVRational{1, fps_}, video_stream_->time_base);

            // 写入数据包
            int ret = av_interleaved_write_frame(format_ctx_, pkt);

            av_packet_free(&pkt);

            if (ret < 0) {
                char errbuf[AV_ERROR_MAX_STRING_SIZE];
                av_strerror(ret, errbuf, sizeof(errbuf));
                fprintf(stderr, "Error writing frame: %s\n", errbuf);
                return false;
            }
            if (frame_count_ % 100 == 0) {
                fprintf(stdout, "Pushed frame %ld, pts=%" PRId64 "\n", frame_count_.load(), pts);
            }
            frame_count_++;
            return true;
        }

        // 推送帧（带自定义PTS）
        bool PushFrameWithPTS(const uint8_t *h264_data, size_t data_size, int64_t pts, bool is_keyframe = false) {
            if (!is_running_ || !format_ctx_) {
                fprintf(stderr, "Pusher not initialized or not running\n");
                return false;
            }

            AVPacket *pkt = av_packet_alloc();
            if (!pkt) {
                fprintf(stderr, "Could not allocate packet\n");
                return false;
            }

            pkt->data = (uint8_t *) av_malloc(data_size);
            if (!pkt->data) {
                av_packet_free(&pkt);
                return false;
            }

            memcpy(pkt->data, h264_data, data_size);
            pkt->size         = data_size;
            pkt->stream_index = video_stream_->index;

            if (is_keyframe) { pkt->flags |= AV_PKT_FLAG_KEY; }

            pkt->pts      = pts;
            pkt->dts      = pts;
            pkt->duration = av_rescale_q(1, AVRational{1, fps_}, video_stream_->time_base);

            int ret = av_interleaved_write_frame(format_ctx_, pkt);
            av_packet_free(&pkt);

            if (ret < 0) {
                char errbuf[AV_ERROR_MAX_STRING_SIZE];
                av_strerror(ret, errbuf, sizeof(errbuf));
                fprintf(stderr, "Error writing frame: %s\n", errbuf);
                return false;
            }

            frame_count_++;
            return true;
        }

        // 停止推流
        void Stop() {
            if (!is_running_) { return; }

            is_running_ = false;

            if (format_ctx_) {
                // 写入流尾
                av_write_trailer(format_ctx_);

                // 关闭输出（只有需要文件I/O的格式才关闭）
                if (format_ctx_->pb && !(format_ctx_->oformat->flags & AVFMT_NOFILE)) { avio_closep(&format_ctx_->pb); }

                // 释放编码器上下文（当前实现未使用，但保留兼容）
                if (codec_ctx_) { avcodec_free_context(&codec_ctx_); }

                // 释放格式上下文（会自动释放 video_stream_）
                avformat_free_context(format_ctx_);
                format_ctx_   = nullptr;
                video_stream_ = nullptr;// 指针置空，避免野指针
            }

            // Reset state so that Start() + PushFrame() can be called again safely.
            header_written_ = false;
            start_time_     = 0;
            frame_count_    = 0;

            printf("%s pusher stopped. Total frames: %ld\n", protocol_ == StreamProtocol::RTSP ? "RTSP" : "RTMP",
                   frame_count_.load());
        }

        // 获取推送帧数
        int64_t GetFrameCount() const { return frame_count_.load(); }

        // 检查是否正在运行
        bool IsRunning() const { return is_running_.load(); }

        // 获取协议类型
        StreamProtocol GetProtocol() const { return protocol_; }

        // 获取URL
        std::string GetURL() const { return url_; }

    private:
        // 检测URL的协议类型
        static StreamProtocol DetectProtocol(const std::string &url) {
            if (url.find("rtmp://") == 0 || url.find("rtmps://") == 0) {
                return StreamProtocol::RTMP;
            } else if (url.find("rtsp://") == 0) {
                return StreamProtocol::RTSP;
            } else if (url.find("rtp://") == 0 || url.find("udp://") == 0) {
                return StreamProtocol::RTP;
            }
            // 默认RTSP
            return StreamProtocol::RTSP;
        }

        std::string url_;
        int width_;
        int height_;
        int fps_;

        AVFormatContext *format_ctx_;
        AVStream *video_stream_;
        AVCodecContext *codec_ctx_;

        std::atomic<bool> is_running_;
        std::atomic<int64_t> frame_count_;
        int64_t start_time_;

        StreamProtocol protocol_;

        bool header_written_ = false;
        std::vector<uint8_t> codec_header_annexb_;
    };
}// namespace controller_native