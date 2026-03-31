
// Header-only H264 (AnnexB) -> MP4 muxer.
// This file intentionally provides an independent, reusable component.

#pragma once

#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
}

namespace controller_native {
    namespace h264_to_mp4 {

        static inline bool IsStartCode3(const uint8_t *p) { return p[0] == 0x00 && p[1] == 0x00 && p[2] == 0x01; }

        static inline bool IsStartCode4(const uint8_t *p) {
            return p[0] == 0x00 && p[1] == 0x00 && p[2] == 0x00 && p[3] == 0x01;
        }

        // Split AnnexB stream into NAL payloads (without start codes).
        inline std::vector<std::vector<uint8_t>> SplitAnnexBNals(const uint8_t *data, size_t size) {
            std::vector<std::vector<uint8_t>> nals;
            if (!data || size < 4) return nals;

            auto find_start = [&](size_t from) -> size_t {
                for (size_t j = from; j + 3 < size; ++j) {
                    if (j + 4 <= size && IsStartCode4(&data[j])) return j;
                    if (IsStartCode3(&data[j])) return j;
                }
                return size;
            };

            size_t start = find_start(0);
            while (start < size) {
                const size_t sc_len  = (start + 4 <= size && IsStartCode4(&data[start])) ? 4 : 3;
                const size_t nal_beg = start + sc_len;
                const size_t next    = find_start(nal_beg);
                size_t nal_end       = (next < size) ? next : size;

                // Trim trailing zeros.
                while (nal_end > nal_beg && data[nal_end - 1] == 0x00) nal_end--;

                if (nal_end > nal_beg) nals.emplace_back(data + nal_beg, data + nal_end);
                start = next;
            }
            return nals;
        }

        // Convert AnnexB NALs into AVCC sample (each NAL prefixed with 4-byte big-endian length).
        inline std::vector<uint8_t> AnnexBToAvccSample(const uint8_t *data, size_t size) {
            std::vector<uint8_t> out;
            auto nals = SplitAnnexBNals(data, size);
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

        // Build AVCDecoderConfigurationRecord (ISO/IEC 14496-15) from AnnexB SPS/PPS.
        // Returns empty on failure.
        inline std::vector<uint8_t> BuildAvcExtradataFromAnnexBHeader(const std::vector<uint8_t> &annexb_hdr) {
            std::vector<uint8_t> extra;
            if (annexb_hdr.empty()) return extra;

            auto nals = SplitAnnexBNals(annexb_hdr.data(), annexb_hdr.size());
            std::vector<uint8_t> sps;
            std::vector<uint8_t> pps;
            for (const auto &nal : nals) {
                if (nal.empty()) continue;
                const uint8_t nal_type = nal[0] & 0x1f;
                if (nal_type == 7 && sps.empty()) sps = nal;// SPS
                if (nal_type == 8 && pps.empty()) pps = nal;// PPS
            }
            if (sps.size() < 4 || pps.empty()) return extra;

            // configurationVersion(1)
            // AVCProfileIndication(sps[1])
            // profile_compatibility(sps[2])
            // AVCLevelIndication(sps[3])
            // lengthSizeMinusOne(3 -> 4 bytes lengths)
            extra.push_back(0x01);
            extra.push_back(sps[1]);
            extra.push_back(sps[2]);
            extra.push_back(sps[3]);
            extra.push_back(0xFC | 0x03);

            // numOfSequenceParameterSets (1)
            extra.push_back(0xE0 | 0x01);
            extra.push_back((uint8_t) ((sps.size() >> 8) & 0xff));
            extra.push_back((uint8_t) (sps.size() & 0xff));
            extra.insert(extra.end(), sps.begin(), sps.end());

            // numOfPictureParameterSets (1)
            extra.push_back(0x01);
            extra.push_back((uint8_t) ((pps.size() >> 8) & 0xff));
            extra.push_back((uint8_t) (pps.size() & 0xff));
            extra.insert(extra.end(), pps.begin(), pps.end());

            return extra;
        }

        // Minimal H264(AnnexB) -> MP4 remuxer.
        // Contract:
        // - Input: H264 AnnexB NAL units (with 0x00000001 start codes).
        // - You must call Init() once with SPS/PPS header (also AnnexB).
        // - Then call WriteAnnexB() for each encoded access unit.
        class Mp4Muxer {
        public:
            // MP4/H264 commonly uses a 90kHz clock.
            static constexpr int kTimeBaseHz = 90000;

            // Enable raw AnnexB H264 dump into a file.
            // If enabled, Mp4Muxer will write SPS/PPS header once (if provided) and then append each WriteAnnexB()
            // payload.
            bool EnableH264Dump(const std::string &path, const std::vector<uint8_t> &annexb_hdr) {
                DisableH264Dump();
                if (path.empty()) return false;
                h264_path_ = path;
                h264_hdr_  = annexb_hdr;
                h264_out_.open(h264_path_, std::ios::out | std::ios::binary);
                h264_hdr_written_ = false;
                return h264_out_.is_open();
            }

            bool IsH264DumpEnabled() const { return h264_out_.is_open(); }

            void DisableH264Dump() {
                if (h264_out_.is_open()) h264_out_.close();
                h264_out_.clear();
                h264_hdr_.clear();
                h264_hdr_written_ = false;
            }

            bool Init(const std::string &mp4_path, int fps) {
                mp4_path_ = mp4_path;
                fps_      = fps;
                return true;
            }

            bool Init(int width, int height, const std::vector<uint8_t> &annexb_hdr) {
                Close();
                if (mp4_path_.empty() || width <= 0 || height <= 0 || fps_ <= 0) return false;
                width_  = width;
                height_ = height;

                hdr_ = annexb_hdr;

                int ret = avformat_alloc_output_context2(&ofmt_ctx_, nullptr, "mp4", mp4_path_.c_str());
                if (ret < 0 || !ofmt_ctx_) return false;

                stream_ = avformat_new_stream(ofmt_ctx_, nullptr);
                if (!stream_) return false;

                stream_->id             = static_cast<int>(ofmt_ctx_->nb_streams - 1);
                stream_->time_base      = AVRational{1, kTimeBaseHz};
                stream_->avg_frame_rate = AVRational{fps_, 1};

                AVCodecParameters *par = stream_->codecpar;
                par->codec_type        = AVMEDIA_TYPE_VIDEO;
                par->codec_id          = AV_CODEC_ID_H264;
                par->width             = width_;
                par->height            = height_;
                par->codec_tag         = 0;// let muxer choose

                // Convert AnnexB SPS/PPS to AVCDecoderConfigurationRecord (extradata) for MP4.
                if (!hdr_.empty()) {
                    auto extra = BuildAvcExtradataFromAnnexBHeader(hdr_);
                    if (!extra.empty()) {
                        static constexpr int kPadding = AV_INPUT_BUFFER_PADDING_SIZE;
                        par->extradata_size           = (int) extra.size();
                        par->extradata                = (uint8_t *) av_malloc(extra.size() + kPadding);
                        if (!par->extradata) return false;
                        memcpy(par->extradata, extra.data(), extra.size());
                        memset(par->extradata + extra.size(), 0, kPadding);
                    }
                }

                if (!(ofmt_ctx_->oformat->flags & AVFMT_NOFILE)) {
                    ret = avio_open(&ofmt_ctx_->pb, mp4_path_.c_str(), AVIO_FLAG_WRITE);
                    if (ret < 0) return false;
                }

                ret = avformat_write_header(ofmt_ctx_, nullptr);
                if (ret < 0) return false;

                opened_ = true;
                return true;
            }

            bool IsOpen() const { return opened_; }

            // Write one access unit of AnnexB H264.
            bool WriteAnnexB(const uint8_t *data, size_t size, bool keyframe_hint) {
                if (!data || size == 0) return false;

                // Optional raw dump.
                if (h264_out_.is_open()) {
                    if (!h264_hdr_written_) {
                        if (!h264_hdr_.empty()) {
                            h264_out_.write(reinterpret_cast<const char *>(h264_hdr_.data()), h264_hdr_.size());
                        }
                        h264_hdr_written_ = true;
                    }
                    h264_out_.write(reinterpret_cast<const char *>(data), (std::streamsize) size);
                    h264_out_.flush();
                }

                // Optional MP4 mux.
                if (!opened_ || !ofmt_ctx_ || !stream_) return true;

                // Convert AnnexB to AVCC sample (length-prefixed NAL units) for MP4.
                auto avcc = AnnexBToAvccSample(data, size);
                if (avcc.empty()) return false;

                AVPacket pkt{};
                pkt.data         = avcc.data();
                pkt.size         = (int) avcc.size();
                pkt.stream_index = stream_->index;

                // Generate monotonic timestamps in the stream timebase.
                // PTS/DTS unit is stream_->time_base (1/90000).
                const int64_t step = fps_ > 0 ? (kTimeBaseHz / fps_) : (kTimeBaseHz / 10);
                pkt.pts            = frame_index_ * step;
                pkt.dts            = pkt.pts;
                pkt.duration       = step;
                pkt.pos            = -1;

                if (keyframe_hint) pkt.flags |= AV_PKT_FLAG_KEY;

                frame_index_++;
                const int ret = av_interleaved_write_frame(ofmt_ctx_, &pkt);
                return ret >= 0;
            }

            std::string Close() {
                DisableH264Dump();
                if (!opened_) {
                    // Still free context if partially created.
                    if (ofmt_ctx_) {
                        if (!(ofmt_ctx_->oformat->flags & AVFMT_NOFILE) && ofmt_ctx_->pb) avio_closep(&ofmt_ctx_->pb);
                        avformat_free_context(ofmt_ctx_);
                        ofmt_ctx_ = nullptr;
                    }
                    stream_      = nullptr;
                    frame_index_ = 0;
                    hdr_.clear();
                    return "";
                }

                if (ofmt_ctx_) {
                    av_write_trailer(ofmt_ctx_);
                    if (!(ofmt_ctx_->oformat->flags & AVFMT_NOFILE) && ofmt_ctx_->pb) avio_closep(&ofmt_ctx_->pb);
                    avformat_free_context(ofmt_ctx_);
                    ofmt_ctx_ = nullptr;
                }
                stream_      = nullptr;
                opened_      = false;
                frame_index_ = 0;
                hdr_.clear();
                return mp4_path_;
            }

            ~Mp4Muxer() { Close(); }

        private:
            int width_  = 0;
            int height_ = 0;
            int fps_    = 0;
            std::string h264_path_;
            std::string mp4_path_;
            bool opened_ = false;

            AVFormatContext *ofmt_ctx_ = nullptr;
            AVStream *stream_          = nullptr;
            int64_t frame_index_       = 0;

            std::vector<uint8_t> hdr_;

            std::ofstream h264_out_;

            std::vector<uint8_t> h264_hdr_;
            bool h264_hdr_written_ = false;
        };

    }// namespace h264_to_mp4
}// namespace controller_native
