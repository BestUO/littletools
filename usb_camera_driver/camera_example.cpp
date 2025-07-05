#include <cstdio>
#include <iostream>
#include <string>
#include <stdexcept>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
#include <libavdevice/avdevice.h>
}

class USBCameraFFmpeg4 {
public:
    // 支持的像素格式
    enum class PixelFormat {
        YUYV422 = AV_PIX_FMT_YUYV422,
        RGB24   = AV_PIX_FMT_RGB24,
        BGR24   = AV_PIX_FMT_BGR24,
        NV12    = AV_PIX_FMT_NV12,
        YUV420P = AV_PIX_FMT_YUV420P,
        GRAY8   = AV_PIX_FMT_GRAY8
    };

    // 构造函数
    USBCameraFFmpeg4(const std::string& device = "/dev/video0", 
                    int width = 1280, 
                    int height = 720, 
                    int fps = 5,
                    PixelFormat format = PixelFormat::YUYV422)
        : device_name(device), width(width), height(height), fps(fps), 
          pixel_format(static_cast<AVPixelFormat>(format)), 
          format_ctx(nullptr), codec_ctx(nullptr),
          video_stream_idx(-1), sws_ctx(nullptr), frame(nullptr),
          is_opened(false) {
        
        avdevice_register_all(); // 注册所有设备
        printf("USBCameraFFmpeg4 initialized with device: %s, width: %d, height: %d, fps: %d, format: %s\n",
               device_name.c_str(), width, height, fps, get_pixel_format_string(pixel_format).c_str());
    }

    // 析构函数
    ~USBCameraFFmpeg4() {
        close();
    }

    // 打开摄像头
    bool open() {
        printf("Opening camera: %s\n", device_name.c_str());
        if (is_opened) {
            return true;
        }

        // 设置输入格式为v4l2 (Video for Linux 2)
        const AVInputFormat* input_format = av_find_input_format("v4l2");
        if (!input_format) {
            std::cerr << "Cannot find v4l2 input format" << std::endl;
            return false;
        }
        printf("Using input format: %s\n", input_format->name);
        // 打开设备
        AVDictionary* options = nullptr;
        av_dict_set(&options, "video_size", (std::to_string(width) + "x" + std::to_string(height)).c_str(), 0);
        av_dict_set(&options, "framerate", std::to_string(fps).c_str(), 0);
        av_dict_set(&options, "pixel_format", get_pixel_format_string(pixel_format).c_str(), 0);

        if (avformat_open_input(&format_ctx, device_name.c_str(), input_format, &options) != 0) {
            std::cerr << "Cannot open video device: " << device_name << std::endl;
            av_dict_free(&options);
            return false;
        }
        av_dict_free(&options);
        printf("Camera opened successfully\n");
        // 查找流信息
        if (avformat_find_stream_info(format_ctx, nullptr) < 0) {
            std::cerr << "Cannot find stream information" << std::endl;
            close();
            return false;
        }
        printf("Stream information found\n");
        // 查找视频流
        for (unsigned int i = 0; i < format_ctx->nb_streams; i++) {
            if (format_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
                video_stream_idx = i;
                break;
            }
            printf("查找视频流\n");
        }

        if (video_stream_idx == -1) {
            std::cerr << "Cannot find video stream" << std::endl;
            close();
            return false;
        }

        // 获取解码器
        AVCodecParameters* codec_params = format_ctx->streams[video_stream_idx]->codecpar;
        const AVCodec* codec = avcodec_find_decoder(codec_params->codec_id);
        if (!codec) {
            std::cerr << "Unsupported codec" << std::endl;
            close();
            return false;
        }

        // 创建解码器上下文
        codec_ctx = avcodec_alloc_context3(codec);
        if (!codec_ctx) {
            std::cerr << "Cannot allocate codec context" << std::endl;
            close();
            return false;
        }

        if (avcodec_parameters_to_context(codec_ctx, codec_params) < 0) {
            std::cerr << "Cannot copy codec parameters to context" << std::endl;
            close();
            return false;
        }

        // 打开解码器
        if (avcodec_open2(codec_ctx, codec, nullptr) < 0) {
            std::cerr << "Cannot open codec" << std::endl;
            close();
            return false;
        }

        // 分配帧
        frame = av_frame_alloc();
        if (!frame) {
            std::cerr << "Cannot allocate frame" << std::endl;
            close();
            return false;
        }

        is_opened = true;
        return true;
    }

    // 关闭摄像头
    void close() {
        if (sws_ctx) {
            sws_freeContext(sws_ctx);
            sws_ctx = nullptr;
        }

        if (frame) {
            av_frame_free(&frame);
            frame = nullptr;
        }

        if (codec_ctx) {
            avcodec_free_context(&codec_ctx);
            codec_ctx = nullptr;
        }

        if (format_ctx) {
            avformat_close_input(&format_ctx);
            format_ctx = nullptr;
        }

        video_stream_idx = -1;
        is_opened = false;
    }

    // 检查摄像头是否打开
    bool isOpen() const {
        return is_opened;
    }

    // 获取一帧数据
    bool grabFrame() {
        if (!is_opened) {
            return false;
        }

        AVPacket* packet = av_packet_alloc();
        if (!packet) {
            return false;
        }

        int ret;
        while ((ret = av_read_frame(format_ctx, packet)) >= 0) {
            if (packet->stream_index == video_stream_idx) {
                // 发送包到解码器
                if (avcodec_send_packet(codec_ctx, packet) < 0) {
                    av_packet_free(&packet);
                    return false;
                }

                // 接收帧
                ret = avcodec_receive_frame(codec_ctx, frame);
                if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                    av_packet_free(&packet);
                    continue;
                } else if (ret < 0) {
                    av_packet_free(&packet);
                    return false;
                }

                av_packet_free(&packet);
                return true;
            }
            av_packet_free(&packet);
        }

        av_packet_free(&packet);
        return false;
    }

    // 获取当前帧并转换为指定格式
    bool retrieveFrame(AVFrame* output_frame, PixelFormat desired_format = PixelFormat::BGR24) {
        if (!is_opened || !frame) {
            return false;
        }

        AVPixelFormat desired_pix_fmt = static_cast<AVPixelFormat>(desired_format);

        // 如果格式相同，直接复制
        if (frame->format == desired_pix_fmt) {
            av_frame_copy(output_frame, frame);
            return true;
        }

        // 否则进行转换
        if (!sws_ctx) {
            sws_ctx = sws_getContext(
                frame->width, frame->height, static_cast<AVPixelFormat>(frame->format),
                frame->width, frame->height, desired_pix_fmt,
                SWS_BILINEAR, nullptr, nullptr, nullptr);
            if (!sws_ctx) {
                return false;
            }
        }

        // 设置输出帧参数
        output_frame->width = frame->width;
        output_frame->height = frame->height;
        output_frame->format = desired_pix_fmt;
        
        if (av_frame_get_buffer(output_frame, 0) < 0) {
            return false;
        }

        // 执行转换
        sws_scale(sws_ctx, 
                 frame->data, frame->linesize, 0, frame->height,
                 output_frame->data, output_frame->linesize);

        return true;
    }

    // 获取当前帧的宽度
    int getWidth() const {
        return is_opened ? codec_ctx->width : -1;
    }

    // 获取当前帧的高度
    int getHeight() const {
        return is_opened ? codec_ctx->height : -1;
    }

    // 获取当前帧的像素格式
    PixelFormat getPixelFormat() const {
        return is_opened ? static_cast<PixelFormat>(codec_ctx->pix_fmt) : PixelFormat::YUYV422;
    }

private:
    // 将像素格式转换为字符串
    std::string get_pixel_format_string(AVPixelFormat format) {
        switch (format) {
            case AV_PIX_FMT_YUYV422: return "yuyv422";
            case AV_PIX_FMT_RGB24:   return "rgb24";
            case AV_PIX_FMT_BGR24:   return "bgr24";
            case AV_PIX_FMT_NV12:    return "nv12";
            case AV_PIX_FMT_YUV420P: return "yuv420p";
            case AV_PIX_FMT_GRAY8:   return "gray";
            default: return "";
        }
    }

    std::string device_name;
    int width;
    int height;
    int fps;
    AVPixelFormat pixel_format;
    
    AVFormatContext* format_ctx;
    AVCodecContext* codec_ctx;
    int video_stream_idx;
    
    struct SwsContext* sws_ctx;
    AVFrame* frame;
    
    bool is_opened;
};

int main() {
    USBCameraFFmpeg4 camera("/dev/video0", 1280, 720, 5, USBCameraFFmpeg4::PixelFormat::YUYV422);
    
    if (!camera.open()) {
        std::cerr << "Failed to open camera" << std::endl;
        return -1;
    }
    printf("Camera opened successfully\n");
    AVFrame* frame = av_frame_alloc();
    if (!frame) {
        std::cerr << "Failed to allocate frame" << std::endl;
        return -1;
    }

    int frame_count = 0;
    while (frame_count < 100) {  // 捕获100帧
        if (camera.grabFrame()) {
            if (camera.retrieveFrame(frame, USBCameraFFmpeg4::PixelFormat::BGR24)) {
                std::cout << "Got frame " << frame_count 
                          << " (" << frame->width << "x" << frame->height 
                          << ")" << std::endl;
                frame_count++;
                
                // 这里可以处理帧数据，例如保存到文件或显示
                printf("aaa\n");
            }
            else {
                std::cerr << "Failed to retrieve frame" << std::endl;
            }
        }
        else {
            std::cerr << "Failed to grab frame" << std::endl;
            break;
        }
    }

    av_frame_free(&frame);
    return 0;
}
