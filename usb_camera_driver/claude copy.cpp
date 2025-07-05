#ifndef USB_CAMERA_H
#define USB_CAMERA_H

#include <string>
#include <memory>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavdevice/avdevice.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
}

enum class CameraFormat {
    MJPEG,
    H264,
    YUV422
};

class USBCamera {
private:
    AVFormatContext* format_ctx;
    AVCodecContext* codec_ctx;
    const AVCodec* codec;
    AVFrame* frame;
    AVFrame* frame_rgb;
    AVPacket* packet;
    SwsContext* sws_ctx;
    const AVCodec* mjpeg_codec;
    
    int video_stream_idx;
    std::string device_path;
    int width;
    int height;
    CameraFormat format;
    
    bool initialized;
    
    // 私有方法
    bool initializeFFmpeg();
    bool openCamera();
    bool setupCodec();
    bool setupScaler();
    void cleanup();
    
public:
    USBCamera(const std::string& device = "/dev/video0");
    ~USBCamera();
    
    // 设置摄像头参数
    bool setResolution(int w, int h);
    bool setFormat(CameraFormat fmt);
    
    // 初始化摄像头
    bool initialize();
    
    // 捕获帧
    bool captureFrame();
    
    // 保存当前帧为JPG
    bool saveFrameAsJPG(const std::string& filename);
    
    // 获取当前帧数据
    uint8_t* getFrameData();
    int getFrameSize();
    
    // 获取摄像头信息
    int getWidth() const { return width; }
    int getHeight() const { return height; }
    CameraFormat getFormat() const { return format; }
    
    // 释放资源
    void release();
};

#endif // USB_CAMERA_H

// 实现文件 USBCamera.cpp
// #include "USBCamera.h"
#include <iostream>
#include <cstring>

USBCamera::USBCamera(const std::string& device)
    : format_ctx(nullptr), codec_ctx(nullptr), codec(nullptr),
      frame(nullptr), frame_rgb(nullptr), packet(nullptr), sws_ctx(nullptr),
      video_stream_idx(-1), device_path(device), width(640), height(480),
      format(CameraFormat::MJPEG), initialized(false) {
    
    // 初始化FFmpeg (FFmpeg 4.0+中不再需要av_register_all())
    avdevice_register_all();
    avformat_network_init();

    // 查找MJPEG编码器
    mjpeg_codec = avcodec_find_encoder(AV_CODEC_ID_MJPEG);
    if (!mjpeg_codec) {
        std::cerr << "警告：无法找到MJPEG编码器" << std::endl;
        
        // 尝试按名称查找
        mjpeg_codec = avcodec_find_encoder_by_name("mjpeg");
        if (!mjpeg_codec) {
            std::cerr << "警告：按名称也无法找到MJPEG编码器" << std::endl;
        }
    }
    
    if (mjpeg_codec) {
        std::cout << "MJPEG编码器找到: " << mjpeg_codec->name << std::endl;
    }

    // 调试：列出所有可用的编码器
    const AVCodec* codec = nullptr;
    void* iter = nullptr;
    std::cout << "可用的编码器：" << std::endl;
    while ((codec = av_codec_iterate(&iter))) {
        if (av_codec_is_encoder(codec)) {
            std::cout << codec->name << std::endl;
        }
    }
}

USBCamera::~USBCamera() {
    cleanup();
}

bool USBCamera::initializeFFmpeg() {
    // 分配AVFormatContext
    format_ctx = avformat_alloc_context();
    if (!format_ctx) {
        std::cerr << "无法分配AVFormatContext" << std::endl;
        return false;
    }
    
    // 查找输入格式
    const AVInputFormat* input_format = av_find_input_format("v4l2");
    if (!input_format) {
        std::cerr << "无法找到v4l2输入格式" << std::endl;
        return false;
    }
    
    // 设置输入选项
    AVDictionary* options = nullptr;
    
    // 根据格式设置参数
    std::string pixel_format;
    switch (format) {
        case CameraFormat::MJPEG:
            pixel_format = "mjpeg";
            break;
        case CameraFormat::H264:
            pixel_format = "h264";
            break;
        case CameraFormat::YUV422:
            pixel_format = "yuyv422";
            break;
    }

    printf("使用输入格式: %s video_size %d x %d, pixel_format: %s\n",
           input_format->name, width, height, pixel_format.c_str());
    
    av_dict_set(&options, "pixel_format", pixel_format.c_str(), 0);
    av_dict_set(&options, "video_size", (std::to_string(width) + "x" + std::to_string(height)).c_str(), 0);
    av_dict_set(&options, "framerate", "5", 0);
    
    // 打开输入设备
    if (avformat_open_input(&format_ctx, device_path.c_str(), input_format, &options) != 0) {
        std::cerr << "无法打开摄像头设备: " << device_path << std::endl;
        av_dict_free(&options);
        return false;
    }
    
    av_dict_free(&options);
    
    printf("查找流信息\n");
    // 查找流信息
    if (avformat_find_stream_info(format_ctx, nullptr) < 0) {
        std::cerr << "无法获取流信息" << std::endl;
        return false;
    }
    printf("查找流信息 结束\n");
    return true;
}

bool USBCamera::openCamera() {
    // 查找视频流
    video_stream_idx = -1;
    for (unsigned int i = 0; i < format_ctx->nb_streams; i++) {
        if (format_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            video_stream_idx = i;
            break;
        }
    }
    
    if (video_stream_idx == -1) {
        std::cerr << "无法找到视频流" << std::endl;
        return false;
    }
    
    return true;
}

bool USBCamera::setupCodec() {
    AVCodecParameters* codecpar = format_ctx->streams[video_stream_idx]->codecpar;

    // 查找解码器
    codec = avcodec_find_decoder(codecpar->codec_id);
    if (!codec) {
        std::cerr << "无法找到解码器" << std::endl;
        return false;
    }
    
    // 分配解码器上下文
    codec_ctx = avcodec_alloc_context3(codec);
    if (!codec_ctx) {
        std::cerr << "无法分配解码器上下文" << std::endl;
        return false;
    }
    
    // 复制参数到解码器上下文
    if (avcodec_parameters_to_context(codec_ctx, codecpar) < 0) {
        std::cerr << "无法复制解码器参数" << std::endl;
        return false;
    }
    
    // 打开解码器
    if (avcodec_open2(codec_ctx, codec, nullptr) < 0) {
        std::cerr << "无法打开解码器" << std::endl;
        return false;
    }
    
    // 更新实际的宽高
    width = codec_ctx->width;
    height = codec_ctx->height;
    
    return true;
}

bool USBCamera::setupScaler() {
    // 分配帧
    frame = av_frame_alloc();
    frame_rgb = av_frame_alloc();
    packet = av_packet_alloc();
    
    if (!frame || !frame_rgb || !packet) {
        std::cerr << "无法分配帧或数据包" << std::endl;
        return false;
    }
    
    // 为RGB帧分配缓冲区
    int num_bytes = av_image_get_buffer_size(AV_PIX_FMT_RGB24, width, height, 1);
    uint8_t* buffer = (uint8_t*)av_malloc(num_bytes * sizeof(uint8_t));
    av_image_fill_arrays(frame_rgb->data, frame_rgb->linesize, buffer, AV_PIX_FMT_RGB24, width, height, 1);
    
    // 创建缩放上下文
    sws_ctx = sws_getContext(width, height, codec_ctx->pix_fmt,
                            width, height, AV_PIX_FMT_RGB24,
                            SWS_BILINEAR, nullptr, nullptr, nullptr);
    
    if (!sws_ctx) {
        std::cerr << "无法创建缩放上下文" << std::endl;
        return false;
    }
    
    return true;
}

bool USBCamera::setResolution(int w, int h) {
    if (initialized) {
        std::cerr << "无法在摄像头初始化后更改分辨率" << std::endl;
        return false;
    }
    
    width = w;
    height = h;
    return true;
}

bool USBCamera::setFormat(CameraFormat fmt) {
    if (initialized) {
        std::cerr << "无法在摄像头初始化后更改格式" << std::endl;
        return false;
    }
    
    format = fmt;
    return true;
}

bool USBCamera::initialize() {
    if (initialized) {
        std::cerr << "摄像头已经初始化" << std::endl;
        return false;
    }
    
    if (!initializeFFmpeg()) {
        cleanup();
        return false;
    }
    
    if (!openCamera()) {
        cleanup();
        return false;
    }
    
    if (!setupCodec()) {
        cleanup();
        return false;
    }
    
    if (!setupScaler()) {
        cleanup();
        return false;
    }
    
    initialized = true;
    std::cout << "摄像头初始化成功 - 分辨率: " << width << "x" << height << std::endl;
    return true;
}

bool USBCamera::captureFrame() {
    if (!initialized) {
        std::cerr << "摄像头未初始化" << std::endl;
        return false;
    }
    
    // 读取数据包
    if (av_read_frame(format_ctx, packet) < 0) {
        std::cerr << "无法读取帧" << std::endl;
        return false;
    }
    
    // 确保是视频流的数据包
    if (packet->stream_index != video_stream_idx) {
        av_packet_unref(packet);
        return captureFrame(); // 递归调用直到获取视频帧
    }
    
    // 发送数据包给解码器
    if (avcodec_send_packet(codec_ctx, packet) < 0) {
        std::cerr << "无法发送数据包给解码器" << std::endl;
        av_packet_unref(packet);
        return false;
    }
    
    // 接收解码后的帧
    int ret = avcodec_receive_frame(codec_ctx, frame);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
        av_packet_unref(packet);
        return captureFrame(); // 需要更多数据
    } else if (ret < 0) {
        std::cerr << "解码错误" << std::endl;
        av_packet_unref(packet);
        return false;
    }
    
    // 转换像素格式为RGB
    sws_scale(sws_ctx, (uint8_t const * const *)frame->data, frame->linesize, 
              0, height, frame_rgb->data, frame_rgb->linesize);
    
    av_packet_unref(packet);
    return true;
}

bool USBCamera::saveFrameAsJPG(const std::string& filename) {
    if (!initialized) {
        std::cerr << "摄像头未初始化" << std::endl;
        return false;
    }
    
    // 创建输出格式上下文
    AVFormatContext* out_format_ctx = nullptr;
    if (avformat_alloc_output_context2(&out_format_ctx, nullptr, nullptr, filename.c_str()) < 0) {
        std::cerr << "无法创建输出格式上下文" << std::endl;
        return false;
    }
    
    // 添加视频流
    AVStream* out_stream = avformat_new_stream(out_format_ctx, nullptr);
    if (!out_stream) {
        std::cerr << "无法创建输出流" << std::endl;
        avformat_free_context(out_format_ctx);
        return false;
    }
    
    // 查找MJPEG编码器
    const AVCodec* mjpeg_codec = avcodec_find_encoder(AV_CODEC_ID_MJPEG);
    if (!mjpeg_codec) {
        std::cerr << "无法找到MJPEG编码器" << std::endl;
        avformat_free_context(out_format_ctx);
        return false;
    }
    
    // 创建编码器上下文
    AVCodecContext* mjpeg_ctx = avcodec_alloc_context3(mjpeg_codec);
    if (!mjpeg_ctx) {
        std::cerr << "无法分配MJPEG编码器上下文" << std::endl;
        avformat_free_context(out_format_ctx);
        return false;
    }
    
    // 设置编码器参数
    mjpeg_ctx->bit_rate = 400000;
    mjpeg_ctx->width = width;
    mjpeg_ctx->height = height;
    mjpeg_ctx->time_base = {1, 25};
    mjpeg_ctx->framerate = {25, 1};
    mjpeg_ctx->gop_size = 10;
    mjpeg_ctx->max_b_frames = 1;
    mjpeg_ctx->pix_fmt = AV_PIX_FMT_YUVJ420P;
    
    if (out_format_ctx->oformat->flags & AVFMT_GLOBALHEADER)
        mjpeg_ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    
    // 打开编码器
    if (avcodec_open2(mjpeg_ctx, mjpeg_codec, nullptr) < 0) {
        std::cerr << "无法打开MJPEG编码器" << std::endl;
        avcodec_free_context(&mjpeg_ctx);
        avformat_free_context(out_format_ctx);
        return false;
    }
    
    // 复制参数到输出流
    if (avcodec_parameters_from_context(out_stream->codecpar, mjpeg_ctx) < 0) {
        std::cerr << "无法复制编码器参数" << std::endl;
        avcodec_free_context(&mjpeg_ctx);
        avformat_free_context(out_format_ctx);
        return false;
    }
    
    // 打开输出文件
    if (avio_open(&out_format_ctx->pb, filename.c_str(), AVIO_FLAG_WRITE) < 0) {
        std::cerr << "无法打开输出文件: " << filename << std::endl;
        avcodec_free_context(&mjpeg_ctx);
        avformat_free_context(out_format_ctx);
        return false;
    }
    
    // 写入文件头
    if (avformat_write_header(out_format_ctx, nullptr) < 0) {
        std::cerr << "无法写入文件头" << std::endl;
        avcodec_free_context(&mjpeg_ctx);
        avio_closep(&out_format_ctx->pb);
        avformat_free_context(out_format_ctx);
        return false;
    }
    
    // 创建YUV420P帧用于编码
    AVFrame* yuv_frame = av_frame_alloc();
    yuv_frame->format = AV_PIX_FMT_YUVJ420P;
    yuv_frame->width = width;
    yuv_frame->height = height;
    av_frame_get_buffer(yuv_frame, 32);
    
    // 创建RGB到YUV的转换上下文
    SwsContext* rgb_to_yuv_ctx = sws_getContext(width, height, AV_PIX_FMT_RGB24,
                                               width, height, AV_PIX_FMT_YUVJ420P,
                                               SWS_BILINEAR, nullptr, nullptr, nullptr);
    
    // 转换RGB到YUV
    sws_scale(rgb_to_yuv_ctx, (uint8_t const * const *)frame_rgb->data, frame_rgb->linesize,
              0, height, yuv_frame->data, yuv_frame->linesize);
    
    yuv_frame->pts = 0;
    
    // 编码帧
    AVPacket* out_packet = av_packet_alloc();
    if (avcodec_send_frame(mjpeg_ctx, yuv_frame) >= 0) {
        while (avcodec_receive_packet(mjpeg_ctx, out_packet) >= 0) {
            out_packet->stream_index = out_stream->index;
            av_write_frame(out_format_ctx, out_packet);
            av_packet_unref(out_packet);
        }
    }
    
    // 写入文件尾
    av_write_trailer(out_format_ctx);
    
    // 清理资源
    av_frame_free(&yuv_frame);
    av_packet_free(&out_packet);
    sws_freeContext(rgb_to_yuv_ctx);
    avcodec_free_context(&mjpeg_ctx);
    avio_closep(&out_format_ctx->pb);
    avformat_free_context(out_format_ctx);
    
    std::cout << "图片保存成功: " << filename << std::endl;
    return true;
}

uint8_t* USBCamera::getFrameData() {
    if (!initialized || !frame_rgb) {
        return nullptr;
    }
    return frame_rgb->data[0];
}

int USBCamera::getFrameSize() {
    if (!initialized) {
        return 0;
    }
    return width * height * 3; // RGB24格式
}

void USBCamera::cleanup() {
    if (sws_ctx) {
        sws_freeContext(sws_ctx);
        sws_ctx = nullptr;
    }
    
    if (frame_rgb) {
        av_freep(&frame_rgb->data[0]);
        av_frame_free(&frame_rgb);
    }
    
    if (frame) {
        av_frame_free(&frame);
    }
    
    if (packet) {
        av_packet_free(&packet);
    }
    
    if (codec_ctx) {
        avcodec_close(codec_ctx);
        avcodec_free_context(&codec_ctx);
    }
    
    if (format_ctx) {
        avformat_close_input(&format_ctx);
    }
    
    initialized = false;
}

void USBCamera::release() {
    cleanup();
}

// 使用示例

#include <iostream>
#include <chrono>
#include <thread>

int main() {
    USBCamera camera("/dev/video0");
    
    // 设置分辨率和格式
    camera.setResolution(640, 480);
    camera.setFormat(CameraFormat::MJPEG);
    
    // 初始化摄像头
    if (!camera.initialize()) {
        std::cerr << "摄像头初始化失败" << std::endl;
        return -1;
    }
    
    // 捕获并保存10张图片
    for (int i = 0; i < 10; i++) {
        if (camera.captureFrame()) {
            std::string filename = "capture_" + std::to_string(i) + ".jpg";
            camera.saveFrameAsJPG(filename);
            std::cout << "已保存: " << filename << std::endl;
        }
        
        // 等待1秒sss
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    camera.release();
    return 0;
}
