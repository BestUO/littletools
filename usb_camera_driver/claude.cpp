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
    #include <libavutil/time.h>
}

enum class CameraFormat
{
    MJPEG,
    H264,
    YUV422
};

class USBCamera
{
private:
    // 输入相关
    AVFormatContext* format_ctx;
    AVCodecContext* decode_ctx;
    const AVCodec* decoder;

    // 输出相关（JPEG编码）
    AVCodecContext* encode_ctx;
    const AVCodec* encoder;

    // 帧和数据包
    AVFrame* frame;
    AVFrame* frame_rgb;
    AVFrame* frame_yuv;
    AVPacket* packet;

    // 缩放上下文
    SwsContext* sws_ctx_rgb;  // 解码后转RGB
    SwsContext* sws_ctx_yuv;  // RGB转YUV用于编码

    int video_stream_idx;
    std::string device_path;
    int width;
    int height;
    int fps;
    CameraFormat format;

    bool initialized;

    // 私有初始化方法
    bool initializeFFmpeg();
    bool initializeEncoder2();
    bool initializeDecoder();
    bool initializeEncoder();
    bool initializeFramesAndScaler();
    bool openCamera();

    // 输出流相关
    bool createOutputContext(const std::string& filename,
        AVFormatContext** out_ctx,
        AVStream** out_stream);
    void cleanupOutputContext(AVFormatContext* out_ctx);

    void cleanup();

public:
    USBCamera(const std::string& device = "/dev/video0",
        int w                           = 640,
        int h                           = 480,
        int framerate                   = 30,
        CameraFormat fmt                = CameraFormat::MJPEG);
    ~USBCamera();

    // 设置摄像头参数（初始化前）
    bool setResolution(int w, int h);
    bool setFormat(CameraFormat fmt);
    bool setFrameRate(int framerate);

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
    int getWidth() const
    {
        return width;
    }
    int getHeight() const
    {
        return height;
    }
    int getFrameRate() const
    {
        return fps;
    }
    CameraFormat getFormat() const
    {
        return format;
    }

    // 释放资源
    void release();
};

#endif  // USB_CAMERA_H

// 实现文件 USBCamera.cpp
#include <iostream>
#include <cstring>

USBCamera::USBCamera(const std::string& device,
    int w,
    int h,
    int framerate,
    CameraFormat fmt)
    : format_ctx(nullptr)
    , decode_ctx(nullptr)
    , decoder(nullptr)
    , encode_ctx(nullptr)
    , encoder(nullptr)
    , frame(nullptr)
    , frame_rgb(nullptr)
    , frame_yuv(nullptr)
    , packet(nullptr)
    , sws_ctx_rgb(nullptr)
    , sws_ctx_yuv(nullptr)
    , video_stream_idx(-1)
    , device_path(device)
    , width(w)
    , height(h)
    , fps(framerate)
    , format(fmt)
    , initialized(false)
{

    // 初始化FFmpeg
    avdevice_register_all();
    avformat_network_init();

    std::cout << "USBCamera created with settings:" << std::endl;
    std::cout << "  Device: " << device_path << std::endl;
    std::cout << "  Resolution: " << width << "x" << height << std::endl;
    std::cout << "  Frame rate: " << fps << " fps" << std::endl;
    std::cout << "  Format: "
              << (format == CameraFormat::MJPEG         ? "MJPEG"
                         : format == CameraFormat::H264 ? "H264"
                                                        : "YUV422")
              << std::endl;
}

USBCamera::~USBCamera()
{
    cleanup();
}

bool USBCamera::initializeFFmpeg()
{
    // 分配AVFormatContext
    format_ctx = avformat_alloc_context();
    if (!format_ctx)
    {
        std::cerr << "无法分配AVFormatContext" << std::endl;
        return false;
    }

    // 查找输入格式
    const AVInputFormat* input_format = av_find_input_format("v4l2");
    if (!input_format)
    {
        std::cerr << "无法找到v4l2输入格式" << std::endl;
        return false;
    }

    // 设置输入选项
    AVDictionary* options = nullptr;

    // 根据格式设置参数
    std::string pixel_format;
    switch (format)
    {
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

    std::cout << "使用输入格式: " << input_format->name << " video_size "
              << width << "x" << height << ", pixel_format: " << pixel_format
              << std::endl;

    av_dict_set(&options, "pixel_format", pixel_format.c_str(), 0);
    av_dict_set(&options,
        "video_size",
        (std::to_string(width) + "x" + std::to_string(height)).c_str(),
        0);
    av_dict_set(&options, "framerate", std::to_string(fps).c_str(), 0);

    // 打开输入设备
    if (avformat_open_input(
            &format_ctx, device_path.c_str(), input_format, &options)
        != 0)
    {
        std::cerr << "无法打开摄像头设备: " << device_path << std::endl;
        av_dict_free(&options);
        return false;
    }

    av_dict_free(&options);

    // 查找流信息
    if (avformat_find_stream_info(format_ctx, nullptr) < 0)
    {
        std::cerr << "无法获取流信息" << std::endl;
        return false;
    }

    return true;
}

bool USBCamera::initializeDecoder()
{
    // 查找视频流
    video_stream_idx = -1;
    for (unsigned int i = 0; i < format_ctx->nb_streams; i++)
    {
        if (format_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO)
        {
            video_stream_idx = i;
            break;
        }
    }

    if (video_stream_idx == -1)
    {
        std::cerr << "无法找到视频流" << std::endl;
        return false;
    }

    AVCodecParameters* codecpar
        = format_ctx->streams[video_stream_idx]->codecpar;
    {
        std::cout << "分辨率: " << codecpar->width << "x" << codecpar->height
                  << " fps: " << avcodec_get_name(codecpar->codec_id)
                  << std::endl;
    }
    // 查找解码器
    decoder = avcodec_find_decoder(codecpar->codec_id);
    if (!decoder)
    {
        std::cerr << "无法找到解码器" << std::endl;
        return false;
    }

    // 分配解码器上下文
    decode_ctx = avcodec_alloc_context3(decoder);
    if (!decode_ctx)
    {
        std::cerr << "无法分配解码器上下文" << std::endl;
        return false;
    }

    // 复制参数到解码器上下文
    if (avcodec_parameters_to_context(decode_ctx, codecpar) < 0)
    {
        std::cerr << "无法复制解码器参数" << std::endl;
        return false;
    }

    // 打开解码器
    if (avcodec_open2(decode_ctx, decoder, nullptr) < 0)
    {
        std::cerr << "无法打开解码器" << std::endl;
        return false;
    }

    // 更新实际的宽高
    width  = decode_ctx->width;
    height = decode_ctx->height;

    std::cout << "解码器初始化成功 - 实际分辨率: " << width << "x" << height
              << std::endl;
    return true;
}

bool USBCamera::initializeEncoder2()
{ }

bool USBCamera::initializeEncoder()
{
    // 查找MJPEG编码器
    encoder = avcodec_find_encoder(AV_CODEC_ID_MJPEG);
    if (!encoder)
    {
        std::cerr << "无法找到MJPEG编码器" << std::endl;
        return false;
    }

    // 创建编码器上下文
    encode_ctx = avcodec_alloc_context3(encoder);
    if (!encode_ctx)
    {
        std::cerr << "无法分配MJPEG编码器上下文" << std::endl;
        return false;
    }

    // 设置编码器参数
    encode_ctx->bit_rate     = 400000;
    encode_ctx->width        = width;
    encode_ctx->height       = height;
    encode_ctx->time_base    = {1, fps};
    encode_ctx->framerate    = {fps, 1};
    encode_ctx->gop_size     = 10;
    encode_ctx->max_b_frames = 0;  // MJPEG不使用B帧
    encode_ctx->pix_fmt      = AV_PIX_FMT_YUVJ420P;

    // 打开编码器
    if (avcodec_open2(encode_ctx, encoder, nullptr) < 0)
    {
        std::cerr << "无法打开MJPEG编码器" << std::endl;
        return false;
    }

    std::cout << "MJPEG编码器初始化成功" << std::endl;
    return true;
}

bool USBCamera::initializeFramesAndScaler()
{
    // 分配帧
    frame     = av_frame_alloc();
    frame_rgb = av_frame_alloc();
    frame_yuv = av_frame_alloc();
    packet    = av_packet_alloc();

    if (!frame || !frame_rgb || !frame_yuv || !packet)
    {
        std::cerr << "无法分配帧或数据包" << std::endl;
        return false;
    }

    // 为RGB帧分配缓冲区
    int num_bytes_rgb
        = av_image_get_buffer_size(AV_PIX_FMT_RGB24, width, height, 1);
    uint8_t* buffer_rgb = (uint8_t*)av_malloc(num_bytes_rgb * sizeof(uint8_t));
    av_image_fill_arrays(frame_rgb->data,
        frame_rgb->linesize,
        buffer_rgb,
        AV_PIX_FMT_RGB24,
        width,
        height,
        1);

    // 为YUV帧分配缓冲区
    frame_yuv->format = AV_PIX_FMT_YUVJ420P;
    frame_yuv->width  = width;
    frame_yuv->height = height;
    if (av_frame_get_buffer(frame_yuv, 32) < 0)
    {
        std::cerr << "无法为YUV帧分配缓冲区" << std::endl;
        return false;
    }

    // 创建缩放上下文 - 解码后转RGB
    sws_ctx_rgb = sws_getContext(width,
        height,
        decode_ctx->pix_fmt,
        width,
        height,
        AV_PIX_FMT_RGB24,
        SWS_BILINEAR,
        nullptr,
        nullptr,
        nullptr);

    if (!sws_ctx_rgb)
    {
        std::cerr << "无法创建RGB缩放上下文" << std::endl;
        return false;
    }

    // 创建缩放上下文 - RGB转YUV用于编码
    sws_ctx_yuv = sws_getContext(width,
        height,
        AV_PIX_FMT_RGB24,
        width,
        height,
        AV_PIX_FMT_YUVJ420P,
        SWS_BILINEAR,
        nullptr,
        nullptr,
        nullptr);

    if (!sws_ctx_yuv)
    {
        std::cerr << "无法创建YUV缩放上下文" << std::endl;
        return false;
    }

    std::cout << "帧和缩放器初始化成功" << std::endl;
    return true;
}

bool USBCamera::openCamera()
{
    return true;  // 这个函数已经在initializeFFmpeg中处理了
}

bool USBCamera::setResolution(int w, int h)
{
    if (initialized)
    {
        std::cerr << "无法在摄像头初始化后更改分辨率" << std::endl;
        return false;
    }

    width  = w;
    height = h;
    return true;
}

bool USBCamera::setFormat(CameraFormat fmt)
{
    if (initialized)
    {
        std::cerr << "无法在摄像头初始化后更改格式" << std::endl;
        return false;
    }

    format = fmt;
    return true;
}

bool USBCamera::setFrameRate(int framerate)
{
    if (initialized)
    {
        std::cerr << "无法在摄像头初始化后更改帧率" << std::endl;
        return false;
    }

    fps = framerate;
    return true;
}

bool USBCamera::initialize()
{
    if (initialized)
    {
        std::cerr << "摄像头已经初始化" << std::endl;
        return false;
    }

    if (!initializeFFmpeg())
    {
        cleanup();
        return false;
    }

    if (!initializeDecoder())
    {
        cleanup();
        return false;
    }

    if (!initializeEncoder())
    {
        cleanup();
        return false;
    }

    if (!initializeFramesAndScaler())
    {
        cleanup();
        return false;
    }

    initialized = true;
    std::cout << "摄像头初始化成功!" << std::endl;
    return true;
}

bool USBCamera::captureFrame()
{
    if (!initialized)
    {
        std::cerr << "摄像头未初始化" << std::endl;
        return false;
    }

    // 读取数据包
    if (av_read_frame(format_ctx, packet) < 0)
    {
        std::cerr << "无法读取帧" << std::endl;
        return false;
    }

    // 确保是视频流的数据包
    if (packet->stream_index != video_stream_idx)
    {
        av_packet_unref(packet);
        return captureFrame();  // 递归调用直到获取视频帧
    }

    // 发送数据包给解码器
    if (avcodec_send_packet(decode_ctx, packet) < 0)
    {
        std::cerr << "无法发送数据包给解码器" << std::endl;
        av_packet_unref(packet);
        return false;
    }

    // 接收解码后的帧
    int ret = avcodec_receive_frame(decode_ctx, frame);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
    {
        av_packet_unref(packet);
        return captureFrame();  // 需要更多数据
    }
    else if (ret < 0)
    {
        std::cerr << "解码错误" << std::endl;
        av_packet_unref(packet);
        return false;
    }

    // if (format == CameraFormat::MJPEG)
    // {
    //     std::string filename
    //         = "frame_" + std::to_string(av_gettime_relative()) + ".jpg";
    //     FILE* f = fopen(filename.c_str(), "wb");
    //     fwrite(packet->data, 1, packet->size, f);
    //     fclose(f);
    //     return true;
    //     printf("帧捕获成功: %s\n", filename.c_str());
    // }

    // 转换像素格式为RGB
    sws_scale(sws_ctx_rgb,
        (uint8_t const* const*)frame->data,
        frame->linesize,
        0,
        height,
        frame_rgb->data,
        frame_rgb->linesize);

    av_packet_unref(packet);
    return true;
}

bool USBCamera::createOutputContext(const std::string& filename,
    AVFormatContext** out_ctx,
    AVStream** out_stream)
{
    // 创建输出格式上下文
    if (avformat_alloc_output_context2(
            out_ctx, nullptr, "mjpeg", filename.c_str())
        < 0)
    {
        std::cerr << "无法创建输出格式上下文" << std::endl;
        return false;
    }

    // 添加视频流
    *out_stream = avformat_new_stream(*out_ctx, nullptr);
    if (!*out_stream)
    {
        std::cerr << "无法创建输出流" << std::endl;
        avformat_free_context(*out_ctx);
        *out_ctx = nullptr;
        return false;
    }

    // 复制编码器参数到输出流
    if (avcodec_parameters_from_context((*out_stream)->codecpar, encode_ctx)
        < 0)
    {
        std::cerr << "无法复制编码器参数" << std::endl;
        avformat_free_context(*out_ctx);
        *out_ctx = nullptr;
        return false;
    }

    // 打开输出文件
    if (avio_open(&(*out_ctx)->pb, filename.c_str(), AVIO_FLAG_WRITE) < 0)
    {
        std::cerr << "无法打开输出文件: " << filename << std::endl;
        avformat_free_context(*out_ctx);
        *out_ctx = nullptr;
        return false;
    }

    // 写入文件头
    if (avformat_write_header(*out_ctx, nullptr) < 0)
    {
        std::cerr << "无法写入文件头" << std::endl;
        avio_closep(&(*out_ctx)->pb);
        avformat_free_context(*out_ctx);
        *out_ctx = nullptr;
        return false;
    }

    std::cout << "输出上下文创建成功: " << filename << std::endl;
    return true;
}

void USBCamera::cleanupOutputContext(AVFormatContext* out_ctx)
{
    if (out_ctx)
    {
        // 写入文件尾
        av_write_trailer(out_ctx);

        // 关闭文件
        if (out_ctx->pb)
        {
            avio_closep(&out_ctx->pb);
        }

        // 释放上下文
        avformat_free_context(out_ctx);
    }
}

bool USBCamera::saveFrameAsJPG(const std::string& filename)
{
    // 1. RGB转YUV（JPEG编码需要YUV格式）
    sws_scale(sws_ctx_yuv,
        frame_rgb->data,
        frame_rgb->linesize,
        0,
        height,
        frame_yuv->data,
        frame_yuv->linesize);

    // 2. 直接编码为JPEG
    AVPacket* pkt = av_packet_alloc();
    avcodec_send_frame(encode_ctx, frame_yuv);
    avcodec_receive_packet(encode_ctx, pkt);

    // 3. 直接写入文件（不需要format上下文）
    FILE* f = fopen(filename.c_str(), "wb");
    fwrite(pkt->data, 1, pkt->size, f);
    fclose(f);

    // 4. 清理
    av_packet_free(&pkt);
    return true;
}

// bool USBCamera::saveFrameAsJPG(const std::string& filename) {
//     if (!initialized) {
//         std::cerr << "摄像头未初始化" << std::endl;
//         return false;
//     }

//     if (!frame_rgb || !frame_rgb->data[0]) {
//         std::cerr << "没有可用的RGB帧数据" << std::endl;
//         return false;
//     }

//     // 创建输出上下文和流
//     AVFormatContext* out_format_ctx = nullptr;
//     AVStream* out_stream = nullptr;

//     if (!createOutputContext(filename, &out_format_ctx, &out_stream)) {
//         return false;
//     }

//     // 转换RGB到YUV
//     sws_scale(sws_ctx_yuv, (uint8_t const * const *)frame_rgb->data,
//     frame_rgb->linesize,
//               0, height, frame_yuv->data, frame_yuv->linesize);

//     static int64_t pts_counter = 0;
//     frame_yuv->pts = pts_counter++;

//     // 编码帧
//     AVPacket* out_packet = av_packet_alloc();
//     if (!out_packet) {
//         std::cerr << "无法分配输出数据包" << std::endl;
//         cleanupOutputContext(out_format_ctx);
//         return false;
//     }

//     bool encode_success = false;
//     if (avcodec_send_frame(encode_ctx, frame_yuv) >= 0) {
//         while (avcodec_receive_packet(encode_ctx, out_packet) >= 0) {
//             out_packet->stream_index = out_stream->index;
//             av_write_frame(out_format_ctx, out_packet);
//             av_packet_unref(out_packet);
//             encode_success = true;
//         }
//     }

//     // 清理资源
//     av_packet_free(&out_packet);
//     cleanupOutputContext(out_format_ctx);

//     if (encode_success) {
//         std::cout << "图片保存成功: " << filename << std::endl;
//         return true;
//     } else {
//         std::cerr << "编码失败" << std::endl;
//         return false;
//     }
// }

uint8_t* USBCamera::getFrameData()
{
    if (!initialized || !frame_rgb)
    {
        return nullptr;
    }
    return frame_rgb->data[0];
}

int USBCamera::getFrameSize()
{
    if (!initialized)
    {
        return 0;
    }
    return width * height * 3;  // RGB24格式
}

void USBCamera::cleanup()
{
    if (sws_ctx_rgb)
    {
        sws_freeContext(sws_ctx_rgb);
        sws_ctx_rgb = nullptr;
    }

    if (sws_ctx_yuv)
    {
        sws_freeContext(sws_ctx_yuv);
        sws_ctx_yuv = nullptr;
    }

    if (frame_rgb)
    {
        av_freep(&frame_rgb->data[0]);
        av_frame_free(&frame_rgb);
    }

    if (frame_yuv)
    {
        av_frame_free(&frame_yuv);
    }

    if (frame)
    {
        av_frame_free(&frame);
    }

    if (packet)
    {
        av_packet_free(&packet);
    }

    if (encode_ctx)
    {
        avcodec_free_context(&encode_ctx);
    }

    if (decode_ctx)
    {
        avcodec_free_context(&decode_ctx);
    }

    if (format_ctx)
    {
        avformat_close_input(&format_ctx);
    }

    initialized = false;
}

void USBCamera::release()
{
    cleanup();
}

// 使用示例
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    // 创建摄像头对象，设置分辨率640x480，帧率30fps，格式MJPEG
    USBCamera camera("/dev/video0", 640, 480, 5, CameraFormat::MJPEG);

    // 可以在初始化前修改设置
    // camera.setResolution(1280, 720);
    // camera.setFrameRate(15);

    // 初始化摄像头
    if (!camera.initialize())
    {
        std::cerr << "摄像头初始化失败" << std::endl;
        return -1;
    }

    // 捕获并保存10张图片
    for (int i = 0; i < 10; i++)
    {
        if (camera.captureFrame())
        {
            std::string filename = "capture_" + std::to_string(i) + ".jpg";
            // camera.saveFrameAsJPG(filename);
            std::cout << "已保存: " << filename << std::endl;
        }

        // 等待1秒
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    camera.release();
    return 0;
}