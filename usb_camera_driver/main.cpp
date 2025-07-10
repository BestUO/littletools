#include <iostream>
#include <string>

extern "C" {
#include <libavdevice/avdevice.h>
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

int find_device()
{
    // 初始化 FFmpeg 设备模块
    avdevice_register_all();

    // 获取视频输入设备格式（如 v4l2）
    const AVInputFormat* fmt = av_find_input_format("v4l2");
    if (!fmt)
    {
        std::cerr << "无法获取 v4l2 设备格式" << std::endl;
        return -1;
    }

    // 枚举设备
    AVDeviceInfoList* device_list = nullptr;
    if (avdevice_list_input_sources(fmt, nullptr, nullptr, &device_list) < 0)
    {
        std::cerr << "无法枚举设备" << std::endl;
        return -1;
    }

    // 打印所有视频设备信息
    std::cout << "找到 " << device_list->nb_devices << " 个视频设备："
              << std::endl;
    for (int i = 0; i < device_list->nb_devices; i++)
    {
        AVDeviceInfo* dev = device_list->devices[i];
        std::cout << "[" << i << "] " << dev->device_name;
        if (dev->device_description)
        {
            std::cout << " (" << dev->device_description << ")";
        }
        std::cout << std::endl;
    }

    // 释放资源
    avdevice_free_list_devices(&device_list);
    return 0;
}

int push_stream()
{

    // 1. 初始化 FFmpeg
    avdevice_register_all();  // 注册设备输入
    avformat_network_init();  // 初始化网络（用于推流）

    // 2. 打开 USB 摄像头（使用 V4L2）
    AVFormatContext* input_ctx = nullptr;
    AVDictionary* options      = nullptr;
    av_dict_set(&options, "video_size", "1280x720", 0);
    av_dict_set(&options, "framerate", "10", 0);
    av_dict_set(&options, "pixel_format", "yuyv422", 0);  // 常见 USB 摄像头格式

    const char* input_device = "/dev/video0";  // USB 摄像头设备路径
    if (avformat_open_input(&input_ctx, input_device, nullptr, &options) < 0)
    {
        std::cerr << "无法打开摄像头设备: " << input_device << std::endl;
        return -1;
    }

    // 3. 查找视频流
    if (avformat_find_stream_info(input_ctx, nullptr) < 0)
    {
        std::cerr << "无法获取视频流信息" << std::endl;
        avformat_close_input(&input_ctx);
        return -1;
    }

    int video_stream_idx = -1;
    for (int i = 0; i < input_ctx->nb_streams; i++)
    {
        if (input_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO)
        {
            video_stream_idx = i;
            break;
        }
    }

    AVCodecParameters* codecpar
        = input_ctx->streams[video_stream_idx]->codecpar;
    {
        std::cout << "分辨率: " << codecpar->width << "x" << codecpar->height
                  << " codecpar->codec_id " << codecpar->codec_id << std::endl;
    }

    if (video_stream_idx == -1)
    {
        std::cerr << "未找到视频流" << std::endl;
        avformat_close_input(&input_ctx);
        return -1;
    }

    // 4. 初始化编码器（H.264）
    const AVCodec* codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (!codec)
    {
        std::cerr << "未找到 H.264 编码器" << std::endl;
        avformat_close_input(&input_ctx);
        return -1;
    }

    AVCodecContext* codec_ctx = avcodec_alloc_context3(codec);
    codec_ctx->width          = 640;
    codec_ctx->height         = 480;
    codec_ctx->pix_fmt        = AV_PIX_FMT_YUV420P;
    codec_ctx->time_base      = (AVRational){1, 30};  // 30 FPS
    codec_ctx->bit_rate       = 2000000;  // 2 Mbps

    // 设置 H.264 编码参数
    av_opt_set(codec_ctx->priv_data, "preset", "fast", 0);
    av_opt_set(codec_ctx->priv_data, "tune", "zerolatency", 0);
    av_opt_set(codec_ctx->priv_data, "profile", "baseline", 0);

    if (avcodec_open2(codec_ctx, codec, nullptr) < 0)
    {
        std::cerr << "无法打开编码器" << std::endl;
        avcodec_free_context(&codec_ctx);
        avformat_close_input(&input_ctx);
        return -1;
    }
    /*
        // 5. 初始化输出流（RTMP 推流）
        AVFormatContext* output_ctx = nullptr;
        const char* output_url
            = "rtmp://localhost/live/stream";  // 替换为你的 RTMP 服务器地址
        // const char* output_url = "udp://localhost:4321";  // 替换为你的 RTMP
        // 服务器地址
        // const char* output_url = "http://172.27.36.144:8080/stream";  //
        // 替换为你的 RTMP 服务器地址

        if (avformat_alloc_output_context2(&output_ctx, nullptr, "flv",
       output_url) < 0)
        {
            std::cerr << "无法创建输出流" << std::endl;
            avcodec_free_context(&codec_ctx);
            avformat_close_input(&input_ctx);
            return -1;
        }

        // 添加视频流到输出
        AVStream* out_stream = avformat_new_stream(output_ctx, codec);
        avcodec_parameters_from_context(out_stream->codecpar, codec_ctx);

        // 打开输出流
        if (avio_open(&output_ctx->pb, output_url, AVIO_FLAG_WRITE) < 0)
        {
            std::cerr << "无法打开输出 URL: " << output_url << std::endl;
            avformat_free_context(output_ctx);
            avcodec_free_context(&codec_ctx);
            avformat_close_input(&input_ctx);
            return -1;
        }

        // 写入流头部
        if (avformat_write_header(output_ctx, nullptr) < 0)
        {
            std::cerr << "无法写入流头部" << std::endl;
            avio_closep(&output_ctx->pb);
            avformat_free_context(output_ctx);
            avcodec_free_context(&codec_ctx);
            avformat_close_input(&input_ctx);
            return -1;
        }
    */
    // 6. 读取摄像头数据并推流
    AVPacket packet;
    AVFrame* frame = av_frame_alloc();
    frame->format  = AV_PIX_FMT_YUV420P;
    frame->width   = 640;
    frame->height  = 480;
    av_frame_get_buffer(frame, 32);

    SwsContext* sws_ctx = sws_getContext(640,
        480,
        AV_PIX_FMT_YUYV422,  // 输入格式
        640,
        480,
        AV_PIX_FMT_YUV420P,  // 输出格式
        SWS_BILINEAR,
        nullptr,
        nullptr,
        nullptr);
    int frame_count     = 0;
    while (true)
    {
        if (av_read_frame(input_ctx, &packet) < 0)
        {
            std::cerr << "无法读取帧" << std::endl;
            break;
        }
        if (packet.stream_index == video_stream_idx)
        {
            // 转换像素格式（YUYV422 → YUV420P）
            uint8_t* src_data[1] = {packet.data};
            int src_linesize[1]  = {packet.size / 480};  // 估算行大小

            sws_scale(sws_ctx,
                src_data,
                src_linesize,
                0,
                480,
                frame->data,
                frame->linesize);
            // 编码帧
            if (avcodec_send_frame(codec_ctx, frame) < 0)
            {
                std::cerr << "编码失败" << std::endl;
                break;
            }

            while (avcodec_receive_packet(codec_ctx, &packet) == 0)
            {
                // 写入输出流
                // av_interleaved_write_frame(output_ctx, &packet);
                av_packet_unref(&packet);
            }
            frame_count++;
            if (frame_count % 30 == 0)
            {
                std::cout << "Pushed " << frame_count << " frames" << std::endl;
            }
        }
    }

    // 7. 清理资源
    sws_freeContext(sws_ctx);
    av_frame_free(&frame);
    // av_write_trailer(output_ctx);
    // avio_closep(&output_ctx->pb);
    // avformat_free_context(output_ctx);
    avcodec_free_context(&codec_ctx);
    avformat_close_input(&input_ctx);
    return 0;
}

int push_stream1()
{

    // 1. 注册所有FFmpeg组件（必需）
    avdevice_register_all();
    avformat_network_init();

    // 2. 打开输入设备（/dev/video0）
    AVFormatContext* input_ctx  = nullptr;
    AVDictionary* input_options = nullptr;
    av_dict_set(&input_options, "framerate", "30", 0);  // 设置帧率
    av_dict_set(&input_options, "video_size", "640x480", 0);  // 设置分辨率

    const char* input_url = "/dev/video0";
    if (avformat_open_input(
            &input_ctx, input_url, av_find_input_format("v4l2"), &input_options)
        < 0)
    {
        std::cerr << "Failed to open input device" << std::endl;
        return -1;
    }

    // 3. 获取流信息
    if (avformat_find_stream_info(input_ctx, nullptr) < 0)
    {
        std::cerr << "Failed to get stream info" << std::endl;
        avformat_close_input(&input_ctx);
        return -1;
    }

    // 4. 查找视频流
    int video_stream_index = -1;
    for (unsigned int i = 0; i < input_ctx->nb_streams; i++)
    {
        if (input_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO)
        {
            video_stream_index = i;
            break;
        }
    }
    if (video_stream_index == -1)
    {
        std::cerr << "No video stream found" << std::endl;
        avformat_close_input(&input_ctx);
        return -1;
    }

    // 5. 创建输出格式上下文（RTMP）
    AVFormatContext* output_ctx = nullptr;
    const char* output_url      = "rtmp://localhost/live/stream";
    if (avformat_alloc_output_context2(&output_ctx, nullptr, "flv", output_url)
        < 0)
    {
        std::cerr << "Failed to create output context" << std::endl;
        avformat_close_input(&input_ctx);
        return -1;
    }

    // 6. 添加视频流到输出
    AVStream* out_stream = avformat_new_stream(output_ctx, nullptr);
    if (!out_stream)
    {
        std::cerr << "Failed to create output stream" << std::endl;
        avformat_free_context(output_ctx);
        avformat_close_input(&input_ctx);
        return -1;
    }

    // 7. 复制编解码器参数并配置编码器
    AVCodecParameters* in_codecpar
        = input_ctx->streams[video_stream_index]->codecpar;
    const AVCodec* codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (!codec)
    {
        std::cerr << "H.264 encoder not found" << std::endl;
        avformat_free_context(output_ctx);
        avformat_close_input(&input_ctx);
        return -1;
    }

    AVCodecContext* codec_ctx = avcodec_alloc_context3(codec);
    avcodec_parameters_to_context(codec_ctx, in_codecpar);
    codec_ctx->pix_fmt   = AV_PIX_FMT_YUV420P;  // 强制使用YUV420P
    codec_ctx->time_base = (AVRational){1, 25};  // 30 FPS
    codec_ctx->bit_rate  = 2000000;  // 2 Mbps
    av_opt_set(codec_ctx->priv_data, "preset", "ultrafast", 0);  // 低延迟编码
    av_opt_set(codec_ctx->priv_data, "tune", "zerolatency", 0);

    if (avcodec_open2(codec_ctx, codec, nullptr) < 0)
    {
        std::cerr << "Failed to open encoder" << std::endl;
        avcodec_free_context(&codec_ctx);
        avformat_free_context(output_ctx);
        avformat_close_input(&input_ctx);
        return -1;
    }

    // 8. 将编码器参数复制到输出流
    avcodec_parameters_from_context(out_stream->codecpar, codec_ctx);
    out_stream->time_base = codec_ctx->time_base;

    // 9. 打开输出（RTMP）
    if (!(output_ctx->oformat->flags & AVFMT_NOFILE))
    {
        if (avio_open(&output_ctx->pb, output_url, AVIO_FLAG_WRITE) < 0)
        {
            std::cerr << "Failed to open output URL" << std::endl;
            avcodec_free_context(&codec_ctx);
            avformat_free_context(output_ctx);
            avformat_close_input(&input_ctx);
            return -1;
        }
    }

    // 10. 写文件头
    if (avformat_write_header(output_ctx, nullptr) < 0)
    {
        std::cerr << "Failed to write header" << std::endl;
        avio_closep(&output_ctx->pb);
        avcodec_free_context(&codec_ctx);
        avformat_free_context(output_ctx);
        avformat_close_input(&input_ctx);
        return -1;
    }

    // 11. 读取帧并推流
    AVPacket packet;
    AVFrame* frame    = av_frame_alloc();
    int64_t frame_pts = 0;

    while (true)
    {
        if (av_read_frame(input_ctx, &packet) < 0)
            break;  // 读取结束或错误

        if (packet.stream_index == video_stream_index)
        {
            // 解码帧（可选，若需处理帧数据）
            // 此处省略解码步骤，直接转发原始数据（需确保格式兼容）

            // 重新封装为RTMP流
            packet.stream_index = out_stream->index;
            packet.pts = packet.dts = av_rescale_q(
                frame_pts++, codec_ctx->time_base, out_stream->time_base);
            packet.duration
                = av_rescale_q(1, codec_ctx->time_base, out_stream->time_base);

            // 发送包
            if (av_interleaved_write_frame(output_ctx, &packet) < 0)
            {
                std::cerr << "Failed to send packet" << std::endl;
                break;
            }
        }
        av_packet_unref(&packet);
    }

    // 12. 清理资源
    av_write_trailer(output_ctx);
    av_frame_free(&frame);
    avcodec_free_context(&codec_ctx);
    if (output_ctx && !(output_ctx->oformat->flags & AVFMT_NOFILE))
    {
        avio_closep(&output_ctx->pb);
    }
    avformat_free_context(output_ctx);
    avformat_close_input(&input_ctx);

    return 0;
}
int main()
{

    std::cout
        << "--------------------------------------find-----------------------"
        << std::endl;

    find_device();
    std::cout
        << "--------------------------------------push-----------------------"
        << std::endl;
    push_stream();

    return 0;
}