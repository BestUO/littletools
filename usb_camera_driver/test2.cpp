#include <iostream>
extern "C" {
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}
int main() {
    // 初始化FFmpeg库
    av_register_all();
    
    // 设置参数
    const char* deviceName = "video=USB Camera"; // 更改为你的摄像头设备名称
    const char* videoSize = "1920x1080"; // 高分辨率设置
    AVFormatContext* formatContext = NULL;
    // 打开设备并设置参数
    if (avformat_open_input(&formatContext, deviceName, av_find_input_format("dshow"), NULL) != 0) {
        std::cerr << "无法打开设备" << std::endl;
        return -1;
    }
    AVDictionary* options = NULL;
    // 设置视频输入格式和分辨率
    av_dict_set(&options, "video_size", videoSize, 0);
   if (avformat_find_stream_info(formatContext,NULL)<0){
      fprintf(stderr,"Could not find stream information\n");
      return -1; 
   }
   int videoStreamIndex = -1;
   for (unsigned int i = 0; i < formatContext->nb_streams; ++i) {
       if (formatContext->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
           videoStreamIndex = i;
           break;
       }
   }
   if (videoStreamIndex == -1) {
       std::cerr << "找不到视频流" << std::endl;
       avformat_close_input(&formatContext);
       return -1;
   }
   // 设置视频输入参数
   AVStream* videoStream = formatContext->streams[videoStreamIndex];
   AVCodecParameters* codecParams = videoStream->codecpar;
   codecParams->width = 1920; // 设置宽度
   codecParams->height = 1080; // 设置高度
   av_dump_format(formatContext, 0, deviceName, 0);
    // 查找视频解码器
    AVCodec* codec = avcodec_find_decoder(codecParams->codec_id);
    if (!codec) {
        std::cerr << "无法找到合适的解码器" << std::endl;
        avformat_close_input(&formatContext);
        return -1;
    }
    // 创建解码器上下文
    AVCodecContext* codecContext = avcodec_alloc_context3(codec);
    if (!codecContext) {
        std::cerr << "无法创建解码器上下文" << std::endl;
        avformat_close_input(&formatContext);
        return -1;
    }
    // 打开解码器
    if (avcodec_open2(codecContext, codec, NULL) < 0) {
        std::cerr << "无法打开解码器" << std::endl;
        avcodec_free_context(&codecContext);
        avformat_close_input(&formatContext);
        return -1;
    }
    // 分配AVPacket和AVFrame用于存储视频数据
    AVPacket* packet = av_packet_alloc();
    AVFrame* frame = av_frame_alloc();
     // 循环读取并播放视频帧
     while (av_read_frame(formatContext, packet) >= 0) {
         if (packet->stream_index == videoStreamIndex) {
             // 解码视频帧
             avcodec_send_packet(codecContext, packet);
             while (avcodec_receive_frame(codecContext, frame) == 0) {
                 // 处理解码后的帧数据
                 // 在这里你可以进行一些其他处理，比如显示、保存等
                 // ...
                 // 播放视频帧（示例：输出到屏幕）
                 SwsContext* swsContext = sws_getContext(codecParams->width, codecParams->height, codecContext->pix_fmt,
                                                         codecParams->width, codecParams->height, AV_PIX_FMT_RGB24,
                                                         SWS_BILINEAR, NULL, NULL, NULL);
                uint8_t* rgbBuffer[1] = {NULL};
                int rgbLinesize[1];
                av_image_alloc(rgbBuffer, rgbLinesize,
                               codecParams->width, codecParams->height,
                               AV_PIX_FMT_RGB24, 1);
                
                sws_scale(swsContext, frame->data, frame->linesize,
                          0, codecParams->height,
                          rgbBuffer, rgbLinesize);
                          
               // 在此处你可以将rgbBuffer中的数据显示在屏幕上或保存为图像文件
               // ...
               av_freep(&rgbBuffer[0]);
               sws_freeContext(swsContext);
            }
        }
       av_packet_unref(packet);
   }
   // 释放资源
   av_frame_free(&frame);
   av_packet_free(&packet);
   avcodec_close(codecContext);
   avformat_close_input(&formatContext);
   return 0;
}