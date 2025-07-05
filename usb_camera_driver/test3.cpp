extern "C" {
    #include <libavformat/avformat.h>
    #include <libavutil/imgutils.h>
    #include <libswscale/swscale.h>
}
#include <iostream>
int main() {
    // 注册所有组件
    av_register_all();
    
    // 输入格式上下文
    AVFormatContext* formatContext = nullptr;
    // 打开USB摄像头（Windows下的DirectShow）
    const char* deviceName = "video=Your_USB_Camera_Name"; // 替换为实际设备名
    if (avformat_open_input(&formatContext, deviceName, nullptr, nullptr) != 0) {
        std::cerr << "Could not open video device." << std::endl;
        return -1;
    }
    // 查找流信息
    if (avformat_find_stream_info(formatContext, nullptr) < 0) {
        std::cerr << "Could not find stream information." << std::endl;
        avformat_close_input(&formatContext);
        return -1;
    }
    // 找到第一个视频流
    int videoStreamIndex = -1;
    for (unsigned int i = 0; i < formatContext->nb_streams; ++i) {
        if (formatContext->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            videoStreamIndex = i;
            break;
        }
    }
    if (videoStreamIndex == -1) {
        std::cerr << "Could not find a video stream." << std::endl;
        avformat_close_input(&formatContext);
        return -1;
    }
    AVCodecParameters* codecParams = formatContext->streams[videoStreamIndex]->codecpar;
    
    // 找到解码器
    AVCodec* codec = avcodec_find_decoder(codecParams->codec_id);
    
    if (!codec) {
        std::cerr << "Unsupported codec!" << std::endl;
        avformat_close_input(&formatContext);
        return -1;
    }
    AVCodecContext* codecContext = avcodec_alloc_context3(codec);
    
    if (!codecContext || 
       avcodec_parameters_to_context(codecContext, codecParams) < 0 || 
       avcodec_open2(codecContext, codec, nullptr) < 0) {
        
       std::cerr << "Could not initialize the decoder context." << std::endl; 
       av_free(codecContext); 
       avformat_close_input(&formatContext); 
       return -1; 
   } 
   AVFrame* frame = av_frame_alloc(); 
   while (true) { 
       AVPacket packet; 
       
       // 从摄像头读取数据包
       if (av_read_frame(formatContext, &packet) >= 0 && packet.stream_index == videoStreamIndex) {  
           // 解码视频帧
           int response = avcodec_send_packet(codecContext, &packet); 
           while (response >= 0) { 
               response = avcodec_receive_frame(codecContext, frame); 
               if (response == AVERROR(EAGAIN) || response == AVERROR_EOF)
                   break; 
               else if (response < 0)
                   break;  
               // 在这里处理解码后的帧（例如显示、保存等）
               std::cout << "Received frame: width=" << frame->width 
                         << ", height=" << frame->height << std::endl;
               // TODO: 在这里添加处理逻辑，如渲染到窗口或保存图像等。
           }  
           
           av_packet_unref(&packet);  
       } else {  
           break;  
       }
   }
   // 清理资源
   av_frame_free(&frame);
   avcodec_free_context(&codecContext);
   avformat_close_input(&formatContext);
   return 0; 
}