extern "C" {
    #include <libavcodec/avcodec.h>
    #include <libavformat/avformat.h>
    #include <libswscale/swscale.h>
    }
    int main() {
        av_register_all();
        AVFormatContext* formatContext = nullptr;
        AVInputFormat* inputFormat = av_find_input_format("dshow");
        // 设置摄像头设备名称
        const char* deviceName = "video=USB Camera";
        if (avformat_open_input(&formatContext, deviceName, inputFormat, NULL) != 0) {
            fprintf(stderr, "无法打开摄像头设备\n");
            return -1;
        }
        if (avformat_find_stream_info(formatContext, NULL) < 0) {
            fprintf(stderr, "无法获取流信息\n");
            return -1;
        }
        int videoStreamIndex = -1;
        for (unsigned int i = 0; i < formatContext->nb_streams; i++) {
            if (formatContext->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
                videoStreamIndex = i;
                break;
            }
        }
        if (videoStreamIndex == -1) {
            fprintf(stderr, "无法找到视频流\n");
            return -1;
        }
        AVCodecParameters* codecParams = formatContext->streams[videoStreamIndex]->codecpar;
        AVCodec* codec = avcodec_find_decoder(codecParams->codec_id);
        
        // 打开解码器
        AVCodecContext* codecContext = avcodec_alloc_context3(codec);
        if (!codecContext) {
            fprintf(stderr, "无法分配解码器上下文\n");
            return -1;
        }
        if (avcodec_parameters_to_context(codecContext, codecParams) < 0) {
            fprintf(stderr, "无法复制解码器参数到上下文\n");
            return -1;
        }
        if (avcodec_open2(codecContext, codec, NULL) < 0) {
            fprintf(stderr, "无法打开解码器\n");
            return -1;
        }
        AVFrame* frame = av_frame_alloc();
        AVPacket packet;
        while (true) {
            if (av_read_frame(formatContext, &packet) < 0)
                break;
            if (packet.stream_index == videoStreamIndex) {
                avcodec_send_packet(codecContext, &packet);
                avcodec_receive_frame(codecContext, frame);
                // 在此处处理每一帧图像，可以进行后续的处理或显示操作
                // ...
                av_packet_unref(&packet);
            }
        }
        avformat_close_input(&formatContext);
        avformat_free_context(formatContext);
        
        return 0;
    }