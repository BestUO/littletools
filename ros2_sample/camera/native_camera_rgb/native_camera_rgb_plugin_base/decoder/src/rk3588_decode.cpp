/*----------------------------------------------------------------------------------------------
*
* This file is Sunny Optical's property. It contains Sunny Optical's trade secret, proprietary and 		
* confidential information. 
* 
* The information and code contained in this file is only for authorized Sunny Optical employees 
* to design, create, modify, or review.
* 
* DO NOT DISTRIBUTE, DO NOT DUPLICATE OR TRANSMIT IN ANY FORM WITHOUT PROPER AUTHORIZATION.
* 
* If you are not an intended recipient of this file, you must not copy, distribute, modify, 
* or take any action in reliance on it. 
* 
* If you have received this file in error, please immediately notify Sunny Optical and 
* permanently delete the original and any copy of any file and any printout thereof.
*
*-------------------------------------------------------------------------------------------------*/
/*
* 
* Author:
* Chunyang Zhang (zngzhangcy@sunnyoptical.com)
* History:
* 23-05-18 create
*/

#ifdef USE_RK_MPP

#include "rk3588_decode.h"
#include <rga/RgaApi.h>

mppDecode::mppDecode()
{
}

mppDecode::~mppDecode()
{
}

int mppDecode::init(int width, int height)
{
    m_width = width;
    m_height = height;

	MPP_RET ret = MPP_OK;
	MpiCmd mpi_cmd = MPP_CMD_BASE;
    MppParam param = NULL;
	
	ret = mpp_create(&mCtx, &mApi);
    if (ret != MPP_OK) 
	{
		MPP_ERR("mpp_create erron (%d) \n", ret);
        return ret;
    }

    ret = mApi->control(mCtx, MPP_SET_DISABLE_THREAD, NULL);

   #ifdef DECO_H264
	ret = mpp_init(mCtx, MPP_CTX_DEC, MppCodingType::MPP_VIDEO_CodingAVC);//MODIFIED BY SHL 20231018
	if (MPP_OK != ret) 
	{
		MPP_ERR("mpp_init erron (%d) \n", ret);
        return ret;
	}
	#else
	ret = mpp_init(mCtx, MPP_CTX_DEC, MppCodingType::MPP_VIDEO_CodingMJPEG);
	// MppFrameFormat frmType =MPP_FMT_RGB888;//IN_FRAME;//MPP_FMT_YUV420SP;//
	// param = &frmType;
	// ret = mApi->control(mCtx, MPP_DEC_SET_OUTPUT_FORMAT, param);
	if (MPP_OK != ret) 
	{
		MPP_ERR("mApi->control(mCtx, MPP_DEC_SET_OUTPUT_FORMAT, param) erron (%d) \n", ret);
        return ret;
	}
	#endif
	
    mpp_dec_cfg_init(&cfg);

    /* get default config from decoder context */
    ret = mApi->control(mCtx, MPP_DEC_GET_CFG, cfg);
    if (ret) {
        MPP_ERR("%p failed to get decoder cfg ret %d\n", mCtx, ret);
        return ret;
    }

    /*
     * split_parse is to enable mpp internal frame spliter when the input
     * packet is not aplited into frames.
     */
    ret = mpp_dec_cfg_set_u32(cfg, "base:split_parse", need_split);
    if (ret) {
        MPP_ERR("%p failed to set split_parse ret %d\n", mCtx, ret);
        return ret;
    }

    ret = mApi->control(mCtx, MPP_DEC_SET_CFG, cfg);
    if (ret) {
        MPP_ERR("%p failed to set cfg %p ret %d\n", mCtx, cfg, ret);
        return ret;
    }

    rgb_data = (RK_U8 *)malloc(m_width*m_height*3);
    if(rgb_data == NULL)
    {
    	MPP_ERR("failed to get memery ");
        return -1;
    }
	frame_cnt=0;
	return MPP_OK;
}

int mppDecode::init_packet_and_frame(void)
{
	#ifdef DECO_H264
	MPP_DBG("decode H264\n");
	int ret;

	ret = mpp_packet_init(&packet, NULL, 0);
	//MPP_ERR("mpp_packet_init get %p\n", packet);
	if (ret) {
		MPP_ERR("mpp_packet_init failed\n");
		return -1;
	}
	return 0;

	#else 
	MPP_DBG("decode MJPEG\n");
	RK_U32 hor_stride = MPP_ALIGN(m_width, 16);
    RK_U32 ver_stride = MPP_ALIGN(m_height, 16);
    
	int ret;
	ret = mpp_buffer_group_get_internal(&frmGrp, MPP_BUFFER_TYPE_ION);
	if(ret)
	{
		MPP_ERR("frmGrp mpp_buffer_group_get_internal erron (%d)\r\n",ret);
		return -1;
	}

	ret = mpp_buffer_group_get_internal(&pktGrp, MPP_BUFFER_TYPE_ION);
	if(ret)
	{
		MPP_ERR("frmGrp mpp_buffer_group_get_internal erron (%d)\r\n",ret);
		return -1;
	}

	ret = mpp_frame_init(&frame); /* output frame */
    if (MPP_OK != ret) 
	{
        MPP_ERR("mpp_frame_init failed erron (%d)\r\n",ret);
        return -1;
    }

	ret = mpp_buffer_get(frmGrp, &frmBuf, hor_stride * ver_stride * 4);
    if (ret) 
	{
        MPP_ERR("frmGrp mpp_buffer_get erron (%d) \n", ret);
        return -1;
    }

	ret = mpp_buffer_get(pktGrp, &pktBuf, m_width * m_height * 2);
    if (ret) 
	{
        MPP_ERR("pktGrp mpp_buffer_get erron (%d) \n", ret);
        return -1;
    }

	mpp_packet_init_with_buffer(&packet, pktBuf);
	dataBuf = (char *)mpp_buffer_get_ptr(pktBuf);
	mpp_frame_set_buffer(frame, frmBuf);
	MPP_DBG("init_packet_and_frame OK\n");

    return 0;
	#endif
}

int mppDecode::uninit()
{
	if (packet) 
	{
        mpp_packet_deinit(&packet);
        packet = NULL;
    }

	if (frame) 
	{
        mpp_frame_deinit(&frame);
        frame = NULL;
    }

	if (mCtx) 
	{
        mpp_destroy(mCtx);
        mCtx = NULL;
    }

	if (pktBuf) 
	{
        mpp_buffer_put(pktBuf);
        pktBuf = NULL;
    }

    if (frmBuf) 
	{
        mpp_buffer_put(frmBuf);
        frmBuf = NULL;
    }

	if (pktGrp) {
        mpp_buffer_group_put(pktGrp);
        pktGrp = NULL;
    }

    if (frmGrp) {
        mpp_buffer_group_put(frmGrp);
        frmGrp = NULL;
    }
	if(rgb_data)
	{
		free(rgb_data);
		rgb_data = NULL;
	}
	return 0;
}

int mppDecode::decode(char* &srcFrm, size_t srcLen)
{
#ifdef DECO_H264

//MPP_DBG("RECV: %d, %02x %02x %02x %02x %02x %02x %02x %02x\n",\
	srcLen,srcFrm[0],srcFrm[1],srcFrm[2],srcFrm[3],srcFrm[4],srcFrm[5],srcFrm[6],srcFrm[7]);
	RK_U32 pkt_done = 0;
    RK_U32 pkt_eos  = 0;
    MPP_RET ret = MPP_OK; 
	RK_S32 times = 15;

    mpp_packet_set_data(packet, srcFrm);
    mpp_packet_set_size(packet, srcLen);
    mpp_packet_set_pos(packet, srcFrm);
    mpp_packet_set_length(packet, srcLen);

    do {
        //RK_U32 frm_eos = 0;
        RK_S32 get_frm = 0;
        MppFrame frame = NULL;

        // send the packet first if packet is not done
		//MPP_DBG("DECODE START\n");
        ret = mApi->decode(mCtx, packet, &frame);
        if (ret)
            MPP_ERR("decode failed ret %d\n", ret);
		//MPP_DBG("DECODE end\n");
		if(frame == NULL && ret == MPP_OK)
		{
			if (times > 0) {
				times--;
				usleep(2000);		
				if(times == 0)
				{
					//MPP_ERR("%p decode 15 times but failed,discard one frame\n",mCtx);
					break;
				}
				continue;
			}	
		}
        // then get all available frame and release
        if (frame) {
            if (mpp_frame_get_info_change(frame)) {
                RK_U32 width = mpp_frame_get_width(frame);
                RK_U32 height = mpp_frame_get_height(frame);
                RK_U32 hor_stride = mpp_frame_get_hor_stride(frame);
                RK_U32 ver_stride = mpp_frame_get_ver_stride(frame);
                RK_U32 buf_size = mpp_frame_get_buf_size(frame);

                MPP_DBG("%p decode_get_frame get info changed found\n", mCtx);
                MPP_DBG("%p decoder require buffer w:h [%d:%d] stride [%d:%d] buf_size %d\n",
                          mCtx, width, height, hor_stride, ver_stride, buf_size);

                if (NULL == frmGrp) {
                    /* If buffer group is not set create one and limit it */
                    ret = mpp_buffer_group_get_internal(&frmGrp, MPP_BUFFER_TYPE_ION);
                    if (ret) {
                        MPP_ERR("%p get mpp buffer group failed ret %d\n", mCtx, ret);
                        break;
                    }

                    /* Set buffer to mpp decoder */
                    ret = mApi->control(mCtx, MPP_DEC_SET_EXT_BUF_GROUP, frmGrp);
                    if (ret) {
                        MPP_ERR("%p set buffer group failed ret %d\n", mCtx, ret);
                        break;
                    }
                } else {
                    /* If old buffer group exist clear it */
                    ret = mpp_buffer_group_clear(frmGrp);
                    if (ret) {
                        MPP_ERR("%p clear buffer group failed ret %d\n", mCtx, ret);
                        break;
                    }
                }

                /* Use limit config to limit buffer count to 24 with buf_size */
                ret = mpp_buffer_group_limit_config(frmGrp, buf_size, 24);
                if (ret) {
                    MPP_ERR("%p limit buffer group failed ret %d\n", mCtx, ret);
                    break;
                }

                /*
                 * All buffer group config done. Set info change ready to let
                 * decoder continue decoding
                 */
                ret = mApi->control(mCtx, MPP_DEC_SET_INFO_CHANGE_READY, NULL);
                if (ret) {
                    MPP_ERR("%p info change ready failed ret %d\n", mCtx, ret);
                    break;
                }
            } 
			else 
			{	
				char log_buf[256];
                RK_S32 log_size = sizeof(log_buf) - 1;
                RK_S32 log_len = 0;
                RK_U32 err_info = mpp_frame_get_errinfo(frame);
                RK_U32 discard = mpp_frame_get_discard(frame);

                //log_len += snprintf(log_buf + log_len, log_size - log_len,\
                                    "decode get frame %d", frame_cnt);

                if (mpp_frame_has_meta(frame)) {
                    MppMeta meta = mpp_frame_get_meta(frame);
                    RK_S32 temporal_id = 0;

                    mpp_meta_get_s32(meta, KEY_TEMPORAL_ID, &temporal_id);

                 //   log_len += snprintf(log_buf + log_len, log_size - log_len,\
                                        " tid %d", temporal_id);
                }

                if (err_info || discard) { 
					mpp_frame_deinit(&frame);
					err_times++;
					if(err_times > 15)
					{
						err_times = 0;
						MPP_ERR("%p err to discard one frame, err %x discard %x \n", mCtx,err_info, discard);
						break;
					}
					usleep(1000);	
					continue;
                }
                //MPP_DBG("%p %s\n", mCtx, log_buf);

                frame_cnt++;
				rga_frame_yuv_to_rgb(frame);
            }
			err_times = 0;
			//frm_eos = mpp_frame_get_eos(frame);//no frm_eos in hsp004
			mpp_frame_deinit(&frame);
			get_frm = 1;
        }
	
        // try get runtime frame memory usage
		if (frmGrp) {
			size_t usage = mpp_buffer_group_usage(frmGrp);
		}
		if (get_frm)
		{
			break;
		}
        /*
         * why sleep here:
         * mpi->decode_put_packet will failed when packet in internal queue is
         * full,waiting the package is consumed .Usually hardware decode one
         * frame which resolution is 1080p needs 2 ms,so here we sleep 1ms
         * * is enough.
         */
        usleep(1000);
    } while (1);

    return ret;
#else
	MppTask task = NULL;
	int ret;
	int pktEos = 0;
	
	memset(dataBuf, 0, sizeof(dataBuf));
	memcpy(dataBuf, srcFrm, srcLen);
	mpp_packet_set_data(packet, dataBuf);
    mpp_packet_set_size(packet, srcLen);
	mpp_packet_set_pos(packet, dataBuf);
    mpp_packet_set_length(packet, srcLen);
	pktEos = mpp_packet_get_eos(packet);
	//MPP_DBG("pktEos = %d\n",pktEos);
	if(pktEos)
	{
		mpp_packet_set_eos(packet);
	}
	//MPP_DBG("start mApi->poll(mCtx, MPP_PORT_INPUT, MPP_POLL_BLOCK)\n");
	ret = mApi->poll(mCtx, MPP_PORT_INPUT, MPP_POLL_BLOCK);
    if (ret) 
	{
        MPP_ERR("mpp input poll failed %d\n",ret);
        return ret;
    }
	//MPP_DBG("start mApi->dequeue\n");
	ret = mApi->dequeue(mCtx, MPP_PORT_INPUT, &task);  /* input queue */
    if (ret) 
	{
        MPP_ERR("mpp task input dequeue failed %d\n",ret);
        return ret;
    }
	mpp_task_meta_set_packet(task, KEY_INPUT_PACKET, packet);
    mpp_task_meta_set_frame (task, KEY_OUTPUT_FRAME,  frame);
	//MPP_DBG("start mApi->enqueue\n");

	ret = mApi->enqueue(mCtx, MPP_PORT_INPUT, task);  /* input queue */
    if (ret) 
	{
        MPP_ERR("mpp task input enqueue failed %d\n",ret);
        return ret;
    }
	//MPP_DBG("start mApi->poll(mCtx, MPP_PORT_OUTPUT, MPP_POLL_BLOCK)\n");
	/* poll and wait here */
    ret = mApi->poll(mCtx, MPP_PORT_OUTPUT,MPP_POLL_BLOCK );// MPP_POLL_BLOCK
    if (ret) 
	{
        MPP_ERR("mpp output poll failed %d\n",ret);
        return ret;
    }
	//MPP_DBG("start mApi->dequeue(mCtx, MPP_PORT_OUTPUT, &task)\n");

	ret = mApi->dequeue(mCtx, MPP_PORT_OUTPUT, &task); /* output queue */
    if (ret) 
	{
        MPP_ERR("mpp task output dequeue failed %d\n",ret);
        //return ret;
    }

	if (task)
	{
		MppFrame frameOut = NULL;
		mpp_task_meta_get_frame(task, KEY_OUTPUT_FRAME, &frameOut);
		if (frame) 
		{
			//MPP_DBG("ok\n");
			rga_frame_yuv_to_rgb(frame);

            if (mpp_frame_get_eos(frameOut))
            {
				MPP_DBG("found eos frame\n");
			}
        }
		ret = mApi->enqueue(mCtx, MPP_PORT_OUTPUT, task);
        if (ret)
        {
			MPP_ERR("mpp task output enqueue failed\n");
		}
	}
	return ret;
	#endif
}

void mppDecode::rga_frame_yuv_to_rgb(MppFrame &frame)
{
	RK_U32 width    = 0;
    RK_U32 height   = 0;
    RK_U32 h_stride = 0;
    RK_U32 v_stride = 0;
    MppFrameFormat fmt  = MPP_FMT_RGB888;//MPP_FMT_YUV420SP;
    MppBuffer buffer    = NULL;
    RK_U8 *base = NULL;
    
    width    = mpp_frame_get_width(frame);
    height   = mpp_frame_get_height(frame);
    h_stride = mpp_frame_get_hor_stride(frame);
    v_stride = mpp_frame_get_ver_stride(frame);
    fmt      = mpp_frame_get_fmt(frame);
    buffer   = mpp_frame_get_buffer(frame);

    if (NULL == buffer){
		MPP_ERR("NULL == buffer\n");
        return ;
    }

    RK_U32 buf_size1 = mpp_frame_get_buf_size(frame);
    //printf("hor_stride:%d ver_stride:%d fmt:%d buf_size:%d\n", h_stride, v_stride, fmt, buf_size1);

    base = (RK_U8 *)mpp_buffer_get_ptr(buffer);

    // MPP_FMT_YUV420SP
    if (fmt != MPP_FMT_YUV420SP) {//
        printf("fmt %d not supported\n", fmt);
        return;
    }

	rga_info_t src_info = {0};
  	rga_info_t dst_info = {0};
	memset(rgb_data, 0, width * height * 3);
	src_info.fd = -1;
	src_info.mmuFlag = 1;
	src_info.virAddr = base;
	src_info.format = RK_FORMAT_YCbCr_420_SP;
	dst_info.fd = -1;
	dst_info.mmuFlag = 1;
	dst_info.virAddr = rgb_data;
	dst_info.format = RK_FORMAT_RGB_888;
	rga_set_rect(&src_info.rect, 0, 0, width, height, width, height, RK_FORMAT_YCbCr_420_SP);
	rga_set_rect(&dst_info.rect, 0, 0, width, height, width, height, RK_FORMAT_RGB_888);
	int ret = c_RkRgaBlit(&src_info, &dst_info, NULL);
	if (ret) {
		printf("c_RkRgaBlit error %d\n" , ret);
		return;
	}
	m_frame = cv::Mat(m_height, m_width, CV_8UC3, (void *)rgb_data);
}

cv::Mat mppDecode::getFrame(void)
{
    return m_frame;
}

HRK3588DEC rk3588_decode_init(int width, int height)
{
	mppDecode* mppDec = new mppDecode;

	int ret = mppDec->init(width, height);
	ret = mppDec->init_packet_and_frame();

	if(ret != 0){
		return NULL;
	}

	return mppDec;
}

void rk3588_decode_uninit(HRK3588DEC &hDec)
{
	if(NULL == hDec)
	{
		return;
	}
	mppDecode* mppDec = (mppDecode*)hDec;
	if(mppDec != NULL){
		delete mppDec;
	}

	return ;	
}

unsigned int rk3588_decode_do(HRK3588DEC &hDec, unsigned char* &pInData, const unsigned int nDataLen)
{
	if((NULL == hDec) || (NULL == pInData) || (0 >= nDataLen))
	{
		return -1;
	}
	mppDecode* mppDec = (mppDecode*)hDec;
    char *tmp = (char *)pInData;
	mppDec->decode(tmp, nDataLen);

	return 0;
}

cv::Mat rk3588_decode_getFrame(HRK3588DEC &hDec)
{
	if(NULL == hDec)
    {
		cv::Mat ret;
        return ret;
    }

	mppDecode* mppDec = (mppDecode*)hDec;

    return mppDec->getFrame();
}

#endif