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

#ifndef _SUNNY_RK3588_DECODE_H_
#define _SUNNY_RK3588_DECODE_H_

#ifdef USE_RK_MPP
#include <rockchip/mpp_buffer.h>
#include <rockchip/mpp_err.h>
#include <rockchip/mpp_frame.h>
#include <rockchip/mpp_meta.h>
#include <rockchip/mpp_packet.h>
#include <rockchip/mpp_task.h>
#include <rockchip/rk_mpi.h>
#include <rockchip/rk_mpi_cmd.h>
#include <rockchip/rk_type.h>
#include <rockchip/vpu.h>
#include <rockchip/vpu_api.h>

#include <opencv2/opencv.hpp>
// #include <rga/RgaUtils.h>
// #include <rga/rga.h>
// #include "rga.h"
// #include "RockchipRga.h"
// #include "RgaUtils.h"

#define DECO_H264 1

class mppDecode {

public:
    mppDecode();
    ~mppDecode();

    int init(int width, int height);
    int uninit();

    int init_packet_and_frame(void);
    int decode(char *&srcFrm, size_t srcLen);
    cv::Mat getFrame(void);

private:
//宏定义
#define MPP_ALIGN(x, a) (((x) + (a) - 1) & ~((a) - 1))
#define IN_FRAME MPP_FMT_YUV420SP
#define ESC_START "\033["
#define ESC_END "\033[0m"
#define COLOR_GREEN "32;40;1m"
#define COLOR_RED "31;40;1m"
#define MPP_DBG(format, args...)                                                                                       \
    (printf(ESC_START COLOR_GREEN "[MPP DBG]-[%s]-[%05d]:" format ESC_END, __FUNCTION__, (int) __LINE__, ##args))
#define MPP_ERR(format, args...)                                                                                       \
    (printf(ESC_START COLOR_RED "[MPP ERR]-[%s]-[%05d]:" format ESC_END, __FUNCTION__, (int) __LINE__, ##args))

private:
    void rga_frame_yuv_to_rgb(MppFrame &frame);

private:
    MppBufferGroup frmGrp = NULL;
    MppBufferGroup pktGrp = NULL;

    MppPacket packet = NULL;
    MppFrame frame = NULL;
    size_t packetSize;

    MppBuffer frmBuf = NULL;
    MppBuffer pktBuf = NULL;

    char *dataBuf = NULL;
    MppCtx mCtx = NULL;
    MppApi *mApi = NULL;
    // config for runtime mode
    MppDecCfg cfg = NULL;
    RK_U32 need_split = 0;

    int buf_size = 0;

    int m_width;
    int m_height;
    cv::Mat m_frame;

    int frame_cnt;
    int err_times = 0;
    int fps;//
    RK_U8 *rgb_data;
};

typedef void *HRK3588DEC;

HRK3588DEC rk3588_decode_init(int width, int height);
void rk3588_decode_uninit(HRK3588DEC &hDec);
unsigned int rk3588_decode_do(HRK3588DEC &hDec, unsigned char *&pInData, const unsigned int nDataLen);
cv::Mat rk3588_decode_getFrame(HRK3588DEC &hDec);
#endif

#endif
