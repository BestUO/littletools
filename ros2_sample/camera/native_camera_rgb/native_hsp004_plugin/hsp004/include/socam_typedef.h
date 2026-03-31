/********************************************************************************************************
 *
 *  @note   : ©2023 Zhejiang Sunny Intelligent Optical Technology Co., Ltd. All Rights Reserved
 *  @file   : socam_typedef.h
 *  @brief  : UVC sdk Profiler Definition
 *  @author : zngshenbo <zngshenbo@sunnyoptical.com>
 *  @date   : createAt 2023-05-22
 *
 *******************************************************************************************************/
#ifndef _SOCAM_TYPEDEF_H_
#define _SOCAM_TYPEDEF_H_

#ifdef WIN32
#include <string>
#else
#include <cstring>
#endif
//#define UVC_DEV_MAX_STREAM_CHAN (6) //设备取流通道最大个数
#define SUNNY_DEV_MAX_CNT (32)//最多支持的设备个数

typedef enum tagSOERR
{
    SOERR_NONE                   = 0,
    SOERR_FAILED                 = -1,
    SOERR_UNKNOWN                = -2,
    SOERR_NO_CAM_DEVICE          = -100,
    SOERR_INITED                 = -101,
    SOERR_INVALID_PARAM          = -102,
    SOERR_WRONG_STATUS           = -104,
    SOERR_FAILED_CHNAGE_WIN_ID   = -105,
    SOERR_FAILED_LIST_RES        = -107,
    SOERR_UNINITED               = -108,
    SOERR_NOT_SUPPORTED          = -109,
    SOERR_NO_MEMEORY             = -110,
    SOERR_FAILED_INIT_ELEMENT    = -121,
    SOERR_FAILED_LINK_ELEMENT    = -122,
    SOERR_FAILED_QUERY_INTERFACE = -200,
    SOERR_FAILED_API_INTERFACE   = -201,
    SOERR_FAILED_WR              = -202,
    SOERR_FAILED_IOCTL           = -203
} SOERR;

#define MAKE_UNIQUE_ID(major, sub, a, b) ((major<<24) | (sub<<16) | (a<<8) | (b))


typedef enum tagSOCAM_UVC_CORE_TYPE
{
    SOCAM_UVC_CORE_DSHOW  = 0,//directshow方式实现uvc
    SOCAM_UVC_CORE_V4L2   = 1,//v4l2方式实现uvc
    SOCAM_UVC_CORE_LIBUVC = 2,//libuvc方式实现uvc
} SOCAM_UVC_CORE_TYPE;


typedef enum tagSOCAM_STREAM_FMT
{
    SOCAM_STREAM_FMT_YUY2 = 0x32595559,//
    SOCAM_STREAM_FMT_MJPG = 0x47504a4d,//
    SOCAM_STREAM_FMT_H264 = 0x34363248,//
    SOCAM_STREAM_FMT_YUYV = 0x56595559,//
    SOCAM_STREAM_FMT_NV12 = 0x3231564e,//
} SOCAM_STREAM_FMT;


typedef struct tagCamRes
{
    int  width;
    int  height;
    int  frameRate; //取值见SOCAM_FRAMERATE
    unsigned int format; //取值见SOCAM_STREAM_FMT
} CamRes;

typedef struct tagCamResList
{
    CamRes *cam_res;
    int num_res;
} CamResList;

typedef enum tagSOCAM_FRAMERATE
{
    SOCAM_FRAMERATE_5 = 5,
    SOCAM_FRAMERATE_10 = 10,
    SOCAM_FRAMERATE_15 = 15,
    SOCAM_FRAMERATE_20 = 20,
    SOCAM_FRAMERATE_25 = 25,
    SOCAM_FRAMERATE_30 = 30,
    SOCAM_FRAMERATE_35 = 35,
    SOCAM_FRAMERATE_40 = 40,
    SOCAM_FRAMERATE_45 = 45,
    SOCAM_FRAMERATE_60 = 60,
} SOCAM_FRAMERATE;


typedef void *HCAM;

#endif