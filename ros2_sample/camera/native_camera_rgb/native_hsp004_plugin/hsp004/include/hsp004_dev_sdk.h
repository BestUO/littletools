#ifndef __HSP004_DEV_SDK__
#define __HSP004_DEV_SDK__

#include "socam_typedef.h"

#ifdef WIN32
#ifdef SUNNY_DEVICE_SDK_EXPORT
#define SUNNYDDLL __declspec(dllexport)
#else
#define SUNNYDDLL __declspec(dllimport)
#endif

#define CALL __cdecl
#else
#define SUNNYDDLL __attribute__((visibility("default")))
#define CALL
#endif

#define STREAM_FMT_MJPG 0x47504a4d
#define STREAM_FRAMERATE_30 30
#define UVC_STREAM_WIDTH 1920
#define UVC_STREAM_HEIGHT 1080
#define DEV_PATH_SIZE 48     //烧写文件存放的设备地址的最大长度
#define MAX_PATH_LEN 256     //文件路径最大长度
#define OTA_FILE_NAME_SIZE 40//OTA文件名最大长度

//OTA errorcode
#define SUNNY_OTA_ING -3                 //固件升级进行中
#define SUNNY_OTA_OVER_BUTNOTREBOOT -4   //固件升级完成但未重启
#define SUNNY_ERROR_FILENAME_TOO_LONG -5 //文件名过长(<40)
#define SUNNY_RSP_FILE_NOT_EXIST -6      //文件不存在
#define SUNNY_ERROR_CMD_SEND_FAILED -7   //命令发送失败
#define SUNNY_ERROR_CMD_RECV_FAILED -8   //应答接收失败
#define SUNNY_RSP_CHECKSUM_ERROR -9      //校验码错误
#define SUNNY_RSP_DATA_LEN_ERROR -10     //数据长度错误
#define SUNNY_RSP_UNSUPPORT_CMD -11      //不支持的命令
#define SUNNY_RSP_MALLOC_FAILED -12      //申请内存失败
#define SUNNY_RSP_PATH_NOT_EXIST -13     //路径不存在
#define SUNNY_RSP_FILE_TOO_LARGE -14     //文件太大
#define SUNNY_RSP_FILE_CRC_ERROR -15     //文件CRC校验失败
#define SUNNY_RSP_FILE_WRITE_FAILED -16  //文件写入失败
#define SUNNY_ERROR_DATA_LEN_TOO_LONG -17//发送数据长度过长
#define SUNNY_PARAM_ERROR -18            //下发参数错误
#define SUNNY_RDWR_FILE_ERROR -19        //读写文件错误
#define SUNNY_EXTENDED_FAILED -20        //扩展通道失败
#define SUNNY_OTA_TIMEOUT -21            //固件OTA升级超时
#define SUNNY_OTA_FAILED -22             //固件OTA升级失败
//OTA success code
#define SUNNY_RET_SUCCESS_UPDATE 100//更新固件成功


typedef struct tagVCUnitInfo {
    int max_value;
    int min_value;
    int default_value;
    int step_value;
    bool bSupAuto;
} stVCUnitInfo;

typedef struct tagVCUnitValue {
    int nValue;
    bool bAuto;
} stVCUnitValue;

typedef struct tagStereoCameraParam {
    float Cx;
    float Cy;
    float Fx;
    float Fy;
    float K1;
    float K2;
    float K3;
    float P1;
    float P2;
} stStereoCameraParam;

typedef struct tagFishEyeCameraParam {
    float Cx;
    float Cy;
    float Fx;
    float Fy;
    float K1;
    float K2;
    float K3;
    float K4;
} stFishEyeCameraParam;

typedef enum tagDEV_STATUS {
    DEV_STATUS_DEV_BROKEN = MAKE_UNIQUE_ID('D', 'E', 'V', 'B'),//设备异常断开
} SUNNYDEV_STATUS;

typedef struct tagSunnyDevStatus {
    SUNNYDEV_STATUS nStatus;
} stSunnyDevStatus;

typedef struct tagSunnyDeviceDescriptor {
    void *hDevice;
} SunnyDeviceDescriptor;

typedef void (*FNSuunyDeviceStatus)(stSunnyDevStatus tofDevStatus, void *pUserData);

typedef void (*FNStreamCallback)(void *buffer, int buf_len, void *user_data);

typedef void (*FNOTAStatus)(const int nValue, int ret, void *pUserData);
//固件升级数据
typedef struct tagFirmwareUpgradeData {
    char szHostFilePath[MAX_PATH_LEN];//本地文件路径
    FNOTAStatus fnUpgradeStatus;      //固件升级实时状态回调函数
    void *pUserData;
    tagFirmwareUpgradeData() {
        memset(this->szHostFilePath, 0, MAX_PATH_LEN);
        this->fnUpgradeStatus = NULL;
        this->pUserData = NULL;
    }
} FirmwareUpgradeData;

#ifdef __cplusplus
extern "C" {
#endif
/*************************************************************
Function: Hsp004_SearchDevice
Description: search camera
Input:
Output:
pDevsDesc - device descriptor list ptr
pDevNum - device number ptr
Return:
0 - succuss     other - failed
Others:
*************************************************************/
SUNNYDDLL int CALL Hsp004_SearchDevice(SunnyDeviceDescriptor **pDevsDesc, int *pDevNum);

/*************************************************************
Function: Hsp004_Open
Description: open camera
Input:
pDevList - device descriptor
fnDevStatus - device status callback function
pUserData - user ptr
Output:
hCamDev - device handle
Return:
0 - succuss     other - failed
Others:
*************************************************************/
SUNNYDDLL int CALL Hsp004_Open(HCAM *hCamDev, SunnyDeviceDescriptor *pDevList, FNSuunyDeviceStatus fnDevStatus,
                               void *pUserData);

/*************************************************************
Function: Hsp004_OpenWithFd
Description: open camera with file description (only for andorid)
Input:
fd - file description
vid - vendor id
pid - product id
busnum - serial number
devaddr - devNumber
usbfs - USBFSName
Output:
hCamDev - device handle
Return:
0 - succuss     other - failed
Others:
*************************************************************/
#ifdef __ANDROID__
SUNNYDDLL int CALL Hsp004_OpenWithFd(HCAM *hCamDev, int vid, int pid, int fd, int busnum, int devaddr,
                                     const char *usbfs);
#endif


/*************************************************************
Function: Hsp004_StartStream
Description: start stream
Input:
hCamDev - device handle
fnCallback - frame callback function
pUserData - user ptr
width - stream width
height - stream height
fps - stream fps
fomat - stream fomat
Return:
0 - succuss     other - failed
Others:
*************************************************************/
SUNNYDDLL int CALL Hsp004_StartStream(HCAM hCamDev, FNStreamCallback fnCallback, void *pUserData,
                                      int width = UVC_STREAM_WIDTH, int height = UVC_STREAM_HEIGHT,
                                      int fps = STREAM_FRAMERATE_30, unsigned int fomat = STREAM_FMT_MJPG);

/*************************************************************
Function: Hsp004_StopStream
Description: stop stream
Input:
hCamDev - device handle
Return:
0 - succuss     other - failed
Others:
*************************************************************/
SUNNYDDLL int CALL Hsp004_StopStream(HCAM hCamDev);

/*************************************************************
Function: Hsp004_Close
Description: close device
Input:
hCamDev - device handle
Return:
0 - succuss     other - failed
Others:
*************************************************************/
SUNNYDDLL int CALL Hsp004_Close(HCAM hCamDev);

/*************************************************************
Function: Hsp004_GetCameraSDKVersion
Description: get sdk version
Output:
szSDKVer - sdk version, len = 128
Return:
0 - succuss     other - failed
Others:
*************************************************************/
SUNNYDDLL int CALL Hsp004_GetCameraSDKVersion(char *szSDKVer);

/*************************************************************
Function: Hsp004_Query_Camcaps
Description: Get the camera capability set
Input:
hCamDev - device handle
nCapSize - pCamCap's size
Output:
pCamCap - capability set, contains stream width,stream height,stream fps,stream fomat
nCapCount - set count
Return:
0 - succuss     other - failed
Others:
*************************************************************/
SUNNYDDLL int CALL Hsp004_Query_Camcaps(HCAM hCamDev, CamRes *pCamCap, int nCapSize, int *nCapCount);

/*************************************************************
Function: Hsp004_Vc_Ext_unit_Transfer
Description: Extended channel transmission
Input:
hCamDev - device handle
dwCS - unit id
nBufLen - data length, must be 60
nTransferType - 0:send  1:receive
Input/Output:
pData - transfer data, size=nBufLen, must be 60
pdwRecvLen - receive data's length, if nTransferType == 1, this is effective
Return:
0 - succuss     other - failed
Others:
*************************************************************/
SUNNYDDLL int CALL Hsp004_Vc_Ext_unit_Transfer(HCAM hCamDev, int dwCS, void *pData, int nBufLen, int nTransferType,
                                               unsigned int *pdwRecvLen = NULL);

/*************************************************************
Function: Hsp004_Get_FirmVer
Description: get camera firmware version
Input:
hCamDev - device handle
Output:
szVer - firmware version, length 60
Return:
0 - succuss     other - failed
Others:
*************************************************************/
SUNNYDDLL int CALL Hsp004_Get_FirmVer(HCAM hCamDev, char *szVer);

/*************************************************************
Function: Hsp004_Set_SysTime
Description: set camera system time
Input:
hCamDev - device handle
tv_sec - unix timestamp + current timezone * 60 * 60
tv_usec - The remaining microseconds
Return:
0 - succuss     other - failed
Others:
*************************************************************/
SUNNYDDLL int CALL Hsp004_Set_SysTime(HCAM hCamDev, int tv_sec, int tv_usec);

/*************************************************************
Function: Hsp004_GetDevUUID
Description: get camera UUID
Input:
hCamDev - device handle
tv_sec - unix timestamp + current timezone * 60 * 60
tv_usec - The remaining microseconds
Return:
0 - succuss     other - failed
Others:
*************************************************************/
SUNNYDDLL int CALL Hsp004_GetDevUUID(HCAM hCamDev, unsigned long long *pUUid);

/*************************************************************
Function: Hsp004_Switch_OSD
Description: enable/disable video OSD
Input:
hCamDev - device handle
enable - 0:disable   1:enable
Return:
0 - succuss     other - failed
Others:
*************************************************************/
SUNNYDDLL int CALL Hsp004_Switch_OSD(HCAM hCamDev, int enable);

/*************************************************************
Function: Hsp004_Upgrade_Firware
Description: firmware upgrade
Input:
hCamDev - device handle
pUpgradeData - upgrade info
Return:
0 - succuss     other - failed
Others:
*************************************************************/
SUNNYDDLL int CALL Hsp004_Upgrade_Firware(HCAM hCamDev, FirmwareUpgradeData *pUpgradeData);

/*************************************************************
Function: Hsp004_Upgrade_Firware
Description: firmware upgrade
Input:
hCamDev - device handle
pUpgradeData - upgrade info
Return:
0 - succuss     other - failed
Others:
*************************************************************/
SUNNYDDLL int CALL Hsp004_HardResetDevice(HCAM hCamDev);

/*************************************************************
Function: Hsp004_CameraCalibParamTransfer
Description: set or get camera calibrate param
Input:
hCamDev - device handle
nCmd - 0(set) 1(get)
pCameraParam - calibrate param
Return:
0 - succuss     other - failed
Others:
*************************************************************/
SUNNYDDLL int CALL Hsp004_CameraCalibParamTransfer(HCAM hCamDev, int nCmd, stStereoCameraParam *pCameraParam);

/*************************************************************
Function: Hsp004_CameraCalibParamTransfer
Description: set or get fisheye camera calibrate param
Input:
hCamDev - device handle
nCmd - 0(set) 1(get)
pCameraParam - fisheye calibrate param
Return:
0 - succuss     other - failed
Others:
*************************************************************/
SUNNYDDLL int CALL Hsp004_FishEyeCameraCalibParamTransfer(HCAM hCamDev, int nCmd, stFishEyeCameraParam *pCameraParam);

/*************************************************************
Function: Hsp004_ReadSerialNumber
Description: read device's serial number
Input:
hCamDev - device handle
Output:
szSerialNumber - device serial number, length = 60
Return:
0 - succuss     other - failed
Others:
*************************************************************/
SUNNYDDLL int CALL Hsp004_ReadSerialNumber(HCAM hCamDev, char *szSerialNumber);

/*************************************************************
Function: Hsp004_WriteSerialNumber
Description: write device's serial number
Input:
hCamDev - device handle
szSerialNumber - device serial number, length = 60
Return:
0 - succuss     other - failed
Others:
*************************************************************/
SUNNYDDLL int CALL Hsp004_WriteSerialNumber(HCAM hCamDev, char *szSerialNumber);

/*************************************************************
Function: Hsp004_QueryBrightnessUnitInfo
Description: query brightness unit info
Input:
hCamDev - device handle
vcUnitInfo - vc init info
Return:
0 - succuss     other - failed
Others:
*************************************************************/
SUNNYDDLL int CALL Hsp004_QueryBrightnessUnitInfo(HCAM hCamDev, stVCUnitInfo *vcUnitInfo);

/*************************************************************
Function: Hsp004_BrightnessUnitTransfer
Description: set or get brightness value
Input:
hCamDev - device handle
nCmd - 0(set) 1(get)
vcUnitValue - vc init value
Return:
0 - succuss     other - failed
Others:
*************************************************************/
SUNNYDDLL int CALL Hsp004_BrightnessUnitTransfer(HCAM hCamDev, int nCmd, stVCUnitValue *vcUnitValue);

/*************************************************************
Function: Hsp004_QueryConstrastUnitInfo
Description: query constrast unit info
Input:
hCamDev - device handle
vcUnitInfo - vc init info
Return:
0 - succuss     other - failed
Others:
*************************************************************/
SUNNYDDLL int CALL Hsp004_QueryConstrastUnitInfo(HCAM hCamDev, stVCUnitInfo *vcUnitInfo);

/*************************************************************
Function: Hsp004_ConstrastUnitTransfer
Description: set or get constrast value
Input:
hCamDev - device handle
nCmd - 0(set) 1(get)
vcUnitValue - vc init value
Return:
0 - succuss     other - failed
Others:
*************************************************************/
SUNNYDDLL int CALL Hsp004_ConstrastUnitTransfer(HCAM hCamDev, int nCmd, stVCUnitValue *vcUnitValue);

/*************************************************************
Function: Hsp004_QuerySaturationUnitInfo
Description: query saturation unit info
Input:
hCamDev - device handle
vcUnitInfo - vc init info
Return:
0 - succuss     other - failed
Others:
*************************************************************/
SUNNYDDLL int CALL Hsp004_QuerySaturationUnitInfo(HCAM hCamDev, stVCUnitInfo *vcUnitInfo);

/*************************************************************
Function: Hsp004_SaturationUnitTransfer
Description: set or get saturation value
Input:
hCamDev - device handle
nCmd - 0(set) 1(get)
vcUnitValue - vc init value
Return:
0 - succuss     other - failed
Others:
*************************************************************/
SUNNYDDLL int CALL Hsp004_SaturationUnitTransfer(HCAM hCamDev, int nCmd, stVCUnitValue *vcUnitValue);

/*************************************************************
Function: Hsp004_QuerySharpnessUnitInfo
Description: query sharpness unit info
Input:
hCamDev - device handle
vcUnitInfo - vc init info
Return:
0 - succuss     other - failed
Others:
*************************************************************/
SUNNYDDLL int CALL Hsp004_QuerySharpnessUnitInfo(HCAM hCamDev, stVCUnitInfo *vcUnitInfo);

/*************************************************************
Function: Hsp004_SharpnessUnitTransfer
Description: set or get sharpness value
Input:
hCamDev - device handle
nCmd - 0(set) 1(get)
vcUnitValue - vc init value
Return:
0 - succuss     other - failed
Others:
*************************************************************/
SUNNYDDLL int CALL Hsp004_SharpnessUnitTransfer(HCAM hCamDev, int nCmd, stVCUnitValue *vcUnitValue);

/*************************************************************
Function: Hsp004_QueryGainUnitInfo
Description: query gain unit info
Input:
hCamDev - device handle
vcUnitInfo - vc init info
Return:
0 - succuss     other - failed
Others:
*************************************************************/
SUNNYDDLL int CALL Hsp004_QueryGainUnitInfo(HCAM hCamDev, stVCUnitInfo *vcUnitInfo);

/*************************************************************
Function: Hsp004_GainUnitTransfer
Description: set or get gain value
Input:
hCamDev - device handle
nCmd - 0(set) 1(get)
vcUnitValue - vc init value
Return:
0 - succuss     other - failed
Others:
*************************************************************/
SUNNYDDLL int CALL Hsp004_GainUnitTransfer(HCAM hCamDev, int nCmd, stVCUnitValue *vcUnitValue);

/*************************************************************
Function: Hsp004_QueryExpUnitInfo
Description: query exp unit info
Input:
hCamDev - device handle
vcUnitInfo - vc init info
Return:
0 - succuss     other - failed
Others:
*************************************************************/
SUNNYDDLL int CALL Hsp004_QueryExpUnitInfo(HCAM hCamDev, stVCUnitInfo *vcUnitInfo);

/*************************************************************
Function: Hsp004_ExpUnitTransfer
Description: set or get exp value
Input:
hCamDev - device handle
nCmd - 0(set) 1(get)
vcUnitValue - vc init value
Return:
0 - succuss     other - failed
Others:
*************************************************************/
SUNNYDDLL int CALL Hsp004_ExpUnitTransfer(HCAM hCamDev, int nCmd, stVCUnitValue *vcUnitValue);


#ifdef __cplusplus
}
#endif

#endif