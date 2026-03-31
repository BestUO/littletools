#include "hsp004_wrap.h"
#include "log/log.h"
#include <chrono>
#include <cstdio>
#include <functional>
#include <opencv2/opencv.hpp>
namespace controller_native {
    bool HSP004Wrap::ShowInfo() {
        auto device_info = GetDeviceUUIDList();
        if (device_info.empty()) {
            LOGGER_ERROR(logger_, "No HSP004 device found.");
            return false;
        } else {
            for (const auto &info : device_info) {
                LOGGER_INFO(logger_, "Device UUID: {}, Serial Number: {}", info.uuid, info.serial_number);
            }
        }
        return true;
    }

    std::vector<HSP004Wrap::HSP004DeviceInfo> HSP004Wrap::GetDeviceUUIDList() {
        SunnyDeviceDescriptor *devs_desc_ptr = nullptr;
        int dev_num = 0;
        std::vector<HSP004DeviceInfo> device_infos;
        Hsp004_SearchDevice(&devs_desc_ptr, &dev_num);
        if (dev_num > 0) {
            for (int i = 0; i < dev_num; i++) {
                HSP004DeviceInfo device_info;
                char serial_number[60] = {0};
                if (!Hsp004_Open(&device_info.hcam, &devs_desc_ptr[i], nullptr, nullptr)) {
                    if (!Hsp004_GetDevUUID(device_info.hcam, &device_info.uuid) &&
                        !Hsp004_ReadSerialNumber(device_info.hcam, serial_number)) {

                        CamRes camRes[128];
                        int nCapCount = 0;
                        memset(camRes, 0x00, 128 * sizeof(CamRes));
                        Hsp004_Query_Camcaps(device_info.hcam, camRes, 128, &nCapCount);
                        for (int j = 0; j < nCapCount; j++) {
                            LOGGER_INFO(logger_, "width = {},height = {},frameRate = {},int format = {}\n",
                                        camRes[j].width, camRes[j].height, camRes[j].frameRate, camRes[j].format);
                        }

                        Hsp004_Close(device_info.hcam);
                        device_info.hcam = nullptr;
                        device_info.serial_number = serial_number;
                        device_infos.push_back(device_info);
                    } else {
                        LOGGER_ERROR(logger_, "Hsp004_Open fail");
                    }
                }
            }
        } else {
            LOGGER_ERROR(logger_, "No sunny device plugin!!\n");
        }
        return device_infos;
    }

    HSP004Wrap::HSP004DeviceInfo *
    HSP004Wrap::OpenTargetDevice(const std::function<bool(const HSP004DeviceInfo *device_info)> compare_fun) {
        SunnyDeviceDescriptor *devs_desc_ptr = nullptr;
        int dev_num = 0;
        Hsp004_SearchDevice(&devs_desc_ptr, &dev_num);
        HSP004DeviceInfo *result = nullptr;
        if (dev_num > 0) {
            for (int i = 0; i < dev_num; i++) {
                HSP004DeviceInfo *device_info = new HSP004DeviceInfo;
                char serial_number[60] = {0};
                if (!Hsp004_Open(&device_info->hcam, &devs_desc_ptr[i], DevStatusCB, device_info)) {
                    if (!Hsp004_GetDevUUID(device_info->hcam, &device_info->uuid) &&
                        !Hsp004_ReadSerialNumber(device_info->hcam, serial_number)) {
                        device_info->serial_number = serial_number;
                        if (compare_fun && compare_fun(device_info)) {
                            result = device_info;
                            break;
                        } else {
                            Hsp004_Close(device_info->hcam);
                            delete device_info;
                        }
                    } else {
                        LOGGER_ERROR(logger_, "Hsp004_Open fail");
                    }
                }
            }
        } else {
            LOGGER_ERROR(logger_, "No sunny device plugin!!\n");
        }
        return result;
    }

    bool HSP004Wrap::CloseDevice(HSP004DeviceInfo *device_info) {
        if (device_info && device_info->hcam) {
            Hsp004_Close(device_info->hcam);
            device_info->hcam = nullptr;
            return true;
        }
        return false;
    }

    bool HSP004Wrap::StartCameraStream(HSP004DeviceInfo *device_info) {
        if (device_info && device_info->hcam) {
            auto now = std::chrono::system_clock::now();
            std::time_t now_time = std::chrono::system_clock::to_time_t(now);
            std::tm local_tm;
            localtime_r(&now_time, &local_tm);
            auto time_count = now_time + local_tm.tm_gmtoff;

            std::stringstream ss;
            ss << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S");
            std::string time_str = ss.str();
            LOGGER_INFO(logger_, "HSP004Wrap::StartCameraStream: Current time is: {}", time_str);

            if (Hsp004_Set_SysTime(device_info->hcam, time_count / 1000000, time_count % 1000000)) {
                LOGGER_INFO(logger_, "HSP004Wrap::StartCameraStream: Set system time fail.");
            }

            SOCAM_STREAM_FMT stream_fmt = SOCAM_STREAM_FMT_H264;
            if (device_info->base_info_ptr->camera_output_format == "MJPG") {
                stream_fmt = SOCAM_STREAM_FMT_MJPG;
            } else if (device_info->base_info_ptr->camera_output_format == "H264") {
                stream_fmt = SOCAM_STREAM_FMT_H264;
            }

            LOGGER_INFO(logger_,
                        "HSP004Wrap::StartCameraStream: Start stream with format: resolution: {}*{} fps: {} format: {}",
                        device_info->base_info_ptr->width, device_info->base_info_ptr->height,
                        device_info->base_info_ptr->frame_rate, (int) stream_fmt);
            return !Hsp004_StartStream(device_info->hcam, StreamCB, device_info, device_info->base_info_ptr->width,
                                       device_info->base_info_ptr->height, device_info->base_info_ptr->frame_rate,
                                       stream_fmt);
        }
        return false;
    }

    bool HSP004Wrap::StopCameraStream(HSP004DeviceInfo *device_info) {
        if (device_info && device_info->hcam) { return !Hsp004_StopStream(device_info->hcam); }
        return false;
    }

    bool HSP004Wrap::SetCalibrationParam(HSP004DeviceInfo *device_info,
                                         controller_native::CameraRGBCalibrateParam camera_calibrate_param) {
        if (device_info && device_info->hcam) {
            stStereoCameraParam calibData;
            calibData.Fx = camera_calibrate_param.Fx;
            calibData.Fy = camera_calibrate_param.Fy;
            calibData.Cx = camera_calibrate_param.Cx;
            calibData.Cy = camera_calibrate_param.Cy;
            calibData.K1 = camera_calibrate_param.K1;
            calibData.K2 = camera_calibrate_param.K2;
            calibData.K3 = camera_calibrate_param.K3;
            calibData.P1 = camera_calibrate_param.P1;
            calibData.P2 = camera_calibrate_param.P2;

            return Hsp004_CameraCalibParamTransfer(device_info->hcam, 0, &calibData) == 0;
        }
        return false;
    }

    void HSP004Wrap::DevStatusCB(stSunnyDevStatus tofDevStatus, void *pUserData) {
        HSP004Wrap::HSP004DeviceInfo *device_info = (HSP004Wrap::HSP004DeviceInfo *) pUserData;
        (void) tofDevStatus;
        if (device_info->base_info_ptr->dev_status_callback) {
            std::string status_message = "Device uuid: " + std::to_string(device_info->uuid) +
                                         " serial_number: " + device_info->serial_number + " status: down";
            device_info->base_info_ptr->dev_status_callback(status_message);
        }
    }

    void HSP004Wrap::StreamCB(void *buffer, int buf_len, void *user_data) {
        HSP004Wrap::HSP004DeviceInfo *device_info = (HSP004Wrap::HSP004DeviceInfo *) user_data;
        if (device_info) { device_info->base_info_ptr->camera_image_callback(buffer, buf_len); }
    }
}// namespace controller_native