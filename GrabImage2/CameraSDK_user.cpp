#include "CameraSDK_user.h"
#include <stdio.h>
#include <conio.h>
#include <string.h>

// 构造函数
CameraHelper::CameraHelper()
    : handle_(nullptr)
{
    memset(&deviceList_, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
}

// 析构函数
CameraHelper::~CameraHelper()
{
    if (handle_)
    {
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
    }
    MV_CC_Finalize();
}

// 初始化SDK
bool CameraHelper::Initialize()
{
    int nRet = MV_CC_Initialize();
    if (MV_OK != nRet)
    {
        printf("Initialize SDK fail! nRet [0x%x]\n", nRet);
        return false;
    }
    return true;
}

// 枚举设备
bool CameraHelper::EnumDevices()
{
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE | MV_GENTL_CAMERALINK_DEVICE | MV_GENTL_CXP_DEVICE | MV_GENTL_XOF_DEVICE, &deviceList_);
    if (MV_OK != nRet)
    {
        printf("Enum Devices fail! nRet [0x%x]\n", nRet);
        return false;
    }

    if (deviceList_.nDeviceNum == 0)
    {
        printf("Find No Devices!\n");
        return false;
    }

    for (unsigned int i = 0; i < deviceList_.nDeviceNum; i++)
    {
        PrintDeviceInfo(i);
    }
    return true;
}

// 创建句柄
bool CameraHelper::CreateHandle(unsigned int index)
{
    if (index >= deviceList_.nDeviceNum)
        return false;

    int nRet = MV_CC_CreateHandle(&handle_, deviceList_.pDeviceInfo[index]);
    if (MV_OK != nRet)
    {
        printf("Create Handle fail! nRet [0x%x]\n", nRet);
        handle_ = nullptr;
        return false;
    }
    return true;
}

// 打开设备
bool CameraHelper::OpenDevice()
{
    if (!handle_) return false;

    int nRet = MV_CC_OpenDevice(handle_);
    if (MV_OK != nRet)
    {
        printf("Open Device fail! nRet [0x%x]\n", nRet);
        return false;
    }

    // 仅针对GigE相机设置最佳包大小
    if (deviceList_.pDeviceInfo[0]->nTLayerType == MV_GIGE_DEVICE)
    {
        int nPacketSize = MV_CC_GetOptimalPacketSize(handle_);
        if (nPacketSize > 0)
        {
            MV_CC_SetIntValueEx(handle_, "GevSCPSPacketSize", nPacketSize);
        }
    }

    // 设置触发模式为off
    MV_CC_SetEnumValue(handle_, "TriggerMode", MV_TRIGGER_MODE_OFF);

    return true;
}

// 关闭设备
bool CameraHelper::CloseDevice()
{
    if (!handle_) return false;
    MV_CC_CloseDevice(handle_);
    return true;
}

// ch:判断相机是否处于连接状态 | en:Is The Device Connected
bool CameraHelper::IsDeviceConnected()
{
    return MV_CC_IsDeviceConnected(handle_);
}

// 开始取流
bool CameraHelper::StartGrabbing()
{
    if (!handle_) return false;
    MV_CC_RegisterImageCallBackEx(handle_, ImageCallBackEx, handle_);
    int nRet = MV_CC_StartGrabbing(handle_);
    if (MV_OK != nRet)
    {
        printf("Start Grabbing fail! nRet [0x%x]\n", nRet);
        return false;
    }
    return true;
}

// 停止取流
bool CameraHelper::StopGrabbing()
{
    if (!handle_) return false;
    MV_CC_StopGrabbing(handle_);
    MV_CC_RegisterImageCallBackEx(handle_, NULL, NULL);
    return true;
}

// ch:主动获取一帧图像数据 | en:Get one frame initiatively
int CameraHelper::GetImageBuffer(MV_FRAME_OUT* pFrame, int nMsec)
{
    return MV_CC_GetImageBuffer(handle_, pFrame, nMsec);
}

// ch:释放图像缓存 | en:Free image buffer
int CameraHelper::FreeImageBuffer(MV_FRAME_OUT* pFrame)
{
    return MV_CC_FreeImageBuffer(handle_, pFrame);
}

// ch:设置显示窗口句柄 | en:Set Display Window Handle
int CameraHelper::DisplayOneFrame(void* hwndDisplay, MV_CC_IMAGE* pImageInfo)
{
    return MV_CC_DisplayOneFrameEx2(handle_, hwndDisplay, pImageInfo, 0);

}

// ch:设置SDK内部图像缓存节点个数 | en:Set the number of the internal image cache nodes in SDK
int CameraHelper::SetImageNodeNum(unsigned int nNum)
{
    return MV_CC_SetImageNodeNum(handle_, nNum);
}

// 注销回调函数
void CameraHelper::UnregisterCallback()
{
    if (handle_)
        MV_CC_RegisterImageCallBackEx(handle_, NULL, NULL);
}

// 反初始化SDK
void CameraHelper::FinalizeSDK()
{
    MV_CC_Finalize();
}

// 等待按键
void CameraHelper::WaitForKeyPress()
{
    while (!_kbhit())
    {
        Sleep(10);
    }
    _getch();
}

void CameraHelper::PrintDeviceInfo(unsigned int index)
{
    if (index >= deviceList_.nDeviceNum) return;

    MV_CC_DEVICE_INFO* pstMVDevInfo = deviceList_.pDeviceInfo[index];
    if (!pstMVDevInfo) return;

    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
        printf("[device %d] CurrentIp: %d.%d.%d.%d, Name: %s\n", index, nIp1, nIp2, nIp3, nIp4, pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else
    {
        printf("[device %d] Name: %s\n", index, pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
}

// 静态回调函数
void __stdcall CameraHelper::ImageCallBackEx(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser)
{
    if (pFrameInfo)
    {
        printf("Get One Frame: Width[%d], Height[%d], FrameNum[%d]\n",
            pFrameInfo->nWidth, pFrameInfo->nHeight, pFrameInfo->nFrameNum);
    }
}

// ch:像素格式转换 | en:Pixel format conversion
int CameraHelper::ConvertPixelType(MV_CC_PIXEL_CONVERT_PARAM_EX* pstCvtParam)
{
    return MV_CC_ConvertPixelTypeEx(handle_, pstCvtParam);
}

// ch:保存图片 | en:save image
int CameraHelper::SaveImage(MV_SAVE_IMAGE_PARAM_EX3* pstParam)
{
    return MV_CC_SaveImageEx3(handle_, pstParam);
}

// ch:保存图片为文件 | en:Save the image as a file
int CameraHelper::SaveImageToFile(MV_CC_IMAGE* pstImage, MV_CC_SAVE_IMAGE_PARAM* pSaveImageParam, const char* pcImagePath)
{
    return MV_CC_SaveImageToFileEx2(handle_, pstImage, pSaveImageParam, pcImagePath);
}



