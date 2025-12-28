#include <stdio.h>
#include <Windows.h>
#include <conio.h>
#include <math.h>
#include "MvCameraControl.h"
#include "Display.h"
#include <opencv2/opencv.hpp>


// 发光小球检测参数
#define MIN_BRIGHTNESS_THRESHOLD 200  // 最小亮度阈值
#define MIN_BLOB_AREA 50              // 最小连通区域面积
#define MAX_BLOB_AREA 5000            // 最大连通区域面积
Display gDisplay;

//// 坐标结构体
//typedef struct {
//    float x;
//    float y;
//    int valid;  // 是否检测到有效小球
//} BallPosition;

// ch:等待按键输入 | en:Wait for key press
void WaitForKeyPress(void)
{
    while (!_kbhit())
    {
        Sleep(10);
    }
    _getch();
}

bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

        // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
        printf("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
        printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
        printf("Device Number: %d\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.nDeviceNumber);
    }
    else if (pstMVDevInfo->nTLayerType == MV_GENTL_GIGE_DEVICE)
    {
        printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
        printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chSerialNumber);
        printf("Model Name: %s\n\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_GENTL_CAMERALINK_DEVICE)
    {
        printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stCMLInfo.chUserDefinedName);
        printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stCMLInfo.chSerialNumber);
        printf("Model Name: %s\n\n", pstMVDevInfo->SpecialInfo.stCMLInfo.chModelName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_GENTL_CXP_DEVICE)
    {
        printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stCXPInfo.chUserDefinedName);
        printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stCXPInfo.chSerialNumber);
        printf("Model Name: %s\n\n", pstMVDevInfo->SpecialInfo.stCXPInfo.chModelName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_GENTL_XOF_DEVICE)
    {
        printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stXoFInfo.chUserDefinedName);
        printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stXoFInfo.chSerialNumber);
        printf("Model Name: %s\n\n", pstMVDevInfo->SpecialInfo.stXoFInfo.chModelName);
    }
    else
    {
        printf("Not support.\n");
    }

    return true;
}

// RGB转灰度（假设输入是RGB24格式）
unsigned char rgb_to_gray(unsigned char r, unsigned char g, unsigned char b)
{
    return (unsigned char)(0.299 * r + 0.587 * g + 0.114 * b);
}

// 简单的连通区域标记和质心计算
BallPosition detectBrightBall(unsigned char* imageData, int width, int height, int pixelFormat)
{
    BallPosition result = { 0, 0, 0 };

    // 创建灰度图像缓冲区
    unsigned char* grayImage = (unsigned char*)malloc(width * height);
    if (!grayImage) return result;

    // 转换为灰度图像
    if (pixelFormat == PixelType_Gvsp_RGB8_Packed)
    {
        // RGB24格式
        for (int i = 0; i < width * height; i++)
        {
            grayImage[i] = rgb_to_gray(imageData[i * 3], imageData[i * 3 + 1], imageData[i * 3 + 2]);
        }
    }
    else if (pixelFormat == PixelType_Gvsp_Mono8)
    {
        // 单色格式，直接复制
        memcpy(grayImage, imageData, width * height);
    }
    else
    {
        // 其他格式的简单处理，假设为单色或取第一个通道
        for (int i = 0; i < width * height; i++)
        {
            grayImage[i] = imageData[i];
        }
    }

    // 二值化
    unsigned char* binaryImage = (unsigned char*)malloc(width * height);
    if (!binaryImage)
    {
        free(grayImage);
        return result;
    }

    for (int i = 0; i < width * height; i++)
    {
        binaryImage[i] = (grayImage[i] > MIN_BRIGHTNESS_THRESHOLD) ? 255 : 0;
    }

    // 查找最大的亮区域
    int maxArea = 0;
    float bestCenterX = 0, bestCenterY = 0;

    unsigned char* visited = (unsigned char*)calloc(width * height, 1);
    if (!visited)
    {
        free(grayImage);
        free(binaryImage);
        return result;
    }

    // 简单的连通区域检测
    for (int y = 1; y < height - 1; y++)
    {
        for (int x = 1; x < width - 1; x++)
        {
            int idx = y * width + x;
            if (binaryImage[idx] == 255 && !visited[idx])
            {
                // 开始区域生长
                int area = 0;
                float sumX = 0, sumY = 0;

                // 简单的4连通区域生长
                int stack[10000];  // 简单的栈实现
                int stackTop = 0;
                stack[stackTop++] = idx;

                while (stackTop > 0 && stackTop < 10000)
                {
                    int current = stack[--stackTop];
                    int cy = current / width;
                    int cx = current % width;

                    if (visited[current] || binaryImage[current] != 255)
                        continue;

                    visited[current] = 1;
                    area++;
                    sumX += cx;
                    sumY += cy;

                    // 添加邻居到栈
                    if (cx > 0 && !visited[current - 1] && binaryImage[current - 1] == 255)
                        stack[stackTop++] = current - 1;
                    if (cx < width - 1 && !visited[current + 1] && binaryImage[current + 1] == 255)
                        stack[stackTop++] = current + 1;
                    if (cy > 0 && !visited[current - width] && binaryImage[current - width] == 255)
                        stack[stackTop++] = current - width;
                    if (cy < height - 1 && !visited[current + width] && binaryImage[current + width] == 255)
                        stack[stackTop++] = current + width;
                }

                // 检查区域大小是否合适
                if (area >= MIN_BLOB_AREA && area <= MAX_BLOB_AREA && area > maxArea)
                {
                    maxArea = area;
                    bestCenterX = sumX / area;
                    bestCenterY = sumY / area;
                    result.valid = 1;
                }
            }
        }
    }

    if (result.valid)
    {
        result.x = bestCenterX;
        result.y = bestCenterY;
    }

    free(grayImage);
    free(binaryImage);
    free(visited);

    return result;
}

void __stdcall ImageCallBackEx(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser)
{
    if (pFrameInfo)
    {
        printf("Frame[%d] Size[%dx%d] ",
            pFrameInfo->nFrameNum, pFrameInfo->nWidth, pFrameInfo->nHeight);

        // 检测发光小球
        BallPosition ballPos = detectBrightBall(pData, pFrameInfo->nWidth, pFrameInfo->nHeight, pFrameInfo->enPixelType);

        if (ballPos.valid)
        {
            printf("-> Ball detected at: (%.2f, %.2f)\n", ballPos.x, ballPos.y);
        }
        else
        {
            printf("-> No ball detected\n");
        }

        // 可视化显示
        gDisplay.showFrame(pData, pFrameInfo, ballPos);
    }
}

int main()
{
    int nRet = MV_OK;
    void* handle = NULL;

    do
    {
        // ch:初始化SDK | en:Initialize SDK
        nRet = MV_CC_Initialize();
        if (MV_OK != nRet)
        {
            printf("Initialize SDK fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:枚举设备 | Enum device
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE | MV_GENTL_CAMERALINK_DEVICE | MV_GENTL_CXP_DEVICE | MV_GENTL_XOF_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("Enum Devices fail! nRet [0x%x]\n", nRet);
            break;
        }

        if (stDeviceList.nDeviceNum > 0)
        {
            for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                printf("[device %d]:\n", i);
                MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    break;
                }
                PrintDeviceInfo(pDeviceInfo);
            }
        }
        else
        {
            printf("Find No Devices!\n");
            break;
        }

        printf("Please Input camera index(0-%d):", stDeviceList.nDeviceNum - 1);
        unsigned int nIndex = 0;
        scanf_s("%d", &nIndex);

        if (nIndex >= stDeviceList.nDeviceNum)
        {
            printf("Input error!\n");
            break;
        }

        // ch:选择设备并创建句柄 | Select device and create handle
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet)
        {
            printf("Create Handle fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:打开设备 | Open device
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            printf("Open Device fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
        if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE)
        {
            int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
            if (nPacketSize > 0)
            {
                nRet = MV_CC_SetIntValueEx(handle, "GevSCPSPacketSize", nPacketSize);
                if (nRet != MV_OK)
                {
                    printf("Warning: Set Packet Size fail nRet [0x%x]!", nRet);
                }
            }
            else
            {
                printf("Warning: Get Packet Size fail nRet [0x%x]!", nPacketSize);
            }
        }

        // ch:设置触发模式为off | eb:Set trigger mode as off
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", MV_TRIGGER_MODE_OFF);
        if (MV_OK != nRet)
        {
            printf("Set Trigger Mode fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:注册抓图回调 | en:Register image callback
        nRet = MV_CC_RegisterImageCallBackEx(handle, ImageCallBackEx, handle);
        if (MV_OK != nRet)
        {
            printf("Register Image CallBack fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:开始取流 | en:Start grab image
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("Start Grabbing fail! nRet [0x%x]\n", nRet);
            break;
        }

        printf("Press a key to stop grabbing.\n");
        printf("Detection Parameters: Brightness Threshold=%d, Min Area=%d, Max Area=%d\n",
            MIN_BRIGHTNESS_THRESHOLD, MIN_BLOB_AREA, MAX_BLOB_AREA);
        WaitForKeyPress();

        // ch:停止取流 | en:Stop grab image
        nRet = MV_CC_StopGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("Stop Grabbing fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:注销抓图回调 | en:Unregister image callback
        nRet = MV_CC_RegisterImageCallBackEx(handle, NULL, NULL);
        if (MV_OK != nRet)
        {
            printf("Unregister Image CallBack fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:关闭设备 | en:Close device
        nRet = MV_CC_CloseDevice(handle);
        if (MV_OK != nRet)
        {
            printf("Close Device fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:销毁句柄 | en:Destroy handle
        nRet = MV_CC_DestroyHandle(handle);
        if (MV_OK != nRet)
        {
            printf("Destroy Handle fail! nRet [0x%x]\n", nRet);
            break;
        }
        handle = NULL;
    } while (0);


    if (handle != NULL)
    {
        MV_CC_DestroyHandle(handle);
        handle = NULL;
    }


    // ch:反初始化SDK | en:Finalize SDK
    MV_CC_Finalize();

    printf("Press a key to exit.\n");
    WaitForKeyPress();

    return 0;
}