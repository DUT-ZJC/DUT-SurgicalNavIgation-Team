/************************************************************************/
/* 以C++接口为基础，对常用函数进行二次封装，方便使用，参考MvCamera.h .cpp*/
/************************************************************************/
#ifndef CAMERASDK_USER_H
#define CAMERASDK_USER_H

//包含头文件
#include <Windows.h>
#include "MvCameraControl.h"
#include <string>

// 相机操作类
/*通过回调函数异步获取图像, 优势是可以减少内存拷贝。
异步操作可规避同步取图时可能存在的耗时长问题。 参考Grab_Asynchronous.cpp */
class CameraHelper
{
public:
	//声明构造函数
	CameraHelper(); 

	// 声明析构函数
	~CameraHelper(); 

	// ch:初始化SDK | en:Initialize SDK
	bool Initialize(); 

	// ch:枚举设备 | en:Enum devices
	bool EnumDevices();

	// ch:创建句柄 | en:Create handle
	bool CreateHandle(unsigned int index);

	// ch:打开设备 | en:Open device
	bool OpenDevice(); 

	// ch:关闭设备 | en:Close device
	bool CloseDevice();

	// ch:判断相机是否处于连接状态 | en:Is The Device Connected
	bool IsDeviceConnected();

	// ch:开始取流 | en:Start grabbing
	bool StartGrabbing(); 

	// ch:停止取流 | en:Stop grabbing
	bool StopGrabbing(); 

	// ch:主动获取一帧图像数据 | en:Get one frame initiatively
	int GetImageBuffer(MV_FRAME_OUT* pFrame, int nMsec);

	// ch:释放图像缓存 | en:Free image buffer
	int FreeImageBuffer(MV_FRAME_OUT* pFrame);

	// ch:显示一帧图像 | en:Display one frame image
	int DisplayOneFrame(void* hDisplay, MV_CC_IMAGE* pImageInfo);

	// ch:设置SDK内部图像缓存节点个数 | en:Set the number of the internal image cache nodes in SDK
	int SetImageNodeNum(unsigned int nNum);

	// ch:注册回调函数 | en:Register callback function
	void RegisterCallback()
	{
		MV_CC_RegisterImageCallBackEx(handle_, ImageCallBackEx, this);
	}

	// ch:注销回调函数 | en:Unregister callback function
	void UnregisterCallback(); 

	// ch:反初始化SDK | en:Finalize SDK
	void FinalizeSDK(); 

	// ch:等待按键 | en:Wait for key press
	void WaitForKeyPress(); 

	// ch:获取设备数量 | en:Get device count
	unsigned int GetDeviceCount() const { return deviceList_.nDeviceNum; } 

	// ch:打印设备信息 | en:Print device info
	void PrintDeviceInfo(unsigned int index); 

	// ch:像素格式转换 | en:Pixel format conversion
	int ConvertPixelType(MV_CC_PIXEL_CONVERT_PARAM_EX* pstCvtParam);

	// ch:保存图片 | en:save image
	int SaveImage(MV_SAVE_IMAGE_PARAM_EX3* pstParam);

	// ch:保存图片到文件 | en:save image to file
	int SaveImageToFile(MV_CC_IMAGE* pstImage, MV_CC_SAVE_IMAGE_PARAM* pSaveImageParam, const char* pcImagePath);


private:
	// ch:静态回调函数 | en:Static callback function
	//作用是当相机有新图像数据时被调用
    static void __stdcall ImageCallBackEx(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser);

	// 相机句柄
	void* handle_; 
	
	// 设备列表
	MV_CC_DEVICE_INFO_LIST deviceList_; 
};

#endif // !CAMERASDK_USER_H