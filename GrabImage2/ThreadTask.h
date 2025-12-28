#pragma once
#include "CameraSDK_user.h"
#include <thread>
#include <vector>
#include <string>
#include <atomic>

class CameraThreadManager
{
public:
	CameraThreadManager(); // 构造函数
	~CameraThreadManager(); // 析构函数

    // 初始化相机系统
    bool Initialize();

    // 启动所有相机线程
    bool StartAllCameras();

    // 停止所有相机线程
    void StopAllCameras();

    // 等待用户输入退出指令
    void WaitForExit();

    // 清理资源
    void Cleanup();

    // 获取相机数量
    unsigned int GetCameraCount() const { return m_cameraCount; }

private:
    // 单个相机线程函数
    void CameraThreadFunction(unsigned int cameraIndex, const std::string& windowName);

    // 创建相机状态显示窗口
    void CreateCameraStatusWindow(unsigned int cameraIndex, const std::string& windowTitle);

    // 更新相机状态显示
    void UpdateCameraStatus(unsigned int cameraIndex, const std::string& status);
    void UpdateFrameCount(unsigned int cameraIndex, int frameCount);

private:
    CameraHelper m_sdk;                           // SDK实例
    std::vector<std::thread> m_cameraThreads;     // 相机线程容器
    std::atomic<bool> m_exitFlag;                 // 退出标志
    unsigned int m_cameraCount;                   // 相机数量

    // 显示相关常量
    static const int CAMERA_BLOCK_HEIGHT = 10;
};