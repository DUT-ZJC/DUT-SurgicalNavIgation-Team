#include "ThreadTask.h"
#include <iostream>
#include <windows.h>

CameraThreadManager::CameraThreadManager()
    : m_exitFlag(false)
    , m_cameraCount(0)
{
}

CameraThreadManager::~CameraThreadManager()
{
    Cleanup();
}

bool CameraThreadManager::Initialize()
{
    // 初始化SDK
    if (!m_sdk.Initialize()) {
        printf("Failed to initialize SDK\n");
        return false;
    }

    // 枚举设备
    if (!m_sdk.EnumDevices()) {
        printf("Failed to enumerate devices\n");
        m_sdk.FinalizeSDK();
        return false;
    }

    m_cameraCount = m_sdk.GetDeviceCount();
    printf("Found %d camera(s)\n", m_cameraCount);

    if (m_cameraCount == 0) {
        printf("No cameras found\n");
        m_sdk.FinalizeSDK();
        return false;
    }

    // 打印所有枚举到的相机的信息
    printf("Camera List:\n");
    for (unsigned int i = 0; i < m_cameraCount; ++i) {
        m_sdk.PrintDeviceInfo(i);
    }

    return true;
}

bool CameraThreadManager::StartAllCameras()
{
    if (m_cameraCount == 0) {
        printf("No cameras to start\n");
        return false;
    }

    printf("Using %d camera(s)\n", m_cameraCount);

    // 重置退出标志
    m_exitFlag = false;

    // 为每个相机创建线程
    for (unsigned int i = 0; i < m_cameraCount; ++i) {
        std::string windowName = "Camera " + std::to_string(i);
        m_cameraThreads.emplace_back(&CameraThreadManager::CameraThreadFunction, this, i, windowName);
        printf("Started thread for camera %d\n", i);
    }

    printf("All camera threads started.\n");
    return true;
}

void CameraThreadManager::StopAllCameras()
{
    printf("\nStopping all cameras...\n");

    // 设置退出标志
    m_exitFlag = true;

    // 等待所有线程结束
    for (auto& thread : m_cameraThreads) {
        if (thread.joinable()) {
            thread.join();
        }
    }

    // 清空线程容器
    m_cameraThreads.clear();

    printf("All cameras stopped.\n");
}

void CameraThreadManager::WaitForExit()
{
    printf("Press 'q' and Enter to stop all cameras...\n\n");

    char input;
    while (true) {
        scanf_s(" %c", &input, 1);
        if (input == 'q' || input == 'Q') {
            break;
        }
    }
}

void CameraThreadManager::Cleanup()
{
    // 确保所有线程都已停止
    if (!m_cameraThreads.empty()) {
        StopAllCameras();
    }

    // 清理SDK
    m_sdk.FinalizeSDK();
}

void CameraThreadManager::CameraThreadFunction(unsigned int cameraIndex, const std::string& windowName)
{
    CameraHelper camera;
    int frameCount = 0;

    try {
        // 初始化SDK
        if (!camera.Initialize()) {
            printf("Camera %d: Failed to initialize SDK\n", cameraIndex);
            return;
        }

        // 枚举设备
        if (!camera.EnumDevices()) {
            printf("Camera %d: Failed to enumerate devices\n", cameraIndex);
            camera.FinalizeSDK();
            return;
        }

        if (cameraIndex >= camera.GetDeviceCount()) {
            printf("Camera %d: Invalid camera index\n", cameraIndex);
            camera.FinalizeSDK();
            return;
        }

        // 创建句柄
        if (!camera.CreateHandle(cameraIndex)) {
            printf("Camera %d: Failed to create handle\n", cameraIndex);
            camera.FinalizeSDK();
            return;
        }

        // 打开设备
        if (!camera.OpenDevice()) {
            printf("Camera %d: Failed to open device\n", cameraIndex);
            camera.FinalizeSDK();
            return;
        }

        // 注册回调（如果需要）
        // camera.RegisterCallback();

        // 开始抓图
        if (!camera.StartGrabbing()) {
            printf("Camera %d: Failed to start grabbing\n", cameraIndex);
            camera.CloseDevice();
            camera.FinalizeSDK();
            return;
        }

        // 创建显示区域
        CreateCameraStatusWindow(cameraIndex, windowName);

        // 持续获取图像
        while (!m_exitFlag) {
            // TODO: 实际图像获取逻辑
            frameCount++;

            // 更新帧数显示
            UpdateFrameCount(cameraIndex, frameCount);

            Sleep(33); // 控制帧率约30fps
        }

        // 停止抓图
        camera.StopGrabbing();
        camera.CloseDevice();
        camera.FinalizeSDK();

        // 更新状态为 Stopped
        UpdateCameraStatus(cameraIndex, "Stopped");

        printf("\nCamera %d: Thread finished\n", cameraIndex);

    }
    catch (const std::exception& e) {
        printf("Camera %d: Exception occurred: %s\n", cameraIndex, e.what());
    }
}

void CameraThreadManager::CreateCameraStatusWindow(unsigned int cameraIndex, const std::string& windowTitle)
{
    COORD pos;
    HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);

    int baseY = 2 + cameraIndex * CAMERA_BLOCK_HEIGHT;

    // 打印分隔线
    pos.X = 0; pos.Y = baseY - 1;
    SetConsoleCursorPosition(hConsole, pos);
    printf("------------------------------------------------------------");

    // 标题
    pos.X = 0; pos.Y = baseY;
    SetConsoleCursorPosition(hConsole, pos);
    printf("=== %s ===", windowTitle.c_str());

    // 状态
    pos.X = 0; pos.Y = baseY + 1;
    SetConsoleCursorPosition(hConsole, pos);
    printf("Status: Running...");

    // 帧数
    pos.X = 0; pos.Y = baseY + 2;
    SetConsoleCursorPosition(hConsole, pos);
    printf("Frame Count: 0");
}

void CameraThreadManager::UpdateCameraStatus(unsigned int cameraIndex, const std::string& status)
{
    COORD pos;
    pos.X = 8; // "Status: " 长度
    pos.Y = 2 + cameraIndex * CAMERA_BLOCK_HEIGHT + 1;
    HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    SetConsoleCursorPosition(hConsole, pos);
    printf("%-15s", status.c_str()); // 固定宽度，左对齐
}

void CameraThreadManager::UpdateFrameCount(unsigned int cameraIndex, int frameCount)
{
    COORD pos;
    pos.X = 13; // "Frame Count: " 长度
    pos.Y = 2 + cameraIndex * CAMERA_BLOCK_HEIGHT + 2;
    HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    SetConsoleCursorPosition(hConsole, pos);
    printf("%-10d", frameCount); // 固定宽度，左对齐
}