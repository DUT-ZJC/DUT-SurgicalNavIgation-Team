//#include "CameraHelper.h"
//#include <stdio.h>
//#include <thread>
//#include <vector>
//#include <iostream>
//#include <windows.h>
//#include <string>
//
//// 全局退出标志
//bool g_exitFlag = false;
//
//// 单个相机线程函数
//void CameraThreadFunction(unsigned int cameraIndex, const std::string& cameraName)
//{
//    CameraHelper camera;
//
//    try {
//        printf("Camera %d (%s): Starting initialization...\n", cameraIndex, cameraName.c_str());
//
//        // 初始化SDK
//        if (!camera.Initialize()) {
//            printf("Camera %d (%s): Failed to initialize SDK\n", cameraIndex, cameraName.c_str());
//            return;
//        }
//
//        // 枚举设备
//        if (!camera.EnumDevices()) {
//            printf("Camera %d (%s): Failed to enumerate devices\n", cameraIndex, cameraName.c_str());
//            camera.FinalizeSDK();
//            return;
//        }
//
//        if (cameraIndex >= camera.GetDeviceCount()) {
//            printf("Camera %d (%s): Invalid camera index\n", cameraIndex, cameraName.c_str());
//            camera.FinalizeSDK();
//            return;
//        }
//
//        // 创建句柄
//        if (!camera.CreateHandle(cameraIndex)) {
//            printf("Camera %d (%s): Failed to create handle\n", cameraIndex, cameraName.c_str());
//            camera.FinalizeSDK();
//            return;
//        }
//
//        // 打开设备
//        if (!camera.OpenDevice()) {
//            printf("Camera %d (%s): Failed to open device\n", cameraIndex, cameraName.c_str());
//            camera.FinalizeSDK();
//            return;
//        }
//
//        // 注册回调函数（这里可能是显示图像的关键）
//        camera.RegisterCallback();
//
//        // 开始抓图
//        if (!camera.StartGrabbing()) {
//            printf("Camera %d (%s): Failed to start grabbing\n", cameraIndex, cameraName.c_str());
//            camera.CloseDevice();
//            camera.FinalizeSDK();
//            return;
//        }
//
//        printf("Camera %d (%s): Started successfully! Display window should be open.\n", cameraIndex, cameraName.c_str());
//
//        // 持续运行直到用户要求退出
//        // 使用类似原始代码的方式，但不阻塞等待按键
//        while (!g_exitFlag) {
//            // 让线程休眠一小段时间，避免占用过多CPU
//            Sleep(100);
//
//            // 如果你的CameraHelper有类似的方法来检查窗口状态，可以在这里使用
//            // 比如检查OpenCV窗口是否被关闭
//        }
//
//        printf("Camera %d (%s): Stopping...\n", cameraIndex, cameraName.c_str());
//
//        // 停止抓图
//        camera.StopGrabbing();
//        camera.CloseDevice();
//        camera.FinalizeSDK();
//
//        printf("Camera %d (%s): Thread finished\n", cameraIndex, cameraName.c_str());
//
//    }
//    catch (const std::exception& e) {
//        printf("Camera %d (%s): Exception occurred: %s\n", cameraIndex, cameraName.c_str(), e.what());
//    }
//}
//
//int main()
//{
//    CameraHelper sdk;
//
//    printf("=== Multi-Camera OpenCV Display Program ===\n\n");
//
//    // 初始化SDK
//    if (!sdk.Initialize()) {
//        printf("Failed to initialize SDK\n");
//        return -1;
//    }
//
//    // 枚举设备
//    if (!sdk.EnumDevices()) {
//        printf("Failed to enumerate devices\n");
//        sdk.FinalizeSDK();
//        return -1;
//    }
//
//    unsigned int deviceCount = sdk.GetDeviceCount();
//    printf("Found %d camera(s)\n", deviceCount);
//
//    if (deviceCount == 0) {
//        printf("No cameras found\n");
//        sdk.FinalizeSDK();
//        return -1;
//    }
//
//    // 显示找到的相机信息（如果你的CameraHelper支持获取相机名称）
//    for (unsigned int i = 0; i < deviceCount; ++i) {
//        printf("[device %d] Index: %d\n", i, i);
//    }
//
//	unsigned int camerasAvailable = deviceCount; // 实际可用相机数量
//    printf("\nUsing %d camera(s)\n", deviceCount);
//    printf("Each camera will open its own OpenCV display window.\n\n");
//
//    // 相机名称数组（根据你之前的输出）
//    std::vector<std::string> cameraNames = { "RGB", "Mono_R", "Mono_L" };
//
//    // 创建线程向量
//    std::vector<std::thread> cameraThreads;
//
//    // 为每个相机创建线程
//    for (unsigned int i = 0; i < deviceCount; ++i) {
//        std::string cameraName = (i < cameraNames.size()) ? cameraNames[i] : ("Camera_" + std::to_string(i));
//        cameraThreads.emplace_back(CameraThreadFunction, i, cameraName);
//        printf("Started thread for camera %d (%s)\n", i, cameraName.c_str());
//
//        // 给每个相机一些时间来启动，避免同时初始化造成冲突
//        Sleep(1000);
//    }
//
//    printf("\n=== All camera threads started ===\n");
//    printf("You should now see %d OpenCV windows displaying camera feeds.\n", deviceCount);
//    printf("Press 'q' and Enter to stop all cameras...\n\n");
//
//    // 等待用户输入 'q' 来退出
//    char input;
//    while (true) {
//        printf("Enter 'q' to quit: ");
//        if (scanf_s(" %c", &input, 1) == 1) {
//            if (input == 'q' || input == 'Q') {
//                g_exitFlag = true;
//                break;
//            }
//        }
//    }
//
//    printf("\nStopping all cameras...\n");
//
//    // 等待所有线程结束
//    for (auto& thread : cameraThreads) {
//        if (thread.joinable()) {
//            thread.join();
//        }
//    }
//
//    // 最后清理SDK
//    sdk.FinalizeSDK();
//
//    printf("\n=== All cameras stopped ===\n");
//    printf("All OpenCV windows should now be closed.\n");
//    printf("Press any key to exit.\n");
//
//    // 清理输入缓冲区
//    while (getchar() != '\n');
//    getchar();
//
//    return 0;
//}