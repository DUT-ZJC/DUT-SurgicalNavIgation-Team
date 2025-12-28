////——————————————————————————————————————————————————————————————————————————————————————————
////// 单个相机的测试代码，完整的相机注册、打开、取流、关闭流程
////////#include "CameraHelper.h"
//////#include <stdio.h>
//////// 在CameraHelper类中使用
//////#include "ImageGet.h"
//////int main()
//////{
//////    CameraHelper camera;
//////
//////	if (!camera.Initialize()) return -1; // 初始化SDK
//////	if (!camera.EnumDevices()) return -1; // 枚举设备
//////
//////	unsigned int index = 0; // 选择第0个设备
//////	printf("Please Input camera index(0-%d): ", camera.GetDeviceCount() - 1); // 输入设备索引
//////	scanf_s("%d", &index); // 读取用户输入
//////
//////	if (!camera.CreateHandle(index)) return -1; // 创建句柄
//////	if (!camera.OpenDevice()) return -1; // 打开设备
//////	//camera.RegisterCallback(); // 注册回调函数,先注册回调函数
//////	if (!camera.StartGrabbing()) return -1; // 开始取流
//////
//////    printf("Press a key to stop grabbing.\n");
//////    camera.WaitForKeyPress();
//////
//////    camera.StopGrabbing();
//////    camera.CloseDevice();
//////    camera.FinalizeSDK();
//////
//////    printf("Press a key to exit.\n");
//////    camera.WaitForKeyPress();
//////
//////    return 0;
//////}
////——————————————————————————————————————————————————————————————————————————————————————————
//
//
////多相机多线程测试代码，完整的相机注册、打开、取流、关闭流程
//
//#include "ThreadTask.h"
//#include <stdio.h>
//
//int main()
//{
//    CameraThreadManager cameraManager;
//
//    // 初始化相机系统
//    if (!cameraManager.Initialize()) {
//        printf("Failed to initialize camera system\n");
//        return -1;
//    }
//
//    // 启动所有相机
//    if (!cameraManager.StartAllCameras()) {
//        printf("Failed to start cameras\n");
//        return -1;
//    }
//
//    // 等待用户退出指令
//    cameraManager.WaitForExit();
//
//    // 停止所有相机
//    cameraManager.StopAllCameras();
//
//    printf("Press any key to exit.\n");
//    getchar();
//    getchar(); // 清除缓冲区
//
//    return 0;
//}
//
