#pragma once
#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include <queue>
#include <deque>  // 需包含deque头文件
#include <vector>
#include <Windows.h>
#include "DetectorBall.h"
#include "DistanceXYZ.h"
#include "Cam_paras.hpp"
#include <QObject>
#include "LogPrint.h"
#include <iostream>
#include "CMVMutex.hpp"
#include "Pose6DOF.h"
#include <chrono>  // 需包含计时库
#include <iomanip> // 用于格式化输出百分比

// 临界区锁  Window下响应更快

/*
// 图像对结构，包含左右图像及相关信息
struct ImagePair {
    cv::Mat leftImage;       // 左相机图像
    cv::Mat rightImage;      // 右相机图像
    int64_t timeInterval;   //时间差 左减右

    int leftFrameNum;            // 帧号
    int rightFrameNum;            // 帧号
    int64_t timeStamp;    // 时间戳
    int64_t DevTimeStampLeft;
    int64_t DevTimeStampRight;
    unsigned int    nDataSize;       // 图像数据大小（字节数，用于安全操作）
    int             nWidth;          // 图像宽度（像素）
    int             nHeight;         // 图像高度（像素）
    int             nPixelFormat;    // 像素格式（如Mono8、RGB24等，可选）
};
*/



// 全局变量存储位置信息
extern std::vector<cv::Point3f> g_globalPoints3D;
extern HANDLE g_mutexGlobalPoints; // 保护全局变量的互斥锁

//
class ICalculateCallback : public QObject
{
    Q_OBJECT  // 必须添加此宏，启用Qt元对象功能

public:
    // 显式构造函数，符合Qt对象树管理规范
    explicit ICalculateCallback(QObject* parent = nullptr)
        : QObject(parent)  // 初始化父对象
    {
    }

    // 纯虚析构函数需要提供实现
    virtual ~ICalculateCallback() override = default;

    // 回调接口声明
    virtual void OnCalculateComplete(const Point3f& Position, float roll, float pitch, float yaw) = 0;
};

class ImageProcessing {
public:
    ImageProcessing();
    ~ImageProcessing();

    void ReadCameraParams(cv::Mat cameraMatrixLeft, cv::Mat cameraMatrixRight, cv::Mat distCoeffsLeft, cv::Mat distCoeffsRight, cv::Mat R, cv::Mat T, double focalLength);
    
    void RegisterCalculateCallback(ICalculateCallback* pCallback);

    // 初始化函数：创建线程和事件
    bool Initialize();

    // 释放资源
    bool Release();

    // 入队操作
    bool enqueueImagePair(const ImagePair& imgPair);

    // 出队操作
    bool dequeueImagePair(ImagePair& imgPair, bool POP = true);
    void printTimeStats(double total, double detect, double stereo, double calc3D, double calcPose);
    void RegisterManager_2(ICalculateCallback* manager);
    // 获取当前队列大小
    size_t getQueueSize();
public:
    std::ofstream outFile;  // ofstream作为类的私有成员
private:
    // 线程函数
    static unsigned int __stdcall WorkerThread(void* pParam);
    unsigned int WorkerThreadProc();
    
    // 处理图像对的核心函数
    void processImagePair(const ImagePair& imgPair);
    //Pose6DOFCalculator poseCalculator;
    CameraParams cameraParams;
    // 队列相关
    std::queue<ImagePair> m_imageQueue;
    const size_t MAX_QUEUE_SIZE = 3;  // 最大存储3组图像对
    //HANDLE m_mutexQueue;              // 队列互斥锁
    CMVMutex m_mutexQueue;
    // 线程和事件
    HANDLE m_hWorkerThread;           // 工作线程句柄
    bool m_bThreadRunning;            // 线程运行标志
    HANDLE m_eventTaskAvailable;      // 任务可用事件 
    HANDLE m_eventShutdown;           // 线程退出事件

    ICalculateCallback* CalculateResultCallback;
    // 处理相关对象
    BallDetector m_ballDetector;      // 球检测器实例
};

#endif // IMAGE_PROCESSING_H#pragma once
