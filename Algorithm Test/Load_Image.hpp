#pragma once
#include <ostream>
#include <queue>
#include <thread>
#include <atomic>
#include <opencv2/opencv.hpp>
#include "CMVMutex.hpp"

//图像对结构体
struct ImagePair {   
    cv::Mat leftImage;       // 左相机图像
    cv::Mat rightImage;      // 右相机图像
    int64_t timeInterval;    // 时间差 左减右
    int leftFrameNum;        // 帧号
    int rightFrameNum;       // 帧号
    int64_t timeStamp;       // 时间戳
    int64_t DevTimeStampLeft;
    int64_t DevTimeStampRight;
    unsigned int nDataSize;  // 图像数据大小（字节数，用于安全操作）
    int nWidth;              // 图像宽度（像素）
    int nHeight;             // 图像高度（像素）
    int nPixelFormat;        // 像素格式（如Mono8、RGB24等，可选）
};

// 定义函数返回的错误码枚举（强类型，避免与其他枚举冲突）
enum class ErrorCode {
    // 通用成功状态（通常用 0 表示成功）
    SUCCESS = 0,
    ERROR_INVALID_PATH,          // 无效路径
    ERROR_LOAD_IMAGE_FAILED,       // 加载图像失败
    ERROR_QUEUE_EMPTY,           // 队列为空
    ERROR_THREAD_ALREADY_RUNNING,// 线程已在运行
    ERROR_THREAD_NOT_RUNNING,    // 线程未运行
    ERROR_INVALID_PARAMETERS      // 无效参数
};

class LoadImages
{
public:
    LoadImages();
    ~LoadImages();

    // 设置参数
    ErrorCode SetParaments(const char* imagePath, unsigned int maxQueueSize = 300);

    // 启动加载线程
    ErrorCode StartLoading();

    // 停止加载线程
    ErrorCode StopLoading();

    // 入队图像（由加载线程调用）
    ErrorCode EnqueueImage(const ImagePair& imagePair);

    // 出队图像（由用户调用）
    ErrorCode DequeueImage(ImagePair& imagePair);

    // 获取当前队列大小
    size_t GetQueueSize() const;

    // 检查队列是否为空
    bool IsQueueEmpty() const;

    // 清空队列
    void ClearQueue();

private:
    // 从路径加载图像的内部函数
    ErrorCode LoadFromPath();

    // 加载线程函数
    void LoadingThreadFunc();

private:
    //std::string m_path = "C:\\Users\\Administrator\\source\\repos\\QtWidgetsApplication2\\2025年12月12日新器械2";
    //std::string m_path = "E:\\Surgical Navigation\\Code&Document\\2025.12.19取图";
    //std::string m_path = "C:\\Users\\Administrator\\source\\repos\\QtWidgetsApplication2\\QtWidgetsApplication2\\Image Library";
    std::string m_path = "E:\\Test20251225_LRW\\20251225_Left_Right";
    unsigned int m_maxQueueSize;            // 队列最大长度
    unsigned int m_currentImageIndex;       // 当前加载的图像索引

    std::queue<ImagePair> m_imageQueue;     // 图像队列
    mutable CMVMutex m_queueMutex;          // 队列互斥锁

    std::thread m_loadingThread;            // 加载线程
    std::atomic<bool> m_isRunning;          // 线程运行标志
    std::atomic<bool> m_shouldStop;         // 停止标志
};