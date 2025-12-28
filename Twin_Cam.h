#pragma once
#ifndef TWIN_CAM_H
#define TWIN_CAM_H

//#include "Image_Processing.h"
#include "Load_Image.hpp"
#include <string>
#include "LogPrint.h"
#include "MvCameraControl.h"
#include <mutex>
#include <system_error>
#include <functional>
#include <memory>
#include <queue>
#include <condition_variable>
#include <cstdint>
#include <Windows.h> 
#include "CMVMutex.hpp"
//前向声明
class StereoCameraSync;
struct ImagePair;
//回调接口
class IImageDataCallback
{
public:
    virtual ~IImageDataCallback() {}
    virtual void OnImageData(const ImagePair& imageData) = 0;
};

//封装的队列及其处理函数

class ImageFrameQueue
{
private:
    std::queue<MV_FRAME_OUT> m_queue;       // 存储ImageFrame的队列
    size_t m_maxSize = 100;           // 队列最大容量
    mutable std::mutex m_mutex;           // 互斥锁，保证线程安全
    std::condition_variable m_notEmpty;   // 非空条件变量
    std::condition_variable m_notFull;    // 非满条件变量

public:
    ImageFrameQueue() = default;
    //ImageFrameQueue(size_t maxSize) :m_maxSize(maxSize) {};
    ~ImageFrameQueue()
    {
        // 析构时清空队列，释放图像数据内存
        Clear();
    }

    // 禁止拷贝构造和赋值操作
    ImageFrameQueue(const ImageFrameQueue&) = delete;
    ImageFrameQueue& operator=(const ImageFrameQueue&) = delete;

    // 向队列添加元素，队列满时移除最旧元素
    void Push(const MV_FRAME_OUT& frame)
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        // 如果队列已满，先移除最旧的元素
        if (m_queue.size() >= m_maxSize)
        {
            // 释放旧元素的图像数据内存
            MV_FRAME_OUT oldFrame = m_queue.front();
            if (oldFrame.pBufAddr != nullptr)
            {
                delete[] oldFrame.pBufAddr;
                oldFrame.pBufAddr = nullptr;
            }
            m_queue.pop();
        }

        // 添加新元素
        m_queue.push(frame);
        lock.unlock();
        // 通知等待的线程队列非空
        m_notEmpty.notify_one();
    }

 
   
    // 尝试获取元素（非阻塞）
    bool TryPop(MV_FRAME_OUT& outFrame)
    {
        std::unique_lock<std::mutex> lock(m_mutex);

        if (m_queue.empty())
            return false;

        outFrame = m_queue.front();
        m_queue.pop();
        lock.unlock();
        m_notFull.notify_one();
        return true;
    }

    // 清空队列并释放内存  
    void Clear()
    {
        std::unique_lock<std::mutex> lock(m_mutex);

        while (!m_queue.empty())
        {
            MV_FRAME_OUT frame = m_queue.front();
            if (frame.pBufAddr != nullptr)
            {
                delete[] frame.pBufAddr;
                frame.pBufAddr = nullptr;
            }
            m_queue.pop();
        }
        lock.unlock();
        m_notFull.notify_all();
    }

    size_t Size() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_queue.size();
    }

    // 检查队列是否为空 未使用
    bool IsEmpty() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_queue.empty();
    }

    // 检查队列是否已满 未使用
    bool IsFull() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_queue.size() >= m_maxSize;
    }
};

//用于实现双相机同步回调 包含两个队列，一个回调函数,初始化话后 开启循环。
class Twin_CallBack
{
public:
    Twin_CallBack(std::function<void(StereoCameraSync*, IImageDataCallback*)> callback, IImageDataCallback* m_pImageCallback, StereoCameraSync* pStereoCameraSync)
        : left_queue(), right_queue(), m_callback(callback),
        left_lastProcessedFrame(0), right_lastProcessedFrame(0), ImageDataCallback(m_pImageCallback), parenttPtr(pStereoCameraSync)
    {
        // 启动监控线程（定时检查是否有新帧对）
        m_monitorThread = std::thread(&Twin_CallBack::MonitorThread, this);

    }

    ~Twin_CallBack()
    {
        m_running = false;
        if (m_monitorThread.joinable())
        {
            m_monitorThread.join();
            left_queue.Clear();
            right_queue.Clear();
        }
    }

    ImageFrameQueue left_queue;
    ImageFrameQueue right_queue;


    void SetFlag()
    {
        m_running = true;
    }
    void ResetFlag()
    {
        m_running = false;
    }
    // 相机1入队后调用：记录最新帧号
    void OnQueue1NewFrame(unsigned int newFrameNum)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        left_latestFrame = newFrameNum;
    }

    // 相机2入队后调用：记录最新帧号
    void OnQueue2NewFrame(unsigned int newFrameNum)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        right_latestFrame = newFrameNum;
    }

private:
    void MonitorThread()
    {
        while (m_running.load())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
            CheckAndTrigger();
        }
    }

    void CheckAndTrigger()
    {
        //std::lock_guard<std::mutex> lock(m_mutex);
        //触发条件：两个队列都有新帧（帧号大于上次处理的帧号）
        /*
        bool hasNewFrame1 = (left_latestFrame > left_lastProcessedFrame);
        bool hasNewFrame2 = (right_latestFrame > right_lastProcessedFrame);
        if (hasNewFrame1 && hasNewFrame2 &&!left_queue.IsEmpty()&& !right_queue.IsEmpty())
        {
            // 从队列取最新帧（实际项目中可能需要按帧号匹配最接近的帧）

            //
            MV_FRAME_OUT frame1, frame2;
            bool got1 = left_queue.TryPop(frame1);
            bool got2 = right_queue.TryPop(frame2);

            if (got1 && got2)
            {
                m_callback(frame1, frame2, ImageDataCallback);
                // 更新上次处理的帧号
                left_lastProcessedFrame = frame1.stFrameInfo.nFrameNum;
                right_lastProcessedFrame = frame2.stFrameInfo.nFrameNum;
            }
        }
        */
   
        m_callback(parenttPtr,ImageDataCallback);
    }

private:
    
    //std::function<void(MV_FRAME_OUT&, MV_FRAME_OUT&, IImageDataCallback*)> m_callback;
    std::function<void(StereoCameraSync*, IImageDataCallback*)> m_callback;
    std::mutex m_mutex;
    unsigned int left_latestFrame = 0;       // 队列1最新帧号
    unsigned int right_latestFrame = 0;       // 队列2最新帧号
    unsigned int left_lastProcessedFrame = 0;// 上次处理的队列1帧号
    unsigned int right_lastProcessedFrame = 0;// 上次处理的队列2帧号
    IImageDataCallback* ImageDataCallback;
    std::thread m_monitorThread;
    std::atomic<bool> m_running{ true };
    StereoCameraSync* parenttPtr;
};

// 双相机同步取图类
class StereoCameraSync
{
public:

    // 构造函数
    StereoCameraSync();

    // 析构函数
    ~StereoCameraSync();

    // 初始化双相机系统
    bool Initialize();

    // 枚举所有可用设备
    bool EnumDevices();

    // 创建双相机句柄
    bool CreateHandles();

    // 打开双相机
    bool OpenCameras();

    // 关闭双相机
    bool CloseCameras();

    //SDK 异常回调注册
    bool RegisterExceptionCallBack();
    // 开始同步取流
    bool StartSyncGrabbing();

    // 停止同步取流
    bool StopSyncGrabbing();

    // 注册主管理者对象|注册回调接口
    void RegisterManager(IImageDataCallback* pImageCallback);

    //存储图像到指定路径
    bool SaveImageToPath();
    bool SetSavePath(std::string PathString);
    std::array<cv::Mat, 2> GetSaveImage();
    //重启相机
    void ReStartCameras(void* handle_, bool RightOrLeft);

    // 反初始化双相机系统
    void Finalize();

private:

    //自旋锁 被用于图像保存方面
    CMVMutex MVMutex;
    //bool m_bGrapRunning;            // 抓取运行标志
    std::atomic<bool> m_LeftThreadFlag;
    std::atomic<bool> m_RightThreadFlag;
    std::atomic<bool> m_SaveTaskFlag;

    unsigned int leftIndex;
    void* leftCamera_handle_;

    unsigned int rightIndex;
    void* rightCamera_handle_;

    //相机列表
    MV_CC_DEVICE_INFO_LIST deviceList_;

    //左右图像队列
    ImageFrameQueue RightCamQueue;
    ImageFrameQueue LeftCamQueue;

    // 同步回调对象指针
    Twin_CallBack* stereoCallback_;
    // 用户数据

    // 保护图像数据的互斥锁
    std::mutex frameMutex_;

    // 左相机回调函数
    static void __stdcall LeftImageCallbackEx(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser);
    static void __stdcall leftCameraExceptionCallBack(unsigned int nMsgType, void* pUser);
    void GetLeftImageFrame();
    // 右相机回调函数
    static void __stdcall RightImageCallbackEx(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser);
    static void __stdcall rightCameraExceptionCallBack(unsigned int nMsgType, void* pUser);
    void GetRightImageFrame();
    // 相机同步回调函数
    static void __stdcall SynImageCallbackEx(StereoCameraSync* ptr, IImageDataCallback* m_pImageCallback);
    //static void __stdcall SynImageCallbackEx(MV_FRAME_OUT& frameLeft, MV_FRAME_OUT& frameRight, IImageDataCallback* m_pImageCallback);

    MV_SAVE_IMAGE_TO_FILE_PARAM_EX RightSaveFileParam;
    MV_SAVE_IMAGE_TO_FILE_PARAM_EX LeftSaveFileParam;


    std::thread leftImageThread;
    std::thread rightImageThread;
    // 图像回调接口
    IImageDataCallback* m_pImageCallback;

};




#endif // !TWIN_CAM_H
