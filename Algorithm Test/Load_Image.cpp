#include "Load_Image.hpp"
#include <iostream>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <Windows.h>
#include "Logger.hpp"

LoadImages::LoadImages()
    : m_maxQueueSize(60)
    , m_currentImageIndex(0)
    , m_isRunning(false)
    , m_shouldStop(false)
{
}

LoadImages::~LoadImages()
{
    StopLoading();
}

ErrorCode LoadImages::SetParaments(const char* imagePath, unsigned int maxQueueSize)
{
    if (imagePath == nullptr || strlen(imagePath) == 0) {
        std::cerr << "Invalid image path" << std::endl;
        return ErrorCode::ERROR_INVALID_PATH;
    }

    if (maxQueueSize == 0) {
        std::cerr << "Queue size must be greater than 0" << std::endl;
        return ErrorCode::ERROR_INVALID_PARAMETERS;
    }

    m_path = imagePath;
    m_maxQueueSize = maxQueueSize;
    m_currentImageIndex = 0;

    return ErrorCode::SUCCESS;
}

ErrorCode LoadImages::StartLoading()
{
    if (m_isRunning) {
        std::cerr << "Loading thread is already running" << std::endl;
        return ErrorCode::ERROR_THREAD_ALREADY_RUNNING;
    }

    if (m_path.empty()) {
        std::cerr << "Path not set. Call SetParaments first" << std::endl;
        return ErrorCode::ERROR_INVALID_PATH;
    }

    m_shouldStop = false;
    m_isRunning = true;
    m_loadingThread = std::thread(&LoadImages::LoadingThreadFunc, this);

    return ErrorCode::SUCCESS;
}

ErrorCode LoadImages::StopLoading()
{
    if (!m_isRunning) {
        return ErrorCode::ERROR_THREAD_NOT_RUNNING;
    }

    m_shouldStop = true;

    if (m_loadingThread.joinable()) {
        m_loadingThread.join();
    }

    m_isRunning = false;

    return ErrorCode::SUCCESS;
}

ErrorCode LoadImages::EnqueueImage(const ImagePair& imagePair)
{
    CMVAutoLock lock(m_queueMutex);

    // 如果队列已满，移除最旧的图像
    while (m_imageQueue.size() >= m_maxQueueSize) {
        m_imageQueue.pop();
        //if(logFlag) std::cout << "Queue full, removing oldest image. Current size: "
            //<< m_imageQueue.size() << std::endl;
    }

    m_imageQueue.push(imagePair);

    return ErrorCode::SUCCESS;
}

ErrorCode LoadImages::DequeueImage(ImagePair& imagePair)
{
    CMVAutoLock lock(m_queueMutex);

    if (m_imageQueue.empty()) {
        return ErrorCode::ERROR_QUEUE_EMPTY;
    }

    imagePair = m_imageQueue.front();
    m_imageQueue.pop();

    return ErrorCode::SUCCESS;
}

size_t LoadImages::GetQueueSize() const
{
    CMVAutoLock lock(m_queueMutex);
    return m_imageQueue.size();
}

bool LoadImages::IsQueueEmpty() const
{
    CMVAutoLock lock(m_queueMutex);
    return m_imageQueue.empty();
}

void LoadImages::ClearQueue()
{
    CMVAutoLock lock(m_queueMutex);
    while (!m_imageQueue.empty()) {
        m_imageQueue.pop();
    }
    std::cout << "Queue cleared" << std::endl;
}

ErrorCode LoadImages::LoadFromPath()
{
    // 这里实现从路径加载图像的逻辑
    // 示例：假设路径下有左右相机图像对
    // 格式：left_0000.jpg, right_0000.jpg, left_0001.jpg, right_0001.jpg...
    try {
        // 格式化文件名，使用4位数字编号
        std::ostringstream leftStream, rightStream;

        leftStream << m_path << "/left_" << std::setw(4) << std::setfill('0')
            << m_currentImageIndex << ".jpg";
        rightStream << m_path << "/right_" << std::setw(4) << std::setfill('0')
            << m_currentImageIndex << ".jpg";

        std::string leftImagePath = leftStream.str();
        std::string rightImagePath = rightStream.str();

        // 检查文件是否存在
        DWORD leftAttr = GetFileAttributesA(leftImagePath.c_str());
        DWORD rightAttr = GetFileAttributesA(rightImagePath.c_str());

        if (leftAttr == INVALID_FILE_ATTRIBUTES || rightAttr == INVALID_FILE_ATTRIBUTES) {
            // 文件不存在，重新从0开始

            /*
            if (m_currentImageIndex > 0) {
                std::cout << "Reached end of images, restarting from index 0" << std::endl;
                m_currentImageIndex = 0;
            }
            */
            m_currentImageIndex++;
            return ErrorCode::ERROR_INVALID_PATH;
        }

        // 加载图像
        cv::Mat leftImage = cv::imread(leftImagePath, cv::IMREAD_GRAYSCALE);
        cv::Mat rightImage = cv::imread(rightImagePath, cv::IMREAD_GRAYSCALE);

        if (leftImage.empty() || rightImage.empty()) {
            std::cerr << "Failed to load images at index " << m_currentImageIndex << std::endl;
            return ErrorCode::ERROR_LOAD_IMAGE_FAILED;
        }

        // 创建图像对
        ImagePair imagePair;
        imagePair.leftImage = leftImage.clone();  // 使用clone确保数据独立
        imagePair.rightImage = rightImage.clone();
        imagePair.leftFrameNum = m_currentImageIndex;
        imagePair.rightFrameNum = m_currentImageIndex;
        imagePair.nWidth = leftImage.cols;
        imagePair.nHeight = leftImage.rows;
        imagePair.nDataSize = static_cast<unsigned int>(leftImage.total() * leftImage.elemSize());
        imagePair.nPixelFormat = leftImage.type();

        // 获取当前时间戳（使用高精度计时器）
        LARGE_INTEGER frequency, counter;
        QueryPerformanceFrequency(&frequency);
        QueryPerformanceCounter(&counter);

        int64_t timestamp = (counter.QuadPart * 1000000) / frequency.QuadPart; // 微秒
        imagePair.timeStamp = timestamp;
        imagePair.DevTimeStampLeft = timestamp;
        imagePair.DevTimeStampRight = timestamp;
        imagePair.timeInterval = 0;

        // 入队
        ErrorCode result = EnqueueImage(imagePair);
        if (result == ErrorCode::SUCCESS) {
            //std::cout << "Loaded image pair " << m_currentImageIndex
               // << ", Queue size: " << GetQueueSize() << std::endl;
        }

        m_currentImageIndex++;

        return ErrorCode::SUCCESS;

    }
    catch (const std::exception& e) {
        std::cerr << "Exception in LoadFromPath: " << e.what() << std::endl;
        return ErrorCode::ERROR_LOAD_IMAGE_FAILED;
    }
    catch (...) {
        std::cerr << "Unknown exception in LoadFromPath" << std::endl;
        return ErrorCode::ERROR_LOAD_IMAGE_FAILED;
    }
}

void LoadImages::LoadingThreadFunc()
{
    std::cout << "Loading thread started, thread ID: "
        << std::this_thread::get_id() << std::endl;

    while (!m_shouldStop) {
        ErrorCode result = LoadFromPath();

        if (result != ErrorCode::SUCCESS) {
            // 加载失败，等待一段时间后重试
            printf("%d", static_cast<int>(result));
            Sleep(100); // Windows API，等待100毫秒
            continue;
        }

        // 控制加载速率，避免过快填满队列
        // 根据实际需求调整，这里设置为约30fps
        Sleep(33); // 约30fps
    }

    std::cout << "Loading thread stopped" << std::endl;
}