#include "Image_Processing.h"

using namespace std;
// 全局变量初始化
std::vector<cv::Point3f> g_globalPoints3D;



std::string getCurrentTimeString() {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch() % std::chrono::seconds(1)
    );

    std::tm timeinfo;
    localtime_s(&timeinfo, &now_c); // 线程安全版本（VS2022支持）

    std::stringstream ss;
    ss << std::put_time(&timeinfo, "%H:%M:%S")
        << "." << std::setw(3) << std::setfill('0') << now_ms.count();
    return ss.str();
}


ImageProcessing::ImageProcessing()
    : m_bThreadRunning(false),
    m_mutexQueue(),
    m_eventTaskAvailable(CreateEvent(NULL, FALSE, FALSE, NULL)),
    m_eventShutdown(CreateEvent(NULL, TRUE, FALSE, NULL)) ,
    cameraParams(), outFile("3d_points.txt", std::ios::out | std::ios::app)
    //poseCalculator()
{


    if (!outFile.is_open()) {
        std::cerr << "无法打开文件 3d_points.txt！" << std::endl;
        // 可根据需求处理错误（如抛出异常、设置标志位等）
    }

}

ImageProcessing::~ImageProcessing() {
    outFile.close();
    Release();
    CloseHandle(m_eventTaskAvailable);
    CloseHandle(m_eventShutdown);
}

void ImageProcessing::RegisterCalculateCallback(ICalculateCallback* pCallback)
{
    CalculateResultCallback = pCallback;
}
bool ImageProcessing::Initialize() {

    if (m_bThreadRunning) {
        std::cerr << "Already initialized" << std::endl;
        return false;
    }
    // 创建工作线程
    unsigned int threadId;
    m_bThreadRunning = true;
    m_hWorkerThread = (HANDLE)_beginthreadex(
        NULL, 0, WorkerThread, this, 0, &threadId
    );

    if (!m_hWorkerThread) {
        std::cerr << "Failed to create worker thread" << std::endl;
        return false;
    }

    return true;
}

bool ImageProcessing::Release() {
    if (!m_bThreadRunning) return 0;

    // 通知线程退出并等待
    m_bThreadRunning = false;
    SetEvent(m_eventShutdown);
    WaitForSingleObject(m_hWorkerThread, INFINITE);
    CloseHandle(m_hWorkerThread);

    // 清空队列
    m_mutexQueue._Lock();
    while (!m_imageQueue.empty()) {
        m_imageQueue.pop();
    }
    m_mutexQueue._Unlock();

    return true;
}

bool ImageProcessing::enqueueImagePair(const ImagePair& imgPair) {
    m_mutexQueue._Lock();

    if (m_imageQueue.size() >= MAX_QUEUE_SIZE) {
        m_imageQueue.pop();
    }

    m_imageQueue.push(imgPair);
    //SetEvent(m_eventTaskAvailable);  // 通知有新任务
    m_mutexQueue._Unlock();  
    return true;
}

bool ImageProcessing::dequeueImagePair(ImagePair& imgPair ,bool POP) {
    m_mutexQueue._Lock();

    if (m_imageQueue.empty()) {
        m_mutexQueue._Unlock();
        printf("Equeue is Empty\n");
        return false;
    }

    if (POP == true)
    {
        imgPair = m_imageQueue.front();
        m_imageQueue.pop();
    }
    else
    {
        imgPair = m_imageQueue.front();
        m_imageQueue.pop();
        //imgPair.leftImage = imgPair.leftImage.clone();
        //imgPair.rightImage = imgPair.rightImage.clone();
    }
    m_mutexQueue._Unlock();
    return true;
}

size_t ImageProcessing::getQueueSize() {
    m_mutexQueue._Lock();
    size_t size = m_imageQueue.size();
    m_mutexQueue._Unlock();
    return size;
}

unsigned int __stdcall ImageProcessing::WorkerThread(void* pParam) {
    ImageProcessing* pThis = (ImageProcessing*)pParam;
    if (pThis) {
        return pThis->WorkerThreadProc();
    }
    return 0;
}

unsigned int ImageProcessing::WorkerThreadProc() {
    HANDLE events[] = { m_eventTaskAvailable, m_eventShutdown };

    ImagePair imgPair;
    while (m_bThreadRunning) {
        // 等待任务或退出事件
        /*
        DWORD waitResult = WaitForMultipleObjects(
            2, events, FALSE, INFINITE
        );

        if (waitResult == WAIT_OBJECT_0 + 1) {
            // 收到退出事件
            printf("*************2*********");
            break;
        }
        //else if (waitResult == WAIT_OBJECT_0) {
            // 处理队列中的图像对
            
        //}
        */
        if (dequeueImagePair(imgPair)) {
            processImagePair(imgPair);
        }
        else
        {
            this_thread::sleep_for(chrono::microseconds(100));
        }
        
    }
    return 0;
}

void ImageProcessing::processImagePair(const ImagePair& imgPair) {
    // 定义计时类型（高分辨率时钟）
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = Clock::time_point;
    using Duration = std::chrono::duration<double>; // 时间差（秒，double类型）

    // 初始化各阶段时间变量
    double totalTime = 0.0;
    double detectBallsTime = 0.0;
    double stereoMatchTime = 0.0;
    double calculate3DTime = 0.0;
    double calculatePoseTime = 0.0;

    TimePoint totalStart = Clock::now(); // 总时间起点

    try {
        // 1. 检测左右图像中的球（计时）
        TimePoint detectLeftStart = Clock::now();
        std::vector<cv::Point2f> leftPoints = m_ballDetector.detectBalls(imgPair.leftImage);
        detectBallsTime += Duration(Clock::now() - detectLeftStart).count();

        TimePoint detectRightStart = Clock::now();
        std::vector<cv::Point2f> rightPoints = m_ballDetector.detectBalls(imgPair.rightImage);
        detectBallsTime += Duration(Clock::now() - detectRightStart).count();

        if (leftPoints.empty() || rightPoints.empty()) {
            std::cout << "No balls detected in one of the images" << std::endl;
            // 计算总时间（仅执行到检测阶段）
            totalTime = Duration(Clock::now() - totalStart).count();
            printTimeStats(totalTime, detectBallsTime, stereoMatchTime, calculate3DTime, calculatePoseTime);
            return;
        }

        // 2. 立体匹配（计时）
        TimePoint stereoStart = Clock::now();
        std::vector<cv::Point2f> matchedRight = m_ballDetector.stereoMatch(
            leftPoints, rightPoints,
            cameraParams.GetLeftCameraMatrix(), cameraParams.GetRightCameraMatrix(),
            cameraParams.GetLeftDistortionCoeffs(), cameraParams.GetRightDistortionCoeffs(),
            cameraParams.GetRotationMatrix(), cameraParams.GetTranslationVector()
        );
        stereoMatchTime = Duration(Clock::now() - stereoStart).count();

        // 3. 计算3D点（计时，累加每次调用耗时）
        std::vector<cv::Point3f> points3D;
        for (size_t i = 0; i < leftPoints.size() && i < matchedRight.size(); ++i) {
            if (DistanceCalculator::isValidPointPair(leftPoints[i], matchedRight[i])) {
                TimePoint calc3DStart = Clock::now();
                cv::Point3f pt3D = DistanceCalculator::calculate3DPoint(
                    leftPoints[i], matchedRight[i],
                    cameraParams.GetFocalLengthMM(), cameraParams.GetBaseline(),
                    cameraParams.GetLeftCx(), cameraParams.GetLeftCy()
                );
                calculate3DTime += Duration(Clock::now() - calc3DStart).count();

                points3D.push_back(pt3D);
                if (points3D.size() >= 4) break;
            }
        }

        // 4. 计算位姿（计时，仅当有4个3D点时执行）
        if (points3D.size() == 4) {
            static Pose6DOF pose;
            TimePoint calcPoseStart = Clock::now();
            Pose6DOFCalculator poseCalculator(points3D);
            poseCalculator.calculatePose(points3D, pose);
            calculatePoseTime = Duration(Clock::now() - calcPoseStart).count();

            // 日志和回调（不计时，属于IO操作）
            LogPrint::PrintPointDate(points3D, imgPair.leftFrameNum, imgPair.rightFrameNum,
                imgPair.timeStamp, imgPair.DevTimeStampLeft, imgPair.DevTimeStampRight);
            CalculateResultCallback->OnCalculateComplete(points3D[0], pose.roll, pose.pitch, pose.yaw);

            // 输出3D点坐标（原逻辑保留）
            for (size_t i = 0; i < 4; ++i) {
                std::cout << "points3D[" << i << "] = ("
                    << points3D[i].x << ", "
                    << points3D[i].y << ", "
                    << points3D[i].z << ")" << std::endl;
            }
        }

        // 5. 结果存入全局变量（原逻辑保留，不计时）
        // WaitForSingleObject(g_mutexGlobalPoints, INFINITE);
        // g_globalPoints3D.insert(g_globalPoints3D.end(), points3D.begin(), points3D.end());
        // ReleaseMutex(g_mutexGlobalPoints);

    }
    catch (const std::exception& e) {
        std::cerr << "Error processing image pair: " << e.what() << std::endl;
    }

    // 计算总时间（无论正常执行还是异常，均统计到结束）
    totalTime = Duration(Clock::now() - totalStart).count();
    // 输出时间统计
    printTimeStats(totalTime, detectBallsTime, stereoMatchTime, calculate3DTime, calculatePoseTime);
}

// 辅助函数：格式化输出时间统计结果
void ImageProcessing::printTimeStats(double total, double detect, double stereo, double calc3D, double calcPose) {
    std::cout << "\n=== 图像处理时间统计 ===" << std::endl;
    std::cout << "总执行时间：" << std::fixed << std::setprecision(6) << total << " 秒" << std::endl;

    // 计算占比（处理总时间为0的特殊情况，避免除0）
    auto getPercentage = [total](double part) {
        return (total < 1e-9) ? 0.0 : (part / total) * 100;
        };

    std::cout << "1. detectBalls（左右图合计）：" << detect << " 秒，占比："
        << std::fixed << std::setprecision(2) << getPercentage(detect) << "%" << std::endl;
    std::cout << "2. stereoMatch：" << stereo << " 秒，占比："
        << getPercentage(stereo) << "%" << std::endl;
    std::cout << "3. calculate3DPoint（累计）：" << calc3D << " 秒，占比："
        << getPercentage(calc3D) << "%" << std::endl;
    std::cout << "4. calculatePose：" << calcPose << " 秒，占比："
        << getPercentage(calcPose) << "%" << std::endl;
    std::cout << "========================\n" << std::endl;
}

void ImageProcessing::RegisterManager_2(ICalculateCallback* manager)
{ 
    CalculateResultCallback = manager;
}
