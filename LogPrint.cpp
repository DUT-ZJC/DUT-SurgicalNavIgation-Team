#include "LogPrint.h"

CMVMutex LogPrint::m_logMutex; // 这里会调用 CMVMutex 的默认构造函数
std::fstream LogPrint::logFile_1("Log_Message.txt", std::ios::out | std::ios::app);
std::string int64TsToBeijingTime(int64_t msTs);
std::string uint64NanoTsToBeijingTimeMs(uint64_t nanoTs);
LogPrint::LogPrint() 
{
    logFile_1 << "\n***************Dividing Line******************" << getCurrentTimeString() << "***************Dividing Line******************\n"
        <<
        "\n***************Dividing Line******************" << getCurrentTimeString() << "***************Dividing Line******************" << std::endl;
}

std::string LogPrint::getCurrentTimeString() {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch() % std::chrono::seconds(1)
    );

    std::tm timeinfo;
    localtime_s(&timeinfo, &now_c); // 线程安全版本（VS2022支持）

    std::stringstream ss;
    ss << "[" <<std::put_time(&timeinfo, "%H:%M:%S")
        << "." << std::setw(3) << std::setfill('0') << now_ms.count() << "]\t";

    return ss.str();

}


void LogPrint::PrintPointDate(const std::vector<cv::Point3f>& points3D, const int& leftFrameNum, const int& rightFrameNum, const int64_t timeStamp, const uint64_t DevTIMEStampLeft, const uint64_t DevTIMEStampRight)
{
    m_logMutex._Lock();
    logFile_1 << getCurrentTimeString()<< "Image TimeStamp:" << "[" << int64TsToBeijingTime(timeStamp) << "]"
        << "[" << uint64NanoTsToBeijingTimeMs(DevTIMEStampLeft) << "]" << "[" << uint64NanoTsToBeijingTimeMs(DevTIMEStampRight) << "]"
        <<"leftFrameNum: "
        << leftFrameNum << "\trightFrameNum: "
        << rightFrameNum << "\tlocation: ";

    // 写入所有3D点
    size_t pointIndex = 1;
    for (const auto& pt : points3D) {
        // 格式：[时间] X Y Z
        logFile_1 << "Index" << pointIndex << " " << pt.x << " "
            << pt.y << " "
            << pt.z << "\t\t";
        pointIndex++;
    }
    logFile_1 << std::endl;
    m_logMutex._Unlock();

}

void LogPrint::PrintSinglePointData(const cv::Point3f& point3D)
{
    m_logMutex._Lock();
    logFile_1 << "\n" << getCurrentTimeString()
        << " Location Send:\t "
        << "X: " << point3D.x << " "
        << "Y: " << point3D.y << " "
        << "Z: " << point3D.z
        << std::endl;
    m_logMutex._Unlock();
}


void LogPrint::PrintString(const char* warnLog)
{
    m_logMutex._Lock();
    logFile_1 << getCurrentTimeString() <<"\n***************" << "WARNING" << "***************\n"
        << warnLog <<
        "\n***************" << "WARNING" << "***************" << std::endl;
    m_logMutex._Unlock();
}


static std::string int64TsToBeijingTime(int64_t msTs) {
    time_t secTs = static_cast<time_t>(msTs / 1000);
    int64_t remainMs = msTs % 1000;

    tm utcTm;
    gmtime_s(&utcTm, &secTs);  // 线程安全的UTC时间转换

    utcTm.tm_hour += 8;  // 转换为北京时间（UTC+8）
    mktime(&utcTm);  // 修正时间（处理跨天情况）

    char buf[20];  // 缓冲区大小可适当减小，只需容纳时间部分
    // 仅保留 时:分:秒.毫秒 格式
    sprintf_s(buf, sizeof(buf), "%02d:%02d:%02d.%03lld",
        utcTm.tm_hour,
        utcTm.tm_min,
        utcTm.tm_sec,
        remainMs);

    return buf;
}

static std::string uint64NanoTsToBeijingTimeMs(uint64_t nanoTs) {
    // 1. 从纳秒拆分出秒和毫秒（忽略微秒和纳秒的低3位）
    time_t secTs = static_cast<time_t>(nanoTs / 1000000000ULL);  // 秒级时间戳（UTC）
    uint64_t msPart = (nanoTs % 1000000000ULL) / 1000000ULL;     // 提取毫秒部分（0-999）

    // 2. 转换为UTC时区的tm结构（线程安全）
    tm utcTm{};  // 初始化避免随机值
    gmtime_s(&utcTm, &secTs);

    // 3. 转换为北京时区（UTC+8），手动修正跨天
    utcTm.tm_hour += 8;
    if (utcTm.tm_hour >= 24) {
        utcTm.tm_hour -= 24;  // 仅处理小时溢出（无需完整日期修正，因不输出日期）
    }

    // 4. 格式化输出"时:分:秒.毫秒"（共12字符 + 结束符，缓冲区13字节足够）
    char buf[13] = { 0 };
    sprintf_s(buf, sizeof(buf), "%02d:%02d:%02d.%03llu",
        utcTm.tm_hour,
        utcTm.tm_min,
        utcTm.tm_sec,
        msPart);  // 毫秒部分（000-999）

    return std::string(buf);
}
