#pragma once
#ifndef LOGPRINT_H
#define LOGPRINT_H

#include "Twin_Cam.h"
#include <fstream>
#include <iostream>
//#include "Image_Processing.h"
#include <ctime>
#include <cstdint>
#include <string>

class CMVMutex;

class LogPrint
{
public:
	LogPrint() ;
	~LogPrint() {}; //默认析构函数

public:

	static void PrintString(const char* stringLog);
	static void PrintPointDate(const std::vector<cv::Point3f>& points3D, const int& leftFrameNum, const int& rightFrameNum, const int64_t timeStamp ,const uint64_t DevTIMEStampLeft,const uint64_t DevTIMEStampRight);
	static void PrintSinglePointData(const cv::Point3f& point3D);

private:

	static std::fstream logFile_1;
	//std::fstream logFile_2;
	static std::string getCurrentTimeString();
	static CMVMutex m_logMutex;
};


#endif // !LOGPRINT_H
