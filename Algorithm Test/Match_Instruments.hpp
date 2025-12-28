#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <bitset>
#include <map>
#include <array>
#include <deque>
#include <yaml-cpp/yaml.h>
#include <format>
#include "spdlog/spdlog.h"
#include <algorithm>
#include "Logger.hpp"
#include "VisualQueue.hpp"

// 为 bitset<3> 重载 < 运算符
struct BitsetCompare {
	bool operator()(const std::bitset<3>& lhs, const std::bitset<3>& rhs) const {
		// 利用 to_ulong() 转成整数进行比较
		return lhs.to_ulong() < rhs.to_ulong();
	}
};

struct InstrumentPose
{
	int ID;
	cv::Point3f Position; 
	float roll;
	float pitch;
	float yaw;

	cv::Mat R;
};

struct  InstrumentTemplate
{
	//输入参数 包括 边、各个坐标系（尖点向量和转换矩阵）
	int ID;
	std::string name;
	std::array<std::array<double,2>,3> edges;
	std::map<std::bitset<3>, std::array<cv::Point3f, 3>, BitsetCompare> vectorMap;
	std::map<std::bitset<3>, std::array<cv::Mat, 3>, BitsetCompare> matrixMap;

	//搜索结果参数  ****如果搜索和计算线程 分开 则需要加锁，使用条件变量。
	std::atomic<bool> IsFind = false;
	//bool IsFind;
	std::bitset<3> pointEncoding;
	int	missingEocoding;
	unsigned int max_Queue = 60;
	std::deque<std::array<cv::Point3f, 4>> pointsQueue;  //第四个点可能为空 

	void AddPoins(std::array<cv::Point3f, 4>& points) {
		if (pointsQueue.size() >= max_Queue) pointsQueue.pop_front();
		pointsQueue.push_back(points); }

	bool GetPoins(std::array<cv::Point3f, 4>& points){
		if (!pointsQueue.empty()) { points = pointsQueue.back(); return true;}
		return false;
	}

	bool GetPoins(int index, std::array<cv::Point3f, 4>& points) {
		if (pointsQueue.size() > index) { points = pointsQueue[index]; return true; }
		return false;
	}
	
	InstrumentTemplate() = default;
	InstrumentTemplate(
		const std::array<std::array<double, 2>, 3>& initEdges,
		const std::initializer_list<std::pair<const std::bitset<3>, std::array<cv::Point3f, 3>>>& initVectorMap,
		const std::initializer_list<std::pair<const std::bitset<3>, std::array<cv::Mat, 3>>>& initMaixMap
	) : edges(initEdges), vectorMap(initVectorMap), matrixMap(initMaixMap){
	}

	// 当 vector 执行 push_back 时，会调用这个函数 
	InstrumentTemplate(const InstrumentTemplate& other)
		: ID(other.ID),
		name(other.name),
		edges(other.edges),
		vectorMap(other.vectorMap),
		matrixMap(other.matrixMap),

		// --- 关键点在这里 ---
		// 我们不拷贝 other.IsFind，而是强制初始化为 false
		IsFind(false),
		// --------------------

		pointEncoding(other.pointEncoding),
		missingEocoding(other.missingEocoding),
		max_Queue(other.max_Queue),
		pointsQueue(other.pointsQueue)
	{
		// 函数体为空即可，所有工作都在初始化列表中完成了
	}
};

enum class AlgorithmStage {
	FirstStage,
	SecondStage,
	ThirdStage,
	FourthStage
};

class InstrumentsMatcher
{
public:
	InstrumentsMatcher(std::string& templatePath);

	~InstrumentsMatcher() {};

	void SetInstrumentsIDs(std::vector<int>& ids);
	void SetInstrumentsIDs(int id);

	bool Match(std::vector<cv::Point3f>& Points);

	void Localize();
	bool Calibrate(std::array<cv::Mat, 12>& rvec, std::array<cv::Mat, 12>& tvec);

	void ResetAllInstrumentFindFlags();

	void Visuable(const cv::Mat& leftImage,
		const cv::Mat& rightImage,
		const std::vector<std::array<cv::Point2f, 2>>& map,
		BrowsableQueue& visualImageQueue);
private:  

	void GolbalMatch(std::vector<cv::Point3f>& Points, std::vector<bool> shadowIndex);
	void RapidMatch();

	void LoadTemplate();
	void LoadTemplate(int& id);
	
	std::array<cv::Mat, 2> ComputeToolFrame(const std::array<cv::Point3f, 4>& pints, int misscoding);
private:
	std::string templateFileName;
	const double telorableError = 3; //单位毫米
	std::vector<int> currtntInstrumentsIDs;
	std::vector<int> pendingInstrumentsIDs;
	std::vector<InstrumentTemplate> Instruments;
	std::vector<bool> shadowIndex;

	int matchCount;
	std::vector<int> idForPoints;
};
