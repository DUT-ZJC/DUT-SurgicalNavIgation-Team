#pragma once
#include "Cam_Paras.hpp"
#include "Load_Image.hpp"
#include "Match_Points.hpp"
#include <condition_variable>
#include <mutex>
#include "Match_Instruments.hpp"
#include "StereoDepthEstimator.hpp"
#include "VisualQueue.hpp"
//#include "Instruments_Calibration.hpp"

/*
class ICalculateCallback 
{

public:
	// 显式构造函数，符合Qt对象树管理规范
	 ICalculateCallback() = default;

	// 纯虚析构函数需要提供实现
	 ~ICalculateCallback() = default;

	// 回调接口声明
	virtual void OnCalculateComplete(const cv::Point3f& Position, float roll, float pitch, float yaw) = 0;
};
*/

class CV_Manager
{
public:
	CV_Manager():params_Manager(), loadImage_Manager()
	{
		p_poinrsMatcher_Manager = std::make_unique<PointsMatcher>(params_Manager.GetLeftCameraMatrix(), 
			params_Manager.GetRightCameraMatrix(),
			params_Manager.GetLeftDistortionCoeffs(),
			params_Manager.GetRightDistortionCoeffs(),
			params_Manager.GetRotationMatrix(),
			params_Manager.GetTranslationVector()
		);

		p_instrumentMatcherManager = std::make_unique<InstrumentsMatcher>(instrumentPath);

		cv::Size imageSize(3072, 2048);
		p_stereoDepthEstimator_Manager = std::make_unique<StereoDepthEstimator>(
			params_Manager.GetLeftCameraMatrix(),
			params_Manager.GetLeftDistortionCoeffs(),
			params_Manager.GetRightCameraMatrix(),
			params_Manager.GetRightDistortionCoeffs(),
			params_Manager.GetRotationMatrix(),
			params_Manager.GetTranslationVector(),
			imageSize
		);

		//p_pivotCalibrator_Manager = std::make_unique<RealTimePivotCalibrator>();
	}

	~CV_Manager() {
		StopProcessing();
		Stopshowing();
	}

	//循环载入图像 
	void LoadImagesSet(bool flag);

	//更新 修改图像地址
	void updateImageSourcePath(std::string path);

	//更新器械模板
	void updateTemplate(std::vector<int>& reInstrumesID);

	//图像处理线程控制
	bool StartProcessing();
	bool StopProcessing();


	//可视化
	void VisualImageSet(bool flag);

	LoadImages loadImage_Manager;
private:
	//取图并处理，找不到点输出false
	bool FetchAndProcessImage();

	bool CorrespondenceAndReconstruction();

	bool MatcherAndLocalize();

private:
	CameraParams params_Manager;
	//LoadImages loadImage_Manager;

	std::unique_ptr<PointsMatcher> p_poinrsMatcher_Manager;
	std::unique_ptr<StereoDepthEstimator> p_stereoDepthEstimator_Manager;
	std::unique_ptr<InstrumentsMatcher> p_instrumentMatcherManager;
	//std::unique_ptr<RealTimePivotCalibrator> p_pivotCalibrator_Manager;

	bool Startshowing();
	bool Stopshowing();
	std::atomic<bool> m_isRunning;
	std::atomic<bool> m_shouldStop ;
	std::thread m_visualThread;
	void VisualThreadFunc();
	void Visualization();
	void ProcessThreadFunc();

	std::thread m_processThread;            // 处理线程
	std::atomic<bool> m_processIsRunning;          // 线程运行标志
	std::atomic<bool> m_processShouldStop;         // 停止标志
private:

	ImagePair nowImage;
	cv::Mat visualImage;
	BrowsableQueue visualImagequeue;

	std::vector<cv::Point2f> leftPoints;
	std::vector<cv::Point2f> rightPoints;

	std::vector<cv::Point2f> filteredLeftPoints;
	std::vector<std::vector<cv::Point2f>> filteredRightPoints;

	std::vector<cv::Point3f> reconstructedPoints;
	std::vector<std::array<cv::Point2f, 2>> map;

	std::string instrumentPath = "C:\\Users\\Administrator\\source\\repos\\QtWidgetsApplication2\\Algorithm Test\\Instruments\\";
	std::vector<int> instrumesID;

	std::array<std::array<cv::Mat, 12>, 2>  toolFrameArrary;
};