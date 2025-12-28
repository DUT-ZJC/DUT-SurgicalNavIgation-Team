#include "CV_Manager.hpp"

void CV_Manager::LoadImagesSet(bool flag)
{
	if (flag)
	{
		loadImage_Manager.StartLoading();
	}
	else
	{
		loadImage_Manager.StopLoading();
	}
}

void CV_Manager::VisualImageSet(bool flag)
{
	if (flag)
	{
		Startshowing();
	}
	else
	{
		Stopshowing();
	}
}

bool CV_Manager::Startshowing()
{
	if (m_isRunning) {
		std::cerr << "Visual thread is already running" << std::endl;
		return false;
	}
	m_shouldStop = false;
	m_isRunning = true;
	m_visualThread = std::thread(&CV_Manager::VisualThreadFunc, this);
	return true;
}

bool CV_Manager::Stopshowing()
{
	if (!m_isRunning) {
		return false;
	}

	m_shouldStop = true;

	if (m_visualThread.joinable()) {
		m_visualThread.join();
	}

	m_isRunning = false;

	return true;
}

void CV_Manager::updateImageSourcePath(std::string path)
{
	loadImage_Manager.SetParaments(path.c_str());
}

void CV_Manager::updateTemplate(std::vector<int>& reInstrumesID)
{
	instrumesID = reInstrumesID;
	p_instrumentMatcherManager->SetInstrumentsIDs(instrumesID);
}

//找不到点输出false
bool CV_Manager::FetchAndProcessImage()
{
	if (loadImage_Manager.DequeueImage(nowImage) != ErrorCode::SUCCESS) return false;
	leftPoints = p_poinrsMatcher_Manager->detectBalls(nowImage.leftImage);
	rightPoints = p_poinrsMatcher_Manager->detectBalls(nowImage.rightImage);
	bool result = p_poinrsMatcher_Manager->stereoMatch(leftPoints, rightPoints, filteredLeftPoints, filteredRightPoints);
	if (result) //应该加一个判断是否需要可视化
	{
		p_poinrsMatcher_Manager->visualizeMatchingProcess(nowImage.leftImage, nowImage.rightImage, leftPoints, rightPoints, filteredLeftPoints, filteredRightPoints, visualImagequeue);
	}
	return result;

}

bool CV_Manager::CorrespondenceAndReconstruction()
{
	return p_stereoDepthEstimator_Manager->Compute(filteredLeftPoints, filteredRightPoints, reconstructedPoints,map);
	
}


bool CV_Manager::MatcherAndLocalize()
{
	bool flag = p_instrumentMatcherManager->Match(reconstructedPoints);
	p_instrumentMatcherManager->Visuable(nowImage.leftImage, nowImage.rightImage, map, visualImagequeue);
	//instrumentMatcherManager.Localize();
	return flag;
}
/*
void CV_Manager::InstrmentsCalibrateSet(bool flag)
{
	if (flag) p_pivotCalibrator_Manager->StartCalibrate();
	else p_pivotCalibrator_Manager->StopCalibrate();
}

bool CV_Manager::MatcheForCalibration()
{
	if (p_instrumentMatcherManager->Calibrate(toolFrameArrary[0], toolFrameArrary[1]))
	{
		p_pivotCalibrator_Manager->EnqueuePose(toolFrameArrary[0], toolFrameArrary[1]);
		return true;
	}
	return false;
}
*/
void CV_Manager::VisualThreadFunc()
{
	cv::namedWindow("Feature Visualization", cv::WINDOW_NORMAL);

	while (!m_shouldStop)
	{
		Visualization();
		Sleep(33); // 约30fps
	}
}

void CV_Manager::Visualization()
{
	char key = cv::waitKey(1);

	if (key == 27) { // ESC键退出
		cv::destroyAllWindows();
		// 可以在这里添加程序退出逻辑
	}
	if (key != -1) {
		char c = static_cast<char>(key & 0xFF);

		if (c == 'a') { // 上一张
			visualImagequeue.move_prev();
		}
		else if (c == 'd') { // 下一张
			visualImagequeue.move_next();
		}
		
		else if (c == 's') { // 恢复实时模式 (Sync)
			visualImagequeue.reset_to_live();
		}
		
	}
	if (visualImagequeue.get_display_image(visualImage))
	{
		cv::putText(visualImage, visualImagequeue.get_status_text(), cv::Point(20, 40),
			cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
		cv::imshow("Feature Visualization", visualImage);
	}

}

bool CV_Manager::StartProcessing()
{
	if (m_processIsRunning)
	{	
		std::cerr << "Process thread is already running" << std::endl;
		return false;
	}
	m_processShouldStop = false;
	m_processIsRunning = true;
	m_processThread = std::thread(&CV_Manager::ProcessThreadFunc, this);
	return true;
}

bool CV_Manager::StopProcessing()
{
	if (!m_processIsRunning)
	{
		return false;
	}
	m_processShouldStop = true;

	if (m_processThread.joinable()) {
		m_processThread.join();
	}

	m_processIsRunning = false;

	return true;
}

void CV_Manager::ProcessThreadFunc()
{
	while (!m_processShouldStop)
	{
		if (!FetchAndProcessImage()) { printf("*********1******\n"); continue; }
		if(!CorrespondenceAndReconstruction()) { printf("*********2******\n"); continue; }
		if(!MatcherAndLocalize()) { printf("*********3******\n"); continue; }
	}
}
