#include "Manager.h"

Manager::Manager() :CalculateThread(), TwinCamThread(), signalFlag(true){}

Manager::~Manager(){}

void Manager::Manager_Init()
{
	if (!TwinCamThread.Initialize()) throw std::runtime_error("相机SDK初始化失败");

	if (!TwinCamThread.EnumDevices()) throw std::runtime_error("无法获取相机设备");

	if (!TwinCamThread.CreateHandles()) throw std::runtime_error("相机句柄获取失败");

	if (!TwinCamThread.OpenCameras()) throw std::runtime_error("相机打开失败");

	TwinCamThread.RegisterManager(this);
	//CalculateThread.RegisterManager_2(this);
	std::vector<int> ids = { 1,2,3,4,5,6 };
	CalculateThread.updateTemplate(ids);
	CalculateThread.VisualImageSet(true);

}

void Manager::GO()
{
	//if (!CalculateThread.Initialize()) throw runtime_error("计算线程初始化失败");
	if (!CalculateThread.StartProcessing()) throw std::runtime_error("计算线程初始化失败");
	if (!TwinCamThread.StartSyncGrabbing()) throw std::runtime_error("捕获图像失败");
	

}

void Manager::STOP()
{
	if(!TwinCamThread.StopSyncGrabbing()) throw std::runtime_error("停止捕获失败");

}

void Manager::OnImageData(const ImagePair& imageData)
{
	
	//CalculateThread.enqueueImagePair(imageData);
	CalculateThread.loadImage_Manager.EnqueueImage(imageData);
} 
 

/*
void Manager::OnCalculateComplete(const cv::Point3f& Position , float roll, float pitch, float yaw)
{
	if (signalFlag.load())
	{
		signalFlag.store(false);
		LogPrint::PrintSinglePointData(Position);
		emit Position_Send(Position,roll, pitch, yaw);

	}
}
*/
