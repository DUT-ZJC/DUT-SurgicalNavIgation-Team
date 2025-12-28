#ifndef MANAGER_H
#define MANAGER_H

#include "Twin_Cam.h"
//#include "Image_Processing.h"
#include "CV_Manager.hpp"
#include <iostream>
#include <future>
#include <QObject>
#include "LogPrint.h"

class Manager: public QObject,public IImageDataCallback, public LogPrint
{
Q_OBJECT

signals:  // 信号声明关键字
	void Position_Send(const cv::Point3f& Position, float roll, float pitch, float yaw);

public:
	Manager();
	~Manager();
	


	void Manager_Init();

	void GO();
	void STOP();

	virtual void OnImageData(const ImagePair& imageData) override;

	//virtual void OnCalculateComplete(const cv::Point3f& Position , float roll, float pitch, float yaw) override;

	//ImageProcessing CalculateThread;
	CV_Manager CalculateThread;
	StereoCameraSync TwinCamThread;
	 
	std::atomic<bool> signalFlag;
	/*std::promise<Point3f> prom_position;
	std::future<Point3f> fut_position = prom_position.get_future();*/
};

#endif //  MANAGER_H

