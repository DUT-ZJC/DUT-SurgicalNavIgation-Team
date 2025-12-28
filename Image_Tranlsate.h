#pragma once
#ifndef IMAGE_TRANSLATE_H
#define IMAGE_TRANSLATE_H

#include <QThread>
#include <opencv2/opencv.hpp>
#include "Manager.h"
//#include "VTK_Viewer.h"
#include <QPixmap>
#include <QImage>
#include <QObject>  // 必须包含，否则 Q_OBJECT 无法解析
#include <QDebug>

static QImage preprocessImage(const cv::Mat& grayMat);

class Image_Translate : public QThread
{
Q_OBJECT
signals:
	// 信号：发送处理后的灰度图（参数为 const 引用，避免拷贝）
    void ImageReady(const QImage& leftImg, const QImage& rightImg);


public:
    Image_Translate(Manager* MainManager) :p_Manager(MainManager) {}

    ~Image_Translate() override {};


protected:
	void run() override
	{
		while (showImage)
		{
            QThread::msleep(100);
            
			ImagePair Pair;
            if (p_Manager->CalculateThread.loadImage_Manager.DequeueImage(Pair) != ErrorCode::SUCCESS)
            {
                QThread::msleep(100);
                continue;
            }
			QImage leftQImage = preprocessImage(Pair.leftImage);
            QImage rightQImage = preprocessImage(Pair.rightImage);

            if (!leftQImage.isNull() && !rightQImage.isNull()) {
                emit ImageReady(leftQImage, rightQImage);
            }
            
            printf("\n******************Send Pix*************\n");
		}
	}  

public:
	void Set() { showImage = true; }
	void ReSet() { showImage = false; }

private:
	std::atomic<bool> showImage = false;
	Manager* p_Manager;


};



    // 静态函数：cv::Mat(灰度图)转QPixmap
// 静态函数：cv::Mat(灰度图)转QPixmap（修复缩放问题）

static QImage preprocessImage(const cv::Mat& mat) {
    if (mat.empty() || mat.channels() != 1) {
        return QImage();
    }

    // 子线程中先缩放到目标尺寸（如1/4大小），减少主线程绘制压力
    cv::Mat scaledMat;
    cv::resize(mat, scaledMat, cv::Size(), 0.25, 0.25, cv::INTER_LINEAR);

    // 转换为QImage（格式匹配，无隐式转换）
    return QImage(
        scaledMat.data,
        scaledMat.cols,
        scaledMat.rows,
        scaledMat.step,
        QImage::Format_Grayscale8
    );
}

inline std::array<QImage, 2> preprocessImageArray(const std::array<cv::Mat, 2>& matArray) {
    std::array<QImage, 2> result;

    // 通用处理函数（复用逻辑，避免重复代码）
    auto processSingleMat = [](const cv::Mat& mat) -> QImage {
        // 1. 校验输入有效性：非空、单通道、8位深度
        if (mat.empty() || mat.channels() != 1 || mat.depth() != CV_8U) {
            qWarning("无效灰度图：空图像/非单通道/非8位深度");
            return QImage();
        }

        // 2. 缩放：3072×2048 → 768×512（0.25倍，尺寸合理且性能高）
        cv::Mat scaledMat;
        cv::resize(mat, scaledMat, cv::Size(), 0.25, 0.25, cv::INTER_LINEAR);
        // 显式确保缩放后仍是8位单通道（避免潜在格式变化）
        if (scaledMat.type() != CV_8UC1) {
            scaledMat.convertTo(scaledMat, CV_8UC1);
        }

        // 3. 深拷贝数据到QImage：避免依赖局部scaledMat的生命周期
        // 方式1：用QImage::copy()触发深拷贝（简单直观）
        QImage tempImg(
            scaledMat.data,
            scaledMat.cols,
            scaledMat.rows,
            scaledMat.step,  // 每行字节数（处理可能的内存对齐）
            QImage::Format_Grayscale8
        );
        return tempImg.copy();  // 关键：copy()后QImage拥有独立内存

        // 方式2：手动复制数据（适用于需要更精细控制的场景）
        // QImage img(scaledMat.cols, scaledMat.rows, QImage::Format_Grayscale8);
        // memcpy(img.bits(), scaledMat.data, scaledMat.total() * scaledMat.elemSize());
        // return img;
        };

    // 处理左右两个Mat
    result[0] = processSingleMat(matArray[0]);
    result[1] = processSingleMat(matArray[1]);

    return result;
}
#endif // !IMAGE_TRANSLATE_H
