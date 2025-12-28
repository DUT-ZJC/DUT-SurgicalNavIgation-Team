#ifndef DETECTOR_H
#define DETECTOR_H

#include <opencv2/opencv.hpp>
#include <vector>

// 检测反光球函数
std::vector<cv::Point2f> detectReflectiveBalls(const cv::Mat& image);

// 双目匹配函数：对右图点进行重新排序以匹配左图
std::vector<cv::Point2f> matchRightImagePoints(const std::vector<cv::Point2f>& leftPoints,
    const std::vector<cv::Point2f>& rightPoints);

// 辅助函数（如果需要在外部使用）
std::vector<cv::Point2f> getOrdered8Points(const std::vector<cv::Point2f>& points);
std::vector<cv::Point2f> alignGroupStartPoint(const std::vector<cv::Point2f>& refPts,
    const std::vector<cv::Point2f>& targetPts);

#endif // DETECTOR_H
//// 球识别模块头文件
//#ifndef DETECTOR_H
//#define DETECTOR_H
//
//#include <opencv2/opencv.hpp>
//#include <vector>
//
//// 检测图像中的反光球（高亮区域），返回其中心点列表
//std::vector<cv::Point2f> detectReflectiveBalls(const cv::Mat& image);
//
//#endif // DETECTOR_H
