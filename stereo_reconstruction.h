//// 三维重建头文件
//#ifndef STEREO_RECONSTRUCTION_H
//#define STEREO_RECONSTRUCTION_H
//
//#include <opencv2/opencv.hpp>
//#include <vector>
//
//// 计算匹配点的三维坐标并输出到控制台
//void compute3DPositions(const std::vector<cv::Point2f>& pts_left,
//    const std::vector<cv::Point2f>& pts_right,
//    double focal_length, double baseline,
//    double cx, double cy);
//
//// 计算三维坐标并绘制到图像上
//void compute3DPositionsAndDraw(cv::Mat& img,
//    const std::vector<cv::Point2f>& pts_left,
//    const std::vector<cv::Point2f>& pts_right,
//    double focal_length, double baseline,
//    double cx, double cy);
//
//#endif // STEREO_RECONSTRUCTION_H

// 三维重建头文件
#ifndef STEREO_RECONSTRUCTION_H
#define STEREO_RECONSTRUCTION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <fstream>

// 计算匹配点的三维坐标并输出到控制台 (单位: mm)
void compute3DPositions(const std::vector<cv::Point2f>& pts_left,
    const std::vector<cv::Point2f>& pts_right,
    double focal_length_px, double baseline_mm,
    double cx, double cy);

// 计算三维坐标并绘制到图像上 (单位: mm)
void compute3DPositionsAndDraw(cv::Mat& img,
    const std::vector<cv::Point2f>& pts_left,
    const std::vector<cv::Point2f>& pts_right,
    double focal_length_px, double baseline_mm,
    double cx, double cy);

// 批量处理双目图像对的三维重建
void compute3DPositionsBatch(const std::vector<std::vector<cv::Point2f>>& pts_left_batch,
    const std::vector<std::vector<cv::Point2f>>& pts_right_batch,
    double focal_length_px, double baseline_mm,
    double cx, double cy);

// 计算点云并保存为PLY格式
void savePointCloudPLY(const std::vector<cv::Point2f>& pts_left,
    const std::vector<cv::Point2f>& pts_right,
    double focal_length_px, double baseline_mm,
    double cx, double cy,
    const std::string& filename);

#endif // STEREO_RECONSTRUCTION_H