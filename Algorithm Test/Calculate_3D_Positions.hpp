#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

/**
 * @brief 计算3D坐标（一对一匹配）
 * @param leftPoints 左图点（已去畸变）
 * @param rightPoints 右图点（已去畸变，与左图点一一对应）
 * @param cameraMatrix1 左相机内参矩阵
 * @param cameraMatrix2 右相机内参矩阵
 * @param baseline_mm 基线距离（毫米）
 * @param points3D 输出：3D坐标（单位：毫米）
 * @param disparities 输出：视差值（可选）
 * @return 成功返回true
 */
bool Calculate3DCoordinates(
    const std::vector<cv::Point2f>& leftPoints,
    const std::vector<cv::Point2f>& rightPoints,
    const cv::Mat& cameraMatrix1,
    const cv::Mat& cameraMatrix2,
    double baseline_mm,
    std::vector<cv::Point3f>& points3D,
    std::vector<double>* disparities = nullptr);

/**
 * @brief 打印3D点坐标
 * @param points3D 3D点集
 * @param verbose 是否打印详细信息
 */
void Print3DPoints(
    const std::vector<cv::Point3f>& points3D,
    bool verbose = true);

/**
 * @brief 打印视差统计信息
 * @param disparities 视差值
 */
void PrintDisparityStatistics(
    const std::vector<double>& disparities);

/**
 * @brief 打印精度估计
 * @param disparities 视差值
 * @param points3D 3D点
 * @param cameraMatrix1 左相机内参
 * @param baseline_mm 基线距离
 */
void PrintAccuracyEstimate(
    const std::vector<double>& disparities,
    const std::vector<cv::Point3f>& points3D,
    const cv::Mat& cameraMatrix1,
    double baseline_mm);

/**
 * @brief 验证视差的有效性
 * @param disparity 视差值
 * @return 有效返回true
 */
bool ValidateDisparity(double disparity);

/**
 * @brief 验证3D点的有效性
 * @param point3D 3D点
 * @return 有效返回true
 */
bool Validate3DPoint(const cv::Point3f& point3D);

/**
 * @brief 估计深度误差
 * @param depth_mm 深度（毫米）
 * @param disparity_error 视差误差（像素）
 * @param focal_length_px 焦距（像素）
 * @param baseline_mm 基线距离（毫米）
 * @return 深度误差（毫米）
 */
double EstimateDepthError(
    double depth_mm,
    double disparity_error,
    double focal_length_px,
    double baseline_mm);