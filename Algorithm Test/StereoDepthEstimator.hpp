#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <array>
#include <iostream>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <iomanip>
#include <mutex>
#include "Logger.hpp"

/**
 * @brief 双目视觉3D坐标计算器
 * 核心功能：
 * 1. 简单的顺序贪婪匹配（解决一对多）
 * 2. 数学级极线校正（Rectification），不依赖图像Remap
 * 3. 视差转深度计算
 */
class StereoDepthEstimator
{
public:
    struct Config {
        double min_disparity = 0.1;     // 最小视差
        double max_disparity = 2500.0;  // 最大视差
        double min_depth_mm = 50.0;     // 最小深度
        double max_depth_mm = 10000.0;  // 最大深度
        double pixel_size_mm = 0.05;    // 像元尺寸(用于精度估算)
        bool verbose = true;            // 打印日志
        bool is_input_undistorted = true; // [重要] 如果输入点已经在外部去过畸变，设为true
    };

public:
    /**
     * @brief 构造函数：初始化立体校正矩阵
     * 需要传入完整的双目标定参数，用于计算 R1, R2, P1, P2
     */
    StereoDepthEstimator(const cv::Mat& K1, const cv::Mat& D1,
        const cv::Mat& K2, const cv::Mat& D2,
        const cv::Mat& R, const cv::Mat& T,
        const cv::Size& imageSize,
        const Config& config = Config());

    /**
     * @brief 核心计算函数
     * 1. 执行贪婪匹配（解决一对多）
     * 2. 执行数学极线校正
     * 3. 计算3D坐标
     * * @param leftPoints 左图点集 (有序)
     * @param rightCandidates 右图候选点集 (与左图一一对应，每个左图点可能有多个右图候选)
     * @param outPoints3D 输出：计算出的3D点 (与输入索引一一对应，无效点为 0,0,0)
     * @param outMatchedPairs 输出：2D匹配点对引用 (必须提供)。
     * outMatchedPairs[i] = {LeftPoint, RightPoint}
     * 如果是无效点，则为 {0,0}。
     * @return 有效点数量
     */
    int Compute(const std::vector<cv::Point2f>& leftPoints,
        const std::vector<std::vector<cv::Point2f>>& rightCandidates,
        std::vector<cv::Point3f>& outPoints3D,
        std::vector<std::array<cv::Point2f, 2>>& outMatchedPairs);

    // --- 辅助显示函数 ---
    void PrintDisparityStatistics() const;
    void PrintAccuracyEstimate() const;

    // --- Setter/Getter ---
    void setConfig(const Config& config) { m_config = config; }

private:
    /**
     * @brief 内部匹配算法：顺序贪婪匹配
     * [ALGORITHM_UPDATE]: 这里是后续更新锚点扩散算法的主要位置
     * 注意：最后一个参数是 vector<int>&，修复了之前的声明错误
     */
    void matchPointsGreedy(
        const std::vector<cv::Point2f>& leftPoints,
        const std::vector<std::vector<cv::Point2f>>& rightCandidates,
        std::vector<cv::Point2f>& outMatchedLeft,
        std::vector<cv::Point2f>& outMatchedRight,
        std::vector<int>& outOriginalIndices);

    /**
     * @brief 内部函数：将任意点投影到极线校正平面
     * 使用 P1/P2 和 R1/R2 将点变换到平行双目系统
     */
    void rectifyPointCoordinates(
        const std::vector<cv::Point2f>& srcLeft,
        const std::vector<cv::Point2f>& srcRight,
        std::vector<cv::Point2f>& dstRectLeft,
        std::vector<cv::Point2f>& dstRectRight);

    bool isPointUsed(const cv::Point2f& pt, const std::vector<cv::Point2f>& usedPoints, float tolerance = 0.1f) const;

private:
    // 原始标定参数
    cv::Mat m_K1, m_D1, m_K2, m_D2, m_R, m_T;
    cv::Size m_imgSize;

    // 立体校正参数 (由构造函数计算)
    cv::Mat m_R1, m_R2; // 校正旋转
    cv::Mat m_P1, m_P2; // 校正投影矩阵 (包含新的 fx, fy, cx, cy 和 基线偏移)
    cv::Mat m_Q;        // 视差-深度映射矩阵

    // 缓存参数
    double m_f_rect;      // 校正后的焦距
    double m_base_rect;   // 校正后的基线 (mm)
    cv::Point2d m_c_rect; // 校正后的光心

    Config m_config;

    // 缓存结果
    std::vector<double> m_lastDisparities;
    std::vector<cv::Point3f> m_lastPoints3D;
};