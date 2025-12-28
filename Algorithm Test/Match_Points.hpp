#pragma 
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include "VisualQueue.hpp"

class PointsMatcher {
public:
    /**
     * @brief 构造函数
     */
    PointsMatcher(const cv::Mat& cameraMatrix1,
        const cv::Mat& cameraMatrix2,
        const cv::Mat& distCoeffs1,
        const cv::Mat& distCoeffs2,
        const cv::Mat& R,
        const cv::Mat& T,
        float minRadius = 2.0f, float maxRadius = 50.0f,
        int brightThreshold = 200, double minArea = 5.0);

    /**
     * @brief 析构函数
     */
    ~PointsMatcher();

    /**
     * @brief 设置检测参数
    */
    void setDetectionParams(float minRadius, float maxRadius,
        int brightThreshold, double minArea);

    /**
     * @brief 设置排序容差
     * @param tolerance Y坐标排序的容差（像素）
     */
    void setSortTolerance(float tolerance);


    /**
     * @brief 获取匹配结果的索引对（基于原始输入点集的索引）
     * @return 匹配的索引对 (左图原始索引, 右图原始索引)
     */
    std::vector<std::pair<int, std::vector<int>>> getMatchedIndices() const;

    /**
     * @brief 获取匹配统计信息
     * @param totalLeftPoints 输出：左图总点数
     * @param totalRightPoints 输出：右图总点数
     * @param matchedCount 输出：成功匹配的点对数
     */
    void getMatchStatistics(int& totalLeftPoints, int& totalRightPoints, int& matchedCount) const;

    /**
     * @brief 可视化完整匹配过程（显示所有点，区分匹配和未匹配）
     * @param leftImage 左图像
     * @param rightImage 右图像
     * @param allLeftPoints 左图所有点
     * @param allRightPoints 右图所有点
     * @param matchedLeftPoints 匹配成功的左图点
     * @param matchedRightPoints 匹配成功的右图点
     */
    void visualizeMatchingProcess(const cv::Mat& leftImage,
        const cv::Mat& rightImage,
        const std::vector<cv::Point2f>& allLeftPoints,
        const std::vector<cv::Point2f>& allRightPoints,
        const std::vector<cv::Point2f>& matchedLeftPoints,
        const std::vector<std::vector<cv::Point2f>>& matchedRightPoints,
        BrowsableQueue& visualImageQueue);

public:
    /**
     * @brief 检测单张图像中的反光球（自动排序）
     * @param image 输入图像（灰度图或彩色图）
     * @return 检测到的球心坐标（已按光栅扫描顺序排序）
     */
    std::vector<cv::Point2f> detectBalls(const cv::Mat& image);

    /**
     * @brief 双目立体匹配（基于极线约束，自动过滤未匹配点）
     * @param leftPoints 左图像中的球心点
     * @param rightPoints 右图像中的球心点
     * @param epipolarThreshold 极线约束阈值（像素）
     * @param filteredLeftPoints 输出：过滤后的左图点（只包含成功匹配的点）
     * @param filteredRightPoints 输出：过滤后的右图点（与左图点一一对应）
     * @return 匹配成功返回true，失败返回false
     */
    bool stereoMatch(
        const std::vector<cv::Point2f>& leftPoints,
        const std::vector<cv::Point2f>& rightPoints,
        std::vector<cv::Point2f>& filteredLeftPoints,
        std::vector<std::vector<cv::Point2f>>& filteredRightPoints,
        double epipolarThreshold = 5.0);

private:
    // 检测参数
    float m_minRadius;          // 最小球半径
    float m_maxRadius;          // 最大球半径
    int m_brightThreshold;      // 亮度阈值
    double m_minArea;           // 最小轮廓面积
    float m_sortTolerance;      // 排序容差

    //输入内参
    cv::Mat cameraMatrix1;
    cv::Mat cameraMatrix2;
    cv::Mat R;
    cv::Mat T;
    cv::Mat distCoeffs1;
    cv::Mat distCoeffs2;
    //基础矩阵    
    cv::Mat F;
    
    // 匹配结果统计
    std::vector<std::pair<int, std::vector<int>>> m_matchedIndices;
    int m_totalLeftPoints;
    int m_totalRightPoints;
    int m_matchedCount;

    /**
     * @brief 预处理图像（增强对比度、去噪等）
     * @param image 输入图像
     * @return 预处理后的图像
     */
    cv::Mat preprocessImage(const cv::Mat& image);

    /**  
     * @brief 通过亮度和形状特征检测反光球
     * @param image 输入灰度图像
     * @return 检测到的球心坐标
     */
    std::vector<cv::Point2f> detectBrightCircles(const cv::Mat& grayImage);

    /**
     * @brief 对检测到的点进行光栅扫描排序（从左上到右下）
     * @param points 输入点集
     * @return 排序后的点集
     */
    std::vector<cv::Point2f> sortPointsRasterScan(const std::vector<cv::Point2f>& points);

    /**
     * @brief 计算点到极线的距离
     * @param point 待检测点
     * @param epiline 极线参数 [a, b, c]，表示 ax + by + c = 0
     * @return 点到极线的距离
     */
    double pointToEpilineDistance(const cv::Point2f& point, const cv::Vec3f& epiline);

    /**
     * @brief 去畸变点坐标
     * @param points 输入点
     * @param cameraMatrix 相机内参
     * @param distCoeffs 畸变系数
     * @return 去畸变后的点
     */
    std::vector<cv::Point2f> undistortPoints(
        const std::vector<cv::Point2f>& points,
        const cv::Mat& cameraMatrix,
        const cv::Mat& distCoeffs);

    /**
     * @brief 计算基础矩阵
     */
    void computeFundamentalMatrix();
};