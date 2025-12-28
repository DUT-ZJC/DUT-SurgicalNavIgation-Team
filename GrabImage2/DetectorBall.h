#ifndef BALL_DETECTOR_H
#define BALL_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <stdexcept>

/**
 * @brief 反光球检测器类
 * 用于检测双目视觉系统中的反光球并进行立体匹配
 */
class BallDetector {
public:
    /**
     * @brief 构造函数
     * @param minRadius 最小球半径阈值
     * @param maxRadius 最大球半径阈值
     * @param brightThreshold 亮度阈值
     * @param minArea 最小轮廓面积
     */
    BallDetector(float minRadius = 3.0f, float maxRadius = 50.0f,
        int brightThreshold = 250, double minArea = 10.0);

    /**
     * @brief 检测单张图像中的反光球
     * @param image 输入图像
     * @return 检测到的球心坐标
     */
    std::vector<cv::Point2f> detectBalls(const cv::Mat& image);

    /**
     * @brief 双目立体匹配
     * @param leftPoints 左图像中的球心点
     * @param rightPoints 右图像中的球心点
     * @param cameraMatrix1 左相机内参矩阵
     * @param cameraMatrix2 右相机内参矩阵
     * @param distCoeffs1 左相机畸变参数
     * @param distCoeffs2 右相机畸变参数
     * @param R 旋转矩阵
     * @param T 平移向量
     * @param epipolarThreshold 极线约束阈值
     * @return 重新排序后的右图像球心点，与左图像对应
     */
    std::vector<cv::Point2f> stereoMatch(
        const std::vector<cv::Point2f>& leftPoints,
        const std::vector<cv::Point2f>& rightPoints,
        const cv::Mat& cameraMatrix1,
        const cv::Mat& cameraMatrix2,
        const cv::Mat& distCoeffs1,
        const cv::Mat& distCoeffs2,
        const cv::Mat& R,
        const cv::Mat& T,
        double epipolarThreshold = 2.0);

    /**
     * @brief 设置检测参数
     */
    void setDetectionParams(float minRadius, float maxRadius,
        int brightThreshold, double minArea);

    /**
     * @brief 获取有序的8个点（分组排序）
     * @param points 输入的8个点
     * @return 排序后的点
     */
    std::vector<cv::Point2f> getOrdered8Points(const std::vector<cv::Point2f>& points);

private:
    // 检测参数
    float minRadius_;
    float maxRadius_;
    int brightThreshold_;
    double minArea_;

    /**
     * @brief 计算点到极线的距离
     * @param point 图像点
     * @param epiline 极线参数 [a, b, c]，表示 ax + by + c = 0
     * @return 点到极线的距离
     */
    double pointToEpilineDistance(const cv::Point2f& point, const cv::Vec3f& epiline);

    /**
     * @brief 使用极角对4个点进行排序
     * @param groupPts 4个点的组
     * @return 排序后的点
     */
    std::vector<cv::Point2f> sortPointsByAngle(const std::vector<cv::Point2f>& groupPts);

    /**
     * @brief 计算点相对于中心的极角
     * @param pt 点坐标
     * @param cx 中心x坐标
     * @param cy 中心y坐标
     * @return 极角值
     */
    double angleFromCenter(const cv::Point2f& pt, double cx, double cy);

    /**
     * @brief KMeans聚类将8个点分成2组
     * @param points 8个输入点
     * @return 分成的两组点
     */
    std::vector<std::vector<cv::Point2f>> clusterPoints(const std::vector<cv::Point2f>& points);

    /**
     * @brief 对齐两组点的起始顺序
     * @param refPts 参考点组
     * @param targetPts 目标点组
     * @return 对齐后的目标点组
     */
    std::vector<cv::Point2f> alignGroupStartPoint(const std::vector<cv::Point2f>& refPts,
        const std::vector<cv::Point2f>& targetPts);

    /**
     * @brief 使用匈牙利算法解决最优匹配问题
     * @param costMatrix 代价矩阵
     * @return 匹配结果的索引对
     */
    std::vector<std::pair<int, int>> hungarianAlgorithm(const std::vector<std::vector<double>>& costMatrix);
};

#endif // BALL_DETECTOR_H