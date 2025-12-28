#ifndef POSE6DOF_H
#define POSE6DOF_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

// ========== 6DOF姿态结构体 ==========
struct Pose6DOF {
    cv::Point3f position;        // 位置 (x, y, z) 单位：mm
    float roll;                  // 横滚角（绕X轴）单位：度
    float pitch;                 // 俯仰角（绕Y轴）单位：度
    float yaw;                   // 偏航角（绕Z轴）单位：度
    cv::Mat rotationMatrix;      // 旋转矩阵 (3x3)
    bool isValid;                // 姿态是否有效

    // 构造函数
    Pose6DOF() : position(0, 0, 0), roll(0), pitch(0), yaw(0), isValid(false) {}

    // 转字符串输出
    std::string toString() const;
};

// ========== 6DOF姿态计算器 ==========
class Pose6DOFCalculator {
public:
    /**
     * 构造函数
     * @param modelPoints 模型点（参考坐标系中的4个3D点）
     */
    Pose6DOFCalculator(const std::vector<cv::Point3f>& modelPoints);
    Pose6DOFCalculator() {};
    /**
     * 计算6DOF姿态
     * @param measuredPoints 测量点（目标坐标系中的4个3D点）
     * @param outPose 输出的姿态结果
     * @return 是否计算成功
     */
    bool calculatePose(const std::vector<cv::Point3f>& measuredPoints, Pose6DOF& outPose);

    /**
     * 使用PnP方法计算姿态（适用于共面点）
     * @param measuredPoints 测量的3D点
     * @param cameraMatrix 相机内参矩阵
     * @param outPose 输出的姿态结果
     * @return 是否计算成功
     */
    bool calculatePoseWithPnP(const std::vector<cv::Point3f>& measuredPoints,
        const cv::Mat& cameraMatrix,
        Pose6DOF& outPose);

private:
    std::vector<cv::Point3f> modelPoints_;  // 模型点
    bool useCoplanarMethod_;                // 是否使用共面方法

    /**
     * 检查点是否共面
     * @param points 待检查的3D点集
     * @return 是否共面
     */
    bool checkCoplanar(const std::vector<cv::Point3f>& points);

    /**
     * 计算点集的质心
     * @param points 3D点集
     * @return 质心坐标
     */
    cv::Point3f calculateCentroid(const std::vector<cv::Point3f>& points);

    /**
     * 使用SVD计算刚体变换（适用于非共面点）
     * @param srcPoints 源点集
     * @param dstPoints 目标点集
     * @param R 输出旋转矩阵
     * @param t 输出平移向量
     * @return 是否计算成功
     */
    bool calculateRigidTransform(const std::vector<cv::Point3f>& srcPoints,
        const std::vector<cv::Point3f>& dstPoints,
        cv::Mat& R, cv::Mat& t);

    /**
     * 计算共面点的姿态
     * @param measuredPoints 测量的共面点
     * @param outPose 输出的姿态结果
     * @return 是否计算成功
     */
    bool calculatePoseCoplanar(const std::vector<cv::Point3f>& measuredPoints,
        Pose6DOF& outPose);

    /**
     * 旋转矩阵转欧拉角（ZYX约定）
     * @param R 旋转矩阵
     * @param roll 输出横滚角（度）
     * @param pitch 输出俯仰角（度）
     * @param yaw 输出偏航角（度）
     */
    void rotationMatrixToEulerAngles(const cv::Mat& R,
        float& roll,
        float& pitch,
        float& yaw);
};

#endif // POSE6DOF_H