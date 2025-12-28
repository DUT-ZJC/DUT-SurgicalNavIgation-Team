#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <map>
#include <string>

// 标定状态枚举
enum class PivotState {
    CollectingBatch, // 正在积攒批次 (Collecting Candidate Batch)
    Rejected,        // 批次被拒绝 (Batch Rejected - High Noise)
    Reset,           // 检测到移动，已重置 (Movement Detected -> Reset)
    Converging,      // 正在收敛/优化 (Optimizing)
    Ready            // ✅ 标定完成 (Ready)
};

// 标定结果结构体
struct CalibrationResult {
    PivotState state;
    std::array<cv::Point3d,12> toolTip;    // 当前主库计算的尖点 (Tool Tip in Tool Frame)

    cv::Point3d pivotPoint; // 当前主库计算的枢轴 (Pivot in World Frame)
    double rmse;            // 均方根误差 (mm)

    // 进度与诊断
    int totalFrames;        // 主库当前帧数
    int coveragePercent;    // 角度覆盖率 (0-100%)
    std::string message;    // 状态描述信息
};

// 内部使用的姿态数据
struct PoseData {
    std::array<cv::Mat,12> R; // 3x3 double Rotation Matrix
    std::array<cv::Mat, 12> T; // 3x1 double Translation Vector
    cv::Point3d zAxis; // 缓存 Z 轴方向用于分桶 (Cached Z-axis for bucketing)

    PoseData(const std::array<cv::Mat, 12>& r,
        const std::array<cv::Mat, 12>& t,
        const cv::Point3d& z = cv::Point3d(0.0, 0.0, 0.0))
        : R(r), T(t), zAxis(z) {
    }
    PoseData() = default;
};

class RealTimePivotCalibrator {
public:
    // 构造函数
    // batchSize: 每次评估的组数 (默认 50)
    // targetTotal: 最终目标的总数 (默认 500)
    RealTimePivotCalibrator(int batchSize = 50, int targetTotal = 500);
    ~RealTimePivotCalibrator()
    {
        StopCalibrate();
    };

    // 重置所有状态
    void Reset();
    
    bool StartCalibrate();
    bool StopCalibrate();

    bool EnqueuePose(const std::array<cv::Mat, 12>& rvec, const std::array<cv::Mat, 12>& tvec);
private:
    // --- 核心算法函数 ---

    // 添加一帧数据
    // 返回值：当前最新的计算结果
    CalibrationResult AddPose(const std::array<cv::Mat, 12>& rvec, const std::array<cv::Mat, 12>& tvec);

    // 将结果写入 YAML 文件
    void WriteYaml();

    // 处理填满的批次 (包含一致性检查、合并、剪枝)
    CalibrationResult ProcessBatch();

    // 内部求解器 (SVD 最小二乘)
    // 返回 RMSE
    double SolveInternal(const std::vector<PoseData>& inputData,
        std::array<cv::Point3d, 12>& outTip, cv::Point3d& outPivot);

    // --- 空间桶 (Spatial Bucket) 辅助 ---

    // 计算覆盖率百分比
    void UpdateCoverage();

    // 根据 Z 轴向量计算桶 ID
    int GetBucketID(const cv::Point3d& zDir);

    int GetMostCrowdedBucketID() const;
    int DeleteWorstPointInBucket(int bucketID);

//与线程有关
private:
    std::thread m_instrumentCalibrateThread;
    std::atomic<bool> m_isRunning;
    std::atomic<bool> m_shouldStop;
    void CalibrateThreadFunc();
    std::mutex m_mutex;
    size_t m_maxQueueSize;
    std::queue<PoseData> PoseQueue;
    bool DequeuePose(PoseData& Pose);
   
private:
    // 配置参数
    int m_batchSize;
    int m_targetTotal;
    const int GRID_SIZE = 15; // 将球面切分为 15x15 的网格

    // --- 数据缓冲区 ---
    std::vector<PoseData> m_candidateBatch; // 候选批次 (Temporary Batch)
    std::vector<PoseData> m_mainPool;       // 主样本库 (Main Pool)

    // --- 桶状态 ---
    // Key: Bucket ID, Value: 该桶内的点数
    std::map<int, int> m_buckets;
    int m_currentCoverage = 0;

    // --- 缓存的计算结果 ---
    std::array<cv::Point3d, 12> m_currentTip;
    cv::Point3d m_currentPivot;
    double m_currentRMSE = 999.9;
};