#include "Instruments_Calibration.hpp"
#include "spdlog/spdlog.h" 
#include <cmath>
#include <algorithm>
#include <iostream>

// 辅助函数：计算两点欧氏距离  
// 使用 static 限制在当前编译单元，避免符号冲突
static double Dist(const cv::Point3d& a, const cv::Point3d& b) {  
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
}

RealTimePivotCalibrator::RealTimePivotCalibrator(int batchSize, int targetTotal)
    : m_batchSize(batchSize), m_targetTotal(targetTotal) {
    Reset();
}

void RealTimePivotCalibrator::Reset() {
    m_candidateBatch.clear();
    m_mainPool.clear();
    m_buckets.clear();

    m_currentRMSE = 999.9;
    m_currentCoverage = 0;
    m_currentTip.fill(cv::Point3d(0, 0, 0));
    m_currentPivot = cv::Point3d(0, 0, 0);

    spdlog::info("[Pivot] Calibrator Reset.");
}

CalibrationResult RealTimePivotCalibrator::AddPose(const std::array<cv::Mat,12>& rvec, const std::array<cv::Mat, 12>& tvec) {
    if (rvec.empty() || tvec.empty()) {
        return { PivotState::CollectingBatch, m_currentTip, m_currentPivot, m_currentRMSE, (int)m_mainPool.size(), m_currentCoverage, "Invalid Input" };
    }

    PoseData pose;
    // 1. 格式转换：确保 R 是 3x3 矩阵，且均为 CV_64F (double)
    for(size_t i =0;i < 12;i++)
    {
        tvec[i].convertTo(pose.T[i], CV_64F);
        rvec[i].convertTo(pose.R[i], CV_64F);
    }


    // 2. 缓存 Z 轴向量 (R 的第3列)，用于后续分桶
    pose.zAxis = cv::Point3d(pose.R[0].at<double>(0, 2), pose.R[0].at<double>(1, 2), pose.R[0].at<double>(2, 2));

    // 3. 加入候选批次
    m_candidateBatch.push_back(pose);

    // 4. 默认返回状态 (正在积攒数据)
    CalibrationResult res;
    res.state = PivotState::CollectingBatch;
    res.rmse = m_currentRMSE;
    res.toolTip = m_currentTip;
    res.pivotPoint = m_currentPivot;
    res.totalFrames = (int)m_mainPool.size();
    res.coveragePercent = m_currentCoverage;
    res.message = fmt::format("Collecting... {}/{}", m_candidateBatch.size(), m_batchSize);

    // 5. 如果批次满了，触发核心处理逻辑
    if ((int)m_candidateBatch.size() >= m_batchSize) {
        return ProcessBatch();
    }

    return res;
}

CalibrationResult RealTimePivotCalibrator::ProcessBatch() {
    CalibrationResult res;

    // ==========================================
    // A. 批次自检 (Local Batch Check)
    // ==========================================
    cv::Point3d batchPivot;
    std::array<cv::Point3d, 12> batchTip;

    double batchRMSE = SolveInternal(m_candidateBatch, batchTip, batchPivot);

    // 1. 过滤垃圾数据：如果这一批数据自己算出来的误差都很大(>2.5mm)，说明手抖严重，直接丢弃
    if (batchRMSE > 2.5) {
        m_candidateBatch.clear();

        res.state = PivotState::Rejected;
        res.message = "Batch Rejected (Unstable input)";
        res.rmse = m_currentRMSE;
        res.toolTip = m_currentTip;
        res.pivotPoint = m_currentPivot;
        res.totalFrames = (int)m_mainPool.size();
        res.coveragePercent = m_currentCoverage;
        return res;
    }

    // ==========================================
    // B. 移动/重置检测 (Consistency Check)
    // ==========================================
    bool shouldReset = false;
    if (!m_mainPool.empty()) {
        // 比较“新批次的尖点”和“历史尖点”的距离
        double shift = Dist(batchPivot, m_currentPivot);

        // 如果突变超过 3.0mm，认为用户改变了器械或枢轴位置
        if (shift > 3.0) {
            shouldReset = true;
            spdlog::warn("[Pivot] Tip shift {:.2f}mm detected. Resetting.", shift);
        }
    }

    if (shouldReset) {
        Reset(); // 清空所有历史
        m_mainPool = m_candidateBatch; // 以当前批次为新起点   asdadawdasd

        // 重建桶
        for (const auto& p : m_candidateBatch) {
            int id = GetBucketID(p.zAxis);
            m_buckets[id]++;
        }
        UpdateCoverage();

        // 立即计算一次新状态
        m_currentRMSE = SolveInternal(m_mainPool, m_currentTip, m_currentPivot);

        res.state = PivotState::Reset;
        res.message = "Movement Detected -> Reset";
    }
    else {
        // ==========================================
        // C. 合并数据 (Merge)
        // ==========================================
        m_mainPool.insert(m_mainPool.end(), m_candidateBatch.begin(), m_candidateBatch.end());

        // 更新桶计数
        for (const auto& p : m_candidateBatch) {
            int id = GetBucketID(p.zAxis);
            m_buckets[id]++;
        }

        // 先计算一次全局结果，得到最新的 Pivot 和 ToolTip
        // 我们需要利用这个最新的“最佳拟合”来找出谁是群害之马
        m_currentRMSE = SolveInternal(m_mainPool, m_currentTip, m_currentPivot);

        // ==========================================
        // D. 激进优化：优胜劣汰 (Smart Pruning)
        //    新逻辑：
        //    D1) Local pruning：只要桶过密就清理（不依赖 size>targetTotal）
        //    D2) Global pruning：size 超过 targetTotal 再全局删
        // ==========================================

        // --- D0. 计算桶容量阈值（与 GRID_SIZE/targetTotal 绑定）---
        const int bucketCount = GRID_SIZE * GRID_SIZE;
        const double expectedPerBucket = (bucketCount > 0) ? (double)m_targetTotal / (double)bucketCount : 1.0;

        // α/β 可做成参数；这里给稳定默认值
        const double alpha = 1.0; // BUCKET_TARGET = 平均密度
        const double beta = 2.0; // BUCKET_MAX = 2倍平均密度

        const int BUCKET_TARGET = std::max(1, (int)std::ceil(expectedPerBucket * alpha));
        const int BUCKET_MAX = std::max(BUCKET_TARGET + 1, (int)std::ceil(BUCKET_TARGET * beta));

        // γ：自适应强度（越大删得越狠）
        const double gamma = 1.0;

        // --- D1. Local pruning：对所有过密桶进行清理 ---
        bool didLocalPrune = false;

        // 为避免在遍历 map 时修改 map，先把需要处理的桶 id 收集出来
        std::vector<int> crowdedIDs;
        crowdedIDs.reserve(m_buckets.size());
        for (const auto& kv : m_buckets) {
            if (kv.second > BUCKET_MAX) crowdedIDs.push_back(kv.first);
        }

        // 逐个拥挤桶处理
        for (int bucketID : crowdedIDs)
        {
            auto it = m_buckets.find(bucketID);
            if (it == m_buckets.end()) continue;

            int count = it->second;
            if (count <= BUCKET_MAX) continue;

            // 自适应删除数量：超多少删多少（并限制最多删到 BUCKET_TARGET）
            int overMax = count - BUCKET_MAX;
            int canRemove = count - BUCKET_TARGET;
            int toRemove = (int)std::ceil(gamma * overMax);
            toRemove = std::min(toRemove, canRemove);
            toRemove = std::max(0, toRemove);

            for (int n = 0; n < toRemove; ++n) {
                int removedIndex = DeleteWorstPointInBucket(bucketID);
                if (removedIndex < 0) break; // 兜底
                didLocalPrune = true;
            }
        }

        // --- D2. Global pruning：总量超标时再全局删（仍用“桶优先”策略）---
        bool didGlobalPrune = false;
        if ((int)m_mainPool.size() > m_targetTotal)
        {
            int overflow = (int)m_mainPool.size() - m_targetTotal;

            // 每次都优先从最拥挤桶删除（全局删也要鼓励均匀分布）
            for (int n = 0; n < overflow; ++n)
            {
                int bucketID = GetMostCrowdedBucketID();
                if (bucketID < 0) break;

                int removedIndex = DeleteWorstPointInBucket(bucketID);
                if (removedIndex < 0) break;

                didGlobalPrune = true;
            }
        }

        // 剪枝后更新覆盖率与重算（仅当发生了剪枝）
        if (didLocalPrune || didGlobalPrune)
        {
            UpdateCoverage();
            m_currentRMSE = SolveInternal(m_mainPool, m_currentTip, m_currentPivot);
        }


        res.state = PivotState::Converging;
        res.message = "Optimizing (Smart Pruning)...";
    }

    // ==========================================
    // E. 最终就绪判断 (Ready Check)
    // ==========================================
    // 1. 数量达标 (至少 80%)
    // 2. 误差达标 (RMSE < 0.8mm)
    // 3. 覆盖度达标 (> 70%)
    bool isCountOK = m_mainPool.size() >= (size_t)(m_targetTotal * 0.8);
    bool isRmseOK = m_currentRMSE < 0.8;
    bool isCoverageOK = m_currentCoverage >= 70;

    if (isCountOK && isRmseOK && isCoverageOK) {
        res.state = PivotState::Ready;
        res.message = "CALIBRATION READY!";
    }
    else if (!isCoverageOK && isCountOK) {
        res.message = "Rotate more! (Need angle coverage)";
    }
    else if (!isRmseOK && isCountOK) {
        // 优胜劣汰正在进行中
        res.message = "Hold steady... (Refining)";
    }

    // 清空候选批次，准备下一轮
    m_candidateBatch.clear();

    // 填充最终结果
    res.rmse = m_currentRMSE;
    res.toolTip = m_currentTip;
    res.pivotPoint = m_currentPivot;
    res.totalFrames = (int)m_mainPool.size();
    res.coveragePercent = m_currentCoverage;

    return res;
}

// 内部求解器
double RealTimePivotCalibrator::SolveInternal(
    const std::vector<PoseData>& inputData,
    std::array<cv::Point3d, 12>& outTip,
    cv::Point3d& outPivot)
{
    const int N = (int)inputData.size();
    if (N < 5) return 999.9;

    std::array<cv::Point3d, 12> pivots{};
    std::array<double, 12> rmses{};
    std::atomic<bool> failed{ false };

    // 如果你担心 OpenCV 内部线程 + 外层并行叠加，可临时设为 1
    // int oldThreads = cv::getNumThreads();
    // cv::setNumThreads(1);

#pragma omp parallel for
    for (int k = 0; k < 12; ++k)
    {
        if (failed.load()) continue;

        cv::Mat A = cv::Mat::zeros(3 * N, 6, CV_64F);
        cv::Mat b = cv::Mat::zeros(3 * N, 1, CV_64F);

        const cv::Mat I = cv::Mat::eye(3, 3, CV_64F);
        const cv::Mat negI = -I;  // 关键：实体化成 Mat，避免 MatExpr 不能 copyTo

        for (int i = 0; i < N; ++i)
        {
            // R_k
            inputData[i].R[k].copyTo(A(cv::Rect(0, 3 * i, 3, 3)));

            // -I
            negI.copyTo(A(cv::Rect(3, 3 * i, 3, 3)));

            // -T_k（关键：实体化成 Mat）
            cv::Mat negT = -inputData[i].T[k];
            negT.copyTo(b(cv::Rect(0, 3 * i, 1, 3)));
        }

        cv::Mat x;
        if (!cv::solve(A, b, x, cv::DECOMP_SVD))
        {
            failed.store(true);
            continue;
        }

        const cv::Point3d tip(x.at<double>(0), x.at<double>(1), x.at<double>(2));
        const cv::Point3d pivot(x.at<double>(3), x.at<double>(4), x.at<double>(5));

        outTip[k] = tip;
        pivots[k] = pivot;

        // --- 计算该通道 RMSE ---
        double sumSq = 0.0;
        cv::Mat tipVec = (cv::Mat_<double>(3, 1) << tip.x, tip.y, tip.z);

        for (int i = 0; i < N; ++i)
        {
            cv::Mat pred = inputData[i].R[k] * tipVec + inputData[i].T[k];
            cv::Point3d predPt(pred.at<double>(0), pred.at<double>(1), pred.at<double>(2));

            const double dx = predPt.x - pivot.x;
            const double dy = predPt.y - pivot.y;
            const double dz = predPt.z - pivot.z;
            sumSq += dx * dx + dy * dy + dz * dz;
        }

        rmses[k] = std::sqrt(sumSq / N);
    }

    // cv::setNumThreads(oldThreads);

    if (failed.load()) return 999.9;

    // --- outPivot 取 12 路均值 ---
    cv::Point3d sumPivot(0, 0, 0);
    for (const auto& p : pivots) sumPivot += p;
    outPivot = (1.0 / 12.0) * sumPivot;

    // --- 总体 RMSE 取 12 路均值 ---
    double sumRmse = 0.0;
    for (double e : rmses) sumRmse += e;
    return sumRmse / 12.0;
}



// 更新覆盖率
void RealTimePivotCalibrator::UpdateCoverage() {
    int validBuckets = (int)m_buckets.size();
    // 假设至少需要覆盖 15 个不同的角度区域才算及格
    int minBucketsNeeded = 15;

    int pct = (int)((validBuckets / (double)minBucketsNeeded) * 100);
    if (pct > 100) pct = 100;
    m_currentCoverage = pct;
}

// 计算桶 ID (二维坐标一维化)
int RealTimePivotCalibrator::GetBucketID(const cv::Point3d& z) {
    // 将向量转为球坐标 (theta, phi)
    // z.z 是 cos(theta)
    double val = std::min(std::max(z.z, -1.0), 1.0); // 数值钳位
    double theta = std::acos(val); // 0 ~ PI
    double phi = std::atan2(z.y, z.x); // -PI ~ PI

    // 归一化到网格索引
    int idx_theta = static_cast<int>((theta / CV_PI) * GRID_SIZE);
    // phi + PI 映射到 0~2PI
    int idx_phi = static_cast<int>(((phi + CV_PI) / (2 * CV_PI)) * GRID_SIZE);

    // 边界保护
    if (idx_theta >= GRID_SIZE) idx_theta = GRID_SIZE - 1;
    if (idx_phi >= GRID_SIZE) idx_phi = GRID_SIZE - 1;

    return idx_theta * GRID_SIZE + idx_phi;
}
/*
// 写入 YAML (LNK2019 修复)
void RealTimePivotCalibrator::WriteYaml() {
    if (m_mainPool.empty()) {
        spdlog::warn("[Pivot] No data to save!");
        return;
    }

    std::string filename = "PivotCalibrationResult.yaml";
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);

    if (fs.isOpened()) {
        fs << "ToolTip" << m_currentTip;
        fs << "PivotPoint" << m_currentPivot;
        fs << "RMSE" << m_currentRMSE;
        fs << "TotalFrames" << (int)m_mainPool.size();
        fs.release();
        spdlog::info("[Pivot] Result saved to {}", filename);
    }
    else {
        spdlog::error("[Pivot] Failed to open file: {}", filename);
    }
}
*/
int RealTimePivotCalibrator::GetMostCrowdedBucketID() const
{
    int maxBucketID = -1;
    int maxCount = -1;
    for (const auto& kv : m_buckets) {
        if (kv.second > maxCount) {
            maxCount = kv.second;
            maxBucketID = kv.first;
        }
    }
    return maxBucketID;
}

int RealTimePivotCalibrator::DeleteWorstPointInBucket(int bucketID)
{
    if (m_mainPool.empty()) return -1;

    // 预先准备 tipVecs（12 路）
    std::array<cv::Mat, 12> tipVecs;
    for (int k = 0; k < 12; ++k) {
        tipVecs[k] = (cv::Mat_<double>(3, 1)
            << m_currentTip[k].x, m_currentTip[k].y, m_currentTip[k].z);
    }

    int worstIndex = -1;
    double worstErr = -1.0;

    // 遍历主库中属于该 bucket 的样本
    for (int idx = 0; idx < (int)m_mainPool.size(); ++idx)
    {
        if (GetBucketID(m_mainPool[idx].zAxis) != bucketID) continue;

        // 计算该样本在 12 路下的预测残差均值
        double errSum = 0.0;

        for (int k = 0; k < 12; ++k)
        {
            // pred = R*tip + T
            const cv::Mat pred = m_mainPool[idx].R[k] * tipVecs[k] + m_mainPool[idx].T[k];

            // predPt
            const cv::Point3d predPt(
                pred.at<double>(0, 0),
                pred.at<double>(1, 0),
                pred.at<double>(2, 0));

            errSum += Dist(predPt, m_currentPivot);
        }

        const double errAvg = errSum / 12.0;

        if (errAvg > worstErr) {
            worstErr = errAvg;
            worstIndex = idx;
        }
    }

    // 若该桶没找到点，兜底：删最老的一个（与原逻辑一致）
    if (worstIndex < 0)
    {
        // 同步更新桶计数：先算这个点属于哪个桶
        int oldBucket = GetBucketID(m_mainPool.front().zAxis);
        auto it = m_buckets.find(oldBucket);
        if (it != m_buckets.end()) {
            it->second--;
            if (it->second <= 0) m_buckets.erase(it);
        }

        m_mainPool.erase(m_mainPool.begin());
        return 0;
    }

    // 正常删除：更新桶计数
    auto it = m_buckets.find(bucketID);
    if (it != m_buckets.end()) {
        it->second--;
        if (it->second <= 0) m_buckets.erase(it);
    }

    // 删除该样本
    m_mainPool.erase(m_mainPool.begin() + worstIndex);
    return worstIndex;
}

void  RealTimePivotCalibrator::CalibrateThreadFunc()
{
    PoseData pose;
    CalibrationResult result;
    while (!m_shouldStop)
    {
        if (DequeuePose(pose))
        {
            result = AddPose(std::move(pose.R), std::move(pose.T));
        }
    }
}

bool RealTimePivotCalibrator::StartCalibrate()
{
    if (m_isRunning) return false;
    m_isRunning = true;
    m_shouldStop = false;
    m_instrumentCalibrateThread = std::thread(&RealTimePivotCalibrator::CalibrateThreadFunc, this);
    return true;
}


bool RealTimePivotCalibrator::StopCalibrate()
{
    if (!m_isRunning) return false;
    m_shouldStop = true;
    if (m_instrumentCalibrateThread.joinable())
        m_instrumentCalibrateThread.join();
    m_isRunning = false;
    return true;
}

bool RealTimePivotCalibrator::EnqueuePose(const std::array<cv::Mat, 12>& rvec, const std::array<cv::Mat, 12>& tvec)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    while (PoseQueue.size() >= m_maxQueueSize)
    {
        PoseQueue.pop();
    }

    PoseQueue.emplace(rvec, tvec);
    return true;
}

bool RealTimePivotCalibrator::DequeuePose(PoseData& Pose)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (PoseQueue.empty()) return false;

    Pose = std::move(PoseQueue.front());
    PoseQueue.pop();
    return true;
}
