#include "StereoDepthEstimator.hpp"

StereoDepthEstimator::StereoDepthEstimator(
    const cv::Mat& K1, const cv::Mat& D1,
    const cv::Mat& K2, const cv::Mat& D2,
    const cv::Mat& R, const cv::Mat& T,
    const cv::Size& imageSize,
    const Config& config)
    : m_K1(K1), m_D1(D1), m_K2(K2), m_D2(D2), m_R(R), m_T(T),
    m_imgSize(imageSize), m_config(config)
{
    // 1. 计算立体校正矩阵
    // 这步至关重要：它计算出了让两个相机“数学上平行”的变换矩阵
    // P1, P2 是新的投影矩阵，P2.at(0,3) 包含了基线信息
    cv::stereoRectify(m_K1, m_D1, m_K2, m_D2, m_imgSize, m_R, m_T,
        m_R1, m_R2, m_P1, m_P2, m_Q,
        cv::CALIB_ZERO_DISPARITY, 0, m_imgSize);

    // 2. 提取校正后的核心参数 (用于后续快速计算)
    m_f_rect = m_P1.at<double>(0, 0); // 新焦距
    m_c_rect = cv::Point2d(m_P1.at<double>(0, 2), m_P1.at<double>(1, 2)); // 新光心

    // 基线 B = |Tx|. 在 P2 中，Tx = P2(0,3) / f
    // P2(0,3) 通常是 T_x * f
    double Tx = m_P2.at<double>(0, 3);
    m_base_rect = std::abs(Tx / m_f_rect);

    if (m_config.verbose) {
        std::cout << "[StereoEstimator] Init:\n"
            << "  Rectified Focal Length: " << m_f_rect << "\n"
            << "  Baseline: " << m_base_rect << " mm\n"
            << "  Input Undistorted: " << (m_config.is_input_undistorted ? "Yes" : "No")
            << std::endl;
    }
}

int StereoDepthEstimator::Compute(
    const std::vector<cv::Point2f>& leftPoints,
    const std::vector<std::vector<cv::Point2f>>& rightCandidates,
    std::vector<cv::Point3f>& outPoints3D,
    std::vector<std::array<cv::Point2f, 2>>& outMatchedPairs)
{

    // 在 Compute 函数的最开头插入这段代码：

    if (logFlag) {
        spdlog::info("========== Input Data Check ==========");
        spdlog::info("Input Left Points: {}", leftPoints.size());
        spdlog::info("Input Right Candidates Groups: {}", rightCandidates.size());

        //以此防止 leftPoints 和 rightCandidates 大小不一致导致的越界
        size_t loopCount = std::min(leftPoints.size(), rightCandidates.size());

        for (size_t i = 0; i < loopCount; ++i) {
            // 1. 获取左图点
            const auto& lPt = leftPoints[i];

            // 2. 将该点对应的所有右图候选点拼接成一个字符串
            std::string candStr = "";
            const auto& candidates = rightCandidates[i];

            if (candidates.empty()) {
                candStr = "None";
            }
            else {
                for (const auto& rPt : candidates) {
                    // fmt::format 是 spdlog 内置的格式化工具
                    candStr += fmt::format("({:.1f},{:.1f}) ", rPt.x, rPt.y);
                }
            }

            // 3. 打印一行：左点坐标 -> 候选点数量 -> 具体候选点坐标
            spdlog::info("Idx [{:03d}] L:({:6.1f},{:6.1f}) | {} Candidates: [ {}]",
                i,
                lPt.x, lPt.y,
                candidates.size(),
                candStr
            );
        }

        if (leftPoints.size() != rightCandidates.size()) {
            spdlog::error("WARNING: Size mismatch! Left: {}, RightCandidates: {}",
                leftPoints.size(), rightCandidates.size());
        }
        spdlog::info("======================================");
    }
    // 1. 初始化输出容器
    // 确保输出与输入 leftPoints 大小一致，默认填充 0
    outPoints3D.clear();
    outPoints3D.resize(leftPoints.size(), cv::Point3f(0, 0, 0));

    outMatchedPairs.clear();
    outMatchedPairs.resize(leftPoints.size(), { cv::Point2f(0,0), cv::Point2f(0,0) });

    m_lastDisparities.clear();
    m_lastPoints3D.clear();

    if (leftPoints.empty()) return 0;

    // --- 步骤 1: 贪婪匹配 (一对多 -> 一对一) ---
    std::vector<cv::Point2f> matchedLeft, matchedRight;
    std::vector<int> originalIndices; // 记录匹配成功的点是 leftPoints 里的第几个

    matchPointsGreedy(leftPoints, rightCandidates, matchedLeft, matchedRight, originalIndices);

    if (matchedLeft.empty()) return 0;

    // --- 步骤 2: 数学极线校正 (Rectification) ---
    // 输入的点可能只是去了畸变，或者还是原始点。
    // 我们必须把它们转换到“平行双目平面”上，才能做 d = xL - xR
    std::vector<cv::Point2f> rectLeft, rectRight;
    rectifyPointCoordinates(matchedLeft, matchedRight, rectLeft, rectRight);

    // --- 步骤 3: 三角测量 (计算 XYZ) ---
    int validCount = 0;

    for (size_t k = 0; k < matchedLeft.size(); ++k) {
        int originalIdx = originalIndices[k];

        cv::Point2f pl = rectLeft[k];
        cv::Point2f pr = rectRight[k];

        // 计算视差
        double disparity = pl.x - pr.x;

        // 简单的Y轴对齐检查 (在校正平面上，y应该相等)
        double y_diff = std::abs(pl.y - pr.y);

        // 视差有效性检查
        bool is_valid = (disparity > m_config.min_disparity) &&
            (disparity < m_config.max_disparity) &&
            (y_diff < 10.0); // 允许 5 像素的校正误差

        if (is_valid) {
            // Z = (f * B) / d
            double Z = (m_f_rect * m_base_rect) / disparity;

            // X = (u - cx) * Z / f
            double X = (pl.x - m_c_rect.x) * Z / m_f_rect;
            // Y = (v - cy) * Z / f
            double Y = (pl.y - m_c_rect.y) * Z / m_f_rect;

            // 深度范围过滤
            if (Z >= m_config.min_depth_mm && Z <= m_config.max_depth_mm) {
                cv::Point3f pt3d(X, Y, Z);

                // [关键] 填入对应的原始索引位置
                outPoints3D[originalIdx] = pt3d;

                // [关键] 填入对应的 2D 坐标对 (使用原始匹配点，方便可视化)
                outMatchedPairs[originalIdx] = { matchedLeft[k], matchedRight[k] };

                // 缓存用于统计
                m_lastPoints3D.push_back(pt3d);
                m_lastDisparities.push_back(disparity);
                validCount++;
            }
        }
    }

    if (logFlag) {
        spdlog::info("========== Stereo Results (Valid: {}/{}) ==========", validCount, leftPoints.size());

        for (size_t i = 0; i < outPoints3D.size(); ++i) {
            const auto& pt3d = outPoints3D[i];
            const auto& pair = outMatchedPairs[i];

            // 过滤逻辑：如果 Z 依然是 0，说明这个点没有匹配成功，跳过不打印
            // 使用 1e-6 防止浮点数精度的微小误差
            if (std::abs(pt3d.z) > 1e-6) {
                // 使用 fmt 格式化输出 (spdlog自带)
                // {:03d}: 索引占3位补0
                // {:.1f}: 保留1位小数
                // {:.2f}: 保留2位小数
                spdlog::info("Idx [{:03d}] | 2D: L({:6.1f},{:6.1f}) <-> R({:6.1f},{:6.1f}) | 3D: [{:6.2f}, {:6.2f}, {:6.2f}]",
                    i,
                    pair[0].x, pair[0].y,  // 左图坐标
                    pair[1].x, pair[1].y,  // 右图坐标
                    pt3d.x, pt3d.y, pt3d.z // 3D坐标
                );
            }
        }
        spdlog::info("========================================================");
    }

    // 原有的 verbose 输出 (如果你想保留的话，或者也可以删掉用上面的代替)
    if (m_config.verbose && !logFlag) { // 避免重复打印
        std::cout << "[StereoEstimator] Computed " << validCount << " valid 3D points." << std::endl;
    }

    return validCount;
}

// ---------------------------------------------------------
// [核心算法] 贪婪匹配策略
// ---------------------------------------------------------
void StereoDepthEstimator::matchPointsGreedy(
    const std::vector<cv::Point2f>& leftPoints,
    const std::vector<std::vector<cv::Point2f>>& rightCandidates,
    std::vector<cv::Point2f>& outMatchedLeft,
    std::vector<cv::Point2f>& outMatchedRight,
    std::vector<int>& outOriginalIndices)
{
    outMatchedLeft.clear();
    outMatchedRight.clear();
    outOriginalIndices.clear();

    // 用于记录已经被使用的右图点（防止重复匹配）
    std::vector<cv::Point2f> usedRightPoints;

    for (size_t i = 0; i < leftPoints.size(); ++i) {
        if (i >= rightCandidates.size()) break;

        const auto& candidates = rightCandidates[i];
        if (candidates.empty()) continue; // 没候选，跳过

        int chosenIdx = -1;

        // 贪婪查找：找第一个没被用过的候选点
        for (size_t j = 0; j < candidates.size(); ++j) {
            const cv::Point2f& candPt = candidates[j];

            if (!isPointUsed(candPt, usedRightPoints, 0.5f)) {
                chosenIdx = (int)j;
                break; // 找到了，就它了
            }
        }

        // 如果找到了有效匹配
        if (chosenIdx != -1) {
            cv::Point2f finalRightPt = candidates[chosenIdx];

            outMatchedLeft.push_back(leftPoints[i]);
            outMatchedRight.push_back(finalRightPt);

            // 记录下：这个匹配对应的是 leftPoints 的第 i 个点
            outOriginalIndices.push_back((int)i);

            // 标记占用
            usedRightPoints.push_back(finalRightPt);
        }
    }
}

// ---------------------------------------------------------
// [核心算法] 坐标校正 (Undistort -> Rectify)
// ---------------------------------------------------------
void StereoDepthEstimator::rectifyPointCoordinates(
    const std::vector<cv::Point2f>& srcLeft,
    const std::vector<cv::Point2f>& srcRight,
    std::vector<cv::Point2f>& dstRectLeft,
    std::vector<cv::Point2f>& dstRectRight)
{
    // 关键点：输入点是否已经去过畸变？
    // 如果 is_input_undistorted 为 true，则传入空的畸变矩阵 (cv::Mat())，防止二次去畸变
    cv::Mat D1_to_use = m_config.is_input_undistorted ? cv::Mat() : m_D1;
    cv::Mat D2_to_use = m_config.is_input_undistorted ? cv::Mat() : m_D2;

    // cv::undistortPoints 不仅去畸变，还可以通过 R 和 P 参数
    // 将点直接变换到 "校正后的平行平面"
    // 公式： dst = P * R * inv(K) * src

    if (!srcLeft.empty()) {
        cv::undistortPoints(srcLeft, dstRectLeft, m_K1, D1_to_use, m_R1, m_P1);
    }
    if (!srcRight.empty()) {
        cv::undistortPoints(srcRight, dstRectRight, m_K2, D2_to_use, m_R2, m_P2);
    }
}

bool StereoDepthEstimator::isPointUsed(const cv::Point2f& pt, const std::vector<cv::Point2f>& usedPoints, float tolerance) const
{
    for (const auto& used : usedPoints) {
        // 简单的曼哈顿距离判断，速度快
        if (std::abs(pt.x - used.x) < tolerance && std::abs(pt.y - used.y) < tolerance) {
            return true;
        }
    }
    return false;
}

void StereoDepthEstimator::PrintDisparityStatistics() const
{
    if (m_lastDisparities.empty()) return;

    double sum = std::accumulate(m_lastDisparities.begin(), m_lastDisparities.end(), 0.0);
    double mean = sum / m_lastDisparities.size();

    double sq_sum = std::inner_product(m_lastDisparities.begin(), m_lastDisparities.end(), m_lastDisparities.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / m_lastDisparities.size() - mean * mean);

    std::cout << "--- Disparity Stats ---\n"
        << "Count: " << m_lastDisparities.size() << "\n"
        << "Mean:  " << mean << " px\n"
        << "StDev: " << stdev << " px\n"
        << "-----------------------" << std::endl;
}

void StereoDepthEstimator::PrintAccuracyEstimate() const
{
    std::cout << "--- Accuracy Estimate (Theoretical) ---\n"
        << "Baseline: " << m_base_rect << " mm\n"
        << "Focal:    " << m_f_rect << " px\n"
        << "Assumed Disparity Error: 0.5 px\n";

    double depths[] = { 500.0, 1000.0, 2000.0 };
    for (double z : depths) {
        // ErrZ = Z^2 * d_err / (f * B)
        double err = (z * z * 0.5) / (m_f_rect * m_base_rect);
        std::cout << "Depth " << z << "mm -> Error +/- " << err << " mm\n";
    }
    std::cout << "---------------------------------------" << std::endl;
}