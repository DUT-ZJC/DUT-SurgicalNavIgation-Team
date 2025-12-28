#include "Match_Points.hpp"
#include <algorithm>
#include <limits>


PointsMatcher::~PointsMatcher()
{
}


PointsMatcher::PointsMatcher(const cv::Mat& cameraMatrix1,
    const cv::Mat& cameraMatrix2,
    const cv::Mat& distCoeffs1,
    const cv::Mat& distCoeffs2,
    const cv::Mat& R,
    const cv::Mat& T,
    float minRadius, float maxRadius,
    int brightThreshold, double minArea)
    : m_minRadius(minRadius)
    , m_maxRadius(maxRadius)
    , m_brightThreshold(brightThreshold)
    , m_minArea(minArea)
    , m_sortTolerance(10.0f)
    , m_totalLeftPoints(0)
    , m_totalRightPoints(0)
    , m_matchedCount(0)
    , cameraMatrix1(cameraMatrix1)
    , cameraMatrix2(cameraMatrix2)
    , distCoeffs1(distCoeffs1)
    , distCoeffs2(distCoeffs2)
    , R(R),T(T)
{
    computeFundamentalMatrix();
}

void PointsMatcher::setDetectionParams(float minRadius, float maxRadius,
    int brightThreshold, double minArea)
{
    m_minRadius = minRadius;
    m_maxRadius = maxRadius;
    m_brightThreshold = brightThreshold;
    m_minArea = minArea;
}

void PointsMatcher::setSortTolerance(float tolerance)
{
    m_sortTolerance = tolerance;
}

cv::Mat PointsMatcher::preprocessImage(const cv::Mat& image)
{
    cv::Mat processed;

    // 转换为灰度图
    if (image.channels() == 3) {
        cv::cvtColor(image, processed, cv::COLOR_BGR2GRAY);
    }
    else {
        processed = image.clone();
    }

    // 高斯模糊去噪
    cv::GaussianBlur(processed, processed, cv::Size(5, 5), 1.5);

    return processed;
}

std::vector<cv::Point2f> PointsMatcher::sortPointsRasterScan(const std::vector<cv::Point2f>& points)
{
    if (points.empty()) {
        return points;
    }

    std::vector<cv::Point2f> sortedPoints = points;

    // 光栅扫描排序：先按y排序，y相近时按x排序
    std::sort(sortedPoints.begin(), sortedPoints.end(),
        [this](const cv::Point2f& a, const cv::Point2f& b) {
            // 先按y排序，y相近时按x排序
            if (std::abs(a.y - b.y) > m_sortTolerance) {
                return a.y < b.y;
            }
            return a.x < b.x;
        });

    std::cout << "Points sorted in raster scan order (top-left to bottom-right)" << std::endl;

    return sortedPoints;
}

std::vector<cv::Point2f> PointsMatcher::detectBrightCircles(const cv::Mat& grayImage)
{
    std::vector<cv::Point2f> centers;

    // 二值化：提取亮点
    cv::Mat binary;
    cv::threshold(grayImage, binary, m_brightThreshold, 255, cv::THRESH_BINARY);

    // 形态学操作：去除噪点
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);

    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 遍历每个轮廓
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);

        // 面积过滤
        if (area < m_minArea) {
            continue;
        }

        // 计算最小外接圆
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(contours[i], center, radius);

        // 半径过滤
        if (radius < m_minRadius || radius > m_maxRadius) {
            continue;
        }

        // 圆度检查（轮廓面积与外接圆面积的比值）
        double circularity = area / (CV_PI * radius * radius);
        if (circularity < 0.6) { // 圆度阈值可调
            continue;
        }

        // 使用矩计算更精确的质心
        cv::Moments m = cv::moments(contours[i]);
        if (m.m00 > 0) {
            center.x = static_cast<float>(m.m10 / m.m00);
            center.y = static_cast<float>(m.m01 / m.m00);
        }

        centers.push_back(center);
    }

    std::cout << "Detected " << centers.size() << " bright circles (before sorting)" << std::endl;

    return centers;
}

std::vector<cv::Point2f> PointsMatcher::detectBalls(const cv::Mat& image)
{
    if (image.empty()) {
        std::cerr << "Input image is empty!" << std::endl;
        return std::vector<cv::Point2f>();
    }

    // 预处理
    cv::Mat processed = preprocessImage(image);

    // 检测亮圆
    std::vector<cv::Point2f> centers = detectBrightCircles(processed);

    // 自动排序：光栅扫描顺序（从左上到右下）
    std::vector<cv::Point2f> sortedCenters = sortPointsRasterScan(centers);

    // 打印排序后的点坐标
    std::cout << "Sorted points coordinates:" << std::endl;
    for (size_t i = 0; i < sortedCenters.size(); i++) {
        std::cout << "  Point[" << i << "]: ("
            << sortedCenters[i].x << ", "
            << sortedCenters[i].y << ")" << std::endl;
    }
    return sortedCenters;
}

void PointsMatcher::computeFundamentalMatrix()
{
    // 计算本质矩阵 E = [T]x * R
    // [T]x 是 T 的反对称矩阵
    cv::Mat Tx = (cv::Mat_<double>(3, 3)<<
        0, -T.at<double>(2, 0), T.at<double>(1, 0),
        T.at<double>(2, 0), 0, -T.at<double>(0, 0),
        -T.at<double>(1, 0), T.at<double>(0, 0), 0);

    cv::Mat E = Tx * R;

    // 计算基础矩阵 F = K2^-T * E * K1^-1
    cv::Mat K1_inv = cameraMatrix1.inv();
    cv::Mat K2_inv_t = cameraMatrix2.inv().t();

    F = K2_inv_t * E * K1_inv;

}

double PointsMatcher::pointToEpilineDistance(const cv::Point2f& point, const cv::Vec3f& epiline)
{
    // 极线方程: ax + by + c = 0
    // 点到直线距离: |ax + by + c| / sqrt(a^2 + b^2)
    double a = epiline[0];
    double b = epiline[1];
    double c = epiline[2];

    double numerator = std::abs(a * point.x + b * point.y + c);
    double denominator = std::sqrt(a * a + b * b);

    if (denominator < 1e-6) {
        return std::numeric_limits<double>::max();
    }

    return numerator / denominator;
}

std::vector<cv::Point2f> PointsMatcher::undistortPoints(
    const std::vector<cv::Point2f>& points,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs)
{
    std::vector<cv::Point2f> undistorted;

    if (points.empty()) {
        return undistorted;
    }

    // 去畸变
    cv::undistortPoints(points, undistorted, cameraMatrix, distCoeffs,
        cv::noArray(), cameraMatrix);

    return undistorted;
}

bool PointsMatcher::stereoMatch(
    const std::vector<cv::Point2f>& leftPoints,
    const std::vector<cv::Point2f>& rightPoints,
    std::vector<cv::Point2f>& filteredLeftPoints,
    std::vector<std::vector<cv::Point2f>>& filteredRightPoints,
    double epipolarThreshold)
{
    // 清空输出和统计信息
    filteredLeftPoints.clear();
    filteredRightPoints.clear();
    m_matchedIndices.clear();

    m_totalLeftPoints = static_cast<int>(leftPoints.size());
    m_totalRightPoints = static_cast<int>(rightPoints.size());
    m_matchedCount = 0;

    if (leftPoints.empty() || rightPoints.empty()) {
        std::cerr << "Input points are empty!" << std::endl;
        return false;
    }

    std::cout << "\n=== Stereo Matching (with filtering) ===" << std::endl;
    std::cout << "Left points: " << leftPoints.size()
        << ", Right points: " << rightPoints.size() << std::endl;

    // 去畸变
    std::vector<cv::Point2f> leftUndistorted = undistortPoints(leftPoints,
        cameraMatrix1, distCoeffs1);
    std::vector<cv::Point2f> rightUndistorted = undistortPoints(rightPoints,
        cameraMatrix2, distCoeffs2);

    std::cout << "Fundamental Matrix:\n" << F << std::endl;

    // 对于左图中的每个点，计算其在右图中的极线
    std::vector<cv::Vec3f> epilines;
    cv::computeCorrespondEpilines(leftUndistorted, 1, F, epilines);

    // 临时存储所有匹配关系
    std::vector<std::vector<int>> tempMatchIndices(leftPoints.size());
    std::vector<std::vector<double>> tempMatchDistances(leftPoints.size());

    // 第一步：对每个左图点进行匹配，找出所有候选
    std::cout << "\n--- Phase 1: Finding all candidate matches ---" << std::endl;
    for (size_t i = 0; i < leftUndistorted.size(); i++) {
        std::vector<int> matchIndx;
        std::vector<double> distVec;

        for (size_t j = 0; j < rightUndistorted.size(); j++) {
            double dist = pointToEpilineDistance(rightUndistorted[j], epilines[i]);

            if (dist < epipolarThreshold) {
                distVec.push_back(dist);
                matchIndx.push_back(j);
            }
        }

        if (!matchIndx.empty()) {
            tempMatchIndices[i] = matchIndx;
            tempMatchDistances[i] = distVec;

            std::cout << "Left[" << i << "] -> " << matchIndx.size() << " candidate(s): ";
            for (size_t k = 0; k < matchIndx.size(); k++) {
                std::cout << "R[" << matchIndx[k] << "](dist="
                    << std::fixed << std::setprecision(2) << distVec[k] << ")";
                if (k < matchIndx.size() - 1) std::cout << ", ";
            }
            std::cout << std::endl;
        }
    }

    // 第二步：迭代清理冲突，直到匹配关系不再变化
    std::cout << "\n--- Phase 2: Iterative conflict resolution ---" << std::endl;

    int iteration = 0;
    bool changed = true;

    while (changed) {
        changed = false;
        iteration++;

        std::cout << "\n[Iteration " << iteration << "]" << std::endl;

        // 找出当前最大的候选数量
        size_t maxCandidates = 0;
        for (const auto& matches : tempMatchIndices) {
            if (matches.size() > maxCandidates) {
                maxCandidates = matches.size();
            }
        }

        if (maxCandidates <= 1) {
            std::cout << "All matches are unique or empty, stopping iteration." << std::endl;
            break;
        }
        // 从size=1开始，逐步处理到size=maxCandidates-1
        for (size_t targetSize = 1; targetSize < maxCandidates; targetSize++) {
            // 收集当前size的所有占用的右图点索引
            std::set<int> occupiedRightIndices;

            for (size_t i = 0; i < tempMatchIndices.size(); i++) {
                if (tempMatchIndices[i].size() == targetSize) {
                    for (int rightIdx : tempMatchIndices[i]) {
                        occupiedRightIndices.insert(rightIdx);
                    }
                }
            }  
            if (occupiedRightIndices.empty()) {
                continue;
            }

            std::cout << "Size=" << targetSize << ": Found " << occupiedRightIndices.size()
                << " occupied right points: ";
            for (int idx : occupiedRightIndices) {
                std::cout << "R[" << idx << "] ";
            }
            std::cout << std::endl;

            // 从size>targetSize的匹配中移除这些占用的右图点
            for (size_t i = 0; i < tempMatchIndices.size(); i++) {
                if (tempMatchIndices[i].size() > targetSize) {
                    std::vector<int> newMatches;
                    std::vector<double> newDistances;
                    int removedCount = 0;

                    for (size_t j = 0; j < tempMatchIndices[i].size(); j++) {
                        int rightIdx = tempMatchIndices[i][j];

                        if (occupiedRightIndices.find(rightIdx) == occupiedRightIndices.end()) {
                            // 这个右图点没有被占用，保留
                            newMatches.push_back(rightIdx);
                            newDistances.push_back(tempMatchDistances[i][j]);
                        }
                        else {
                            // 这个右图点被占用了，移除
                            removedCount++;
                        }
                    }
                    if (removedCount > 0) {
                        tempMatchIndices[i] = newMatches;
                        tempMatchDistances[i] = newDistances;
                        changed = true;

                        std::cout << "  Left[" << i << "]: "
                            << (newMatches.size() + removedCount) << " -> "
                            << newMatches.size() << " (removed " << removedCount << ")" << std::endl;
                    }
                }
            }
        }
        if (!changed) {
            std::cout << "No changes in this iteration, converged." << std::endl;
        }
    }

    std::cout << "\nConverged after " << iteration << " iteration(s)" << std::endl;

    // 第三步：统计最终结果
    std::cout << "\n--- Phase 3: Final results ---" << std::endl;

    m_matchedCount = 0;

    for (size_t i = 0; i < leftPoints.size(); i++) {
        if (tempMatchIndices[i].empty()) {
            std::cout << "✗ Left[" << i << "]: No match - FILTERED OUT" << std::endl;
            continue;
        }

        filteredLeftPoints.push_back(leftUndistorted[i]);  // 改为去畸变

        std::vector<cv::Point2f> matchPoints;
        for (int idx : tempMatchIndices[i]) {
            matchPoints.push_back(rightUndistorted[idx]);  // 改为去畸变
        }
        filteredRightPoints.push_back(matchPoints);

        m_matchedIndices.push_back(std::make_pair(static_cast<int>(i), tempMatchIndices[i]));

        if (tempMatchIndices[i].size() == 1) {
            m_matchedCount++;
        }

        // 详细输出
        std::cout << "✓ Left[" << i << "] (" << leftPoints[i].x << ", " << leftPoints[i].y
            << ") -> ";
        for (size_t j = 0; j < tempMatchIndices[i].size(); j++) {
            std::cout << "R[" << tempMatchIndices[i][j] << "]("
                << rightPoints[tempMatchIndices[i][j]].x << ", "
                << rightPoints[tempMatchIndices[i][j]].y << "), dist="
                << std::fixed << std::setprecision(2) << tempMatchDistances[i][j] << " px";
            if (j < tempMatchIndices[i].size() - 1) std::cout << " | ";
        }
        std::cout << std::endl;
    }

    // 统计信息
    std::cout << "\n=== Matching Summary ===" << std::endl;
    std::cout << "Total left points: " << m_totalLeftPoints << std::endl;
    std::cout << "Total right points: " << m_totalRightPoints << std::endl;
    std::cout << "Unique (1-to-1) matches: " << m_matchedCount << std::endl;
    std::cout << "Final matched left points: " << filteredLeftPoints.size() << std::endl;

    int totalRightMatched = 0;
    int oneToMany = 0;

    for (const auto& rightPts : filteredRightPoints) {
        totalRightMatched += rightPts.size();
        if (rightPts.size() > 1) {
            oneToMany++;
        }
    }

    std::cout << "One-to-many matches: " << oneToMany << std::endl;
    std::cout << "Total right points matched: " << totalRightMatched << std::endl;
    std::cout << "Left points filtered out: " << (m_totalLeftPoints - filteredLeftPoints.size()) << std::endl;
    std::cout << "Match rate: " << (filteredLeftPoints.size() * 100.0 / m_totalLeftPoints) << "%" << std::endl;
    std::cout << "========================\n" << std::endl;

    return !filteredLeftPoints.empty();
}

std::vector<std::pair<int, std::vector<int>>> PointsMatcher::getMatchedIndices() const
{
    return m_matchedIndices;
}

void PointsMatcher::getMatchStatistics(int& totalLeftPoints, int& totalRightPoints, int& matchedCount) const
{
    totalLeftPoints = m_totalLeftPoints;
    totalRightPoints = m_totalRightPoints;
    matchedCount = m_matchedCount;
}

void PointsMatcher::visualizeMatchingProcess(const cv::Mat& leftImage,
    const cv::Mat& rightImage,
    const std::vector<cv::Point2f>& allLeftPoints,
    const std::vector<cv::Point2f>& allRightPoints,
    const std::vector<cv::Point2f>& matchedLeftPoints,
    const std::vector<std::vector<cv::Point2f>>& matchedRightPoints,  // 改正：一对多
    BrowsableQueue& visualImageQueue)
{
    // 直接使用原始图像，不去畸变
    cv::Mat leftDisplay, rightDisplay , visualImage;
    if (leftImage.channels() == 1) {
        cv::cvtColor(leftImage, leftDisplay, cv::COLOR_GRAY2BGR);
    }
    else {
        leftDisplay = leftImage.clone();
    }
    if (rightImage.channels() == 1) {
        cv::cvtColor(rightImage, rightDisplay, cv::COLOR_GRAY2BGR);
    }
    else {
        rightDisplay = rightImage.clone();
    }

    cv::hconcat(leftDisplay, rightDisplay, visualImage);
    int offset = leftImage.cols;

    // 去畸变所有点坐标（快速操作）
    std::vector<cv::Point2f> allLeftUndistorted = undistortPoints(allLeftPoints,
        cameraMatrix1, distCoeffs1);
    std::vector<cv::Point2f> allRightUndistorted = undistortPoints(allRightPoints,
        cameraMatrix2, distCoeffs2);

    // matchedLeftPoints 和 matchedRightPoints 已经是去畸变后的坐标

    // 先绘制所有未匹配的点（灰色）
    for (const auto& pt : allLeftUndistorted) {
        bool isMatched = false;
        for (const auto& mpt : matchedLeftPoints) {
            if (std::abs(pt.x - mpt.x) < 0.1 && std::abs(pt.y - mpt.y) < 0.1) {
                isMatched = true;
                break;
            }
        }
        if (!isMatched) {
            cv::circle(visualImage, pt, 8, cv::Scalar(128, 128, 128), 2);
            cv::drawMarker(visualImage, pt, cv::Scalar(128, 128, 128),
                cv::MARKER_TILTED_CROSS, 15, 2);
        }
    }

    for (const auto& pt : allRightUndistorted) {
        cv::Point2f displayPt = pt;
        displayPt.x += offset;
        bool isMatched = false;
        for (const auto& mpts : matchedRightPoints) {
            for (const auto& mpt : mpts) {
                if (std::abs(pt.x - mpt.x) < 0.1 && std::abs(pt.y - mpt.y) < 0.1) {
                    isMatched = true;
                    break;
                }
            }
            if (isMatched) break;
        }
        if (!isMatched) {
            cv::circle(visualImage, displayPt, 8, cv::Scalar(128, 128, 128), 2);
            cv::drawMarker(visualImage, displayPt, cv::Scalar(128, 128, 128),
                cv::MARKER_TILTED_CROSS, 15, 2);
        }
    }

    // 预定义颜色列表
    std::vector<cv::Scalar> colors = {
        cv::Scalar(255, 0, 0),     // 蓝色
        cv::Scalar(0, 255, 0),     // 绿色
        cv::Scalar(0, 0, 255),     // 红色
        cv::Scalar(255, 255, 0),   // 青色
        cv::Scalar(255, 0, 255),   // 品红
        cv::Scalar(0, 255, 255),   // 黄色
        cv::Scalar(255, 128, 0),   // 橙色
        cv::Scalar(128, 0, 255),   // 紫色
        cv::Scalar(0, 255, 128),   // 青绿
        cv::Scalar(255, 200, 100)  // 浅橙
    };

    // 计算极线（matchedLeftPoints 已经是去畸变的）
    std::vector<cv::Vec3f> epilines;
    cv::computeCorrespondEpilines(matchedLeftPoints, 1, F, epilines);

    // 创建右图点到左图索引的映射
    std::map<std::pair<float, float>, std::vector<int>> rightPointToLeftIndices;
    for (size_t i = 0; i < matchedLeftPoints.size(); i++) {
        const std::vector<cv::Point2f>& rightPts = matchedRightPoints[i];
        for (const auto& rightPt : rightPts) {
            auto key = std::make_pair(std::round(rightPt.x * 10) / 10,
                std::round(rightPt.y * 10) / 10);
            rightPointToLeftIndices[key].push_back(i);
        }
    }

    // 绘制匹配的点对和极线
    int uniqueMatches = 0;
    int multipleMatches = 0;

    for (size_t i = 0; i < matchedLeftPoints.size(); i++) {
        cv::Point2f leftPt = matchedLeftPoints[i];  // 去畸变坐标
        const std::vector<cv::Point2f>& rightPts = matchedRightPoints[i];  // 去畸变坐标

        // 每个左图点用不同颜色
        cv::Scalar color = colors[i % colors.size()];

        // 统计唯一匹配和多重匹配
        if (rightPts.size() == 1) {
            uniqueMatches++;
        }
        else {
            multipleMatches++;
        }

        // 绘制左图点（使用去畸变坐标在原图上绘制）
        cv::circle(visualImage, leftPt, 10, color, -1);
        cv::circle(visualImage, leftPt, 10, cv::Scalar(255, 255, 255), 2);

        // 标注左图点索引
        cv::putText(visualImage, std::to_string(i),
            leftPt + cv::Point2f(15, -15),
            cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 5);
        cv::putText(visualImage, std::to_string(i),
            leftPt + cv::Point2f(15, -15),
            cv::FONT_HERSHEY_SIMPLEX, 1.0, color, 3);

        // 在右图绘制对应的极线
        cv::Vec3f epiline = epilines[i];
        double a = epiline[0];
        double b = epiline[1];
        double c = epiline[2];

        // 计算极线与图像边界的交点
        cv::Point2f pt1, pt2;
        int rightWidth = rightImage.cols;
        int rightHeight = rightImage.rows;

        if (std::abs(b) > 1e-6) {
            pt1.x = 0;
            pt1.y = -c / b;
            pt2.x = rightWidth - 1;
            pt2.y = -(a * (rightWidth - 1) + c) / b;
        }
        else {
            pt1.x = -c / a;
            pt1.y = 0;
            pt2.x = -c / a;
            pt2.y = rightHeight - 1;
        }

        // 裁剪到图像范围内
        pt1.x = std::max(0.0f, std::min((float)(rightWidth - 1), pt1.x));
        pt1.y = std::max(0.0f, std::min((float)(rightHeight - 1), pt1.y));
        pt2.x = std::max(0.0f, std::min((float)(rightWidth - 1), pt2.x));
        pt2.y = std::max(0.0f, std::min((float)(rightHeight - 1), pt2.y));

        // 偏移到右图位置
        pt1.x += offset;
        pt2.x += offset;

        // 绘制极线
        cv::line(visualImage, pt1, pt2, color, 2, cv::LINE_AA);

        // 绘制右图匹配点（使用去畸变坐标在原图上绘制）
        for (const auto& rightPt : rightPts) {
            cv::Point2f displayPt = rightPt;
            displayPt.x += offset;
            cv::circle(visualImage, displayPt, 10, color, -1);
            cv::circle(visualImage, displayPt, 10, cv::Scalar(255, 255, 255), 2);
        }
    }

    // 统一标注右图点的左图索引
    for (const auto& entry : rightPointToLeftIndices) {
        float x = entry.first.first;
        float y = entry.first.second;
        const std::vector<int>& leftIndices = entry.second;

        // 构建标签文本
        std::string label;
        for (size_t i = 0; i < leftIndices.size(); i++) {
            label += std::to_string(leftIndices[i]);
            if (i < leftIndices.size() - 1) {
                label += ",";
            }
        }

        // 如果被多个左图点匹配,用红色高亮标注
        cv::Scalar labelColor = (leftIndices.size() > 1) ?
            cv::Scalar(0, 100, 255) : cv::Scalar(255, 255, 255);

        cv::Point2f displayPt(x + offset, y);

        // 黑色描边
        cv::putText(visualImage, label,
            displayPt + cv::Point2f(15, -15),
            cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 5);
        // 实际文字
        cv::putText(visualImage, label,
            displayPt + cv::Point2f(15, -15),
            cv::FONT_HERSHEY_SIMPLEX, 1.0, labelColor, 3);
    }

    // 显示统计信息
    int totalRightMatched = 0;
    for (const auto& pts : matchedRightPoints) {
        totalRightMatched += pts.size();
    }

    std::string info1 = "Left: " + std::to_string(allLeftPoints.size()) +
        " (matched: " + std::to_string(matchedLeftPoints.size()) + ")";
    std::string info2 = "Right: " + std::to_string(allRightPoints.size()) +
        " (matched: " + std::to_string(totalRightMatched) + ")";
    std::string info3 = "Unique: " + std::to_string(uniqueMatches) +
        ", Multiple: " + std::to_string(multipleMatches);

    // 黑色背景矩形
    cv::rectangle(visualImage, cv::Point(5, 5), cv::Point(700, 130),
        cv::Scalar(0, 0, 0), -1);

    cv::putText(visualImage, info1, cv::Point(10, 40),
        cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 255), 2);
    cv::putText(visualImage, info2, cv::Point(10, 75),
        cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 255), 2);
    cv::putText(visualImage, info3, cv::Point(10, 110),
        cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(255, 255, 255), 2);

    visualImageQueue.push(visualImage);
}