#include "DetectorBall.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

using namespace cv;
using namespace std;

BallDetector::BallDetector(float minRadius, float maxRadius,
    int brightThreshold, double minArea)
    : minRadius_(minRadius), maxRadius_(maxRadius),
    brightThreshold_(brightThreshold), minArea_(minArea) {
}

void BallDetector::setDetectionParams(float minRadius, float maxRadius,
    int brightThreshold, double minArea) {
    minRadius_ = minRadius;
    maxRadius_ = maxRadius;
    brightThreshold_ = brightThreshold;
    minArea_ = minArea;
}

double BallDetector::angleFromCenter(const Point2f& pt, double cx, double cy) {
    double dx = pt.x - cx;
    double dy = pt.y - cy;
    return atan2(dy, dx);  // [-π, π]
}

vector<Point2f> BallDetector::sortPointsByAngle(const vector<Point2f>& groupPts) {
    if (groupPts.size() != 4) {
        throw invalid_argument("组内必须是4个点");
    }

    // 计算组内中心点
    double cx = 0, cy = 0;
    for (const auto& pt : groupPts) {
        cx += pt.x;
        cy += pt.y;
    }
    cx /= 4.0;
    cy /= 4.0;

    // 计算每个点的极角并排序
    vector<pair<Point2f, double>> ptsWithAngle;
    for (const auto& pt : groupPts) {
        double angle = angleFromCenter(pt, cx, cy);
        ptsWithAngle.push_back({ pt, angle });
    }

    // 按角度升序排序
    sort(ptsWithAngle.begin(), ptsWithAngle.end(),
        [](const pair<Point2f, double>& a, const pair<Point2f, double>& b) {
            return a.second < b.second;
        });

    vector<Point2f> result;
    for (const auto& item : ptsWithAngle) {
        result.push_back(item.first);
    }
    return result;
}

vector<vector<Point2f>> BallDetector::clusterPoints(const vector<Point2f>& points) {
    if (points.size() != 8) {
        throw invalid_argument("必须是8个点");
    }

    // 计算所有点的中心
    double centerX = 0, centerY = 0;
    for (const auto& pt : points) {
        centerX += pt.x;
        centerY += pt.y;
    }
    centerX /= 8.0;
    centerY /= 8.0;

    // 使用x坐标进行简单分组（假设两组在x方向上分布）
    vector<Point2f> leftPoints, rightPoints;
    for (const auto& pt : points) {
        if (pt.x < centerX) {
            leftPoints.push_back(pt);
        }
        else {
            rightPoints.push_back(pt);
        }
    }

    // 如果按x分组不均匀，改用y坐标分组
    if (leftPoints.size() != 4 || rightPoints.size() != 4) {
        leftPoints.clear();
        rightPoints.clear();
        for (const auto& pt : points) {
            if (pt.y < centerY) {
                leftPoints.push_back(pt);
            }
            else {
                rightPoints.push_back(pt);
            }
        }
    }

    // 如果还是不均匀，使用距离分组
    if (leftPoints.size() != 4 || rightPoints.size() != 4) {
        leftPoints.clear();
        rightPoints.clear();

        // 找到距离最远的两个点作为初始聚类中心
        double maxDist = 0;
        Point2f center1, center2;
        for (size_t i = 0; i < points.size(); ++i) {
            for (size_t j = i + 1; j < points.size(); ++j) {
                double dist = norm(points[i] - points[j]);
                if (dist > maxDist) {
                    maxDist = dist;
                    center1 = points[i];
                    center2 = points[j];
                }
            }
        }

        // 根据距离分组
        for (const auto& pt : points) {
            double dist1 = norm(pt - center1);
            double dist2 = norm(pt - center2);
            if (dist1 < dist2) {
                leftPoints.push_back(pt);
            }
            else {
                rightPoints.push_back(pt);
            }
        }
    }

    return { leftPoints, rightPoints };
}

vector<Point2f> BallDetector::getOrdered8Points(const vector<Point2f>& points) {
    if (points.size() != 8) {
        throw invalid_argument("必须是8个点");
    }

    // 使用聚类分成两组
    vector<vector<Point2f>> groups = clusterPoints(points);

    if (groups[0].size() != 4 || groups[1].size() != 4) {
        throw runtime_error("聚类结果异常，无法分成两组各4个点");
    }

    // 计算每组的中心点
    Point2f center1(0, 0), center2(0, 0);
    for (const auto& pt : groups[0]) {
        center1 += pt;
    }
    center1 /= 4.0f;

    for (const auto& pt : groups[1]) {
        center2 += pt;
    }
    center2 /= 4.0f;

    // 按组中心的x坐标排序（左组在前）
    if (center1.x > center2.x) {
        swap(groups[0], groups[1]);
    }

    // 对每组内的点进行极角排序
    vector<Point2f> group1Ordered = sortPointsByAngle(groups[0]);
    vector<Point2f> group2Ordered = sortPointsByAngle(groups[1]);

    // 合并结果
    vector<Point2f> finalOrdered;
    finalOrdered.insert(finalOrdered.end(), group1Ordered.begin(), group1Ordered.end());
    finalOrdered.insert(finalOrdered.end(), group2Ordered.begin(), group2Ordered.end());

    return finalOrdered;
}

vector<Point2f> BallDetector::alignGroupStartPoint(const vector<Point2f>& refPts,
    const vector<Point2f>& targetPts) {
    if (refPts.size() != 4 || targetPts.size() != 4) {
        throw invalid_argument("两组都必须是4个点");
    }

    vector<Point2f> bestOrder = targetPts;
    double minTotalDist = numeric_limits<double>::max();

    // 尝试4种旋转对齐方式
    for (int i = 0; i < 4; ++i) {
        vector<Point2f> rotated;
        // 环状旋转
        for (int j = 0; j < 4; ++j) {
            rotated.push_back(targetPts[(i + j) % 4]);
        }

        // 计算总距离
        double totalDist = 0;
        for (int j = 0; j < 4; ++j) {
            totalDist += norm(refPts[j] - rotated[j]);
        }

        if (totalDist < minTotalDist) {
            minTotalDist = totalDist;
            bestOrder = rotated;
        }
    }

    return bestOrder;
}

vector<Point2f> BallDetector::detectBalls(const Mat& image) {
    Mat gray, bin;
    // 转灰度图
    cvtColor(image, gray, COLOR_BGR2GRAY);

    // 高斯模糊减少噪声
    GaussianBlur(gray, gray, Size(5, 5), 0);

    // 二值化：提取高亮度区域
    threshold(gray, bin, brightThreshold_, 255, THRESH_BINARY);

    // 提取轮廓
    vector<vector<Point>> contours;
    findContours(bin, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    vector<Point2f> centers;
    for (const auto& contour : contours) {
        Point2f center;
        float radius;
        minEnclosingCircle(contour, center, radius);

        // 过滤条件
        if (radius >= minRadius_ && radius <= maxRadius_ &&
            contourArea(contour) >= minArea_) {
            centers.push_back(center);
        }
    }

    // 如果检测到8个点，使用新的排序算法
    if (centers.size() == 8) {
        try {
            return getOrdered8Points(centers);
        }
        catch (const exception& e) {
            cout << "新排序算法失败，使用原始排序: " << e.what() << endl;
        }
    }

    // 原始排序方式（左上→右下）
    sort(centers.begin(), centers.end(), [](const Point2f& a, const Point2f& b) {
        const float row_thresh = 30.0f;
        if (fabs(a.y - b.y) < row_thresh) {
            return a.x < b.x;
        }
        else {
            return a.y < b.y;
        }
        });

    return centers;
}

double BallDetector::pointToEpilineDistance(const Point2f& point, const Vec3f& epiline) {
    // 极线方程: ax + by + c = 0
    // 点到直线距离: |ax + by + c| / sqrt(a² + b²)
    double a = epiline[0];
    double b = epiline[1];
    double c = epiline[2];

    double numerator = abs(a * point.x + b * point.y + c);
    double denominator = sqrt(a * a + b * b);

    return (denominator > 1e-6) ? (numerator / denominator) : numeric_limits<double>::max();
}

vector<pair<int, int>> BallDetector::hungarianAlgorithm(const vector<vector<double>>& costMatrix) {
    int n = costMatrix.size();
    if (n == 0) return {};

    int m = costMatrix[0].size();

    // 简化版匈牙利算法实现
    // 对于小规模问题，使用贪心近似算法
    vector<bool> rowUsed(n, false);
    vector<bool> colUsed(m, false);
    vector<pair<int, int>> matches;

    // 创建代价-索引对并排序
    vector<tuple<double, int, int>> costs;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            costs.push_back(make_tuple(costMatrix[i][j], i, j));
        }
    }

    sort(costs.begin(), costs.end());

    // 贪心分配
    for (const auto& cost : costs) {
        int row = get<1>(cost);
        int col = get<2>(cost);

        if (!rowUsed[row] && !colUsed[col]) {
            matches.push_back({ row, col });
            rowUsed[row] = true;
            colUsed[col] = true;

            if (matches.size() == min(n, m)) {
                break;
            }
        }
    }

    return matches;
}

vector<Point2f> BallDetector::stereoMatch(
    const vector<Point2f>& leftPoints,
    const vector<Point2f>& rightPoints,
    const Mat& cameraMatrix1,
    const Mat& cameraMatrix2,
    const Mat& distCoeffs1,
    const Mat& distCoeffs2,
    const Mat& R,
    const Mat& T,
    double epipolarThreshold) {

    if (leftPoints.size() != rightPoints.size()) {
        throw invalid_argument("左右图像点数必须相同");
    }

    size_t numPoints = leftPoints.size();
    if (numPoints == 0) {
        return {};
    }

    // 计算基础矩阵
    Mat F;
    Mat E = cameraMatrix2.t() * T.cross(R) * cameraMatrix2;  // 本质矩阵的简化计算
    F = cameraMatrix2.inv().t() * E * cameraMatrix1.inv();   // 基础矩阵

    // 计算极线
    vector<Vec3f> epilinesLeft, epilinesRight;

    // 为右图点计算在左图中的极线
    computeCorrespondEpilines(rightPoints, 2, F, epilinesLeft);
    // 为左图点计算在右图中的极线  
    computeCorrespondEpilines(leftPoints, 1, F, epilinesRight);

    // 构建代价矩阵 (基于极线约束)
    vector<vector<double>> costMatrix(numPoints, vector<double>(numPoints, 0.0));

    for (size_t i = 0; i < numPoints; ++i) {
        for (size_t j = 0; j < numPoints; ++j) {
            // 计算左图点i到右图点j对应极线的距离
            double dist1 = pointToEpilineDistance(leftPoints[i], epilinesLeft[j]);
            // 计算右图点j到左图点i对应极线的距离  
            double dist2 = pointToEpilineDistance(rightPoints[j], epilinesRight[i]);

            // 双向极线约束的平均距离作为代价
            double avgDist = (dist1 + dist2) / 2.0;

            // 如果超过阈值，设置为很大的代价
            if (avgDist > epipolarThreshold) {
                costMatrix[i][j] = 1000.0 + avgDist;
            }
            else {
                // 加入额外的几何约束：y坐标差异（双目校正后应该在同一水平线上）
                double yDiff = abs(leftPoints[i].y - rightPoints[j].y);
                costMatrix[i][j] = avgDist + 0.1 * yDiff;
            }
        }
    }

    // 使用匈牙利算法找到最优匹配
    vector<pair<int, int>> matches = hungarianAlgorithm(costMatrix);

    // 构建匹配后的右图点序列
    vector<Point2f> matchedRightPoints(numPoints);
    vector<bool> matched(numPoints, false);

    for (const auto& match : matches) {
        int leftIdx = match.first;
        int rightIdx = match.second;

        // 验证匹配质量
        if (costMatrix[leftIdx][rightIdx] < epipolarThreshold + 50.0) {
            matchedRightPoints[leftIdx] = rightPoints[rightIdx];
            matched[leftIdx] = true;
        }
    }

    // 对于未匹配的点，使用原始顺序或最近邻匹配
    for (size_t i = 0; i < numPoints; ++i) {
        if (!matched[i]) {
            if (i < rightPoints.size()) {
                matchedRightPoints[i] = rightPoints[i];
            }
        }
    }

    // 如果是8个点的情况，尝试使用分组排序进行进一步优化
    if (numPoints == 8) {
        try {
            vector<Point2f> leftOrdered = getOrdered8Points(leftPoints);
            vector<Point2f> rightOrderedRaw = getOrdered8Points(matchedRightPoints);

            // 分组对齐
            vector<Point2f> leftGroup1(leftOrdered.begin(), leftOrdered.begin() + 4);
            vector<Point2f> leftGroup2(leftOrdered.begin() + 4, leftOrdered.end());

            vector<Point2f> rightGroup1(rightOrderedRaw.begin(), rightOrderedRaw.begin() + 4);
            vector<Point2f> rightGroup2(rightOrderedRaw.begin() + 4, rightOrderedRaw.end());

            // 对齐两组
            vector<Point2f> alignedGroup1 = alignGroupStartPoint(leftGroup1, rightGroup1);
            vector<Point2f> alignedGroup2 = alignGroupStartPoint(leftGroup2, rightGroup2);

            // 合并结果
            vector<Point2f> result;
            result.insert(result.end(), alignedGroup1.begin(), alignedGroup1.end());
            result.insert(result.end(), alignedGroup2.begin(), alignedGroup2.end());

            return result;
        }
        catch (const exception& e) {
            cout << "8点分组匹配失败: " << e.what() << endl;
        }
    }

    return matchedRightPoints;
}