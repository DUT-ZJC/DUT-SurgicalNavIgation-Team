// 球识别模块实现
#include "detector.h"
#include <algorithm>
#include <cmath>
using namespace cv;
using namespace std;

// 计算点相对于中心的极角
double angleFromCenter(const Point2f& pt, double cx, double cy) {
    double dx = pt.x - cx;
    double dy = pt.y - cy;
    return atan2(dy, dx);  // [-π, π]
}

// 使用极角对一组4个点进行排序（逆时针）
vector<Point2f> sortCrossShapeAngle(const vector<Point2f>& groupPts) {
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

// 简化的KMeans聚类（将8个点分成2组）
vector<vector<Point2f>> clusterPointsKMeans(const vector<Point2f>& points) {
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

// 获取有序的8个点
vector<Point2f> getOrdered8Points(const vector<Point2f>& points) {
    if (points.size() != 8) {
        throw invalid_argument("必须是8个点");
    }

    // 使用聚类分成两组
    vector<vector<Point2f>> groups = clusterPointsKMeans(points);

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
    vector<Point2f> group1Ordered = sortCrossShapeAngle(groups[0]);
    vector<Point2f> group2Ordered = sortCrossShapeAngle(groups[1]);

    // 合并结果
    vector<Point2f> finalOrdered;
    finalOrdered.insert(finalOrdered.end(), group1Ordered.begin(), group1Ordered.end());
    finalOrdered.insert(finalOrdered.end(), group2Ordered.begin(), group2Ordered.end());

    return finalOrdered;
}

// 组间对齐函数
vector<Point2f> alignGroupStartPoint(const vector<Point2f>& refPts, const vector<Point2f>& targetPts) {
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

// 实现：检测反光球函数（更新版）
vector<Point2f> detectReflectiveBalls(const Mat& image) {
    Mat gray, bin;
    // 转灰度图（便于亮度分析）
    cvtColor(image, gray, COLOR_BGR2GRAY);

    // 高斯模糊减少噪声
    GaussianBlur(gray, gray, Size(5, 5), 0);

    // 二值化：提取亮度高于250的区域（发光球区域）
    threshold(gray, bin, 250, 255, THRESH_BINARY);

    // 提取轮廓（外边界）
    vector<vector<Point>> contours;
    findContours(bin, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    vector<Point2f> centers;
    for (const auto& contour : contours) {
        Point2f center;
        float radius;
        // 拟合最小外接圆，获取中心点和半径
        minEnclosingCircle(contour, center, radius);
        // 排除过小或过大的轮廓
        if (radius > 3 && radius < 50 && contourArea(contour) > 10) {
            centers.push_back(center);
        }
    }

    // 如果检测到8个点，使用新的排序算法
    if (centers.size() == 8) {
        try {
            return getOrdered8Points(centers);
        }
        catch (const exception& e) {
            // 如果新算法失败，回退到原来的排序方式
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

// 双目匹配函数：对右图点进行重新排序以匹配左图
vector<Point2f> matchRightImagePoints(const vector<Point2f>& leftPoints, const vector<Point2f>& rightPoints) {
    if (leftPoints.size() != 8 || rightPoints.size() != 8) {
        throw invalid_argument("左右图像都必须包含8个点");
    }

    try {
        // 获取左图的有序点
        vector<Point2f> leftOrdered = getOrdered8Points(leftPoints);
        vector<Point2f> rightOrderedRaw = getOrdered8Points(rightPoints);

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
        cout << "双目匹配失败，使用原始排序: " << e.what() << endl;
        return rightPoints;  // 返回原始排序
    }
}
//// 球识别模块实现
//#include "detector.h"
//
//using namespace cv;
//using namespace std;
//
//// 实现：检测反光球函数
//vector<Point2f> detectReflectiveBalls(const Mat& image) {
//    Mat gray, bin;
//    // 转灰度图（便于亮度分析）
//    cvtColor(image, gray, COLOR_BGR2GRAY);
//
//    //// 二值化：提取亮度高于220的区域（发光球区域）
//    //threshold(gray, bin, 220, 255, THRESH_BINARY);
//
//    // 二值化：提取亮度高于300的区域（发光球区域）
//    threshold(gray, bin, 250, 255, THRESH_BINARY);
//
//    // 中值滤波，去除椒盐噪声
//    medianBlur(bin, bin, 5);
//
//    // 提取轮廓（外边界）
//    vector<vector<Point>> contours;
//    findContours(bin, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//
//    vector<Point2f> centers;
//
//    for (const auto& contour : contours) {
//        Point2f center;
//        float radius;
//        // 拟合最小外接圆，获取中心点和半径
//        minEnclosingCircle(contour, center, radius);
//
//        // 排除过小或过大的轮廓
//        if (radius > 3 && radius < 50 && contourArea(contour) > 10) {
//            centers.push_back(center);
//        }
//    }
//
//    // 按x坐标从左到右排序，便于双目图像匹配
//    //sort(centers.begin(), centers.end(), [](const Point2f& a, const Point2f& b) {
//    //    return a.x < b.x;
//    //    });
//
//    // 左上 → 右下排序逻辑
//    std::sort(centers.begin(), centers.end(), [](const Point2f& a, const Point2f& b) {
//        const float row_thresh = 30.0f;  // 行间判断阈值（像素，根据小球大小调整）
//        if (fabs(a.y - b.y) < row_thresh) {
//            return a.x < b.x; // 同一行按 x 从小到大
//        }
//        else {
//            return a.y < b.y; // 不同行按 y 从小到大
//        }
//        });
//
//
//    return centers;
//}
