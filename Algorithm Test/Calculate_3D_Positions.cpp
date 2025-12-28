#include "Calculate_3D_Positions.hpp"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <iomanip>

bool Calculate3DCoordinates(
    const std::vector<cv::Point2f>& leftPoints,
    const std::vector<cv::Point2f>& rightPoints,
    const cv::Mat& cameraMatrix1,
    const cv::Mat& cameraMatrix2,
    double baseline_mm,
    std::vector<cv::Point3f>& points3D,
    std::vector<double>* disparities)
{
    points3D.clear();
    if (disparities != nullptr) {
        disparities->clear();
    }

    if (leftPoints.empty() || rightPoints.empty()) {
        std::cerr << "Error: Input data is empty!" << std::endl;
        return false;
    }

    if (leftPoints.size() != rightPoints.size()) {
        std::cerr << "Error: Points size mismatch! Left=" << leftPoints.size()
            << ", Right=" << rightPoints.size() << std::endl;
        return false;
    }

    // 提取左相机内参
    double fx_left = cameraMatrix1.at<double>(0, 0);
    double fy_left = cameraMatrix1.at<double>(1, 1);
    double cx_left = cameraMatrix1.at<double>(0, 2);
    double cy_left = cameraMatrix1.at<double>(1, 2);

    int validPoints = 0;
    int invalidPoints = 0;

    for (size_t i = 0; i < leftPoints.size(); i++) {
        const cv::Point2f& leftPt = leftPoints[i];
        const cv::Point2f& rightPt = rightPoints[i];

        // 计算视差 (disparity = x_left - x_right)
        double disparity = leftPt.x - rightPt.x;

        if (!ValidateDisparity(disparity)) {
            std::cout << "Point[" << i << "]: Invalid disparity " << disparity
                << " pixels (skipped)" << std::endl;
            points3D.push_back(cv::Point3f(0, 0, 0));
            if (disparities != nullptr) {
                disparities->push_back(0.0);
            }
            invalidPoints++;
            continue;
        }

        // 计算深度 Z = (fx * baseline) / disparity
        double Z = (fx_left * baseline_mm) / disparity;

        // 计算X和Y坐标
        // X = (u - cx) * Z / fx
        // Y = (v - cy) * Z / fy
        double X = (leftPt.x - cx_left) * Z / fx_left;
        double Y = (leftPt.y - cy_left) * Z / fy_left;

        cv::Point3f point3D(X, Y, Z);

        if (!Validate3DPoint(point3D)) {
            std::cout << "Point[" << i << "]: Invalid 3D point ("
                << X << ", " << Y << ", " << Z << ") mm (skipped)" << std::endl;
            points3D.push_back(cv::Point3f(0, 0, 0));
            if (disparities != nullptr) {
                disparities->push_back(0.0);
            }
            invalidPoints++;
            continue;
        }

        points3D.push_back(point3D);
        if (disparities != nullptr) {
            disparities->push_back(disparity);
        }
        validPoints++;

        // 计算3D距离
        double distance = std::sqrt(X * X + Y * Y + Z * Z);

        std::cout << "Point[" << i << "]: "
            << "Left(" << std::fixed << std::setprecision(2) << leftPt.x << ", " << leftPt.y << "), "
            << "Right(" << rightPt.x << ", " << rightPt.y << ") "
            << "-> Disparity: " << disparity << " px "
            << "-> 3D(" << X << ", " << Y << ", " << Z << ") mm, "
            << "Distance: " << distance << " mm" << std::endl;
    }

    std::cout << "\n=== 3D Calculation Summary ===" << std::endl;
    std::cout << "Total input points: " << leftPoints.size() << std::endl;
    std::cout << "Valid 3D points: " << validPoints << std::endl;
    std::cout << "Invalid 3D points: " << invalidPoints << std::endl;
    std::cout << "====================================\n" << std::endl;

    return validPoints > 0;
}

void Print3DPoints(const std::vector<cv::Point3f>& points3D, bool verbose)
{
    std::cout << "\n=== 3D Points (Camera Coordinate System) ===" << std::endl;
    std::cout << std::fixed << std::setprecision(2);

    int validCount = 0;

    for (size_t i = 0; i < points3D.size(); i++) {
        const cv::Point3f& pt = points3D[i];

        // 跳过无效点
        if (pt.x == 0 && pt.y == 0 && pt.z == 0) {
            if (verbose) {
                std::cout << "Point[" << i << "]: Invalid (skipped)" << std::endl;
            }
            continue;
        }

        validCount++;

        // 计算到相机的距离
        double distance = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);

        std::cout << "Point[" << i << "]: "
            << "X=" << std::setw(10) << pt.x << " mm, "
            << "Y=" << std::setw(10) << pt.y << " mm, "
            << "Z=" << std::setw(10) << pt.z << " mm"
            << "  (Distance: " << std::setw(10) << distance << " mm)" << std::endl;
    }

    std::cout << "\nTotal valid 3D points: " << validCount << " / " << points3D.size() << std::endl;
    std::cout << "============================================\n" << std::endl;
}

void PrintDisparityStatistics(const std::vector<double>& disparities)
{
    if (disparities.empty()) {
        std::cout << "No disparities to analyze" << std::endl;
        return;
    }

    // 过滤掉无效的视差（0值）
    std::vector<double> validDisparities;
    for (double d : disparities) {
        if (d > 0) {
            validDisparities.push_back(d);
        }
    }

    if (validDisparities.empty()) {
        std::cout << "No valid disparities to analyze" << std::endl;
        return;
    }

    double sum = std::accumulate(validDisparities.begin(), validDisparities.end(), 0.0);
    double mean = sum / validDisparities.size();

    double sq_sum = std::inner_product(validDisparities.begin(), validDisparities.end(),
        validDisparities.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / validDisparities.size() - mean * mean);

    double minDisp = *std::min_element(validDisparities.begin(), validDisparities.end());
    double maxDisp = *std::max_element(validDisparities.begin(), validDisparities.end());

    std::cout << "\n=== Disparity Statistics ===" << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Total points: " << disparities.size() << std::endl;
    std::cout << "Valid disparities: " << validDisparities.size() << std::endl;
    std::cout << "Mean disparity: " << mean << " pixels" << std::endl;
    std::cout << "Std deviation: " << stdev << " pixels" << std::endl;
    std::cout << "Min disparity: " << minDisp << " pixels" << std::endl;
    std::cout << "Max disparity: " << maxDisp << " pixels" << std::endl;
    std::cout << "=============================\n" << std::endl;
}

void PrintAccuracyEstimate(
    const std::vector<double>& disparities,
    const std::vector<cv::Point3f>& points3D,
    const cv::Mat& cameraMatrix1,
    double baseline_mm)
{
    if (disparities.empty() || points3D.empty()) {
        std::cout << "No data for accuracy estimation" << std::endl;
        return;
    }

    double fx_left = cameraMatrix1.at<double>(0, 0);

    std::cout << "\n=== 3D Accuracy Estimation ===" << std::endl;
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Assumptions:" << std::endl;
    std::cout << "  - Pixel size: 0.05 mm/pixel" << std::endl;
    std::cout << "  - Epipolar error: 2-5 pixels (typical)" << std::endl;
    std::cout << "  - Formula: ΔZ = Z² / (f × b) × Δd" << std::endl;
    std::cout << std::endl;

    // 收集有效的深度值
    std::vector<double> validDepths;
    for (const auto& pt : points3D) {
        if (pt.z > 0) {
            validDepths.push_back(pt.z);
        }
    }

    if (validDepths.empty()) {
        std::cout << "No valid depths for analysis" << std::endl;
        return;
    }

    double minDepth = *std::min_element(validDepths.begin(), validDepths.end());
    double maxDepth = *std::max_element(validDepths.begin(), validDepths.end());
    double avgDepth = std::accumulate(validDepths.begin(), validDepths.end(), 0.0) / validDepths.size();

    std::cout << "Depth statistics:" << std::endl;
    std::cout << "  Min depth: " << minDepth << " mm" << std::endl;
    std::cout << "  Max depth: " << maxDepth << " mm" << std::endl;
    std::cout << "  Avg depth: " << avgDepth << " mm" << std::endl;
    std::cout << std::endl;

    // 对几个典型深度估计误差
    std::vector<double> testDepths = { 500.0, 1000.0, 1500.0, 2000.0 };
    std::vector<double> testErrors = { 2.0, 5.0 };  // 像素

    std::cout << "Expected 3D errors at different depths:" << std::endl;
    std::cout << "Depth(mm) | Epipolar(px) | Depth Error(mm) | XY Error(mm)" << std::endl;
    std::cout << "----------|--------------|-----------------|-------------" << std::endl;

    double pixel_size_mm = 0.05;
    double f_mm = fx_left * pixel_size_mm;

    for (double depth : testDepths) {
        for (double err_px : testErrors) {
            double depth_error = EstimateDepthError(depth, err_px, fx_left, baseline_mm);
            double xy_error = (depth / f_mm) * (err_px * pixel_size_mm);

            std::cout << std::setw(9) << depth << " | "
                << std::setw(12) << err_px << " | "
                << std::setw(15) << depth_error << " | "
                << std::setw(11) << xy_error << std::endl;
        }
    }

    std::cout << "\nNote: Actual errors depend on calibration quality and matching accuracy." << std::endl;
    std::cout << "======================================\n" << std::endl;
}

bool ValidateDisparity(double disparity)
{
    // 视差应该为正值（左图点在右图点的右侧）
    if (disparity <= 0) {
        return false;
    }

    // 视差不应该过大（通常小于图像宽度的1/2）
    if (disparity > 1536) {
        return false;
    }

    // 视差不应该过小（避免无穷远点）
    if (disparity < 0.1) {
        return false;
    }

    return true;
}

bool Validate3DPoint(const cv::Point3f& point3D)
{
    // 深度应该为正值
    if (point3D.z <= 0) {
        return false;
    }

    // 深度范围检查（根据实际应用调整，这里设置为100mm到10m）
    if (point3D.z < 100 || point3D.z > 10000) {
        return false;
    }

    // X和Y坐标不应该过大（根据深度合理性检查）
    if (std::abs(point3D.x) > 5000 || std::abs(point3D.y) > 5000) {
        return false;
    }

    // 检查是否为NaN或Inf
    if (std::isnan(point3D.x) || std::isnan(point3D.y) || std::isnan(point3D.z) ||
        std::isinf(point3D.x) || std::isinf(point3D.y) || std::isinf(point3D.z)) {
        return false;
    }

    return true;
}

double EstimateDepthError(
    double depth_mm,
    double disparity_error,
    double focal_length_px,
    double baseline_mm)
{
    // ΔZ = Z² / (f × b) × Δd
    return (depth_mm * depth_mm) / (focal_length_px * baseline_mm) * disparity_error;
}