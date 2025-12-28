#ifndef DISTANCE_CALCULATOR_H
#define DISTANCE_CALCULATOR_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace std;
using namespace cv;

class DistanceCalculator {
public:
    // ===================== 核心计算函数 =====================

    // 计算归一化的 XY 坐标 (从像素坐标直接转换，与深度无关)
    static Point2f calculateNormalizedXY(const Point2f& pt_pixel,
        double focal_length_px, double cx, double cy);

    // 计算深度 Z 坐标 (基于双目视差)
    static double calculateDepthZ(const Point2f& pt_left, const Point2f& pt_right,
        double focal_length_px, double baseline_mm);

    // 组合函数：计算完整的3D坐标 (整合XY和Z的计算)
    static Point3f calculate3DPoint(const Point2f& pt_left, const Point2f& pt_right,
        double focal_length_px, double baseline_mm,
        double cx, double cy);

    // 计算两个3D点之间的欧几里得距离
    static double calculateEuclideanDistance(const Point3f& p1, const Point3f& p2);

    // ===================== 输出和显示函数 =====================

    // 打印所有检测点的序号和XYZ坐标
    static void printPointsCoordinates(const vector<Point2f>& pts_left,
        const vector<Point2f>& pts_right,
        double focal_length_px, double baseline_mm,
        double cx, double cy,
        const string& output_file = "");

    // 打印单个点的详细信息 (包括坐标系转换过程)
    static void printPointDetails(int point_index, const Point2f& pt_left, const Point2f& pt_right,
        double focal_length_px, double baseline_mm,
        double cx, double cy);

    // ===================== 距离计算函数 =====================

    // 计算所有相邻点之间的距离 (点1-点2, 点2-点3, ..., 点n-1到点n)
    static void calculateAdjacentDistances(const vector<Point2f>& pts_left,
        const vector<Point2f>& pts_right,
        double focal_length_px, double baseline_mm,
        double cx, double cy,
        const string& output_file);

    // 计算特定点对之间的距离
    static void calculateSpecificDistances(const vector<Point2f>& pts_left,
        const vector<Point2f>& pts_right,
        double focal_length_px, double baseline_mm,
        double cx, double cy,
        const string& output_file);

    // 计算所有点对之间的距离矩阵
    static void calculateAllDistances(const vector<Point2f>& pts_left,
        const vector<Point2f>& pts_right,
        double focal_length_px, double baseline_mm,
        double cx, double cy,
        const string& output_file);

    // ===================== 验证和工具函数 =====================

    // 检查点对是否有效 (检查视差、边界等)
    static bool isValidPointPair(const Point2f& pt_left, const Point2f& pt_right,
        double min_disparity = 1e-6);

    // 验证相机参数的合理性
    static bool validateCameraParameters(double focal_length_px, double baseline_mm,
        double cx, double cy);

private:
    // ===================== 辅助函数 =====================

    // 检查视差是否有效
    static bool isValidDisparity(double disparity, double threshold = 1e-6);

    // 格式化输出距离信息到文件
    static void writeDistanceInfo(ofstream& file, int point1, int point2,
        const Point3f& p1, const Point3f& p2,
        double distance);

    // 写入文件头部信息
    static void writeFileHeader(ofstream& file, const string& title,
        double focal_length_px, double baseline_mm,
        double cx, double cy, size_t point_count);

    // 格式化输出3D坐标
    static string formatPoint3D(const Point3f& point, int precision = 2);

    // 格式化输出2D坐标
    static string formatPoint2D(const Point2f& point, int precision = 2);
};

#endif // DISTANCE_CALCULATOR_H