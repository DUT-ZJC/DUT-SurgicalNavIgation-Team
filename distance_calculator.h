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
    // 计算两个3D点之间的欧几里得距离
    static double calculateEuclideanDistance(const Point3f& p1, const Point3f& p2);

    // 从2D点计算3D坐标
    static Point3f calculate3DPoint(const Point2f& pt_left, const Point2f& pt_right,
        double focal_length_px, double baseline_mm,
        double cx, double cy);

    // 计算所有相邻点之间的距离 (点1-点2, 点2-点3, ..., 点7-点8)
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

private:
    // 辅助函数：检查视差是否有效
    static bool isValidDisparity(double disparity, double threshold = 1e-6);

    // 辅助函数：格式化输出距离信息
    static void writeDistanceInfo(ofstream& file, int point1, int point2,
        const Point3f& p1, const Point3f& p2,
        double distance);
};

#endif // DISTANCE_CALCULATOR_H