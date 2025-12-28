//#include "camera_params.h"
//#include <iostream>
//#include <cmath>
//
//// 假设这些是全局变量，如果不是请根据实际情况调整
//extern cv::Mat camera_matrix;
//extern cv::Mat dist_coeffs;
//
//// ==================== 辅助函数实现 ====================
//
//// 打印相机参数
//void printCameraParams() {
//    std::cout << "\n========== 相机参数 ==========\n";
//    std::cout << "相机内参矩阵 (Camera Matrix):\n";
//    std::cout << "fx = " << camera_matrix.at<double>(0, 0) << "\n";
//    std::cout << "fy = " << camera_matrix.at<double>(1, 1) << "\n";
//    std::cout << "cx = " << camera_matrix.at<double>(0, 2) << "\n";
//    std::cout << "cy = " << camera_matrix.at<double>(1, 2) << "\n";
//
//    std::cout << "\n畸变系数 (Distortion Coefficients):\n";
//    std::cout << "k1 = " << dist_coeffs.at<double>(0, 0) << "\n";
//    std::cout << "k2 = " << dist_coeffs.at<double>(0, 1) << "\n";
//    std::cout << "p1 = " << dist_coeffs.at<double>(0, 2) << "\n";
//    std::cout << "p2 = " << dist_coeffs.at<double>(0, 3) << "\n";
//    std::cout << "k3 = " << dist_coeffs.at<double>(0, 4) << "\n";
//    std::cout << "==============================\n\n";
//}
//
//// 验证相机参数合理性
//bool validateCameraParams() {
//    // 检查相机内参矩阵是否有效
//    if (camera_matrix.empty() || camera_matrix.rows != 3 || camera_matrix.cols != 3) {
//        std::cerr << "错误: 相机内参矩阵无效\n";
//        return false;
//    }
//
//    // 检查畸变系数是否有效
//    if (dist_coeffs.empty() || dist_coeffs.rows != 1 || dist_coeffs.cols != 5) {
//        std::cerr << "错误: 畸变系数无效\n";
//        return false;
//    }
//
//    // 检查焦距是否合理 (应该为正数)
//    double fx = camera_matrix.at<double>(0, 0);
//    double fy = camera_matrix.at<double>(1, 1);
//    if (fx <= 0 || fy <= 0) {
//        std::cerr << "错误: 焦距参数无效 (fx=" << fx << ", fy=" << fy << ")\n";
//        return false;
//    }
//
//    // 检查主点坐标是否合理
//    double cx = camera_matrix.at<double>(0, 2);
//    double cy = camera_matrix.at<double>(1, 2);
//    if (cx < 0 || cy < 0 || cx > 2000 || cy > 2000) {
//        std::cerr << "警告: 主点坐标可能异常 (cx=" << cx << ", cy=" << cy << ")\n";
//    }
//
//    // 检查畸变系数是否在合理范围内
//    double k1 = dist_coeffs.at<double>(0, 0);
//    double k2 = dist_coeffs.at<double>(0, 1);
//    if (std::abs(k1) > 1.0 || std::abs(k2) > 1.0) {
//        std::cerr << "警告: 畸变系数可能过大 (k1=" << k1 << ", k2=" << k2 << ")\n";
//    }
//
//    std::cout << "相机参数验证通过\n";
//    return true;
//}
//
//// 计算重投影误差
//double calculateReprojectionError(const std::vector<cv::Point3f>& object_points,
//    const std::vector<cv::Point2f>& image_points,
//    const cv::Mat& camera_matrix,
//    const cv::Mat& dist_coeffs,
//    const cv::Mat& rvec,
//    const cv::Mat& tvec) {
//
//    // 参数有效性检查
//    if (object_points.empty() || image_points.empty()) {
//        std::cerr << "错误: 点集为空\n";
//        return -1.0;
//    }
//
//    if (object_points.size() != image_points.size()) {
//        std::cerr << "错误: 3D点和2D点数量不匹配\n";
//        return -1.0;
//    }
//
//    // 将3D点重投影到2D
//    std::vector<cv::Point2f> projected_points;
//    cv::projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs, projected_points);
//
//    // 计算重投影误差
//    double total_error = 0.0;
//    for (size_t i = 0; i < image_points.size(); ++i) {
//        double dx = image_points[i].x - projected_points[i].x;
//        double dy = image_points[i].y - projected_points[i].y;
//        double error = std::sqrt(dx * dx + dy * dy);
//        total_error += error;
//    }
//
//    // 返回平均误差
//    double mean_error = total_error / image_points.size();
//
//    std::cout << "重投影误差统计:\n";
//    std::cout << "总误差: " << total_error << "\n";
//    std::cout << "平均误差: " << mean_error << " 像素\n";
//    std::cout << "点对数量: " << image_points.size() << "\n";
//
//    return mean_error;
//}