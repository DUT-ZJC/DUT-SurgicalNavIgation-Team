////__________________________________________旧代码测试
////// camera_params.h
////#ifndef CAMERA_PARAMS_H
////#define CAMERA_PARAMS_H
////
////// 相机内参
////const double focal_length_px = 5000.0;  // 像素单位焦距
////const double baseline_m = 0.500;        // 左右相机间距（米）
////const double cx = 1536.0;               // 图像中心（宽度方向）
////const double cy = 1024.0;               // 图像中心（高度方向）
////
////#endif // CAMERA_PARAMS_H
////#pragma once
////___________________________________________旧代码测试
//
//// camera_params.h
//#ifndef CAMERA_PARAMS_H
//#define CAMERA_PARAMS_H
//
//#include <opencv2/opencv.hpp>
//
//// ==================== 双目相机内参 (单位: mm和像素) ====================
//// 左相机内参
//const double fx_left = 5000.0;          // 左相机x方向焦距 (像素)
//const double fy_left = 5000.0;          // 左相机y方向焦距 (像素)
//const double cx_left = 1536.0;          // 左相机光心x坐标 (像素)
//const double cy_left = 1024.0;          // 左相机光心y坐标 (像素)
//
//// 右相机内参
//const double fx_right = 5000.0;         // 右相机x方向焦距 (像素)
//const double fy_right = 5000.0;         // 右相机y方向焦距 (像素)
//const double cx_right = 1536.0;         // 右相机光心x坐标 (像素)
//const double cy_right = 1024.0;         // 右相机光心y坐标 (像素)
//
//// 左相机畸变系数 [k1, k2, p1, p2, k3]
//const double k1_left = -0.2;            // 径向畸变系数1
//const double k2_left = 0.1;             // 径向畸变系数2
//const double p1_left = 0.001;           // 切向畸变系数1
//const double p2_left = 0.001;           // 切向畸变系数2
//const double k3_left = 0.0;             // 径向畸变系数3
//
//// 右相机畸变系数 [k1, k2, p1, p2, k3]
//const double k1_right = -0.2;           // 径向畸变系数1
//const double k2_right = 0.1;            // 径向畸变系数2
//const double p1_right = 0.001;          // 切向畸变系数1
//const double p2_right = 0.001;          // 切向畸变系数2
//const double k3_right = 0.0;            // 径向畸变系数3
//
//// ==================== 双目相机外参 (单位: mm和度) ====================
//// 基线距离
//const double baseline_mm = 500.0;       // 左右相机间距 (mm)
//
//// 旋转矩阵 (右相机相对于左相机的旋转)
//// 理想情况下为单位矩阵 (平行双目)
//const double R_data[9] = {
//    1.0, 0.0, 0.0,
//    0.0, 1.0, 0.0,
//    0.0, 0.0, 1.0
//};
//
//// 平移向量 (右相机相对于左相机的平移, mm)
//const double T_data[3] = {
//    -baseline_mm,    // x方向平移 (负值表示右相机在左相机左侧)
//    0.0,             // y方向平移
//    0.0              // z方向平移
//};
//
//// ==================== 物理参数 ====================
//// 像素尺寸
//const double pixel_size_mm = 0.0034;    // 像素物理尺寸 (mm/pixel)
//
//// 物理焦距
//const double focal_length_mm = fx_left * pixel_size_mm;  // 物理焦距 (mm)
//
//// 传感器尺寸
//const double sensor_width_mm = 3072 * pixel_size_mm;    // 传感器宽度 (mm)
//const double sensor_height_mm = 2048 * pixel_size_mm;   // 传感器高度 (mm)
//
//// ==================== OpenCV矩阵格式 ====================
//// 左相机内参矩阵
//const cv::Mat camera_matrix_left = (cv::Mat_<double>(3, 3) <<
//    fx_left, 0, cx_left,
//    0, fy_left, cy_left,
//    0, 0, 1);
//
//// 右相机内参矩阵
//const cv::Mat camera_matrix_right = (cv::Mat_<double>(3, 3) <<
//    fx_right, 0, cx_right,
//    0, fy_right, cy_right,
//    0, 0, 1);
//
//// 左相机畸变系数向量
//const cv::Mat dist_coeffs_left = (cv::Mat_<double>(5, 1) <<
//    k1_left, k2_left, p1_left, p2_left, k3_left);
//
//// 右相机畸变系数向量
//const cv::Mat dist_coeffs_right = (cv::Mat_<double>(5, 1) <<
//    k1_right, k2_right, p1_right, p2_right, k3_right);
//
//// 旋转矩阵
//const cv::Mat R = (cv::Mat_<double>(3, 3) <<
//    R_data[0], R_data[1], R_data[2],
//    R_data[3], R_data[4], R_data[5],
//    R_data[6], R_data[7], R_data[8]);
//
//// 平移向量
//const cv::Mat T = (cv::Mat_<double>(3, 1) <<
//    T_data[0], T_data[1], T_data[2]);
//
//// ==================== 兼容性参数 (向后兼容) ====================
//// 为了兼容旧代码，保留原有参数名
//const double focal_length_px = fx_left;  // 像素单位焦距
//const double baseline_m = baseline_mm / 1000.0;  // 转换为米
//const double cx = cx_left;               // 图像中心（宽度方向）
//const double cy = cy_left;               // 图像中心（高度方向）
//
//// ==================== 辅助函数声明 ====================
//// 打印相机参数
//void printCameraParams();
//
//// 验证相机参数合理性
//bool validateCameraParams();
//
//// 计算重投影误差
//double calculateReprojectionError(const std::vector<cv::Point3f>& object_points,
//    const std::vector<cv::Point2f>& image_points,
//    const cv::Mat& camera_matrix,
//    const cv::Mat& dist_coeffs,
//    const cv::Mat& rvec,
//    const cv::Mat& tvec);
//
//#endif // CAMERA_PARAMS_H