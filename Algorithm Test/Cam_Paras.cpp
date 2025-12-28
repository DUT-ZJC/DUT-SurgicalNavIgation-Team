#include "Cam_Paras.hpp"
#include <iostream>
#include <cmath>
#include <fstream>

// ==================== 构造函数 ====================

CameraParams::CameraParams()
    : m_baseline_mm(500.5842)
    , m_pixel_size_mm(0.0024)
    , m_sensor_width_mm(0.0)
    , m_sensor_height_mm(0.0)
    , m_initialized(false)
{
    InitializeDefaultParams();
}

CameraParams::CameraParams(const cv::Mat& camera_matrix_left, const cv::Mat& camera_matrix_right,
    const cv::Mat& dist_coeffs_left, const cv::Mat& dist_coeffs_right,
    const cv::Mat& R, const cv::Mat& T)
    : m_baseline_mm(500.5842)
    , m_pixel_size_mm(0.0024)
    , m_sensor_width_mm(0.0)
    , m_sensor_height_mm(0.0)
    , m_initialized(false)
{
    m_camera_matrix_left = camera_matrix_left.clone();
    m_camera_matrix_right = camera_matrix_right.clone();
    m_dist_coeffs_left = dist_coeffs_left.clone();
    m_dist_coeffs_right = dist_coeffs_right.clone();
    m_R = R.clone();
    m_T = T.clone();

    // 从T向量计算基线距离
    if (!T.empty() && T.rows >= 3) {
        m_baseline_mm = std::abs(T.at<double>(0, 0));
    }

    m_initialized = ValidateParams();
}

// ==================== 参数设置接口 ====================

void CameraParams::SetLeftCameraMatrix(double fx, double fy, double cx, double cy)
{
    m_camera_matrix_left = (cv::Mat_<double>(3, 3) <<
        fx, 0, cx,
        0, fy, cy,
        0, 0, 1);
}

void CameraParams::SetLeftCameraMatrix(const cv::Mat& camera_matrix)
{
    m_camera_matrix_left = camera_matrix.clone();
}

void CameraParams::SetRightCameraMatrix(double fx, double fy, double cx, double cy)
{
    m_camera_matrix_right = (cv::Mat_<double>(3, 3) <<
        fx, 0, cx,
        0, fy, cy,
        0, 0, 1);
}

void CameraParams::SetRightCameraMatrix(const cv::Mat& camera_matrix)
{
    m_camera_matrix_right = camera_matrix.clone();
}

void CameraParams::SetLeftDistortionCoeffs(double k1, double k2, double p1, double p2, double k3)
{
    m_dist_coeffs_left = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);
}

void CameraParams::SetLeftDistortionCoeffs(const cv::Mat& dist_coeffs)
{
    m_dist_coeffs_left = dist_coeffs.clone();
}

void CameraParams::SetRightDistortionCoeffs(double k1, double k2, double p1, double p2, double k3)
{
    m_dist_coeffs_right = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);
}

void CameraParams::SetRightDistortionCoeffs(const cv::Mat& dist_coeffs)
{
    m_dist_coeffs_right = dist_coeffs.clone();
}

void CameraParams::SetStereoExtrinsics(const cv::Mat& R, const cv::Mat& T)
{
    m_R = R.clone();
    m_T = T.clone();

    // 更新基线距离
    if (!T.empty() && T.rows >= 3) {
        m_baseline_mm = std::abs(T.at<double>(0, 0));
    }
}

void CameraParams::SetBaseline(double baseline_mm)
{
    m_baseline_mm = baseline_mm;

    // 更新平移向量
    m_T = (cv::Mat_<double>(3, 1) << -baseline_mm, 0.0, 0.0);
}

void CameraParams::SetPixelSize(double pixel_size_mm)
{
    m_pixel_size_mm = pixel_size_mm;
}

void CameraParams::SetSensorSize(double width_mm, double height_mm)
{
    m_sensor_width_mm = width_mm;
    m_sensor_height_mm = height_mm;
}

// ==================== 功能接口实现 ====================

double CameraParams::GetFocalLengthMM() const
{
    return GetLeftFx() * m_pixel_size_mm;
}

void CameraParams::PrintParams() const
{
    std::cout << "\n==================== 双目相机参数 ====================\n";
    PrintLeftCameraParams();
    PrintRightCameraParams();
    PrintStereoParams();
    std::cout << "=====================================================\n\n";
}

void CameraParams::PrintLeftCameraParams() const
{
    std::cout << "\n--- 左相机参数 ---\n";
    PrintCameraMatrix(m_camera_matrix_left, "左相机");
    PrintDistortionCoeffs(m_dist_coeffs_left, "左相机");
}

void CameraParams::PrintRightCameraParams() const
{
    std::cout << "\n--- 右相机参数 ---\n";
    PrintCameraMatrix(m_camera_matrix_right, "右相机");
    PrintDistortionCoeffs(m_dist_coeffs_right, "右相机");
}

void CameraParams::PrintStereoParams() const
{
    std::cout << "\n--- 双目外参 ---\n";
    std::cout << "基线距离: " << m_baseline_mm << " mm\n";

    std::cout << "旋转矩阵 R:\n";
    for (int i = 0; i < 3; ++i) {
        std::cout << "  ";
        for (int j = 0; j < 3; ++j) {
            std::cout << m_R.at<double>(i, j) << "\t";
        }
        std::cout << "\n";
    }

    std::cout << "平移向量 T: ["
        << m_T.at<double>(0, 0) << ", "
        << m_T.at<double>(1, 0) << ", "
        << m_T.at<double>(2, 0) << "] mm\n";

    std::cout << "\n--- 物理参数 ---\n";
    std::cout << "像素尺寸: " << m_pixel_size_mm << " mm/pixel\n";
    std::cout << "物理焦距: " << GetFocalLengthMM() << " mm\n";
    std::cout << "传感器尺寸: " << m_sensor_width_mm << " x " << m_sensor_height_mm << " mm\n";
}

bool CameraParams::ValidateParams() const
{
    bool valid = true;

    if (!ValidateLeftCamera()) {
        std::cerr << "左相机参数验证失败\n";
        valid = false;
    }

    if (!ValidateRightCamera()) {
        std::cerr << "右相机参数验证失败\n";
        valid = false;
    }

    if (!ValidateStereoParams()) {
        std::cerr << "双目外参验证失败\n";
        valid = false;
    }

    if (valid) {
        std::cout << "所有相机参数验证通过\n";
    }

    return valid;
}

bool CameraParams::ValidateLeftCamera() const
{
    return ValidateCameraMatrix(m_camera_matrix_left, "左相机") &&
        ValidateDistortionCoeffs(m_dist_coeffs_left, "左相机");
}

bool CameraParams::ValidateRightCamera() const
{
    return ValidateCameraMatrix(m_camera_matrix_right, "右相机") &&
        ValidateDistortionCoeffs(m_dist_coeffs_right, "右相机");
}

bool CameraParams::ValidateStereoParams() const
{
    // 检查旋转矩阵
    if (m_R.empty() || m_R.rows != 3 || m_R.cols != 3) {
        std::cerr << "错误: 旋转矩阵尺寸无效\n";
        return false;
    }

    // 检查平移向量
    if (m_T.empty() || m_T.rows != 3 || m_T.cols != 1) {
        std::cerr << "错误: 平移向量尺寸无效\n";
        return false;
    }

    // 检查基线距离
    if (m_baseline_mm <= 0) {
        std::cerr << "错误: 基线距离无效 (" << m_baseline_mm << " mm)\n";
        return false;
    }

    return true;
}

double CameraParams::CalculateReprojectionError(const std::vector<cv::Point3f>& object_points,
    const std::vector<cv::Point2f>& image_points,
    const cv::Mat& rvec,
    const cv::Mat& tvec,
    bool use_left_camera) const
{
    // 参数有效性检查
    if (object_points.empty() || image_points.empty()) {
        std::cerr << "错误: 点集为空\n";
        return -1.0;
    }

    if (object_points.size() != image_points.size()) {
        std::cerr << "错误: 3D点和2D点数量不匹配\n";
        return -1.0;
    }

    // 选择使用的相机参数
    const cv::Mat& camera_matrix = use_left_camera ? m_camera_matrix_left : m_camera_matrix_right;
    const cv::Mat& dist_coeffs = use_left_camera ? m_dist_coeffs_left : m_dist_coeffs_right;
    const std::string camera_name = use_left_camera ? "左相机" : "右相机";

    // 将3D点重投影到2D
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs, projected_points);

    // 计算重投影误差
    double total_error = 0.0;
    for (size_t i = 0; i < image_points.size(); ++i) {
        double dx = image_points[i].x - projected_points[i].x;
        double dy = image_points[i].y - projected_points[i].y;
        double error = std::sqrt(dx * dx + dy * dy);
        total_error += error;
    }

    // 返回平均误差
    double mean_error = total_error / image_points.size();
    std::cout << camera_name << " 重投影误差统计:\n";
    std::cout << "总误差: " << total_error << "\n";
    std::cout << "平均误差: " << mean_error << " 像素\n";
    std::cout << "点对数量: " << image_points.size() << "\n";

    return mean_error;
}

bool CameraParams::SaveToFile(const std::string& filename) const
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "无法打开文件进行写入: " << filename << "\n";
        return false;
    }

    fs << "camera_matrix_left" << m_camera_matrix_left;
    fs << "camera_matrix_right" << m_camera_matrix_right;
    fs << "dist_coeffs_left" << m_dist_coeffs_left;
    fs << "dist_coeffs_right" << m_dist_coeffs_right;
    fs << "R" << m_R;
    fs << "T" << m_T;
    fs << "baseline_mm" << m_baseline_mm;
    fs << "pixel_size_mm" << m_pixel_size_mm;
    fs << "sensor_width_mm" << m_sensor_width_mm;
    fs << "sensor_height_mm" << m_sensor_height_mm;

    fs.release();
    std::cout << "相机参数已保存到: " << filename << "\n";
    return true;
}

bool CameraParams::LoadFromFile(const std::string& filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "无法打开文件进行读取: " << filename << "\n";
        return false;
    }

    fs["camera_matrix_left"] >> m_camera_matrix_left;
    fs["camera_matrix_right"] >> m_camera_matrix_right;
    fs["dist_coeffs_left"] >> m_dist_coeffs_left;
    fs["dist_coeffs_right"] >> m_dist_coeffs_right;
    fs["R"] >> m_R;
    fs["T"] >> m_T;
    fs["baseline_mm"] >> m_baseline_mm;
    fs["pixel_size_mm"] >> m_pixel_size_mm;
    fs["sensor_width_mm"] >> m_sensor_width_mm;
    fs["sensor_height_mm"] >> m_sensor_height_mm;

    fs.release();

    m_initialized = ValidateParams();
    if (m_initialized) {
        std::cout << "相机参数已从文件加载: " << filename << "\n";
    }
    else {
        std::cerr << "加载的相机参数验证失败\n";
    }

    return m_initialized;
}

void CameraParams::ResetToDefault()
{
    InitializeDefaultParams();
    m_initialized = true;
    std::cout << "相机参数已重置为默认值\n";
}

bool CameraParams::IsInitialized() const
{
    return m_initialized;
}

cv::Mat CameraParams::GetStereoRectificationParams() const
{
    // 返回立体校正参数，这里简化返回基本信息
    cv::Mat params = (cv::Mat_<double>(4, 1) <<
        GetLeftFx(), GetLeftFy(), GetLeftCx(), GetLeftCy());
    return params;
}

// ==================== 私有辅助函数实现 ====================

void CameraParams::InitializeDefaultParams()
{
    // 初始化默认内参
    SetLeftCameraMatrix(3171.99071321173, 3170.70689791314, 1555.86108945639, 1027.47517097725);
    SetRightCameraMatrix(3173.7686128581, 3175.26005023897, 1511.09922938155, 1015.21699355011);

    // 初始化默认畸变系数
    SetLeftDistortionCoeffs(-0.4188734, 0.259017759395356, 0.00105310972446147, -0.00124237165166607, -0.125590623130476);
    SetRightDistortionCoeffs(-0.413351201990888, 0.211833118277202, 0.000555966562103498, 0.000183216416669935, -0.0576098751427282);

    // 初始化默认外参
    m_R = (cv::Mat_<double>(3, 3) <<
        0.999944943459456, -0.0102713214765485, 0.00214709221752841,
        0.0102758340022601, 0.999944994305481, -0.00210133266507487,
        -0.00212539065189761, 0.00212328013618268, 0.999995487187837);

    m_T = (cv::Mat_<double>(3, 1) << -m_baseline_mm, -2.68116596404073, 2.94754468851408);

    // 设置传感器尺寸
    m_sensor_width_mm = 3072 * m_pixel_size_mm;
    m_sensor_height_mm = 2048 * m_pixel_size_mm;
}

bool CameraParams::ValidateCameraMatrix(const cv::Mat& camera_matrix, const std::string& camera_name) const
{
    // 检查矩阵尺寸
    if (camera_matrix.empty() || camera_matrix.rows != 3 || camera_matrix.cols != 3) {
        std::cerr << "错误: " << camera_name << "内参矩阵尺寸无效\n";
        return false;
    }

    // 检查焦距参数
    double fx = camera_matrix.at<double>(0, 0);
    double fy = camera_matrix.at<double>(1, 1);
    if (fx <= 0 || fy <= 0) {
        std::cerr << "错误: " << camera_name << "焦距参数无效 (fx=" << fx << ", fy=" << fy << ")\n";
        return false;
    }

    // 检查主点坐标
    double cx = camera_matrix.at<double>(0, 2);
    double cy = camera_matrix.at<double>(1, 2);
    if (cx < 0 || cy < 0 || cx > 4000 || cy > 4000) {
        std::cerr << "警告: " << camera_name << "主点坐标可能异常 (cx=" << cx << ", cy=" << cy << ")\n";
    }

    return true;
}

bool CameraParams::ValidateDistortionCoeffs(const cv::Mat& dist_coeffs, const std::string& camera_name) const
{
    // 检查矩阵尺寸
    if (dist_coeffs.empty() || dist_coeffs.rows != 1 || dist_coeffs.cols != 5) {
        std::cerr << "错误: " << camera_name << "畸变系数尺寸无效\n";
        return false;
    }

    // 检查畸变系数范围
    double k1 = dist_coeffs.at<double>(0, 0);
    double k2 = dist_coeffs.at<double>(0, 1);
    if (std::abs(k1) > 1.0 || std::abs(k2) > 1.0) {
        std::cerr << "警告: " << camera_name << "畸变系数可能过大 (k1=" << k1 << ", k2=" << k2 << ")\n";
    }

    return true;
}

void CameraParams::PrintCameraMatrix(const cv::Mat& camera_matrix, const std::string& camera_name) const
{
    if (camera_matrix.empty()) {
        std::cout << camera_name << " 内参矩阵: 未初始化\n";
        return;
    }

    std::cout << camera_name << " 内参矩阵: \n";
    std::cout << "  fx = " << camera_matrix.at<double>(0, 0) << "\n";
    std::cout << "  fy = " << camera_matrix.at<double>(1, 1) << "\n";
    std::cout << "  cx = " << camera_matrix.at<double>(0, 2) << "\n";
    std::cout << "  cy = " << camera_matrix.at<double>(1, 2) << "\n";
}

void CameraParams::PrintDistortionCoeffs(const cv::Mat& dist_coeffs, const std::string& camera_name) const
{
    if (dist_coeffs.empty()) {
        std::cout << camera_name << " 畸变系数: 未初始化\n";
        return;
    }

    std::cout << camera_name << " 畸变系数:\n";
    std::cout << "  k1 = " << dist_coeffs.at<double>(0, 0) << "\n";
    std::cout << "  k2 = " << dist_coeffs.at<double>(0, 1) << "\n";
    std::cout << "  p1 = " << dist_coeffs.at<double>(0, 2) << "\n";
    std::cout << "  p2 = " << dist_coeffs.at<double>(0, 3) << "\n";
    std::cout << "  k3 = " << dist_coeffs.at<double>(0, 4) << "\n";
}