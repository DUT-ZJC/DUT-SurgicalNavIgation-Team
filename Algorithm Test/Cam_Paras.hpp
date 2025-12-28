#ifndef CAMERA_PARAMS_HPP
#define CAMERA_PARAMS_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

/**   类/继承 封装 多态 
 * @brief 双目相机参数管理类
 *
 * 封装双目相机的内参、外参、畸变系数等参数，
 * 提供参数验证、打印、保存/加载等功能
 */   
class CameraParams
{
public:
    // 构造函数
    CameraParams();
    CameraParams(const cv::Mat& camera_matrix_left, const cv::Mat& camera_matrix_right,
        const cv::Mat& dist_coeffs_left, const cv::Mat& dist_coeffs_right,
        const cv::Mat& R, const cv::Mat& T);

    // 析构函数
    ~CameraParams() = default;

    // ==================== 参数设置接口 ====================

    // 设置左相机内参
    void SetLeftCameraMatrix(double fx, double fy, double cx, double cy);
    void SetLeftCameraMatrix(const cv::Mat& camera_matrix);

    // 设置右相机内参
    void SetRightCameraMatrix(double fx, double fy, double cx, double cy);
    void SetRightCameraMatrix(const cv::Mat& camera_matrix);

    // 设置左相机畸变系数
    void SetLeftDistortionCoeffs(double k1, double k2, double p1, double p2, double k3);
    void SetLeftDistortionCoeffs(const cv::Mat& dist_coeffs);

    // 设置右相机畸变系数
    void SetRightDistortionCoeffs(double k1, double k2, double p1, double p2, double k3);
    void SetRightDistortionCoeffs(const cv::Mat& dist_coeffs);

    // 设置双目外参
    void SetStereoExtrinsics(const cv::Mat& R, const cv::Mat& T);
    void SetBaseline(double baseline_mm);

    // 设置物理参数
    void SetPixelSize(double pixel_size_mm);
    void SetSensorSize(double width_mm, double height_mm);

    // ==================== 参数获取接口 ====================

    // 获取相机内参矩阵
    const cv::Mat& GetLeftCameraMatrix() const { return  m_camera_matrix_left; }
    const cv::Mat& GetRightCameraMatrix() const { return m_camera_matrix_right; }

    // 获取畸变系数
    const cv::Mat& GetLeftDistortionCoeffs() const { return m_dist_coeffs_left; }
    const cv::Mat& GetRightDistortionCoeffs() const { return m_dist_coeffs_right; }

    // 获取双目外参
    const cv::Mat& GetRotationMatrix() const { return m_R; }
    const cv::Mat& GetTranslationVector() const { return m_T; }

    // 获取基线距离
    double GetBaseline() const { return m_baseline_mm; }

    // 获取物理参数
    double GetPixelSize() const { return m_pixel_size_mm; }
    double GetFocalLengthMM() const;

    // 获取具体参数值
    double GetLeftFx() const { return m_camera_matrix_left.at<double>(0, 0); }
    double GetLeftFy() const { return m_camera_matrix_left.at<double>(1, 1); }
    double GetLeftCx() const { return m_camera_matrix_left.at<double>(0, 2); }
    double GetLeftCy() const { return m_camera_matrix_left.at<double>(1, 2); }

    double GetRightFx() const { return m_camera_matrix_right.at<double>(0, 0); }
    double GetRightFy() const { return m_camera_matrix_right.at<double>(1, 1); }
    double GetRightCx() const { return m_camera_matrix_right.at<double>(0, 2); }
    double GetRightCy() const { return m_camera_matrix_right.at<double>(1, 2); }

    // ==================== 功能接口 ====================

    // 打印相机参数
    void PrintParams() const;
    void PrintLeftCameraParams() const;
    void PrintRightCameraParams() const;
    void PrintStereoParams() const;

    // 验证参数合理性
    bool ValidateParams() const;
    bool ValidateLeftCamera() const;
    bool ValidateRightCamera() const;
    bool ValidateStereoParams() const;

    // 计算重投影误差
    double CalculateReprojectionError(const std::vector<cv::Point3f>& object_points,
        const std::vector<cv::Point2f>& image_points,
        const cv::Mat& rvec,
        const cv::Mat& tvec,
        bool use_left_camera = true) const;

    // 参数文件操作
    bool SaveToFile(const std::string& filename) const;
    bool LoadFromFile(const std::string& filename);

    // 参数转换
    cv::Mat GetStereoRectificationParams() const;

    // 重置为默认参数
    void ResetToDefault();

    // 参数是否已初始化
    bool IsInitialized() const;

    //// ==================== 兼容性接口 (向后兼容) ====================

    //// 兼容旧版本代码的全局参数访问
    //const cv::Mat& GetCameraMatrix() const { return GetLeftCameraMatrix(); }
    //const cv::Mat& GetDistortionCoeffs() const { return GetLeftDistortionCoeffs(); }
    //double GetFocalLengthPx() const { return GetLeftFx(); }
    //double GetBaselineM() const { return m_baseline_mm / 1000.0; }
    //double GetCx() const { return GetLeftCx(); }
    //double GetCy() const { return GetLeftCy(); }

private:
    // ==================== 内参矩阵 ====================
    cv::Mat m_camera_matrix_left;     // 左相机内参矩阵 3x3
    cv::Mat m_camera_matrix_right;    // 右相机内参矩阵 3x3

    // ==================== 畸变系数 ====================
    cv::Mat m_dist_coeffs_left;       // 左相机畸变系数 1x5
    cv::Mat m_dist_coeffs_right;      // 右相机畸变系数 1x5

    // ==================== 双目外参 ====================
    cv::Mat m_R;                      // 旋转矩阵 3x3
    cv::Mat m_T;                      // 平移向量 3x1

    // ==================== 物理参数 ====================
    double m_baseline_mm;             // 基线距离 (mm)
    double m_pixel_size_mm;           // 像素物理尺寸 (mm/pixel)
    double m_sensor_width_mm;         // 传感器宽度 (mm)
    double m_sensor_height_mm;        // 传感器高度 (mm)

    // ==================== 初始化标志 ====================
    bool m_initialized;

    // ==================== 私有辅助函数 ====================
    void InitializeDefaultParams();
    bool ValidateCameraMatrix(const cv::Mat& camera_matrix, const std::string& camera_name) const;
    bool ValidateDistortionCoeffs(const cv::Mat& dist_coeffs, const std::string& camera_name) const;
    void PrintCameraMatrix(const cv::Mat& camera_matrix, const std::string& camera_name) const;
    void PrintDistortionCoeffs(const cv::Mat& dist_coeffs, const std::string& camera_name) const;
};

#endif // CAMERA_PARAMS_H