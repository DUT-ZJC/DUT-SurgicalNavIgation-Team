#include "Pose6DOF.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>

using namespace cv;
using namespace std;

// ========== Pose6DOF 实现 ==========
string Pose6DOF::toString() const {
    stringstream ss;
    ss << fixed << setprecision(2);
    ss << "位置: (" << position.x << ", " << position.y << ", " << position.z << ") mm\n";
    ss << "旋转: Roll=" << roll << "°, Pitch=" << pitch << "°, Yaw=" << yaw << "°";
    return ss.str();
}

// ========== Pose6DOFCalculator 实现 ==========
Pose6DOFCalculator::Pose6DOFCalculator(const vector<Point3f>& modelPoints)
    : modelPoints_(modelPoints) {
    if (modelPoints_.size() != 4) {
        cerr << "错误：模型点必须是4个！" << endl;
    }

    // 检查是否共面
    if (checkCoplanar(modelPoints_)) {
        cout << "提示：检测到模型点共面，将使用PnP方法计算姿态" << endl;
        useCoplanarMethod_ = true;
    }
    else {
        cout << "提示：模型点非共面，将使用SVD方法计算姿态" << endl;
        useCoplanarMethod_ = false;
    }
}

bool Pose6DOFCalculator::checkCoplanar(const vector<Point3f>& points) {
    if (points.size() < 4) return true;

    // 计算前3个点构成的平面
    Point3f v1 = points[1] - points[0];
    Point3f v2 = points[2] - points[0];
    Point3f normal = v1.cross(v2);

    double normal_length = norm(normal);
    if (normal_length < 1e-6) return true; // 共线

    normal = normal / normal_length;

    // 检查第4个点到平面的距离
    Point3f v3 = points[3] - points[0];
    double distance = abs(normal.dot(v3));

    return distance < 1.0; // 距离小于1mm认为共面
}

Point3f Pose6DOFCalculator::calculateCentroid(const vector<Point3f>& points) {
    Point3f centroid(0, 0, 0);
    for (const auto& p : points) {
        centroid += p;
    }
    return centroid * (1.0f / points.size());
}

bool Pose6DOFCalculator::calculateRigidTransform(const vector<Point3f>& srcPoints,
    const vector<Point3f>& dstPoints,
    Mat& R, Mat& t) {
    if (srcPoints.size() != dstPoints.size() || srcPoints.size() < 3) {
        cerr << "错误：点数不匹配或点数不足3个！" << endl;
        return false;
    }

    // 1. 计算质心
    Point3f srcCentroid = calculateCentroid(srcPoints);
    Point3f dstCentroid = calculateCentroid(dstPoints);

    // 2. 去中心化
    vector<Point3f> srcCentered, dstCentered;
    for (size_t i = 0; i < srcPoints.size(); ++i) {
        srcCentered.push_back(srcPoints[i] - srcCentroid);
        dstCentered.push_back(dstPoints[i] - dstCentroid);
    }

    // 3. 构建协方差矩阵 H = sum(src_i * dst_i^T)
    Mat H = Mat::zeros(3, 3, CV_64F);
    for (size_t i = 0; i < srcCentered.size(); ++i) {
        Mat src = (Mat_<double>(3, 1) << srcCentered[i].x, srcCentered[i].y, srcCentered[i].z);
        Mat dst = (Mat_<double>(3, 1) << dstCentered[i].x, dstCentered[i].y, dstCentered[i].z);
        H += src * dst.t();
    }

    // 4. SVD分解
    Mat U, W, Vt;
    SVD::compute(H, W, U, Vt);

    // 检查矩阵秩
    double min_singular = W.at<double>(min(W.rows, W.cols) - 1);
    if (min_singular < 1e-6) {
        cerr << "错误：点共面或接近共线，无法用SVD计算！请使用PnP方法" << endl;
        return false;
    }

    // 5. 计算旋转矩阵
    R = Vt.t() * U.t();

    // 6. 处理反射情况
    if (determinant(R) < 0) {
        Vt.row(2) = -Vt.row(2);
        R = Vt.t() * U.t();
    }

    // 7. 计算平移向量
    Mat srcCentroidMat = (Mat_<double>(3, 1) << srcCentroid.x, srcCentroid.y, srcCentroid.z);
    Mat dstCentroidMat = (Mat_<double>(3, 1) << dstCentroid.x, dstCentroid.y, dstCentroid.z);
    t = dstCentroidMat - R * srcCentroidMat;

    return true;
}

bool Pose6DOFCalculator::calculatePoseWithPnP(const vector<Point3f>& measuredPoints,
    const Mat& cameraMatrix,
    Pose6DOF& outPose) {
    // 将3D点反投影到2D（从世界坐标计算像素坐标）
    // 这里假设你有相机内参，如果没有需要从双目系统获取

    vector<Point2f> imagePoints;
    for (const auto& pt : measuredPoints) {
        // 简化投影：假设Z是深度
        double fx = cameraMatrix.at<double>(0, 0);
        double fy = cameraMatrix.at<double>(1, 1);
        double cx = cameraMatrix.at<double>(0, 2);
        double cy = cameraMatrix.at<double>(1, 2);

        float u = (pt.x / pt.z) * fx + cx;
        float v = (pt.y / pt.z) * fy + cy;
        imagePoints.push_back(Point2f(u, v));
    }

    // 使用PnP求解
    Mat rvec, tvec;
    bool success = solvePnP(modelPoints_, imagePoints, cameraMatrix, Mat(), rvec, tvec, false, SOLVEPNP_ITERATIVE);

    if (!success) {
        cerr << "PnP求解失败！" << endl;
        outPose.isValid = false;
        return false;
    }

    // 提取位置
    outPose.position.x = tvec.at<double>(0);
    outPose.position.y = tvec.at<double>(1);
    outPose.position.z = tvec.at<double>(2);

    // 旋转向量转旋转矩阵
    Mat R;
    Rodrigues(rvec, R);

    // 提取欧拉角
    rotationMatrixToEulerAngles(R, outPose.roll, outPose.pitch, outPose.yaw);

    outPose.rotationMatrix = R.clone();
    outPose.isValid = true;

    return true;
}

void Pose6DOFCalculator::rotationMatrixToEulerAngles(const Mat& R, float& roll, float& pitch, float& yaw) {
    // 使用ZYX欧拉角约定
    double sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

    bool singular = sy < 1e-6;

    if (!singular) {
        roll = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        pitch = atan2(-R.at<double>(2, 0), sy);
        yaw = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    }
    else {
        roll = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        pitch = atan2(-R.at<double>(2, 0), sy);
        yaw = 0;
    }

    // 转换为度
    roll = roll * 180.0 / CV_PI;
    pitch = pitch * 180.0 / CV_PI;
    yaw = yaw * 180.0 / CV_PI;
}

bool Pose6DOFCalculator::calculatePose(const vector<Point3f>& measuredPoints, Pose6DOF& outPose) {
    // 验证输入
    if (measuredPoints.size() != 4) {
        cerr << "错误：测量点必须是4个！" << endl;
        outPose.isValid = false;
        return false;
    }

    // 根据模型点是否共面选择方法
    if (useCoplanarMethod_) {
        // 共面情况：直接使用平面约束计算
        return calculatePoseCoplanar(measuredPoints, outPose);
    }
    else {
        // 非共面情况：使用SVD方法
        Mat R, t;
        if (!calculateRigidTransform(modelPoints_, measuredPoints, R, t)) {
            cerr << "错误：计算刚体变换失败！" << endl;
            outPose.isValid = false;
            return false;
        }

        outPose.position.x = t.at<double>(0, 0);
        outPose.position.y = t.at<double>(1, 0);
        outPose.position.z = t.at<double>(2, 0);

        rotationMatrixToEulerAngles(R, outPose.roll, outPose.pitch, outPose.yaw);

        outPose.rotationMatrix = R.clone();
        outPose.isValid = true;

        return true;
    }
}

bool Pose6DOFCalculator::calculatePoseCoplanar(const vector<Point3f>& measuredPoints, Pose6DOF& outPose) {
    cout << "\n========== 共面点姿态计算 ==========" << endl;

    // 打印输入点
    cout << "测量点:" << endl;
    for (size_t i = 0; i < measuredPoints.size(); i++) {
        cout << "  点" << i << ": (" << measuredPoints[i].x << ", "
            << measuredPoints[i].y << ", " << measuredPoints[i].z << ")" << endl;
    }

    // 计算测量点的质心作为位置
    Point3f centroid = calculateCentroid(measuredPoints);
    outPose.position = centroid;
    cout << "质心位置: (" << centroid.x << ", " << centroid.y << ", " << centroid.z << ")" << endl;

    // 计算平面的两个向量
    Point3f v1 = measuredPoints[1] - measuredPoints[0];
    Point3f v2 = measuredPoints[2] - measuredPoints[0];

    cout << "向量v1: (" << v1.x << ", " << v1.y << ", " << v1.z << ")" << endl;
    cout << "向量v2: (" << v2.x << ", " << v2.y << ", " << v2.z << ")" << endl;

    // 计算平面法向量（Z轴方向）
    Point3f normal = v1.cross(v2);
    double len_normal = norm(normal);

    cout << "法向量长度: " << len_normal << endl;

    if (len_normal < 1e-6) {
        cerr << "错误：点共线，无法计算姿态！" << endl;
        outPose.isValid = false;
        return false;
    }

    normal = normal / len_normal;  // 归一化 Z轴
    cout << "归一化法向量(Z轴): (" << normal.x << ", " << normal.y << ", " << normal.z << ")" << endl;

    // 计算X轴（取第一个向量方向）
    double len_v1 = norm(v1);
    if (len_v1 < 1e-6) {
        cerr << "错误：向量v1长度为0！" << endl;
        outPose.isValid = false;
        return false;
    }
    Point3f x_axis = v1 / len_v1;
    cout << "X轴: (" << x_axis.x << ", " << x_axis.y << ", " << x_axis.z << ")" << endl;

    // 计算Y轴（通过Z轴×X轴得到正交的Y轴）
    Point3f y_axis = normal.cross(x_axis);
    double len_y = norm(y_axis);

    if (len_y < 1e-6) {
        cerr << "错误：Y轴计算失败！" << endl;
        outPose.isValid = false;
        return false;
    }

    y_axis = y_axis / len_y;
    cout << "Y轴: (" << y_axis.x << ", " << y_axis.y << ", " << y_axis.z << ")" << endl;

    // 构建旋转矩阵（列向量为坐标轴）
    Mat R = (Mat_<double>(3, 3) <<
        x_axis.x, y_axis.x, normal.x,
        x_axis.y, y_axis.y, normal.y,
        x_axis.z, y_axis.z, normal.z);

    cout << "\n旋转矩阵 R:\n" << R << endl;
    cout << "行列式: " << determinant(R) << " (应接近±1)" << endl;

    // 提取欧拉角
    rotationMatrixToEulerAngles(R, outPose.roll, outPose.pitch, outPose.yaw);

    cout << "\n欧拉角:" << endl;
    cout << "  Roll  = " << outPose.roll << "°" << endl;
    cout << "  Pitch = " << outPose.pitch << "°" << endl;
    cout << "  Yaw   = " << outPose.yaw << "°" << endl;

    outPose.rotationMatrix = R.clone();
    outPose.isValid = true;

    cout << "========== 计算成功 ==========\n" << endl;

    return true;
}