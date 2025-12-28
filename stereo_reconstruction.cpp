#include "stereo_reconstruction.h"
#include <iostream>
#include <iomanip>
#include <sstream>
using namespace std;
using namespace cv;

// 计算匹配点的三维坐标并输出到控制台 (单位: mm)
void compute3DPositions(const vector<Point2f>& pts_left,
    const vector<Point2f>& pts_right,
    double focal_length_px, double baseline_mm,
    double cx, double cy) {

    size_t n = min(pts_left.size(), pts_right.size());
    cout << "\n=== 三维重建结果 (单位: mm) ===" << endl;
    cout << "相机参数:" << endl;
    cout << "  焦距: " << focal_length_px << " 像素" << endl;
    cout << "  基线: " << baseline_mm << " mm" << endl;
    cout << "  光心: (" << cx << ", " << cy << ") 像素" << endl;
    cout << "----------------------------------------" << endl;

    for (size_t i = 0; i < n; ++i) {
        double disparity = pts_left[i].x - pts_right[i].x;

        if (fabs(disparity) < 1e-6) {
            cout << "点 " << i << ": 视差太小 (" << disparity << " 像素)，跳过" << endl;
            continue;
        }

        // 计算深度 (mm)
        double Z_mm = (focal_length_px * baseline_mm) / disparity;

        // 计算X, Y坐标 (mm)
        double X_mm = ((pts_left[i].x - cx) * Z_mm) / focal_length_px;
        double Y_mm = ((pts_left[i].y - cy) * Z_mm) / focal_length_px;

        // 输出结果
        cout << fixed << setprecision(2);
        cout << "点 " << i+1 << ":" << endl;
        cout << "  左图像素: (" << pts_left[i].x << ", " << pts_left[i].y << ")" << endl;
        cout << "  右图像素: (" << pts_right[i].x << ", " << pts_right[i].y << ")" << endl;
        cout << "  视差: " << disparity << " 像素" << endl;
        cout << "  三维坐标: X=" << X_mm << "mm, Y=" << Y_mm << "mm, Z=" << Z_mm << "mm" << endl;
        cout << "  距离: " << sqrt(X_mm * X_mm + Y_mm * Y_mm + Z_mm * Z_mm) << " mm" << endl;
        cout << "----------------------------------------" << endl;
    }
}

// 计算三维坐标并绘制到图像上 (单位: mm)
void compute3DPositionsAndDraw(Mat& img,
    const vector<Point2f>& pts_left,
    const vector<Point2f>& pts_right,
    double focal_length_px, double baseline_mm,
    double cx, double cy) {

    size_t n = min(pts_left.size(), pts_right.size());
    cout << "\n=== 三维重建结果（绘制到图像）(单位: mm) ===" << endl;

    for (size_t i = 0; i < n; ++i) {
        double disparity = pts_left[i].x - pts_right[i].x;

        if (fabs(disparity) < 1e-6) {
            cout << "点 " << i << ": 视差太小，跳过" << endl;
            // 在图像上标记无效点
            string invalid_label = "Invalid";
            putText(img, invalid_label, pts_left[i] + Point2f(10, -10),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 2); // 红色字体
            continue;
        }

        // 计算三维坐标 (mm)
        double Z_mm = (focal_length_px * baseline_mm) / disparity;
        double X_mm = ((pts_left[i].x - cx) * Z_mm) / focal_length_px;
        double Y_mm = ((pts_left[i].y - cy) * Z_mm) / focal_length_px;

        // 打印到控制台
        cout << fixed << setprecision(1);
        cout << "点 " << i+1 << ": X=" << X_mm << "mm, Y=" << Y_mm << "mm, Z=" << Z_mm << "mm" << endl;

        // 创建标签文本
        ostringstream oss1, oss2, oss3;
        oss1 << fixed << setprecision(1);
        oss2 << fixed << setprecision(1);
        oss3 << fixed << setprecision(1);

        // 分三行显示，避免文本过长
        //oss1 << "P" << i << ": X=" << X_mm << "mm";
        oss1 << "X=" << X_mm << "mm";
        oss2 << "Y=" << Y_mm << "mm";
        oss3 << "Z=" << Z_mm << "mm";

        string label1 = oss1.str();
        string label2 = oss2.str();
        string label3 = oss3.str();

        // 计算文本位置，避免重叠
        Point2f text_pos = pts_left[i] + Point2f(15, -10);

        // 确保文本不会超出图像边界
        if (text_pos.x + 150 > img.cols) {
            text_pos.x = pts_left[i].x - 160;
        }
        if (text_pos.y - 45 < 0) {
            text_pos.y = pts_left[i].y + 60;
        }

        // 计算文本背景大小
        Size text_size1 = getTextSize(label1, FONT_HERSHEY_SIMPLEX, 0.4, 1, nullptr);
        Size text_size2 = getTextSize(label2, FONT_HERSHEY_SIMPLEX, 0.4, 1, nullptr);
        Size text_size3 = getTextSize(label3, FONT_HERSHEY_SIMPLEX, 0.4, 1, nullptr);

        int max_width = max({ text_size1.width, text_size2.width, text_size3.width });
        int total_height = text_size1.height + text_size2.height + text_size3.height + 20;

        // 绘制半透明背景
        Mat overlay;
        img.copyTo(overlay);
        rectangle(overlay,
            Point(text_pos.x - 3, text_pos.y - text_size1.height - 3),
            Point(text_pos.x + max_width + 3, text_pos.y + total_height - 5),
            Scalar(0, 0, 0), -1);
        addWeighted(overlay, 0.7, img, 0.3, 0, img);

        // 绘制文本（分三行显示）
        putText(img, label1, text_pos,
            FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 255, 255), 3); // 黄色字体
        putText(img, label2, text_pos + Point2f(0, 40),
            FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 255, 255), 3); // 黄色字体
        putText(img, label3, text_pos + Point2f(0, 80),
            FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 255, 255), 3); // 黄色字体


        // 绘制点编号（在检测点上）
        string point_num = to_string(i);
        putText(img, point_num, pts_left[i] + Point2f(-8, 4),
            FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 2); // 蓝色点编号

        // 绘制检测到的点
        circle(img, pts_left[i], 3, Scalar(0, 255, 0), -1); // 绿色实心圆
        circle(img, pts_left[i], 8, Scalar(0, 255, 0), 2);  // 绿色圆环
    }
}

// 新增：批量处理双目图像对的三维重建
void compute3DPositionsBatch(const vector<vector<Point2f>>& pts_left_batch,
    const vector<vector<Point2f>>& pts_right_batch,
    double focal_length_px, double baseline_mm,
    double cx, double cy) {

    cout << "\n=== 批量三维重建结果 (单位: mm) ===" << endl;

    for (size_t img_idx = 0; img_idx < pts_left_batch.size(); ++img_idx) {
        cout << "\n--- 图像对 " << img_idx << " ---" << endl;
        compute3DPositions(pts_left_batch[img_idx], pts_right_batch[img_idx],
            focal_length_px, baseline_mm, cx, cy);
    }
}

// 新增：计算点云并保存为PLY格式
void savePointCloudPLY(const vector<Point2f>& pts_left,
    const vector<Point2f>& pts_right,
    double focal_length_px, double baseline_mm,
    double cx, double cy,
    const string& filename) {

    ofstream ply_file(filename);
    if (!ply_file.is_open()) {
        cerr << "无法创建PLY文件: " << filename << endl;
        return;
    }

    // 计算有效点数
    size_t n = min(pts_left.size(), pts_right.size());
    size_t valid_points = 0;

    vector<Point3f> points_3d;
    for (size_t i = 0; i < n; ++i) {
        double disparity = pts_left[i].x - pts_right[i].x;
        if (fabs(disparity) > 1e-6) {
            double Z_mm = (focal_length_px * baseline_mm) / disparity;
            double X_mm = ((pts_left[i].x - cx) * Z_mm) / focal_length_px;
            double Y_mm = ((pts_left[i].y - cy) * Z_mm) / focal_length_px;
            points_3d.push_back(Point3f(X_mm, Y_mm, Z_mm));
            valid_points++;
        }
    }

    // 写入PLY头部
    ply_file << "ply\n";
    ply_file << "format ascii 1.0\n";
    ply_file << "element vertex " << valid_points << "\n";
    ply_file << "property float x\n";
    ply_file << "property float y\n";
    ply_file << "property float z\n";
    ply_file << "end_header\n";

    // 写入点云数据
    for (const auto& pt : points_3d) {
        ply_file << pt.x << " " << pt.y << " " << pt.z << "\n";
    }

    ply_file.close();
    cout << "点云已保存至: " << filename << " (共 " << valid_points << " 个点)" << endl;
}
//#include "stereo_reconstruction.h"
//#include <iostream>
//#include <iomanip>
//#include <sstream>
//using namespace std;
//using namespace cv;
//
//// 计算匹配点的三维坐标并输出到控制台
//void compute3DPositions(const vector<Point2f>& pts_left,
//    const vector<Point2f>& pts_right,
//    double focal_length, double baseline,
//    double cx, double cy) {
//
//    size_t n = min(pts_left.size(), pts_right.size());
//    cout << "\n=== 三维重建结果 ===" << endl;
//
//    for (size_t i = 0; i < n; ++i) {
//        double disparity = pts_left[i].x - pts_right[i].x;
//        if (fabs(disparity) < 1e-6) {
//            cout << "点 " << i << ": 视差太小，跳过" << endl;
//            continue;
//        }
//
//        double Z = (focal_length * baseline) / disparity;
//        double X = ((pts_left[i].x - cx) * Z) / focal_length;
//        double Y = ((pts_left[i].y - cy) * Z) / focal_length;
//
//        cout << fixed << setprecision(3);
//        cout << "点 " << i << ":\n";
//        cout << "  左图坐标: (" << pts_left[i].x << ", " << pts_left[i].y << ")\n";
//        cout << "  右图坐标: (" << pts_right[i].x << ", " << pts_right[i].y << ")\n";
//        cout << "  视差: " << disparity << " 像素\n";
//        cout << "  三维坐标: X=" << X << "m, Y=" << Y << "m, Z=" << Z << "m\n" << endl;
//    }
//}
//
//// 计算三维坐标并绘制到图像上
//void compute3DPositionsAndDraw(Mat& img,
//    const vector<Point2f>& pts_left,
//    const vector<Point2f>& pts_right,
//    double focal_length, double baseline,
//    double cx, double cy) {
//
//    size_t n = min(pts_left.size(), pts_right.size());
//    cout << "\n=== 三维重建结果（绘制到图像）===" << endl;
//
//    for (size_t i = 0; i < n; ++i) {
//        double disparity = pts_left[i].x - pts_right[i].x;
//        if (fabs(disparity) < 1e-6) {
//            cout << "点 " << i << ": 视差太小，跳过" << endl;
//            // 在图像上标记无效点
//            string invalid_label = "Invalid";
//            putText(img, invalid_label, pts_left[i] + Point2f(10, -10),
//                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 2); // 红色字体
//            continue;
//        }
//
//        double Z = (focal_length * baseline) / disparity;
//        double X = ((pts_left[i].x - cx) * Z) / focal_length;
//        double Y = ((pts_left[i].y - cy) * Z) / focal_length;
//
//        // 打印到控制台
//        cout << fixed << setprecision(3);
//        cout << "点 " << i << ": X=" << X << "m, Y=" << Y << "m, Z=" << Z << "m" << endl;
//
//        // 创建标签文本
//        ostringstream oss;
//        oss << fixed << setprecision(2);
//        oss << "P" << i+1 << ": X=" << X << "m";
//        string label1 = oss.str();
//
//        oss.str(""); // 清空字符串流
//        oss << "Y=" << Y << "m, Z=" << Z << "m";
//        string label2 = oss.str();
//
//        // 计算文本位置，避免重叠
//        Point2f text_pos = pts_left[i] + Point2f(15, -5);
//
//        // 确保文本不会超出图像边界
//        if (text_pos.x + 200 > img.cols) {
//            text_pos.x = pts_left[i].x - 250;
//        }
//        if (text_pos.y - 30 < 0) {
//            text_pos.y = pts_left[i].y + 40;
//        }
//
//        // 绘制文本背景（可选，提高可读性）
//        Size text_size1 = getTextSize(label1, FONT_HERSHEY_SIMPLEX, 0.5, 1, nullptr);
//        Size text_size2 = getTextSize(label2, FONT_HERSHEY_SIMPLEX, 0.5, 1, nullptr);
//
//        // 绘制半透明背景
//        rectangle(img,
//            Point(text_pos.x - 2, text_pos.y - text_size1.height - 2),
//            Point(text_pos.x + max(text_size1.width, text_size2.width) + 2,
//                text_pos.y + text_size2.height + 15),
//            Scalar(0, 0, 0), -1);
//
//        // 绘制文本（分两行显示）
//        putText(img, label1, text_pos,
//            FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 255, 255), 3); // 黄色字体
//        putText(img, label2, text_pos + Point2f(0, 35),
//            FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 255, 255), 3); // 黄色字体
//
//        // 绘制点编号
//        string point_num = to_string(i);
//        putText(img, point_num, pts_left[i] + Point2f(-10, -10),
//            FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 2); // 蓝色点编号
//    }
//}