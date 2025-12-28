#include "distance_calculator.h"

// 计算两个3D点之间的欧几里得距离
double DistanceCalculator::calculateEuclideanDistance(const Point3f& p1, const Point3f& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

// 从2D点计算3D坐标
Point3f DistanceCalculator::calculate3DPoint(const Point2f& pt_left, const Point2f& pt_right,
                                           double focal_length_px, double baseline_mm,
                                           double cx, double cy) {
    double disparity = pt_left.x - pt_right.x;
    
    if (!isValidDisparity(disparity)) {
        return Point3f(0, 0, 0); // 返回无效点
    }
    
    // 计算三维坐标 (单位: mm)
    double Z_mm = (focal_length_px * baseline_mm) / disparity;
    double X_mm = ((pt_left.x - cx) * Z_mm) / focal_length_px;
    double Y_mm = ((pt_left.y - cy) * Z_mm) / focal_length_px;
    
    return Point3f(X_mm, Y_mm, Z_mm);
}

// 计算所有相邻点之间的距离 (点1-点2, 点2-点3, ..., 点7-点8)
void DistanceCalculator::calculateAdjacentDistances(const vector<Point2f>& pts_left,
                                                   const vector<Point2f>& pts_right,
                                                   double focal_length_px, double baseline_mm,
                                                   double cx, double cy,
                                                   const string& output_file) {
    ofstream file(output_file);
    if (!file.is_open()) {
        cerr << "错误：无法创建输出文件 " << output_file << endl;
        return;
    }
    
    size_t n = min(pts_left.size(), pts_right.size());
    if (n < 2) {
        cerr << "错误：需要至少2个匹配点对来计算距离" << endl;
        file.close();
        return;
    }
    
    // 写入文件头部信息
    file << "=== 相邻点距离计算结果 ===" << endl;
    file << "相机参数:" << endl;
    file << "  焦距: " << focal_length_px << " 像素" << endl;
    file << "  基线: " << baseline_mm << " mm" << endl;
    file << "  光心: (" << cx << ", " << cy << ") 像素" << endl;
    file << "计算时间: " << endl; // 可以添加时间戳
    file << "========================================" << endl;
    
    cout << "\n=== 计算相邻点距离 ===" << endl;
    
    // 首先计算所有点的3D坐标
    vector<Point3f> points_3d;
    for (size_t i = 0; i < n; ++i) {
        Point3f p3d = calculate3DPoint(pts_left[i], pts_right[i], 
                                      focal_length_px, baseline_mm, cx, cy);
        points_3d.push_back(p3d);
    }
    
    // 计算相邻点之间的距离
    for (size_t i = 0; i < n - 1; ++i) {
        Point3f p1 = points_3d[i];
        Point3f p2 = points_3d[i + 1];
        
        // 检查点的有效性 (检查是否为无效计算结果)
        if (p1.x == 0 && p1.y == 0 && p1.z == 0) {
            cout << "点" << (i+1) << " 到 点" << (i+2) << ": 点" << (i+1) << "无效，跳过" << endl;
            file << "点" << (i+1) << " 到 点" << (i+2) << ": 点" << (i+1) << "无效，跳过" << endl;
            continue;
        }
        if (p2.x == 0 && p2.y == 0 && p2.z == 0) {
            cout << "点" << (i+1) << " 到 点" << (i+2) << ": 点" << (i+2) << "无效，跳过" << endl;
            file << "点" << (i+1) << " 到 点" << (i+2) << ": 点" << (i+2) << "无效，跳过" << endl;
            continue;
        }
        
        double distance = calculateEuclideanDistance(p1, p2);
        
        // 输出到控制台
        cout << fixed << setprecision(2);
        cout << "点" << (i+1) << " 到 点" << (i+2) << ": " << distance << " mm" << endl;
        
        // 写入详细信息到文件
        writeDistanceInfo(file, i+1, i+2, p1, p2, distance);
    }
    
    file.close();
    cout << "距离计算结果已保存到: " << output_file << endl;
}

// 计算特定点对之间的距离 (点1-点2, 点2-点3, 点3-点4, 点4-点5, 点5-点6, 点6-点7, 点7-点8)
void DistanceCalculator::calculateSpecificDistances(const vector<Point2f>& pts_left,
                                                   const vector<Point2f>& pts_right,
                                                   double focal_length_px, double baseline_mm,
                                                   double cx, double cy,
                                                   const string& output_file) {
    ofstream file(output_file);
    if (!file.is_open()) {
        cerr << "错误：无法创建输出文件 " << output_file << endl;
        return;
    }
    
    size_t n = min(pts_left.size(), pts_right.size());
    if (n < 8) {
        cerr << "警告：检测到的点数少于8个，当前点数: " << n << endl;
    }
    
    // 写入文件头部信息
    file << "=== 指定点对距离计算结果 ===" << endl;
    file << "相机参数:" << endl;
    file << "  焦距: " << focal_length_px << " 像素" << endl;
    file << "  基线: " << baseline_mm << " mm" << endl;
    file << "  光心: (" << cx << ", " << cy << ") 像素" << endl;
    file << "检测点数: " << n << endl;
    file << "========================================" << endl;
    
    cout << "\n=== 计算指定点对距离 ===" << endl;
    
    // 首先计算所有点的3D坐标
    vector<Point3f> points_3d;
    for (size_t i = 0; i < n; ++i) {
        Point3f p3d = calculate3DPoint(pts_left[i], pts_right[i], 
                                      focal_length_px, baseline_mm, cx, cy);
        points_3d.push_back(p3d);
        
        // 输出每个点的3D坐标
        file << fixed << setprecision(2);
        file << "点" << (i+1) << " 3D坐标: (" 
             << p3d.x << ", " << p3d.y << ", " << p3d.z << ") mm" << endl;
    }
    
    file << "========================================" << endl;
    
    // 定义要计算的点对
    vector<pair<int, int>> point_pairs = {
        {1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6}, {6, 7}, {7, 8}
    };
    
    double total_distance = 0.0;
    int valid_distances = 0;
    
    for (const auto& pair : point_pairs) {
        int idx1 = pair.first - 1;   // 转换为0-based索引
        int idx2 = pair.second - 1;
        
        if (idx1 >= n || idx2 >= n) {
            cout << "点" << pair.first << " 到 点" << pair.second << ": 点索引超出范围，跳过" << endl;
            file << "点" << pair.first << " 到 点" << pair.second << ": 点索引超出范围，跳过" << endl;
            continue;
        }
        
        Point3f p1 = points_3d[idx1];
        Point3f p2 = points_3d[idx2];
        
        // 检查点的有效性 (检查是否为无效计算结果)
        if (p1.x == 0 && p1.y == 0 && p1.z == 0) {
            cout << "点" << pair.first << " 到 点" << pair.second << ": 点" << pair.first << "无效，跳过" << endl;
            file << "点" << pair.first << " 到 点" << pair.second << ": 点" << pair.first << "无效，跳过" << endl;
            continue;
        }
        if (p2.x == 0 && p2.y == 0 && p2.z == 0) {
            cout << "点" << pair.first << " 到 点" << pair.second << ": 点" << pair.second << "无效，跳过" << endl;
            file << "点" << pair.first << " 到 点" << pair.second << ": 点" << pair.second << "无效，跳过" << endl;
            continue;
        }
        
        double distance = calculateEuclideanDistance(p1, p2);
        total_distance += distance;
        valid_distances++;
        
        // 输出到控制台
        cout << fixed << setprecision(2);
        cout << "点" << pair.first << " 到 点" << pair.second << ": " << distance << " mm" << endl;
        
        // 写入详细信息到文件
        writeDistanceInfo(file, pair.first, pair.second, p1, p2, distance);
    }
    
    // 输出统计信息
    file << "========================================" << endl;
    file << "统计信息:" << endl;
    file << "  有效距离数: " << valid_distances << endl;
    if (valid_distances > 0) {
        file << "  总距离: " << fixed << setprecision(2) << total_distance << " mm" << endl;
        file << "  平均距离: " << fixed << setprecision(2) << (total_distance / valid_distances) << " mm" << endl;
    }
    
    file.close();
    cout << "距离计算结果已保存到: " << output_file << endl;
}

// 计算所有点对之间的距离矩阵
void DistanceCalculator::calculateAllDistances(const vector<Point2f>& pts_left,
                                             const vector<Point2f>& pts_right,
                                             double focal_length_px, double baseline_mm,
                                             double cx, double cy,
                                             const string& output_file) {
    ofstream file(output_file);
    if (!file.is_open()) {
        cerr << "错误：无法创建输出文件 " << output_file << endl;
        return;
    }
    
    size_t n = min(pts_left.size(), pts_right.size());
    if (n < 2) {
        cerr << "错误：需要至少2个匹配点对来计算距离" << endl;
        file.close();
        return;
    }
    
    // 写入文件头部信息
    file << "=== 所有点对距离矩阵 ===" << endl;
    file << "相机参数:" << endl;
    file << "  焦距: " << focal_length_px << " 像素" << endl;
    file << "  基线: " << baseline_mm << " mm" << endl;
    file << "  光心: (" << cx << ", " << cy << ") 像素" << endl;
    file << "检测点数: " << n << endl;
    file << "========================================" << endl;
    
    cout << "\n=== 计算所有点对距离矩阵 ===" << endl;
    
    // 首先计算所有点的3D坐标
    vector<Point3f> points_3d;
    for (size_t i = 0; i < n; ++i) {
        Point3f p3d = calculate3DPoint(pts_left[i], pts_right[i], 
                                      focal_length_px, baseline_mm, cx, cy);
        points_3d.push_back(p3d);
    }
    
    // 创建距离矩阵
    vector<vector<double>> distance_matrix(n, vector<double>(n, 0.0));
    
    // 计算所有点对之间的距离
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            Point3f p1 = points_3d[i];
            Point3f p2 = points_3d[j];
            
            if (p1.x == 0 && p1.y == 0 && p1.z == 0) {
                distance_matrix[i][j] = -1.0; // 标记为无效
                distance_matrix[j][i] = -1.0;
                continue;
            }
            if (p2.x == 0 && p2.y == 0 && p2.z == 0) {
                distance_matrix[i][j] = -1.0; // 标记为无效
                distance_matrix[j][i] = -1.0;
                continue;
            }
            
            double distance = calculateEuclideanDistance(p1, p2);
            distance_matrix[i][j] = distance;
            distance_matrix[j][i] = distance;
        }
    }
    
    // 输出距离矩阵
    file << "距离矩阵 (单位: mm):" << endl;
    file << "     ";
    for (size_t j = 0; j < n; ++j) {
        file << "  点" << (j+1) << "   ";
    }
    file << endl;
    
    for (size_t i = 0; i < n; ++i) {
        file << "点" << (i+1) << " ";
        for (size_t j = 0; j < n; ++j) {
            if (i == j) {
                file << "   0.00  ";
            } else if (distance_matrix[i][j] < 0) {
                file << " Invalid ";
            } else {
                file << fixed << setprecision(2) << setw(7) << distance_matrix[i][j] << " ";
            }
        }
        file << endl;
    }
    
    file.close();
    cout << "距离矩阵已保存到: " << output_file << endl;
}

// 辅助函数：检查视差是否有效
bool DistanceCalculator::isValidDisparity(double disparity, double threshold) {
    return fabs(disparity) > threshold;
}

// 辅助函数：格式化输出距离信息
void DistanceCalculator::writeDistanceInfo(ofstream& file, int point1, int point2, 
                                          const Point3f& p1, const Point3f& p2, 
                                          double distance) {
    file << fixed << setprecision(2);
    file << "点" << point1 << " 到 点" << point2 << ":" << endl;
    file << "  点" << point1 << " 坐标: (" << p1.x << ", " << p1.y << ", " << p1.z << ") mm" << endl;
    file << "  点" << point2 << " 坐标: (" << p2.x << ", " << p2.y << ", " << p2.z << ") mm" << endl;
    file << "  距离: " << distance << " mm" << endl;
    file << "----------------------------------------" << endl;
}