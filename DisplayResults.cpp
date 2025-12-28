//#include "DisplayResults.h"
//#include <iostream>
//#include <sstream>
//
//using namespace std;
//using namespace cv;
//using namespace cv::viz;
//
//int Visualization3D::widget_counter = 0;
//
//// 构造函数
//Visualization3D::Visualization3D(const string& window_name)
//    : window(window_name), baseline_length(60.0), show_coordinate_frames(true), show_connections(true) {
//
//    // 设置默认值
//    world_origin = Point3D(0, 0, 0);
//    baseline_center = Point3D(0, 0, 0);
//
//    // 设置窗口背景色为深色
//    window.setBackgroundColor(Color::black());
//
//    // 设置相机视点
//    window.setViewerPose(makeCameraPose(Vec3f(200, 200, 200), Vec3f(0, 0, 0), Vec3f(0, 1, 0)));
//
//    cout << "3D可视化窗口已创建: " << window_name << endl;
//}
//
//// 析构函数
//Visualization3D::~Visualization3D() {
//    cout << "3D可视化窗口已关闭" << endl;
//}
//
//// 设置基线参数
//void Visualization3D::setBaseline(const Point3D& center, double length) {
//    baseline_center = center;
//    baseline_length = length;
//    cout << "基线设置: 中心(" << center.x << ", " << center.y << ", " << center.z
//        << "), 长度=" << length << "mm" << endl;
//}
//
//// 添加相机
//void Visualization3D::addCamera(const CameraParams& camera) {
//    cameras.push_back(camera);
//    cout << "添加相机: " << camera.name << " 位置(" << camera.position.x
//        << ", " << camera.position.y << ", " << camera.position.z << ")" << endl;
//}
//
//// 添加器械
//void Visualization3D::addInstrument(const Instrument& instrument) {
//    instruments.push_back(instrument);
//    cout << "添加器械: " << instrument.name << " 颜色("
//        << instrument.color[0] << ", " << instrument.color[1] << ", " << instrument.color[2] << ")" << endl;
//}
//
//// 添加器械上的点
//void Visualization3D::addInstrumentPoint(const string& instrument_name, const Point3D& point) {
//    for (auto& inst : instruments) {
//        if (inst.name == instrument_name) {
//            inst.points.push_back(point);
//            cout << "为器械 " << instrument_name << " 添加点("
//                << point.x << ", " << point.y << ", " << point.z << ")" << endl;
//            return;
//        }
//    }
//    cout << "警告: 未找到器械 " << instrument_name << endl;
//}
//
//// 设置世界坐标系原点
//void Visualization3D::setWorldOrigin(const Point3D& origin) {
//    world_origin = origin;
//    cout << "世界坐标系原点设置为(" << origin.x << ", " << origin.y << ", " << origin.z << ")" << endl;
//}
//
//// 显示/隐藏坐标系
//void Visualization3D::showCoordinateFrames(bool show) {
//    show_coordinate_frames = show;
//    cout << "坐标系显示: " << (show ? "开启" : "关闭") << endl;
//}
//
//// 显示/隐藏器械连线
//void Visualization3D::showConnections(bool show) {
//    show_connections = show;
//    cout << "器械连线显示: " << (show ? "开启" : "关闭") << endl;
//}
//
//// 更新显示
//void Visualization3D::updateDisplay() {
//    // 清除之前的显示内容
//    window.removeAllWidgets();
//    widget_counter = 0;
//
//    cout << "更新3D显示..." << endl;
//
//    // 绘制世界坐标系
//    if (show_coordinate_frames) {
//        drawCoordinateFrame(world_origin, "World", 50.0, Scalar(255, 255, 255));
//    }
//
//    // 绘制基线中心坐标系
//    if (show_coordinate_frames) {
//        drawCoordinateFrame(baseline_center, "Baseline_Center", 40.0, Scalar(255, 255, 0));
//    }
//
//    // 绘制基线
//    drawBaseline();
//
//    // 绘制相机
//    for (const auto& camera : cameras) {
//        drawCamera(camera);
//    }
//
//    // 绘制器械
//    for (const auto& instrument : instruments) {
//        drawInstrument(instrument);
//    }
//
//    cout << "3D显示更新完成" << endl;
//}
//
//// 显示窗口并等待用户交互
//void Visualization3D::show() {
//    updateDisplay();
//
//    cout << "\n=== 3D立体显示控制 ===" << endl;
//    cout << "操作说明:" << endl;
//    cout << "  - 鼠标左键拖拽: 旋转视角" << endl;
//    cout << "  - 鼠标右键拖拽: 平移视角" << endl;
//    cout << "  - 滚轮: 缩放" << endl;
//    cout << "  - ESC键: 退出显示" << endl;
//    cout << "======================" << endl;
//
//    // 显示窗口直到用户按键退出
//    while (!window.wasStopped()) {
//        window.spinOnce(1, true);
//    }
//}
//
//// 保存当前视图为图片
//void Visualization3D::saveView(const string& filename) {
//    window.saveScreenshot(filename);
//    cout << "视图已保存至: " << filename << endl;
//}
//
//// 清除所有数据
//void Visualization3D::clear() {
//    instruments.clear();
//    cameras.clear();
//    window.removeAllWidgets();
//    widget_counter = 0;
//    cout << "所有数据已清除" << endl;
//}
//
//// 绘制坐标系
//void Visualization3D::drawCoordinateFrame(const Point3D& origin, const string& name,
//    double axis_length, const Scalar& color) {
//    Point3f center(origin.x, origin.y, origin.z);
//
//    // 创建坐标系widget
//    WCoordinateSystem coord_widget(axis_length);
//
//    // 设置位置
//    Affine3d pose = Affine3d::Identity();
//    pose.translation(Vec3d(origin.x, origin.y, origin.z));
//
//    // 添加到场景
//    string widget_name = generateWidgetName("coord_" + name);
//    window.showWidget(widget_name, coord_widget, pose);
//
//    // 添加文本标签
//    WText3D text_widget(name, center + Point3f(axis_length * 0.1, axis_length * 0.1, axis_length * 0.1),
//        axis_length * 0.1, true, Color(color));
//    string text_name = generateWidgetName("text_" + name);
//    window.showWidget(text_name, text_widget);
//}
//
//// 绘制相机
//void Visualization3D::drawCamera(const CameraParams& camera) {
//    Point3f cam_pos(camera.position.x, camera.position.y, camera.position.z);
//
//    // 绘制相机坐标系
//    if (show_coordinate_frames) {
//        drawCoordinateFrame(camera.position, camera.name + "_Frame", 25.0, camera.color);
//    }
//
//    // 绘制相机模型（简单的金字塔形状）
//    double cam_size = 15.0;
//    vector<Point3f> cam_points = {
//        cam_pos,  // 顶点
//        cam_pos + Point3f(-cam_size, -cam_size, cam_size),  // 底面四个角
//        cam_pos + Point3f(cam_size, -cam_size, cam_size),
//        cam_pos + Point3f(cam_size, cam_size, cam_size),
//        cam_pos + Point3f(-cam_size, cam_size, cam_size)
//    };
//
//    // 连接相机的边
//    for (int i = 1; i < 5; ++i) {
//        WLine line_widget(cam_points[0], cam_points[i], Color(camera.color));
//        string line_name = generateWidgetName("cam_line_" + camera.name);
//        window.showWidget(line_name, line_widget);
//
//        // 连接底面
//        WLine bottom_line(cam_points[i], cam_points[i % 4 + 1], Color(camera.color));
//        string bottom_name = generateWidgetName("cam_bottom_" + camera.name);
//        window.showWidget(bottom_name, bottom_line);
//    }
//
//    // 添加相机标签
//    WText3D cam_text(camera.name, cam_pos + Point3f(0, -20, 0), 8.0, true, Color(camera.color));
//    string text_name = generateWidgetName("cam_text_" + camera.name);
//    window.showWidget(text_name, cam_text);
//}
//
//// 绘制器械
//void Visualization3D::drawInstrument(const Instrument& instrument) {
//    if (instrument.points.empty()) return;
//
//    // 绘制器械上的小球
//    for (size_t i = 0; i < instrument.points.size(); ++i) {
//        Point3f pt = instrument.points[i].toPoint3f();
//
//        // 绘制小球
//        WSphere sphere_widget(pt, instrument.sphere_radius, 10, Color(instrument.color));
//        string sphere_name = generateWidgetName("sphere_" + instrument.name);
//        window.showWidget(sphere_name, sphere_widget);
//
//        // 添加点编号
//        stringstream ss;
//        ss << instrument.name << "_" << i;
//        WText3D point_text(ss.str(), pt + Point3f(3, 3, 3), 5.0, true, Color(instrument.color));
//        string text_name = generateWidgetName("point_text_" + instrument.name);
//        window.showWidget(text_name, point_text);
//    }
//
//    // 绘制器械上点与点之间的连线
//    if (show_connections && instrument.points.size() > 1) {
//        for (size_t i = 0; i < instrument.points.size() - 1; ++i) {
//            Point3f pt1 = instrument.points[i].toPoint3f();
//            Point3f pt2 = instrument.points[i + 1].toPoint3f();
//
//            WLine connection(pt1, pt2, Color(instrument.color));
//            string line_name = generateWidgetName("connection_" + instrument.name);
//            window.showWidget(line_name, connection);
//        }
//    }
//
//    // 添加器械名称标签
//    if (!instrument.points.empty()) {
//        Point3f center = instrument.points[0].toPoint3f();
//        WText3D inst_text(instrument.name, center + Point3f(0, -15, 0), 8.0, true, Color(instrument.color));
//        string text_name = generateWidgetName("inst_text_" + instrument.name);
//        window.showWidget(text_name, inst_text);
//    }
//}
//
//// 绘制基线
//void Visualization3D::drawBaseline() {
//    // 计算左右相机位置（假设沿X轴分布）
//    Point3f left_pos(baseline_center.x - baseline_length / 2, baseline_center.y, baseline_center.z);
//    Point3f right_pos(baseline_center.x + baseline_length / 2, baseline_center.y, baseline_center.z);
//
//    // 绘制基线
//    WLine baseline_widget(left_pos, right_pos, Color::yellow());
//    string baseline_name = generateWidgetName("baseline");
//    window.showWidget(baseline_name, baseline_widget);
//
//    // 在基线中心添加标记
//    WSphere center_sphere(Point3f(baseline_center.x, baseline_center.y, baseline_center.z),
//        3.0, 10, Color::yellow());
//    string center_name = generateWidgetName("baseline_center");
//    window.showWidget(center_name, center_sphere);
//}
//
//// 生成唯一的widget名称
//string Visualization3D::generateWidgetName(const string& prefix) {
//    stringstream ss;
//    ss << prefix << "_" << widget_counter++;
//    return ss.str();
//}