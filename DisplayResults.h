//#ifndef 3D_VISUALIZATION_H
//#define 3D_VISUALIZATION_H
//
//#include <opencv2/opencv.hpp>
//#include <opencv2/viz.hpp>
//#include <vector>
//#include <string>
//
//using namespace cv;
//using namespace cv::viz;
//using namespace std;
//
//// 三维点结构体
//struct Point3D {
//    double x, y, z;
//    Point3D(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
//    Point3f toPoint3f() const { return Point3f(x, y, z); }
//};
//
//// 器械结构体
//struct Instrument {
//    vector<Point3D> points;     // 器械上的点
//    Scalar color;               // 器械颜色
//    string name;                // 器械名称
//    double sphere_radius;       // 小球半径
//
//    Instrument(const string& n, const Scalar& c, double radius = 2.0)
//        : name(n), color(c), sphere_radius(radius) {
//    }
//};
//
//// 相机参数结构体
//struct CameraParams {
//    Point3D position;           // 相机位置
//    Point3D orientation;        // 相机方向(欧拉角)
//    double focal_length;        // 焦距
//    Scalar color;              // 显示颜色
//    string name;               // 相机名称
//
//    CameraParams(const string& n, const Point3D& pos, const Scalar& c)
//        : name(n), position(pos), color(c), focal_length(50.0), orientation(0, 0, 0) {
//    }
//};
//
//class Visualization3D {
//private:
//    Viz3d window;                           // 3D显示窗口
//    vector<Instrument> instruments;         // 器械列表
//    vector<CameraParams> cameras;           // 相机列表
//    Point3D world_origin;                   // 世界坐标系原点
//    Point3D baseline_center;                // 基线中心点
//    double baseline_length;                 // 基线长度
//    bool show_coordinate_frames;            // 是否显示坐标系
//    bool show_connections;                  // 是否显示连线
//
//public:
//    // 构造函数
//    Visualization3D(const string& window_name = "3D Stereo Visualization");
//
//    // 析构函数
//    ~Visualization3D();
//
//    // 设置基线参数
//    void setBaseline(const Point3D& center, double length);
//
//    // 添加相机
//    void addCamera(const CameraParams& camera);
//
//    // 添加器械
//    void addInstrument(const Instrument& instrument);
//
//    // 添加器械上的点
//    void addInstrumentPoint(const string& instrument_name, const Point3D& point);
//
//    // 设置世界坐标系原点
//    void setWorldOrigin(const Point3D& origin);
//
//    // 显示/隐藏坐标系
//    void showCoordinateFrames(bool show);
//
//    // 显示/隐藏器械连线
//    void showConnections(bool show);
//
//    // 更新显示
//    void updateDisplay();
//
//    // 显示窗口并等待用户交互
//    void show();
//
//    // 保存当前视图为图片
//    void saveView(const string& filename);
//
//    // 清除所有数据
//    void clear();
//
//private:
//    // 绘制坐标系
//    void drawCoordinateFrame(const Point3D& origin, const string& name,
//        double axis_length = 30.0, const Scalar& color = Scalar(255, 255, 255));
//
//    // 绘制相机
//    void drawCamera(const CameraParams& camera);
//
//    // 绘制器械
//    void drawInstrument(const Instrument& instrument);
//
//    // 绘制基线
//    void drawBaseline();
//
//    // 生成唯一的widget名称
//    string generateWidgetName(const string& prefix);
//
//    static int widget_counter;  // 用于生成唯一名称的计数器
//};
//
//#endif // 3D_VISUALIZATION_H