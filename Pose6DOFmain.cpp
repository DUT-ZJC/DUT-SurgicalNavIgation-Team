/*
#include "Pose6DOF.h"
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    // ========== 步骤1: 定义器械模型点（在器械坐标系下的相对位置）==========
    // 例如：器械上4个标记点的位置关系（单位：mm）
    vector<Point3f> modelPoints = {
        Point3f(0, 0, 0),        // 参考点1
        Point3f(100, 0, 0),      // 参考点2
        Point3f(0, 100, 0),      // 参考点3
        Point3f(0, 0, 100)       // 参考点4
    };

    // 创建计算器（只需创建一次）
    Pose6DOFCalculator poseCalculator(modelPoints);


    // ========== 步骤2: 从你的双目视觉系统获取4个3D点 ==========
    // 假设你已经通过 DistanceCalculator::calculate3DPoint() 计算出了4个点
    vector<Point3f> measuredPoints(4);

    // 示例：从你的双目视觉程序中获取
    // measuredPoints[0] = distanceCalc.calculate3DPoint(pt_left1, pt_right1, focal, baseline, cx, cy);
    // measuredPoints[1] = distanceCalc.calculate3DPoint(pt_left2, pt_right2, focal, baseline, cx, cy);
    // measuredPoints[2] = distanceCalc.calculate3DPoint(pt_left3, pt_right3, focal, baseline, cx, cy);
    // measuredPoints[3] = distanceCalc.calculate3DPoint(pt_left4, pt_right4, focal, baseline, cx, cy);

    // 这里用模拟数据演示
    measuredPoints[0] = Point3f(150.5, 200.3, 1000.2);
    measuredPoints[1] = Point3f(250.5, 200.3, 1000.2);
    measuredPoints[2] = Point3f(150.5, 300.3, 1000.2);
    measuredPoints[3] = Point3f(150.5, 200.3, 1100.2);


    // ========== 步骤3: 计算6自由度姿态 ==========
    Pose6DOF pose;
    if (poseCalculator.calculatePose(measuredPoints, pose)) {
        cout << "✓ 姿态计算成功！\n" << endl;
        cout << pose.toString() << endl;

        // 可以访问具体数值
        cout << "\n详细数据：" << endl;
        cout << "X = " << pose.position.x << " mm" << endl;
        cout << "Y = " << pose.position.y << " mm" << endl;
        cout << "Z = " << pose.position.z << " mm" << endl;
        cout << "Roll = " << pose.roll << "°" << endl;
        cout << "Pitch = " << pose.pitch << "°" << endl;
        cout << "Yaw = " << pose.yaw << "°" << endl;
    }
    else {
        cout << "✗ 姿态计算失败！" << endl;
    }

    return 0;
}


// ========== 在你的实际程序中集成的示例 ==========
/*
// 假设这是你的主循环
void processFrame() {
    // 1. 检测4个标记点的左右图像坐标
    Point2f leftPoints[4], rightPoints[4];
    // ... 你的点检测代码 ...

    // 2. 计算4个点的3D坐标
    vector<Point3f> measured3DPoints(4);
    for (int i = 0; i < 4; i++) {
        measured3DPoints[i] = distanceCalc.calculate3DPoint(
            leftPoints[i], rightPoints[i],
            focal_length_px, baseline_mm, cx, cy
        );
    }

    // 3. 计算6自由度姿态
    Pose6DOF currentPose;
    if (poseCalculator.calculatePose(measured3DPoints, currentPose)) {
        // 姿态计算成功，使用结果
        updateDisplay(currentPose);
        sendToRobot(currentPose);
    }
}
*/
