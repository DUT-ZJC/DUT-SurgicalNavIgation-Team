// image_drawer.cpp
#include "image_drawer.h"

using namespace cv;
using namespace std;

void drawDetectedBalls(Mat& img_left, Mat& img_right, const vector<Point2f>& pts_left, const vector<Point2f>& pts_right) {
    for (size_t i = 0; i < pts_left.size(); ++i) {
        circle(img_left, pts_left[i], 8, Scalar(0, 255, 0), 2);
        putText(img_left, to_string(i+1), pts_left[i] + Point2f(-30, -15), FONT_HERSHEY_SIMPLEX, 3.0, Scalar(0, 255, 0), 5);
    }

    for (size_t i = 0; i < pts_right.size(); ++i) {
        circle(img_right, pts_right[i], 8, Scalar(0, 0, 255), 2);
        putText(img_right, to_string(i+1), pts_right[i] + Point2f(-30, -15), FONT_HERSHEY_SIMPLEX, 3.0, Scalar(0, 0, 255), 5);
    }
    // putText：在图像上绘制文字。
    //to_string(i)：将当前反光球的索引转换为字符串，用作编号。
    //pts_left[i] + Point2f(10, -10)：文本的起始位置，将文本放置在反光球的右上方。
    //FONT_HERSHEY_SIMPLEX：使用的字体。
    //3.0：字体大小。
    //Scalar(0, 255, 0)：文本的颜色，绿色。
    //5：文本的线宽

}
