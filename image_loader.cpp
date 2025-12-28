// image_loader.cpp
#include "image_loader.h"
#include <iostream>

using namespace std;
using namespace cv;

bool loadImages(const string& left_img_path, const string& right_img_path, Mat& img_left, Mat& img_right) {
    img_left = imread(left_img_path);
    img_right = imread(right_img_path);

    if (img_left.empty() || img_right.empty()) {
        cerr << "无法加载图像，请检查路径" << endl;
        return false;
    }
    return true;
}
