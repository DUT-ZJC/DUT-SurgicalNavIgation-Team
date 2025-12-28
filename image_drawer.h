// image_drawer.h
#ifndef IMAGE_DRAWER_H
#define IMAGE_DRAWER_H

#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;

// 在图像上绘制标记的小球
void drawDetectedBalls(Mat& img_left, Mat& img_right, const std::vector<Point2f>& pts_left, const std::vector<Point2f>& pts_right);

#endif // IMAGE_DRAWER_H
#pragma once
