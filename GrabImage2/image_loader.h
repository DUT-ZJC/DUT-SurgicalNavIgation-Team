// image_loader.h
#ifndef IMAGE_LOADER_H
#define IMAGE_LOADER_H

#include <opencv2/opencv.hpp>
#include <string>

using namespace cv;

// ¼ÓÔØ×óÓÒÍ¼ÏñµÄº¯Êı
bool loadImages(const std::string& left_img_path, const std::string& right_img_path, Mat& img_left, Mat& img_right);

#endif // IMAGE_LOADER_H
#pragma once
