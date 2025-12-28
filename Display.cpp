#include "Display.h"

Display::Display() {}

Display::~Display() {}

void Display::showFrame(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, BallPosition ball)
{
    if (!pFrameInfo || !pData) return;

    int width = pFrameInfo->nWidth;
    int height = pFrameInfo->nHeight;

    cv::Mat image;

    // 根据像素格式转换到 OpenCV Mat
    if (pFrameInfo->enPixelType == PixelType_Gvsp_Mono8)
    {
        image = cv::Mat(height, width, CV_8UC1, pData);
        cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
    }
    else if (pFrameInfo->enPixelType == PixelType_Gvsp_RGB8_Packed)
    {
        image = cv::Mat(height, width, CV_8UC3, pData);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    }
    else
    {
        // 其他格式简单处理
        image = cv::Mat(height, width, CV_8UC1, pData);
        cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
    }

    // 如果检测到小球，在图像上画圆和坐标
    if (ball.valid)
    {
        cv::circle(image, cv::Point((int)ball.x, (int)ball.y), 10, cv::Scalar(0, 0, 255), 2);
        char text[64];
        sprintf_s(text, "(%.1f, %.1f)", ball.x, ball.y);
        cv::putText(image, text, cv::Point((int)ball.x + 15, (int)ball.y),
            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    }

    // 显示图像

    //cv::namedWindow("Camera View", cv::WINDOW_NORMAL);   // 创建可缩放窗口
    //cv::resizeWindow("Camera View", 800, 600);           // 设置窗口大小 800x600

    cv::imshow("Camera View", image);
    cv::waitKey(1); // 非阻塞刷新
}
