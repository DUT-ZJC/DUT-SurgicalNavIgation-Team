#pragma once
#include <opencv2/opencv.hpp>
#include "MvCameraControl.h"

// 坐标结构体
typedef struct {
    float x;
    float y;
    int valid;
} BallPosition;

class Display
{
public:
    Display();
    ~Display();
    void showFrame(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, BallPosition ball);
};
#pragma once
