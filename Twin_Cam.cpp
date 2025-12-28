#include "Twin_Cam.h"

StereoCameraSync::StereoCameraSync() :leftCamera_handle_(nullptr), rightCamera_handle_(nullptr), m_pImageCallback(nullptr), leftIndex(0),rightIndex(0), m_LeftThreadFlag(false), m_RightThreadFlag(false), MVMutex()
{
    LeftSaveFileParam.enImageType = MV_Image_Png;
    LeftSaveFileParam.iMethodValue = 3;
    LeftSaveFileParam.pcImagePath = new char[256];
    RightSaveFileParam.enImageType = MV_Image_Png;
    RightSaveFileParam.iMethodValue = 3;
    RightSaveFileParam.pcImagePath = new char[256];

	memset(&deviceList_, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
}

StereoCameraSync::~StereoCameraSync()
{

    if (leftCamera_handle_&& rightCamera_handle_)
    {
        this->CloseCameras();
        MV_CC_DestroyHandle(leftCamera_handle_);
        MV_CC_DestroyHandle(rightCamera_handle_);
        rightCamera_handle_ = nullptr;
        leftCamera_handle_ = nullptr;
    }
    MV_CC_Finalize();
}

bool StereoCameraSync::Initialize()
{
    int nRet = MV_CC_Initialize();
    if (MV_OK != nRet)
    {
        printf("Initialize SDK fail! nRet [0x%x]\n", nRet);
        return false;
    }
    return true;
}

bool StereoCameraSync::EnumDevices()
{
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE | MV_GENTL_CAMERALINK_DEVICE | MV_GENTL_CXP_DEVICE | MV_GENTL_XOF_DEVICE, &deviceList_);
    if (MV_OK != nRet)
    {
        printf("Enum Devices fail! nRet [0x%x]\n", nRet);
        return false;
    }

    if (deviceList_.nDeviceNum == 0)
    {
        printf("Find No Devices!\n");
        return false;
    }

    if (deviceList_.nDeviceNum == 1) printf("Only one Devices!\n");

    if (deviceList_.nDeviceNum == 2)
    {
        printf("TWO Camera  is ready\n");

        unsigned char* device_0 = deviceList_.pDeviceInfo[0]->SpecialInfo.stUsb3VInfo.chSerialNumber;
        unsigned char* device_1 = deviceList_.pDeviceInfo[1]->SpecialInfo.stUsb3VInfo.chSerialNumber;

        if (strcmp(reinterpret_cast<char*>(device_0), "DA6731323") == 0) //这是左相机序列号    右侧为：DA6731328
        {
			leftIndex = 1;
			rightIndex = 0;
        }
		else if (strcmp(reinterpret_cast<char*>(device_1), "DA6731323") == 0)
        {
            leftIndex = 0;
            rightIndex = 1;
		}
        else
        {
            printf("Please check the Serial Number of Cameras\n");
            return false;
        }
    }
    return true;
}

bool StereoCameraSync::CreateHandles()
{

    if (1 >= deviceList_.nDeviceNum)
        return false;

    int nRet1 = MV_CC_CreateHandle(&leftCamera_handle_, deviceList_.pDeviceInfo[leftIndex]);
    int nRet2 = MV_CC_CreateHandle(&rightCamera_handle_, deviceList_.pDeviceInfo[rightIndex]);
    if (nRet1||nRet2)
    {
        printf("Create LeftHandle error! nRet [0x%x]\n", nRet1);
        printf("Create RightHandle error! nRet [0x%x]\n", nRet2);
        rightCamera_handle_ = nullptr;
        leftCamera_handle_ = nullptr;
        return false;
    }
    return true;
}

bool StereoCameraSync::OpenCameras()
{
    if (!(rightCamera_handle_ && leftCamera_handle_))
    {
        return false;
    }

    int nRet1 = MV_CC_OpenDevice(rightCamera_handle_);
    int nRet2 = MV_CC_OpenDevice(leftCamera_handle_);
    if (nRet1 || nRet2)
    {
        printf("Open RightDevice error! nRet [0x%x]\n", nRet1);
        printf("Open LeftDevice error! nRet [0x%x]\n", nRet2);
        return false;
    }

    // 仅针对GigE相机设置最佳包大小
    /*
    if (deviceList_.pDeviceInfo[0]->nTLayerType == MV_GIGE_DEVICE)
    {
        int nPacketSize = MV_CC_GetOptimalPacketSize(handle_);
        if (nPacketSize > 0)
        {
            MV_CC_SetIntValueEx(handle_, "GevSCPSPacketSize", nPacketSize);
        }
    }
    */
    // 设置触发模式为off
    bool bEnable = true;

    if (MV_CC_SetBoolValue(rightCamera_handle_, "AcquisitionFrameRateEnable", bEnable) != MV_OK || MV_CC_SetBoolValue(leftCamera_handle_, "AcquisitionFrameRateEnable", bEnable) != MV_OK) {
        std::cout << "开启帧率控制失败" << std::endl;
        return false;
    }
   
    // 步骤4：设置目标帧率
    if (MV_CC_SetFloatValue(rightCamera_handle_, "AcquisitionFrameRate", 20) != MV_OK || MV_CC_SetFloatValue(leftCamera_handle_, "AcquisitionFrameRate", 20) != MV_OK) {
        std::cout << "设置帧率失败" << std::endl;
        return false;
    }

    MV_CC_SetEnumValue(rightCamera_handle_, "TriggerMode", MV_TRIGGER_MODE_OFF);
    MV_CC_SetEnumValue(leftCamera_handle_, "TriggerMode", MV_TRIGGER_MODE_OFF);
    RegisterExceptionCallBack();

    return true;
}

void StereoCameraSync::ReStartCameras(void* handle_ ,bool RightOrLeft)
{
    printf("重启取流\n");


    //if(!EnumDevices())  return;
    if (RightOrLeft == 0)
    {
        MV_CC_StopGrabbing(leftCamera_handle_);
        //MV_CC_SetGrabStrategy(leftCamera_handle_, MV_GrabStrategy_LatestImagesOnly);
        //MV_CC_CloseDevice(leftCamera_handle_);
        //MV_CC_DestroyHandle(leftCamera_handle_);
        //leftCamera_handle_ = nullptr;
        //int nRet1 = MV_CC_CreateHandle(&leftCamera_handle_, deviceList_.pDeviceInfo[leftIndex]);
        //int nRet2 = MV_CC_OpenDevice(leftCamera_handle_);
        printf("左相机重新取流开始\n");
        int nRet3 = MV_CC_StartGrabbing(leftCamera_handle_);
    }
    else
    {
        MV_CC_StopGrabbing(rightCamera_handle_);
        //MV_CC_SetGrabStrategy(rightCamera_handle_, MV_GrabStrategy_LatestImagesOnly);
        printf("右相机重新取流开始\n");
        //MV_CC_CloseDevice(rightCamera_handle_);
        //MV_CC_DestroyHandle(rightCamera_handle_);
        //leftCamera_handle_ = nullptr;
        //int nRet1 = MV_CC_CreateHandle(&handle_, deviceList_.pDeviceInfo[rightIndex]);
        //printf("Open RightDevice error! nRet [0x%x]\n", nRet1);
        //int nRet2 = MV_CC_OpenDevice(handle_);
       // printf("Open RightDevice error! nRet [0x%x]\n", nRet2);

        int nRet3 = MV_CC_StartGrabbing(handle_);
        //printf("Open RightDevice error! nRet [0x%x]\n", nRet3);
    }

}
bool StereoCameraSync::CloseCameras()
{
    if (!(rightCamera_handle_ && leftCamera_handle_)) return false;
    MV_CC_CloseDevice(rightCamera_handle_);
    MV_CC_CloseDevice(leftCamera_handle_);
    return true;
}

bool StereoCameraSync::RegisterExceptionCallBack()
{
    if (!(rightCamera_handle_ && leftCamera_handle_)) return false;
    MV_CC_RegisterExceptionCallBack(rightCamera_handle_, rightCameraExceptionCallBack, this);
    MV_CC_RegisterExceptionCallBack(leftCamera_handle_, leftCameraExceptionCallBack, this);
    return true;
}

bool StereoCameraSync::StartSyncGrabbing()
{
    //stereoCallback_->SetFlag();
    if (!(rightCamera_handle_ && leftCamera_handle_)) return false;
    int nRet3 = MV_CC_SetGrabStrategy(rightCamera_handle_, MV_GrabStrategy_LatestImagesOnly);
    int nRet4 = MV_CC_SetGrabStrategy(leftCamera_handle_, MV_GrabStrategy_LatestImagesOnly);
    
    //MV_CC_RegisterImageCallBackEx(rightCamera_handle_, RightImageCallbackEx, stereoCallback_);//直接传递指针
    //MV_CC_RegisterImageCallBackEx(leftCamera_handle_, LeftImageCallbackEx, stereoCallback_);
    
    MV_CC_SetImageNodeNum(rightCamera_handle_, 10);
    MV_CC_SetImageNodeNum(leftCamera_handle_, 10);
    
    if (nRet3 || nRet4)
    {
        printf("Start RightSetGrabStrateg error! nRet [0x%x]\n", nRet3);
        printf("Start LeftSetGrabStrateg error! nRet [0x%x]\n", nRet4);
        return false;
    }
    
    //MV_CC_SetOutputQueueSize(rightCamera_handle_, 1);
    //MV_CC_SetOutputQueueSize(leftCamera_handle_,1);
    
    int nRet1 = MV_CC_StartGrabbing(rightCamera_handle_);
    int nRet2 = MV_CC_StartGrabbing(leftCamera_handle_);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    stereoCallback_ = new Twin_CallBack(SynImageCallbackEx, m_pImageCallback, this);
    /*
    m_RightThreadFlag.store(true);
    m_LeftThreadFlag.store(true);
    leftImageThread = std::thread(&StereoCameraSync::GetLeftImageFrame, this);
    rightImageThread = std::thread(&StereoCameraSync::GetRightImageFrame, this);
    */
    
    if (nRet1||nRet2)
    {
        printf("Start RightGrabbing error! nRet [0x%x]\n", nRet1);
        printf("Start LeftGrabbing error! nRet [0x%x]\n", nRet2);
        return false;
    }
    
    
    return true;
}
# if 0
void StereoCameraSync::GetLeftImageFrame()
{
    MV_CC_StartGrabbing(leftCamera_handle_);
    this_thread::sleep_for(chrono::seconds(1));
    static MV_FRAME_OUT stOutLeftFrame;
    stOutLeftFrame.pBufAddr = new unsigned char[3072 * 2048 * 1];
    while (m_LeftThreadFlag.load())
    {
        int nRetLeft = MV_CC_GetImageBuffer(leftCamera_handle_, &stOutLeftFrame, 10);
        //int nRetLeft = MV_CC_GetOneFrameTimeout(this->leftCamera_handle_, stOutLeftFrame.pBufAddr, 3072 * 2048 * 1, &stOutLeftFrame.stFrameInfo, 100);
        
        if (!nRetLeft)
        {
            MV_FRAME_OUT MVFrame;
            MVFrame.pBufAddr = new unsigned char[3072 * 2048 * 1];
            std::memcpy(MVFrame.pBufAddr, stOutLeftFrame.pBufAddr, 3072 * 2048 * 1);
            MVFrame.stFrameInfo = stOutLeftFrame.stFrameInfo;
            stereoCallback_->left_queue.Push(MVFrame);
            stereoCallback_->OnQueue1NewFrame(stOutLeftFrame.stFrameInfo.nFrameNum);
            printf("Get Left Frame: Width[%d], Height[%d], FrameNum[%d]\n",
                stOutLeftFrame.stFrameInfo.nWidth, stOutLeftFrame.stFrameInfo.nHeight, stOutLeftFrame.stFrameInfo.nFrameNum);
        }
        else
        {
            //delete[] stOutLeftFrame.pBufAddr;
            //stOutLeftFrame.pBufAddr = nullptr;
            char buffer[100];
            snprintf(buffer, sizeof(buffer), "Left Camera is Died nRet [0x%x]", nRetLeft);
            LogPrint::PrintString(buffer);
            printf("%s\n", buffer);
            this->ReStartCameras(this->leftCamera_handle_, 0);
           
        }
    }
    
}

void StereoCameraSync::GetRightImageFrame()
{
    MV_CC_StartGrabbing(rightCamera_handle_);
    this_thread::sleep_for(chrono::seconds(1));
    static MV_FRAME_OUT stOutRightFrame;
    stOutRightFrame.pBufAddr = new unsigned char[3072 * 2048 * 1];
    while (m_RightThreadFlag.load())
    {
        int nRetRight = MV_CC_GetImageBuffer(rightCamera_handle_, &stOutRightFrame, 10);
        //int nRetRight = MV_CC_GetOneFrameTimeout(this->rightCamera_handle_, stOutRightFrame.pBufAddr, 3072 * 2048 * 1, &stOutRightFrame.stFrameInfo, 100);
        
     
        if (!nRetRight)
        {
            MV_FRAME_OUT MVFrame;
            MVFrame.pBufAddr = new unsigned char[3072 * 2048 * 1];
            std::memcpy(MVFrame.pBufAddr, stOutRightFrame.pBufAddr, 3072 * 2048 * 1);
            MVFrame.stFrameInfo = stOutRightFrame.stFrameInfo;
            stereoCallback_->right_queue.Push(MVFrame);

            stereoCallback_->OnQueue2NewFrame(stOutRightFrame.stFrameInfo.nFrameNum);
            printf("Get Right Frame: Width[%d], Height[%d], FrameNum[%d]\n",
                stOutRightFrame.stFrameInfo.nWidth, stOutRightFrame.stFrameInfo.nHeight, stOutRightFrame.stFrameInfo.nFrameNum);
        }
        else
        {
            //delete[] stOutRightFrame.pBufAddr;
           // stOutRightFrame.pBufAddr = nullptr;
            char buffer[100];
            snprintf(buffer, sizeof(buffer), "Right Camera is Died nRet [0x%x]", nRetRight);
            LogPrint::PrintString(buffer);
            printf("%s\n", buffer);
            this->ReStartCameras(this->rightCamera_handle_, 1);
        }
        //MV_CC_StopGrabbing(rightCamera_handle_);
    }
    
}
#endif 

#if 0
void __stdcall StereoCameraSync::RightImageCallbackEx(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser)
{
    if (pFrameInfo)
    {
        Twin_CallBack* pCallback = static_cast<Twin_CallBack*>(pUser);
        ImageFrame image;

        image.nFrameNum = pFrameInfo->nFrameNum;
        image.nHeight = pFrameInfo->nHeight;
        image.nWidth = pFrameInfo->nWidth;
        image.nDataSize = image.nHeight * image.nWidth * 1;
        //深拷贝
        if (image.nDataSize > 0 && pData != nullptr) {
            image.pData = new unsigned char[image.nDataSize];  // 分配新内存
            memcpy(image.pData, pData, image.nDataSize);       // 复制数据内容
        }
        else {
            image.pData = nullptr;
        }
        image.nHostTimeStamp = pFrameInfo->nHostTimeStamp;
        pCallback->right_queue.Push(image);
        pCallback->OnQueue2NewFrame(pFrameInfo->nFrameNum);
        printf("Get Right Frame: Width[%d], Height[%d], FrameNum[%d]\n",
            pFrameInfo->nWidth, pFrameInfo->nHeight, pFrameInfo->nFrameNum);
    }
}
#endif


void __stdcall StereoCameraSync::rightCameraExceptionCallBack(unsigned int nMsgType, void* pUser)
{
    if (nMsgType == MV_EXCEPTION_DEV_DISCONNECT)
    {
        StereoCameraSync* ptr = static_cast<StereoCameraSync*>(pUser);
        printf("rightCamera discnnect\n**************************************");
        //可以进行异常处理，比如关闭相机，销毁句柄，重新枚举相机，打开相机等;
        ptr->StopSyncGrabbing();
    }
    printf("\n****************************************************\n");
}

#if 0
void __stdcall StereoCameraSync::LeftImageCallbackEx(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser)
{
    if (pFrameInfo)
    {
        Twin_CallBack* pCallback = static_cast<Twin_CallBack*>(pUser);
        ImageFrame image;
        image.nFrameNum = pFrameInfo->nFrameNum;
        image.nHeight = pFrameInfo->nHeight;
        image.nWidth = pFrameInfo->nWidth;
        image.nDataSize = image.nHeight * image.nWidth * 1;
        //深拷贝
        if (image.nDataSize > 0 && pData != nullptr) {
            image.pData = new unsigned char[image.nDataSize];  // 分配新内存
            memcpy(image.pData, pData, image.nDataSize);       // 复制数据内容
        }
        else {
            image.pData = nullptr;
        }
        
        image.nHostTimeStamp = pFrameInfo->nHostTimeStamp;
        pCallback->left_queue.Push(image);
        pCallback->OnQueue1NewFrame(pFrameInfo->nFrameNum);
        printf("Get Left Frame: Width[%d], Height[%d], FrameNum[%d]\n",
           pFrameInfo->nWidth, pFrameInfo->nHeight, pFrameInfo->nFrameNum);
    }
}
#endif

void __stdcall StereoCameraSync::leftCameraExceptionCallBack(unsigned int nMsgType, void* pUser)
{
    if (nMsgType == MV_EXCEPTION_DEV_DISCONNECT)
    {
        StereoCameraSync* ptr = static_cast<StereoCameraSync*>(pUser);
        printf("\nleftCamera discnnect\n**************************************");
        //可以进行异常处理，比如关闭相机，销毁句柄，重新枚举相机，打开相机等;
        ptr->StopSyncGrabbing();
    }
    printf("\n****************************************************\n");
}

void StereoCameraSync::RegisterManager(IImageDataCallback* pImageCallback)
{
    m_pImageCallback = pImageCallback;
}

void __stdcall StereoCameraSync::SynImageCallbackEx(StereoCameraSync* ptr, IImageDataCallback* m_pImageCallback)
{
    // 内置Marager接口函数

    static MV_FRAME_OUT stOutLeftFrame = {0};
    static MV_FRAME_OUT stOutRightFrame = {0};
    
    static unsigned char* pDataLeft = new unsigned char[2000 * 2000 * 2];
    static unsigned char* pDataRight = new unsigned char[2000 * 2000 * 2];
    //MV_CC_StartGrabbing(ptr->leftCamera_handle_);
    int nRetLeft = MV_CC_GetImageBuffer(ptr->leftCamera_handle_, &stOutLeftFrame, 100);
    //int nRetLeft = MV_CC_GetOneFrameTimeout(ptr->leftCamera_handle_, pDataLeft, 2000 * 2000 * 2, &stOutLeftFrame.stFrameInfo,10000);
    //MV_CC_StopGrabbing(ptr->leftCamera_handle_);


    //MV_CC_StartGrabbing(ptr->rightCamera_handle_);
    int nRetRight = MV_CC_GetImageBuffer(ptr->rightCamera_handle_, &stOutRightFrame, 100);
    //int nRetRight = MV_CC_GetOneFrameTimeout(ptr->rightCamera_handle_, pDataRight, 2000 * 2000 * 2, &stOutRightFrame.stFrameInfo, 10000);
   // MV_CC_StopGrabbing(ptr->rightCamera_handle_);
      
    static bool SavePathInitFlag = false;

    if (!(nRetLeft || nRetRight))
    {
        if (!SavePathInitFlag)
        {
            ptr->LeftSaveFileParam.enPixelType = stOutRightFrame.stFrameInfo.enPixelType;
            ptr->LeftSaveFileParam.nDataLen = stOutRightFrame.stFrameInfo.nHeight * stOutRightFrame.stFrameInfo.nWidth;
            ptr->LeftSaveFileParam.nWidth = stOutRightFrame.stFrameInfo.nWidth;
            ptr->LeftSaveFileParam.nHeight = stOutRightFrame.stFrameInfo.nHeight;
            ptr->LeftSaveFileParam.pData = new unsigned char[ptr->LeftSaveFileParam.nDataLen];

            ptr->RightSaveFileParam.enPixelType = stOutRightFrame.stFrameInfo.enPixelType;
            ptr->RightSaveFileParam.nDataLen = stOutRightFrame.stFrameInfo.nHeight * stOutRightFrame.stFrameInfo.nWidth;
            ptr->RightSaveFileParam.nWidth = stOutRightFrame.stFrameInfo.nWidth;
            ptr->RightSaveFileParam.nHeight = stOutRightFrame.stFrameInfo.nHeight;
            ptr->RightSaveFileParam.pData = new unsigned char[ptr->RightSaveFileParam.nDataLen];
            SavePathInitFlag = true;
        }

        ImagePair Pair;
        cv::Mat leftImage(stOutLeftFrame.stFrameInfo.nHeight, stOutLeftFrame.stFrameInfo.nWidth, CV_8UC1, stOutLeftFrame.pBufAddr);
        cv::Mat rightImage(stOutRightFrame.stFrameInfo.nHeight, stOutRightFrame.stFrameInfo.nWidth, CV_8UC1, stOutRightFrame.pBufAddr);


        Pair.leftImage = leftImage.clone();
        Pair.rightImage = rightImage.clone();
        Pair.timeStamp = stOutLeftFrame.stFrameInfo.nHostTimeStamp;
        Pair.timeInterval = stOutLeftFrame.stFrameInfo.nHostTimeStamp - stOutRightFrame.stFrameInfo.nHostTimeStamp;
        Pair.leftFrameNum = stOutLeftFrame.stFrameInfo.nFrameNum;
        Pair.rightFrameNum = stOutRightFrame.stFrameInfo.nFrameNum;

        Pair.DevTimeStampLeft = (static_cast<int64_t>(stOutLeftFrame.stFrameInfo.nDevTimeStampHigh) << 32)
            | static_cast<int64_t>(stOutLeftFrame.stFrameInfo.nDevTimeStampLow);

        Pair.DevTimeStampRight = (static_cast<int64_t>(stOutRightFrame.stFrameInfo.nDevTimeStampHigh) << 32)
            | static_cast<int64_t>(stOutRightFrame.stFrameInfo.nDevTimeStampLow);
        
        // 创建窗口（只需要创建一次）
        static bool windowsCreated = false;
        static int64_t lastShowTime = 0;
        const int MIN_FRAME_INTERVAL = 33; // 约30帧/秒 (1000ms/30≈33ms)

        if (!windowsCreated) {
            cv::namedWindow("Left Image", cv::WINDOW_NORMAL);
            cv::namedWindow("Right Image", cv::WINDOW_NORMAL);
            // 可以设置窗口初始大小
            cv::resizeWindow("Left Image", 640, 480);
            cv::resizeWindow("Right Image", 640, 480);
            windowsCreated = true;
        }

        // 控制帧率，避免刷新过快
        int64_t currentTime = cv::getTickCount() / cv::getTickFrequency() * 1000; // 毫秒
        if (currentTime - lastShowTime >= MIN_FRAME_INTERVAL) {
            // 显示图像
            cv::imshow("Left Image", Pair.leftImage);
            cv::imshow("Right Image", Pair.rightImage);
            lastShowTime = currentTime;
        }

        // 处理窗口事件，等待1ms确保窗口响应
        char key = cv::waitKey(1);
        if (key == 27) { // ESC键退出
            cv::destroyAllWindows();
            // 可以在这里添加程序退出逻辑
        }


        ptr->MVMutex._Lock();
        memcpy(ptr->RightSaveFileParam.pData, stOutRightFrame.pBufAddr, stOutLeftFrame.stFrameInfo.nHeight * stOutRightFrame.stFrameInfo.nWidth);
        memcpy(ptr->LeftSaveFileParam.pData, stOutLeftFrame.pBufAddr, stOutLeftFrame.stFrameInfo.nHeight * stOutRightFrame.stFrameInfo.nWidth);
        ptr->MVMutex._Unlock();

        MV_CC_FreeImageBuffer(ptr->rightCamera_handle_, &stOutRightFrame);
        MV_CC_FreeImageBuffer(ptr->leftCamera_handle_, &stOutLeftFrame);
        //MV_CC_ClearImageBuffer(ptr->rightCamera_handle_);
        //MV_CC_ClearImageBuffer(ptr->leftCamera_handle_);
        m_pImageCallback->OnImageData(Pair);
        printf("leftFrameNum:%d\trightFrameNum:%d\n", Pair.leftFrameNum, Pair.rightFrameNum);
    }
    else if(!nRetLeft)
    {
        MV_CC_FreeImageBuffer(ptr->leftCamera_handle_, &stOutLeftFrame);
        
        char buffer[100];
        snprintf(buffer,sizeof(buffer), "Right Camera is Died nRet [0x%x]", nRetRight);
        LogPrint::PrintString(buffer);
        printf("%s\n", buffer);
        ptr->ReStartCameras(ptr->rightCamera_handle_, 1);

        //std::ofstream outFile("3d_points.txt", std::ios::app);

        return;
    }
	else if (!nRetRight)
	{
		MV_CC_FreeImageBuffer(ptr->rightCamera_handle_, &stOutRightFrame);
        char buffer[100];
        snprintf(buffer, sizeof(buffer), "Left Camera is Died nRet [0x%x]", nRetLeft);
        LogPrint::PrintString(buffer);
        printf("%s\n", buffer);
        ptr->ReStartCameras(ptr->leftCamera_handle_, 0);
		return;
    }
    else
    {
        LogPrint::PrintString("Two camera died");

        printf("Two camera died \n");
		return;
    }

}

#if 0
void __stdcall StereoCameraSync::SynImageCallbackEx(MV_FRAME_OUT& frameLeft, MV_FRAME_OUT& frameRight,IImageDataCallback* m_pImageCallback)
{
    static ImagePair Pair;
    cv::Mat leftImage(frameLeft.stFrameInfo.nHeight, frameLeft.stFrameInfo.nWidth, CV_8UC1, frameLeft.pBufAddr);
    cv::Mat rightImage(frameRight.stFrameInfo.nHeight, frameRight.stFrameInfo.nWidth, CV_8UC1, frameRight.pBufAddr);


    Pair.leftImage = leftImage.clone();
    Pair.rightImage = rightImage.clone();
    Pair.timeStamp = frameLeft.stFrameInfo.nHostTimeStamp;
    Pair.timeInterval = frameLeft.stFrameInfo.nHostTimeStamp - frameRight.stFrameInfo.nHostTimeStamp;

    Pair.DevTimeStampLeft = (static_cast<int64_t>(frameLeft.stFrameInfo.nDevTimeStampHigh) << 32)
        | static_cast<int64_t>(frameLeft.stFrameInfo.nDevTimeStampLow);

    Pair.DevTimeStampRight = (static_cast<int64_t>(frameRight.stFrameInfo.nDevTimeStampHigh) << 32)
        | static_cast<int64_t>(frameRight.stFrameInfo.nDevTimeStampLow);

    if (frameLeft.pBufAddr != nullptr) {
        delete[] frameLeft.pBufAddr;  // 注意用 delete[] 匹配数组分配
        frameLeft.pBufAddr = nullptr; // 避免野指针
    }
    if (frameRight.pBufAddr != nullptr) {
        delete[] frameRight.pBufAddr;  // 
        frameRight.pBufAddr = nullptr; // 
    }
    // 创建窗口（只需要创建一次）
    static bool windowsCreated = false;
    static int64_t lastShowTime = 0;
    const int MIN_FRAME_INTERVAL = 33; // 约30帧/秒 (1000ms/30≈33ms)

    if (!windowsCreated) {
        cv::namedWindow("Left Image", cv::WINDOW_NORMAL);
        cv::namedWindow("Right Image", cv::WINDOW_NORMAL);
        // 可以设置窗口初始大小
        cv::resizeWindow("Left Image", 640, 480);
        cv::resizeWindow("Right Image", 640, 480);
        windowsCreated = true;
    }

    // 控制帧率，避免刷新过快
    int64_t currentTime = cv::getTickCount() / cv::getTickFrequency() * 1000; // 毫秒
    if (currentTime - lastShowTime >= MIN_FRAME_INTERVAL) {
        // 显示图像
        cv::imshow("Left Image", Pair.leftImage);
        cv::imshow("Right Image", Pair.rightImage);
        lastShowTime = currentTime;
    }

    // 处理窗口事件，等待1ms确保窗口响应
    char key = cv::waitKey(1);
    if (key == 27) { // ESC键退出
        cv::destroyAllWindows();
        // 可以在这里添加程序退出逻辑
    }
    Pair.leftFrameNum = frameLeft.stFrameInfo.nFrameNum;
    Pair.rightFrameNum = frameRight.stFrameInfo.nFrameNum;
    m_pImageCallback->OnImageData(Pair);
    printf("leftFrameNum:%d\trightFrameNum:%d\n", Pair.leftFrameNum, Pair.rightFrameNum);
}

#endif

bool StereoCameraSync::StopSyncGrabbing()
{
    if (rightCamera_handle_ || leftCamera_handle_) return false;
    m_RightThreadFlag.store(false);
    m_LeftThreadFlag.store(false);
    if (leftImageThread.joinable())
    {
        leftImageThread.join();
    }
    if (rightImageThread.joinable())
    {
        rightImageThread.join();
    }
    MV_CC_StopGrabbing(rightCamera_handle_);
    MV_CC_StopGrabbing(leftCamera_handle_);
    stereoCallback_->ResetFlag();
    delete stereoCallback_;
    stereoCallback_ = nullptr; 
    MV_CC_RegisterImageCallBackEx(rightCamera_handle_, NULL, NULL);
    MV_CC_RegisterImageCallBackEx(leftCamera_handle_, NULL, NULL);
    return true;
}

void StereoCameraSync::Finalize()
{
    MV_CC_Finalize();
}


bool StereoCameraSync::SaveImageToPath()
{
    
    MVMutex._Lock();
    int nRet1 = MV_CC_SaveImageToFileEx(rightCamera_handle_, &RightSaveFileParam);
    int nRet2 = MV_CC_SaveImageToFileEx(leftCamera_handle_, &LeftSaveFileParam);
    MVMutex._Unlock();
    if (nRet1 == 0 && nRet2 == 0 && LeftSaveFileParam.pData != nullptr && RightSaveFileParam.pData != nullptr)
    {
        return true;
    }
    return false;
}

bool StereoCameraSync::SetSavePath(std::string PathString)
{
    std::string PathStringLeft = "Image Library/" + PathString + "_Left" + ".png";
    std::string PathStringRight = "Image Library/" +PathString + "_Right"+".png";


    strcpy_s(LeftSaveFileParam.pcImagePath,256, PathStringLeft.c_str());
    strcpy_s(RightSaveFileParam.pcImagePath,256 ,PathStringRight.c_str());
    return true;
 }

std::array<cv::Mat, 2> StereoCameraSync::GetSaveImage()
{
    MVMutex._Lock();
    std::array<cv::Mat, 2> matArray;
    cv::Mat leftSaveImage(LeftSaveFileParam.nHeight, LeftSaveFileParam.nWidth, CV_8UC1);
    memcpy(leftSaveImage.data, LeftSaveFileParam.pData, LeftSaveFileParam.nHeight * LeftSaveFileParam.nWidth);

    cv::Mat rightSaveImage(RightSaveFileParam.nHeight, RightSaveFileParam.nWidth, CV_8UC1);
    memcpy(rightSaveImage.data, RightSaveFileParam.pData, RightSaveFileParam.nHeight * RightSaveFileParam.nWidth);
    MVMutex._Unlock();
    matArray[0] = leftSaveImage;
    matArray[1] = rightSaveImage;
    return matArray;
}