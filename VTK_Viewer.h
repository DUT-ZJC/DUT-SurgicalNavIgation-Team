#ifndef VTK_VIEWER_H
#define VTK_VIEWER_H

#include "Twin_Cam.h"
#include "Manager.h"
#include "Image_Tranlsate.h"
#include "GL_Image.hpp"

#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkAxesActor.h>
#include <vtkMultiBlockDataSet.h>
#include <vtkSTLReader.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkOCCTReader.h>
#include <QFileDialog>
#include <QMessageBox>
#include <QMenuBar>
#include <QFileInfo>
#include <stdexcept>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QComboBox>
#include <QSlider>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QCheckBox>
#include <vtkCellPicker.h>
#include <vtkTransform.h>
#include <vtkInteractorStyle.h>
#include <QStatusBar>

#include <QTimer>  // 需包含定时器头文件
#include <QMainWindow>
#include <QAction>
#include <QList>
#include <windows.h>
#include <stdio.h>
#include <iostream>
// VTK 相关头文件
#include <vtkSmartPointer.h>
#include <QVTKOpenGLNativeWidget.h>


// 前向声明
class QComboBox;
class QSlider;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QPushButton;
class QCheckBox;

// VTK 前向声明
class vtkPolyData;
class vtkActor;
class vtkAxesActor;
class vtkInteractorStyle;
class CustomInteractorStyle;

struct ModelInfo {
    vtkSmartPointer<vtkActor> actor;
    vtkSmartPointer<vtkAxesActor> axesActor;
    double initialPosition[3];
    double initialOrientation[3];
    double initialColor[3];
    bool showAxes;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

    Manager manager;
    // 新增方法：更新模型选择状态
    void updateModelSelection(vtkActor* actor);
    // 新增方法：更新坐标系位置
    void updateAxesPosition(vtkActor* actor);

    void createGrayImageWindows(); // 创建单通道图像窗口

    void showImage(bool falg);
    void saveImage();

public slots:
    // 主线程中实际执行VTK更新的槽函数
    void updateVtkWindow(const cv::Point3f& Position , float roll, float pitch, float yaw);
    //void onImageReceived(const QImage& leftImg, const QImage& rightImg); // 更新单通道图像
private:
    // 模型信息结构体
    
    void createRibbonMenu();
    void setupVTK();
    void setupControlPanel();
    void openModel();
    bool handlePlyFile(const QString& fileName, const QFileInfo& fileInfo);
    bool handleObjFile(const QString& fileName, const QFileInfo& fileInfo);
    bool handleStlFile(const QString& fileName, const QFileInfo& fileInfo);
    bool handleStepFile(const QString& fileName, const QFileInfo& fileInfo);
    void addModelToRenderer(vtkSmartPointer<vtkPolyData> polyData, const QString& modelName = "");

    template<typename T, size_t N>
    void copyArray(T(&dest)[N], const T* src)
    {
        for (size_t i = 0; i < N; i++) {
            dest[i] = src[i];
        }
    }

    //QPixmap grayMatToPixmap(const cv::Mat& grayMat);
    void onImgTimerTimeout();
private slots:
    void onModelSelected(int index);
    void onDeleteModel();
    void onToggleAxes(bool show);

private:
    QMenuBar* ribbonMenuBar;
    QStatusBar* m_statusBar;
    QMenu* fileMenu;
    QAction* openAction;
    QVTKOpenGLNativeWidget* vtkWidget;
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow;
    QComboBox* modelSelector;
    QPushButton* deleteButton;
    QPushButton* saveImageButton;
    QComboBox* saveFileName1Set;
    QComboBox* saveFileName2Set;
    QComboBox* saveFileName3Set;
    QCheckBox* showAxesCheckBox;
    QList<ModelInfo> modelActors;
    vtkActor* currentSelectedActor;

    QStringList Pathlist;
    // 新增成员：自定义交互器
    vtkSmartPointer<CustomInteractorStyle> customInteractorStyle;

    QWidget* imageWindow;  // 图像总窗口
    QWidget* modelViewWidget;
    QLabel* leftImgLabel;  // 左图显示
    QLabel* rightImgLabel; // 右图显示

    GLDualGrayImageWidget* dualGrayWidget;

    Image_Translate TranslateThread;
};

#endif // VTK_VIEWER_H
 
 