#include "VTK_Viewer.h"
#include <chrono>

// 自定义交互器类
class CustomInteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
    static CustomInteractorStyle* New();
    vtkTypeMacro(CustomInteractorStyle, vtkInteractorStyleTrackballCamera);

    CustomInteractorStyle()
        : MainWindow(nullptr), SelectedActor(nullptr)
    {
        LastPickedProperty = vtkProperty::New();
    }

    virtual ~CustomInteractorStyle() {
        if (LastPickedProperty) {
            LastPickedProperty->Delete();
        }
    }

    void SetMainWindow(MainWindow* window) {
        MainWindow = window;
    }

    virtual void OnRightButtonDown() override {
        int* clickPos = this->GetInteractor()->GetEventPosition();

        vtkSmartPointer<vtkCellPicker> picker = vtkSmartPointer<vtkCellPicker>::New();
        picker->SetTolerance(0.0005);

        picker->Pick(clickPos[0], clickPos[1], 0, this->GetDefaultRenderer());
        vtkActor* actor = picker->GetActor();

        if (actor) {
            // 检查是否是坐标系actor，如果是则忽略
            if (dynamic_cast<vtkAxesActor*>(actor) != nullptr) {
                vtkInteractorStyleTrackballCamera::OnRightButtonDown();
                return;
            }

            // 切换选择状态
            if (SelectedActor == actor) {
                // 取消选择
                actor->GetProperty()->DeepCopy(LastPickedProperty);
                SelectedActor = nullptr;
                LastPickedProperty->SetOpacity(1.0);

           
            }
            else {
                // 选择新actor
                if (SelectedActor) {
                    // 恢复之前选择的actor
                    SelectedActor->GetProperty()->DeepCopy(LastPickedProperty);
                }

                // 保存当前actor的属性
                LastPickedProperty->DeepCopy(actor->GetProperty());

                // 设置选中效果：黄色、半透明
                actor->GetProperty()->SetColor(1.0, 1.0, 0.0); // 黄色
                actor->GetProperty()->SetOpacity(0.5); // 半透明

                SelectedActor = actor;
                // 通知主窗口更新UI
                if (MainWindow) {
                    MainWindow->updateModelSelection(actor);
                }
            }

            this->GetInteractor()->Render();
        }
        else {
            // 如果没有选中任何actor，调用父类方法
            vtkInteractorStyleTrackballCamera::OnRightButtonDown();
        }
    }

    

private:
    MainWindow* MainWindow;
    vtkActor* SelectedActor;
    vtkProperty* LastPickedProperty;
};

vtkStandardNewMacro(CustomInteractorStyle);

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ribbonMenuBar(nullptr)
    , fileMenu(nullptr)
    , openAction(nullptr)
    , m_statusBar(nullptr)
    , vtkWidget(nullptr)
    , currentSelectedActor(nullptr)
    , manager()
    , TranslateThread(&manager)
{
    

    m_statusBar = this->statusBar();
    this->setStatusBar(m_statusBar);

    createRibbonMenu();
    setupVTK();
    setupControlPanel();
    createGrayImageWindows();
    try {
        manager.Manager_Init();
        printf("ALL Init");
    }
    catch (const runtime_error& e) {
        qDebug() << "Error:" << e.what();
        printf(e.what());
    }
    //QObject::connect(&manager, &Manager::Position_Send, this, &MainWindow::updateVtkWindow);
    //QObject::connect(&TranslateThread, &Image_Translate::ImageReady, this, &MainWindow::onImageReceived, Qt::QueuedConnection);
}

MainWindow::~MainWindow()
{
    
    if (TranslateThread.isRunning())
    {
        TranslateThread.quit();
        TranslateThread.wait();
    }
    
}

void MainWindow::createRibbonMenu()
{
    ribbonMenuBar = new QMenuBar(this);
    ribbonMenuBar->setStyleSheet("QMenuBar { background-color: #f0f0f0; spacing: 5px; }"
        "QMenuBar::item { background: transparent; padding: 5px 10px; }"
        "QMenuBar::item:selected { background: #d0d0d0; }");

    fileMenu = ribbonMenuBar->addMenu("File");
    openAction = new QAction("Open Model", this);
    openAction->setShortcut(QKeySequence::Open);
    connect(openAction, &QAction::triggered, this, &MainWindow::openModel);
    fileMenu->addAction(openAction);

    setMenuBar(ribbonMenuBar);
}

void MainWindow::setupVTK()
{
    QWidget* centralWidget = new QWidget(this);
    QVBoxLayout* mainLayout = new QVBoxLayout(centralWidget);

    modelViewWidget = new QWidget(this);

    QHBoxLayout* vtkLayout = new QHBoxLayout(modelViewWidget);
    //QHBoxLayout* mainLayout = new QHBoxLayout(centralWidget);
      
    vtkWidget = new QVTKOpenGLNativeWidget(centralWidget);
    vtkLayout->addWidget(vtkWidget, 4);

    mainLayout->addWidget(modelViewWidget,1);

    renderer = vtkSmartPointer<vtkRenderer>::New();
    renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);

    vtkWidget->setRenderWindow(renderWindow);

 
    customInteractorStyle = vtkSmartPointer<CustomInteractorStyle>::New();
    customInteractorStyle->SetMainWindow(this);
    customInteractorStyle->SetDefaultRenderer(renderer);
    renderWindow->GetInteractor()->SetInteractorStyle(customInteractorStyle);
  

    // 添加全局坐标系（基坐标系）
    vtkSmartPointer<vtkAxesActor> baseAxes = vtkSmartPointer<vtkAxesActor>::New();
    baseAxes->SetTotalLength(500.0, 500.0, 500.0);
    baseAxes->SetShaftTypeToCylinder();
    baseAxes->SetCylinderRadius(0.01);
    baseAxes->SetConeRadius(0.1);
    baseAxes->AxisLabelsOff();
    baseAxes->SetPosition(0, 0, 0);
    baseAxes->SetOrigin(0, 0, 0);

    renderer->AddActor(baseAxes);

    renderer->SetBackground(0.2, 0.3, 0.4);
    renderer->ResetCamera();

    setCentralWidget(centralWidget);
}

void MainWindow::setupControlPanel()
{
    QWidget* controlPanel = new QWidget(this);
    QVBoxLayout* controlLayout = new QVBoxLayout(controlPanel);

    QGroupBox* modelSelectionGroup = new QGroupBox("模型选择", controlPanel);
    QVBoxLayout* selectionLayout = new QVBoxLayout(modelSelectionGroup);

    modelSelector = new QComboBox(modelSelectionGroup);
    modelSelector->setMinimumWidth(150);
    connect(modelSelector, QOverload<int>::of(&QComboBox::currentIndexChanged),
        this, &MainWindow::onModelSelected);
    selectionLayout->addWidget(modelSelector);

    deleteButton = new QPushButton("删除模型", modelSelectionGroup);
    connect(deleteButton, &QPushButton::clicked, this, &MainWindow::onDeleteModel);
    selectionLayout->addWidget(deleteButton);


    QGroupBox* imageGroup = new QGroupBox("图像处理", controlPanel);
    QVBoxLayout* imageLayout = new QVBoxLayout(imageGroup);

    saveImageButton = new QPushButton("保存图像到路径", imageGroup);
    connect(saveImageButton, &QPushButton::clicked, this, &MainWindow::saveImage);
    


    QWidget* FileNameSet = new QWidget(imageGroup);
    QHBoxLayout * FileNameSetLayout = new QHBoxLayout(FileNameSet);
    
    saveFileName1Set = new QComboBox(FileNameSet);
    saveFileName2Set = new QComboBox(FileNameSet);
    saveFileName3Set = new QComboBox(FileNameSet);

    saveFileName1Set->addItems({"1.2m","1.5m","2.0m"});
    saveFileName2Set->addItems({"1","2","3","4","5","6","7","8","9"});
    saveFileName3Set->addItems({"预留选项","",""});


    FileNameSetLayout->addWidget(saveFileName1Set);
    FileNameSetLayout->addWidget(saveFileName2Set);
    FileNameSetLayout->addWidget(saveFileName3Set);
    imageLayout->addWidget(FileNameSet);
    imageLayout->addWidget(saveImageButton);

    controlLayout->addWidget(imageGroup);
    controlLayout->addWidget(modelSelectionGroup);
    QHBoxLayout* modelViewWidgetLayout = qobject_cast<QHBoxLayout*>(modelViewWidget->layout());
    if (modelViewWidgetLayout) {
        modelViewWidgetLayout->addWidget(controlPanel, 1);
    }

    deleteButton->setEnabled(false);
}

void MainWindow::openModel()
{
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open 3D Model"), "",
        tr("3D Files (*.ply *.obj *.stl *.step *.stp);;PLY Files (*.ply);;OBJ Files (*.obj);;STL Files (*.stl);;STEP Files (*.step *.stp)"));

    if (fileName.isEmpty()) return;

    QFileInfo fileInfo(fileName);
    QString extension = fileInfo.suffix().toLower();

    try {
        bool success = false;

        if (extension == "ply") {
            success = handlePlyFile(fileName, fileInfo);
        }
        else if (extension == "obj") {
            success = handleObjFile(fileName, fileInfo);
        }
        else if (extension == "stl") {
            success = handleStlFile(fileName, fileInfo);
        }
        else if (extension == "step" || extension == "stp") {
            success = handleStepFile(fileName, fileInfo);
        }
        else {
            QMessageBox::warning(this, "Unsupported Format",
                "The selected file format (" + extension + ") is not supported.");
            return;
        }

        if (success) {
            renderer->ResetCamera();
            renderer->ResetCameraClippingRange();
            vtkWidget->renderWindow()->Render();
        }

    }
    catch (const std::exception& e) {
        QMessageBox::critical(this, "Model Load Failed", "Error loading file: " + QString(e.what()));
    }
    catch (...) {
        QMessageBox::critical(this, "Model Load Failed", "Unknown error occurred while loading the model.");
    }
}

bool MainWindow::handlePlyFile(const QString& fileName, const QFileInfo& fileInfo)
{
    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
    reader->SetFileName(fileName.toStdString().c_str());
    reader->Update();

    vtkSmartPointer<vtkPolyData> polyData = reader->GetOutput();
    if (!polyData || polyData->GetNumberOfPoints() == 0) {
        throw std::runtime_error("No valid geometry data found in PLY file.");
    }

    addModelToRenderer(polyData, fileInfo.fileName());
    return true;
}

bool MainWindow::handleObjFile(const QString& fileName, const QFileInfo& fileInfo)
{
    vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
    reader->SetFileName(fileName.toStdString().c_str());
    reader->Update();

    vtkSmartPointer<vtkPolyData> polyData = reader->GetOutput();
    if (!polyData || polyData->GetNumberOfPoints() == 0) {
        throw std::runtime_error("No valid geometry data found in OBJ file.");
    }

    addModelToRenderer(polyData, fileInfo.fileName());
    return true;
}

bool MainWindow::handleStlFile(const QString& fileName, const QFileInfo& fileInfo)
{
    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(fileName.toStdString().c_str());
    reader->Update();

    vtkSmartPointer<vtkPolyData> polyData = reader->GetOutput();
    if (!polyData || polyData->GetNumberOfPoints() == 0) {
        throw std::runtime_error("No valid data found in STL file.");
    }

    addModelToRenderer(polyData, fileInfo.fileName());
    return true;
}

bool MainWindow::handleStepFile(const QString& fileName, const QFileInfo& fileInfo)
{
    vtkSmartPointer<vtkOCCTReader> stepReader = vtkSmartPointer<vtkOCCTReader>::New();
    stepReader->SetFileName(fileName.toStdString().c_str());
    stepReader->Update();

    if (stepReader->GetErrorCode() != 0) {
        throw std::runtime_error("Failed to read STEP file. Error: " + std::to_string(stepReader->GetErrorCode()));
    }

    vtkSmartPointer<vtkMultiBlockDataSet> multiBlockData = stepReader->GetOutput();
    if (!multiBlockData || multiBlockData->GetNumberOfBlocks() == 0) {
        throw std::runtime_error("STEP file has no valid blocks.");
    }

    bool hasValidData = false;
    for (unsigned int i = 0; i < multiBlockData->GetNumberOfBlocks(); i++) {
        vtkSmartPointer<vtkPolyData> blockPolyData = vtkPolyData::SafeDownCast(multiBlockData->GetBlock(i));
        if (!blockPolyData) continue;

        if (blockPolyData->GetNumberOfPoints() == 0 || blockPolyData->GetNumberOfCells() == 0) {
            continue;
        }

        QString blockName = fileInfo.fileName() + QString(" - Part %1").arg(i + 1);
        addModelToRenderer(blockPolyData, blockName);
        hasValidData = true;
    }

    if (!hasValidData) {
        throw std::runtime_error("STEP file contains no valid geometry data.");
    }

    return true;
}

void MainWindow::addModelToRenderer(vtkSmartPointer<vtkPolyData> polyData, const QString& modelName)
{
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);
    mapper->ScalarVisibilityOff();

    vtkSmartPointer<vtkActor> modelActor = vtkSmartPointer<vtkActor>::New();
    modelActor->SetMapper(mapper);
    modelActor->GetProperty()->SetColor(0.8, 0.8, 0.8);
    modelActor->GetProperty()->SetSpecular(0.5);
    modelActor->GetProperty()->SetSpecularPower(30);
    modelActor->GetProperty()->SetAmbient(0.2);
    modelActor->GetProperty()->SetDiffuse(0.8);

    renderer->AddActor(modelActor);

    // 创建并调整坐标系指示器大小
    vtkSmartPointer<vtkAxesActor> axesActor = vtkSmartPointer<vtkAxesActor>::New();
    axesActor->SetTotalLength(200.0, 200.0, 200.0);
    axesActor->SetShaftTypeToCylinder();
    axesActor->SetCylinderRadius(0.03);
    axesActor->SetConeRadius(0.06);
    axesActor->AxisLabelsOff();
    axesActor->SetVisibility(false);

    renderer->AddActor(axesActor);
    

    // 创建模型信息
    ModelInfo modelInfo;
    modelInfo.actor = modelActor;
    modelInfo.axesActor = axesActor;
    modelInfo.showAxes = false;

    // 保存初始位置 (0,0,0)
    modelInfo.initialPosition[0] = 0;
    modelInfo.initialPosition[1] = 0;
    modelInfo.initialPosition[2] = 0;

    // 保存初始方向 (0,0,0)
    modelInfo.initialOrientation[0] = 0;
    modelInfo.initialOrientation[1] = 0;
    modelInfo.initialOrientation[2] = 0;

    // 保存初始颜色
    double* color = modelActor->GetProperty()->GetColor();
    copyArray(modelInfo.initialColor, color);

    modelActors.append(modelInfo);
    modelSelector->addItem(modelName);

    if (modelSelector->count() > 0) {
        modelSelector->setCurrentIndex(modelSelector->count() - 1);
    }
}

void MainWindow::onModelSelected(int index)
{
    if (index < 0 || index >= modelActors.size()) {
        currentSelectedActor = nullptr;
        return;
    }

    currentSelectedActor = modelActors[index].actor;

    // 更新位置控制
    

    // 更新坐标系显示状态
   // showAxesCheckBox->setChecked(modelActors[index].showAxes);
}



void MainWindow::saveImage()
{
    QString saveFileName1 = saveFileName1Set->currentText();
    QString saveFileName2 = saveFileName2Set->currentText();
    QString saveFileName3 = saveFileName3Set->currentText();


    Pathlist << saveFileName1 << saveFileName2 << saveFileName3;
    QString FileName = Pathlist.join("_");

    std::string str = FileName.toUtf8().toStdString();

    manager.TwinCamThread.SetSavePath(str);

    if (manager.TwinCamThread.SaveImageToPath())
    {
        m_statusBar->showMessage("保存成功", 5000);
        auto mats = manager.TwinCamThread.GetSaveImage();

        auto Qmats = preprocessImageArray(mats);
        dualGrayWidget->updateBothImages(Qmats[1], Qmats[0]);
    }
    Pathlist.clear();
}

void MainWindow::onDeleteModel()
{
    if (!currentSelectedActor) return;

    int currentIndex = modelSelector->currentIndex();
    if (currentIndex < 0 || currentIndex >= modelActors.size()) return;

    renderer->RemoveActor(modelActors[currentIndex].actor);
    renderer->RemoveActor(modelActors[currentIndex].axesActor);

    modelActors.removeAt(currentIndex);
    modelSelector->removeItem(currentIndex);

    if (modelSelector->count() == 0) {
        currentSelectedActor = nullptr;
    }
    else {
        int newIndex = currentIndex >= modelSelector->count() ? modelSelector->count() - 1 : currentIndex;
        modelSelector->setCurrentIndex(newIndex);
    }

    vtkWidget->renderWindow()->Render();
}

void MainWindow::onToggleAxes(bool show)
{
    int currentIndex = modelSelector->currentIndex();
    if (currentIndex < 0 || currentIndex >= modelActors.size()) return;

    modelActors[currentIndex].showAxes = show;
    modelActors[currentIndex].axesActor->SetVisibility(show);

    vtkWidget->renderWindow()->Render();
}

void MainWindow::updateModelSelection(vtkActor* actor)
{
    // 查找对应的模型索引
    int index = -1;
    for (int i = 0; i < modelActors.size(); i++) {
        if (modelActors[i].actor == actor) {
            index = i;
            break;
        }
    }

    if (index >= 0) {
        modelSelector->setCurrentIndex(index);
        currentSelectedActor = actor;
        onModelSelected(index);
    }
    else {
        currentSelectedActor = nullptr;
    }
}

void MainWindow::updateVtkWindow(const Point3f& position,float roll,float pitch,float yaw)
{

    // 检查演员指针是否有效
    if (!currentSelectedActor) {
        qWarning() << "\n currentSelectedActor is not init";
        manager.signalFlag.store(true);
        return;
    }

    // 直接使用Point3f的成员变量更新演员位置
    currentSelectedActor->SetPosition(position.x, position.y, position.z);
    currentSelectedActor->SetOrientation(roll, pitch, yaw);

    // 可选：如果需要立即刷新渲染窗口
    if (vtkWidget) {  
        vtkWidget->renderWindow()->Render();
    }
    manager.signalFlag.store(true);
    printf("*************Rset Signal *************");
}
void MainWindow::updateAxesPosition(vtkActor* actor)
{
    // 查找对应的模型索引
    int index = -1;
    for (int i = 0; i < modelActors.size(); i++) {
        if (modelActors[i].actor == actor) {
            index = i;
            break;
        }
    }

    if (index >= 0) {
        double* position = actor->GetPosition();
        modelActors[index].axesActor->SetPosition(position);
        vtkWidget->renderWindow()->Render();
    }
}


void MainWindow::createGrayImageWindows()
{
    QWidget* centralWidget = this->centralWidget();
    if (!centralWidget) {
        qWarning() << "主窗口中心部件不存在";
        return;
    }

    // 若主布局不存在，创建默认的垂直布局
    QVBoxLayout* mainLayout = qobject_cast<QVBoxLayout*>(centralWidget->layout());

    dualGrayWidget = new GLDualGrayImageWidget(this);
    dualGrayWidget->setMinimumSize(1800,600); // 适合显示大尺寸灰度图

    // 添加到布局
    mainLayout->addWidget(dualGrayWidget,1);

    // 更新灰度图像
    //dualGrayWidget->updateBothImages(leftGray, rightGray);
}
#if 0
void MainWindow::createGrayImageWindows()
{
    // 1. 先释放旧部件，防止内存泄漏（支持重复调用）
    if (imageWindow) {
        // 手动置空子标签指针（避免悬空指针）
        leftImgLabel = nullptr;
        rightImgLabel = nullptr;
        // 删除窗口会自动释放所有子部件和布局
        delete imageWindow;
        imageWindow = nullptr;
    }

    // 2. 获取主窗口的中心部件和主布局
    QWidget* centralWidget = this->centralWidget();
    if (!centralWidget) {
        qWarning() << "主窗口中心部件不存在";
        return;
    }

    // 若主布局不存在，创建默认的垂直布局
    QVBoxLayout* mainLayout = qobject_cast<QVBoxLayout*>(centralWidget->layout());
    if (!mainLayout) {
        qWarning() << "主布局不存在，创建默认垂直布局";
        mainLayout = new QVBoxLayout(centralWidget);
        centralWidget->setLayout(mainLayout);
        // 可在此处添加原有VTK窗口（如果需要）
        // mainLayout->addWidget(vtkWidget, 4); // 保持4:1的比例
    }

    // 3. 创建图像显示的容器部件
    imageWindow = new QWidget(centralWidget);
    QVBoxLayout* imageLayout = new QVBoxLayout(imageWindow);
    imageLayout->setContentsMargins(5, 5, 5, 5);
    imageLayout->setSpacing(10);

    // 4. 创建左右图像显示标签（优化初始化）
    leftImgLabel = new QLabel("左图");
    leftImgLabel->setStyleSheet("border:1px solid #999; background:#000; color:#fff; text-align:center;");
    leftImgLabel->setMinimumSize(320, 240); // 明确最小宽高，避免缩放异常
    leftImgLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding); // 允许扩展

    rightImgLabel = new QLabel("右图");
    rightImgLabel->setStyleSheet("border:1px solid #999; background:#000; color:#fff; text-align:center;");
    rightImgLabel->setMinimumSize(320, 240);
    rightImgLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    // 5. 水平排列两个图像标签
    QHBoxLayout* imgPairLayout = new QHBoxLayout();
    imgPairLayout->addWidget(leftImgLabel, 1);
    imgPairLayout->addWidget(rightImgLabel, 1);
    imageLayout->addLayout(imgPairLayout);

    // 6. 将图像容器添加到主布局（比例1:4）
    mainLayout->addWidget(imageWindow, 1);
}


void MainWindow::onImageReceived(const QImage& leftImg, const QImage& rightImg) {
    // 1. 严格检查指针有效性，避免空指针崩溃
    if (!leftImgLabel || !rightImgLabel) {
        qWarning() << "图像标签未初始化，忽略此次更新";
        return;
    }

    // 2. 定义复用的缩放显示函数
    auto updateLabel = [this](const QImage& img, QLabel* label, const QString& errorText) {
        // 处理空图像
        if (img.isNull()) {
            label->setPixmap(QPixmap()); // 清除旧图像
            label->setText(errorText);   // 显示错误提示
            return;
        }

        // 确保目标尺寸有效（优先用当前尺寸，否则用最小尺寸）
        QSize targetSize = label->size();
        if (targetSize.isEmpty()) {
            targetSize = label->minimumSize();
        }

        // 快速缩放到目标尺寸
        QPixmap scaledPix = QPixmap::fromImage(img.scaled(
            targetSize,
            Qt::KeepAspectRatio,
            Qt::FastTransformation // 效率优先
        ));

        label->setPixmap(scaledPix);
        label->setText(""); // 清空错误提示
        };

    // 3. 更新左右图像标签
    updateLabel(leftImg, leftImgLabel, "左图异常");
    updateLabel(rightImg, rightImgLabel, "右图异常");
}
#endif

void MainWindow::showImage(bool flag)
{
    if (flag == true)
    {
        TranslateThread.Set();
        TranslateThread.start();
    }
    else TranslateThread.ReSet();
}
