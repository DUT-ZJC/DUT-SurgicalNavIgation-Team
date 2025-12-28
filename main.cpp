#include "VTK_Viewer.h"
#include <QApplication>

int main(int argc, char* argv[])
{
    AllocConsole();

    FILE* fp = nullptr;
    freopen_s(&fp, "CONOUT$", "w", stdout);
    freopen_s(&fp, "CONOUT$", "w", stderr);
    freopen_s(&fp, "CONIN$", "r", stdin);

    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);

    // 关键：把 C++ iostream 也重新绑定到控制台缓冲区
    std::ios::sync_with_stdio(true);
    std::cout.clear();
    std::cerr.clear();
    std::cin.clear();

    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    
    try {
        w.manager.GO();
        printf("start grap");
        //w.showImage(true);
    }
    catch (const runtime_error& e) {
        qDebug() << "Error:" << e.what();
        printf(e.what());
    }
    return a.exec();
    
}
