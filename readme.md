# DUT-SurgicalNavIgation-Team

大连理工大学（DUT）手术导航团队代码仓库  
(DUT Surgical Navigation Team Repository)

本仓库主要用于手术导航相关的三维可视化、图像处理与实验性算法验证，当前以 **Qt + VTK 的 3D 模型查看与交互系统** 为核心实现。

---

## 项目简介

本项目实现了一个基于 **C++ / Qt / VTK** 的桌面端三维模型可视化应用，支持多种常见 3D 模型格式的加载、显示与交互操作，适用于：

- 手术导航系统中的三维模型预览
- 三维重建结果验证
- 教学与算法调试用途

项目以工程实践为导向，代码结构清晰，便于在此基础上扩展导航、标定、跟踪等模块。

---

## 功能特性

- 支持多种 3D 模型格式加载（STL / OBJ / PLY / STEP）
- 基于 VTK 的高性能三维渲染
- Qt 图形界面，支持交互式操作
- 多模型同时显示与管理
- 模型选择、平移、旋转、颜色调整
- 显示世界坐标系
- 视角重置与模型状态恢复
- 预留接口，便于集成手术导航算法模块

---

## 支持的模型格式

| 类型 | 扩展名 |
|----|----|
| STL | `.stl` |
| OBJ | `.obj` |
| PLY | `.ply` |
| STEP | `.step`, `.stp` |

---

## 运行环境与依赖

### 操作系统
- Windows 10 / 11（已验证）
- Linux（Ubuntu 20.04 / 22.04，理论支持）

### 开发环境
- C++ 标准：C++14
- 构建工具：CMake ≥ 3.16
- 编译器：
  - Windows：Visual Studio 2022（MSVC）
  - Linux：GCC / Clang

### 第三方库
- Qt 6.x
- VTK 9.x

---

## 构建与运行

### 1. 克隆仓库

```bash
git clone https://github.com/DUT-ZJC/DUT-SurgicalNavIgation-Team.git
cd DUT-SurgicalNavIgation-Team
