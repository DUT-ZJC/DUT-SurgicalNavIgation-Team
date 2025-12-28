markdown
# 3D Model Viewer Application

A comprehensive 3D model viewer application developed using Qt and VTK, supporting multiple 3D file formats with interactive visualization capabilities.

## Features

### Supported Formats
- PLY files (*.ply)
- OBJ files (*.obj)
- STL files (*.stl)
- STEP files (*.step, *.stp)

### Interactive Controls
- **Left mouse drag**: Rotate camera view
- **Right mouse click**: Select/deselect models
- **Left mouse drag on selected model**: Reposition model in 3D space
- **Multi-model management**: Load and manipulate multiple models simultaneously

### Control Panel
- **Translation Control**: Move models along X, Y, and Z axes
- **Rotation Control**: Rotate models around X, Y, and Z axes
- **Color Control**: Adjust RGB color values of models
- **Coordinate System Display**: Toggle local coordinate system visibility for each model
- **Reset Function**: Restore models to initial position, rotation, and color
- **Model Deletion**: Remove selected models from the scene

## Building and Running

### Dependencies
- Qt 6.9.2 
- VTK 9.5.1
- C++14
- VS2022

### Build Instructions

```bash
# Create build directory
mkdir build
cd build

# Configure project with CMake
cmake ..

# Build project
make

# Run application
./QtWidgetsApplication2