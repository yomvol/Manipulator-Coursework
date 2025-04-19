# What is it?
This is my project for the "Robotics" class (Spring 2025) at my uni. The goal was to visualize a simple manipulator and solve its forward kinematics. I used CoppeliaSim as simulation environment and Eigen as matrix handling library for the task at hand.

# How to build
CMake is used as build system. So compiling the source should be relatively easy. I tested the build process only in Windows 10, Microsoft Visual Studio 17 2022 as generator. However, firstly you'll need to install CoppeliaSim (free version).

1) Install CoppeliaSim from the official site https://www.coppeliarobotics.com/
2) Create an environment variable COPPELIASIM_ROOT_DIR on your system. The path must lead to the folder of the installed simulator. 
For example, C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu
3) Clone the repository:
git clone --recursive https://github.com/yomvol/Manipulator-Coursework
4) On the command line:

mkdir build && cd build

cmake .. -G "Visual Studio 17 2022"

cmake --build . --config Release

In the build/Release folder you'll have Manipulator.exe
