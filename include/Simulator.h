#pragma once
#include <stdexcept>
#include <iostream>
#include <filesystem>
#include <windows.h>
#include <functional>
#include "simLib/simLib.h"

namespace rk
{
    class Simulator
    {
    public:
        static std::string getAppDir();
        static std::string getCoppeliaSimDLLPath();

        Simulator();
        ~Simulator();
        void runGUI();
        std::function<void()> getSimulationFunc();

    private:
        LIBRARY m_hLib;
        static std::string appDir;
        
        void simulationThread();
        void loadScene(const std::string& scenePame);
        void setJointPosition(int jointHandle, float position);
    };
}