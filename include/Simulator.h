#pragma once
#include <stdexcept>
#include <iostream>
#include <filesystem>
#include <windows.h>
#include <functional>

namespace rk
{
    class Simulator
    {
    public:
        static std::string getCoppeliaSimDLLPath();

        Simulator();
        ~Simulator();
        void runGUI();
        std::function<void()> getSimulationFunc();

    private:
        HMODULE m_hLib;
        std::function<int()> simRunSimulator;
        std::function<int(const char*)> simLoadScene;
        std::function<int(const char*, int*)> simGetObjectHandle;
        std::function<int(int, float)> simSetJointPosition;
        std::function<int()> simStartSimulation;

        void simulationThread();
        void loadScene(const std::string& scenePame);
        int getObjectHandle(const std::string& objectName);
        void setJointPosition(int jointHandle, float position);

        template <typename Func>
        std::function<Func> loadFunction(const std::string& funcName) {
            FARPROC proc = GetProcAddress(m_hLib, funcName.c_str());
            auto funcPtr = reinterpret_cast<Func*>(proc);
            if (!funcPtr) {
                throw std::runtime_error("Failed to load function: " + funcName);
            }
            return std::function<Func>(funcPtr);
        }
    };
}