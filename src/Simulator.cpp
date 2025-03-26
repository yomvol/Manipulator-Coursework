#include "Simulator.h"

namespace rk
{
    std::string Simulator::getCoppeliaSimDLLPath()
    {
        char pathBuffer[MAX_PATH];
        size_t pathSize;

        if (getenv_s(&pathSize, pathBuffer, MAX_PATH, "COPPELIASIM_ROOT_DIR") != 0 || pathSize == 0) {
            throw std::runtime_error("Environment variable COPPELIASIM_ROOT_DIR is not set. Please define it.");
        }

        auto dllPath = std::filesystem::path(pathBuffer) / "coppeliaSim.dll";

        if (!std::filesystem::exists(dllPath)) {
            throw std::runtime_error("CoppeliaSim DLL not found at: " + dllPath.string());
        }

        std::cout << "CoppeliaSim DLL found at: " << dllPath << std::endl;

        return dllPath.string();
    }

    Simulator::Simulator()
    {
        std::string dllPath = getCoppeliaSimDLLPath();
        //const wchar_t* path = L"C:\\Program Files\\CoppeliaRobotics\\CoppeliaSimEdu\\coppeliaSim.dll";
        const size_t newSize = dllPath.size() + 1;
        size_t convertedChars = 0;
        wchar_t* wcstr = new wchar_t[newSize];
        mbstowcs_s(&convertedChars, wcstr, newSize, dllPath.c_str(), _TRUNCATE);
        m_hLib = LoadLibraryW(wcstr);
        delete[] wcstr;

        /*char buffer[100];
        itoa(GetLastError(), buffer, 10);
        if (!hLib) {
            throw std::runtime_error("Failed to load CoppeliaSim DLL from: " + dllPath + "\nError: " + buffer);
        }*/

        simRunSimulator = loadFunction<int()>("simRunSimulator");
        simLoadScene = loadFunction<int(const char*)>("simLoadScene");
        simGetObjectHandle = loadFunction<int(const char*, int*)>("simGetObjectHandle");
        simSetJointPosition = loadFunction<int(int, float)>("simSetJointPosition");
        simStartSimulation = loadFunction<int()>("simStartSimulation");

        std::cout << "CoppeliaSim DLL loaded successfully from: " << dllPath << std::endl;
    }

    Simulator::~Simulator()
    {
        if (m_hLib != NULL) FreeLibrary(m_hLib);
    }

    void Simulator::runGUI()
    {
        if (simRunSimulator) simRunSimulator();
    }

    std::function<void()> Simulator::getSimulationFunc()
    {
        std::function<void()> wrapper = std::bind(&Simulator::simulationThread, this);
        return wrapper;
    }

    void Simulator::simulationThread()
    {
        try
        {
            loadScene("C:\\Program Files\\CoppeliaRobotics\\CoppeliaSimEdu\\scenes\\tutorials\\BubbleRob\\bubbleRob-python.ttt");
            simStartSimulation();
        }
        catch (const std::exception& e)
        {
            std::cerr << "[Simulation Thread] Error: " << e.what() << std::endl;
        }
    }

    void Simulator::loadScene(const std::string& scenePath)
    {
        const char* str = scenePath.c_str();
        if (simLoadScene && simLoadScene(scenePath.c_str()) == -1) {
            throw std::runtime_error("Failed to load scene: " + scenePath);
        }
        std::cout << "Scene loaded: " << scenePath << std::endl;
    }

    int Simulator::getObjectHandle(const std::string& objectName)
    {
        int handle;
        if (simGetObjectHandle && simGetObjectHandle(objectName.c_str(), &handle) != 0) {
            throw std::runtime_error("Failed to get object handle: " + objectName);
        }
        return handle;
    }

    void Simulator::setJointPosition(int jointHandle, float position)
    {
        if (simSetJointPosition) {
            simSetJointPosition(jointHandle, position);
            std::cout << "Moved joint to position: " << position << " radians\n";
        }
    }


}