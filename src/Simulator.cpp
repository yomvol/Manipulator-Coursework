#include "Simulator.h"

namespace rk
{
    std::string Simulator::appDir = "";

    std::string Simulator::getAppDir()
    {
        if (!appDir.empty()) return appDir;
        
        char pathBuffer[MAX_PATH];
        size_t pathSize;
        if (getenv_s(&pathSize, pathBuffer, MAX_PATH, "COPPELIASIM_ROOT_DIR") != 0 || pathSize == 0) {
            throw std::runtime_error("Environment variable COPPELIASIM_ROOT_DIR is not set. Please define it.");
        }
        
        auto path = std::filesystem::path(pathBuffer);
        appDir = path.string();
        return appDir;
    }

    std::string Simulator::getCoppeliaSimDLLPath()
    {
        auto dllPath = std::filesystem::path(getAppDir()) / "coppeliaSim.dll";

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
        SetDllDirectory(getAppDir().c_str());
        m_hLib = LoadLibraryW(wcstr);
        delete[] wcstr;

        if (m_hLib == NULL) {
            char buffer[100];
            itoa(GetLastError(), buffer, 10);
            throw std::runtime_error("Failed to load CoppeliaSim DLL from: " + dllPath + "\nError: " + buffer);
        }

        if (getSimProcAddresses(m_hLib) == 0)
        {
            FreeLibrary(m_hLib);
            std::cerr << "Could not find all required functions in the CoppeliaSim library.\n";
            return;
        }
        
        std::cout << "CoppeliaSim DLL loaded successfully from: " << dllPath << std::endl;
    }

    Simulator::~Simulator()
    {
        if (m_hLib != NULL) FreeLibrary(m_hLib);
    }

    void Simulator::runGUI()
    {
        simRunGui(sim_gui_all);
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
            simInitialize(getAppDir().c_str(), 0);

            loadScene("C:\\Program Files\\CoppeliaRobotics\\CoppeliaSimEdu\\scenes\\tutorials\\BubbleRob\\bubbleRob-python.ttt");
            while (!simGetExitRequest())
            {
                simLoop(nullptr, 0);
            }
            simDeinitialize();
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

    void Simulator::setJointPosition(int jointHandle, float position)
    {
        if (simSetJointPosition) {
            simSetJointPosition(jointHandle, position);
            std::cout << "Moved joint to position: " << position << " radians\n";
        }
    }


}