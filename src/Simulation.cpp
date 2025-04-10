#include "simLib/simLib.h"
#include <Eigen/Core>

namespace rk
{
#define M_PI 3.14159265358979323846
    // Attention!!! Coppelia uses 3x4 row-major order, Eigen uses column-major by default
    using Transform = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
    Transform T01, T12, T2k;
    int joint1Handle, joint2Handle, joint3Handle;

    double phi_1 = M_PI / 6, phi_2 = M_PI / 4, phi_3 = -M_PI / 4, l_1 = 1, l_2 = 0.1, l_3 = 0.1, l_4 = 0.1, l_5 = 1, l_6 = 0.1;

    Eigen::Vector3d getEndEffectorWorldPos(double alpha, double beta, double gamma)
    {
        Eigen::Vector3d p_world;
        p_world << -l_2 * cos(alpha) + l_5 * sin(beta) * cos(alpha) +
            l_6 * (cos(alpha) * cos(beta) * cos(gamma) - sin(beta) * sin(gamma) * cos(alpha)) - l_1,
            -l_2 * sin(alpha) + l_5 * sin(alpha) * sin(beta) +
            l_6 * (sin(alpha) * cos(beta) * cos(gamma) - sin(alpha) * sin(beta) * sin(gamma)),
            l_5* cos(beta) + l_6 * (sin(beta) * (-cos(gamma)) - sin(gamma) * cos(beta)) + l_3 + l_4;
        return p_world;
    }

    void drawCoordinateFrame(Transform m, double thicknessPx, double axisLength = 0.1f)
    {
        float red[3] = { 1.0f, 0.0f, 0.0f }, green[3] = { 0.0f, 1.0f, 0.0f }, blue[3] = {0.0f, 0.0f, 1.0f};

        int redHandle = simAddDrawingObject(sim_drawing_lines, thicknessPx, 0, -1, 0, red, nullptr, nullptr, nullptr);
        int greenHandle = simAddDrawingObject(sim_drawing_lines, thicknessPx, 0, -1, 0, green, nullptr, nullptr, nullptr);
        auto blueHandle = simAddDrawingObject(sim_drawing_lines, thicknessPx, 0, -1, 0, blue, nullptr, nullptr, nullptr);

        Eigen::Matrix3d rotation = m.block<3, 3>(0, 0); // 3x3 rotation matrix
        Eigen::Vector4d origin = m.col(3); // 4x1 translation vector

        std::vector<double> line;
        // X axis (red)
        line = {
            origin.x(), origin.y(), origin.z(),
            origin.x() + axisLength * rotation(0,0),
            origin.y() + axisLength * rotation(1,0),
            origin.z() + axisLength * rotation(2,0)
        };
        simAddDrawingObjectItem(redHandle, line.data());

        // Y axis (green)
        line = {
            origin.x(), origin.y(), origin.z(),
            origin.x() + axisLength * rotation(0,1),
            origin.y() + axisLength * rotation(1,1),
            origin.z() + axisLength * rotation(2,1)
        };
        simAddDrawingObjectItem(greenHandle, line.data());

        // Z axis (blue)
        line = {
            origin.x(), origin.y(), origin.z(),
            origin.x() + axisLength * rotation(0,2),
            origin.y() + axisLength * rotation(1,2),
            origin.z() + axisLength * rotation(2,2)
        };
        simAddDrawingObjectItem(blueHandle, line.data());
    }

    void drawPoint(Eigen::Vector3d point, double thicknessPx, float r, float g, float b)
    {
        float color[3] = { r, g, b };
        int handle = simAddDrawingObject(sim_drawing_points, thicknessPx, 0, -1, 0, color, nullptr, nullptr, nullptr);
        std::vector<double> line = { point.x(), point.y(), point.z() };
        simAddDrawingObjectItem(handle, line.data());
    }

    inline double deg2Rad(double degrees) { return  degrees * (M_PI / 180); };

    int simulationInit()
    {
        joint1Handle = simGetObject("/fire_engine/joint1", -1, -1, 0);
        joint2Handle = simGetObject("/fire_engine/joint2", -1, -1, 0);
        joint3Handle = simGetObject("/fire_engine/joint3", -1, -1, 0);
        if (joint1Handle == -1 || joint2Handle == -1 || joint3Handle == -1) {
            simAddLog("CoppeliaSim:error", sim_verbosity_errors, "Failed to get joint handles");
            return -1;
        }

        T01 << cos(phi_1), -sin(phi_1), 0, -l_1,
            sin(phi_1), cos(phi_1), 0, 0,
            0, 0, 1, l_3,
            0, 0, 0, 1;

        T12 << cos(phi_2), 0, sin(phi_2), -l_2,
            0, 1, 0, 0,
            -sin(phi_2), 0, cos(phi_2), l_4,
            0,           0,    0,       1;

        T2k << cos(phi_3), 0, sin(phi_3), 0,
            0, 1, 0, 0,
            -sin(phi_3), 0, cos(phi_3), l_5,
            0,           0,  0,         1;

        drawCoordinateFrame(Transform::Identity(), 2, 0.5);
        drawCoordinateFrame(T01, 2, 0.2);
        drawCoordinateFrame(T01 * T12, 2, 0.5);
        drawCoordinateFrame(T01 * T12 * T2k, 2, 0.5);
        drawPoint(getEndEffectorWorldPos(phi_1, phi_2, phi_3), 20, 1, 0, 0);
        
        simSetJointPosition(joint1Handle, phi_1);
        simSetJointPosition(joint2Handle, phi_2);
        simSetJointPosition(joint3Handle, M_PI/2 + phi_3);
        return 0;
    }

    void simulationLoop()
    {
        if (simGetSimulationState() == sim_simulation_advancing_running)
        {
            float color[3] = { 0, 200, 255 };
            int handle = simAddDrawingObject(sim_drawing_points, 2, 0, -1, 1000000, color, nullptr, nullptr, nullptr);

            for (double alpha = 0; alpha < 360; alpha += 2) {
                for (double beta = 90; beta >= 0; beta -= 5) {
                    for (double gamma = -180; gamma <= 0; gamma += 5) {
                        double a = deg2Rad(alpha);
                        double b = deg2Rad(beta);
                        double g = deg2Rad(gamma);

                        auto pos = getEndEffectorWorldPos(a, b, g);
                        std::vector<double> point = { pos.x(), pos.y(), pos.z() };
                        simAddDrawingObjectItem(handle, point.data());
                    }
                }
            }
            
            simPauseSimulation();
        }
    }
}