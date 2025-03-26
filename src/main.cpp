#include "Simulator.h"
#include <thread>

int main() {
    try {
        rk::Simulator sim;

        std::thread simThread(sim.getSimulationFunc());
        sim.runGUI();

        simThread.join();
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}


