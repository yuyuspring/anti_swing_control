#include "simulation/closed_loop_simulation.hpp"

#include <iostream>
#include <string>

int main(int argc, char* argv[]) {
    pendulum::ClosedLoopConfig config;

    // Parse simple command-line overrides
    if (argc > 1) {
        config.pTarget = std::stod(argv[1]);
    }
    if (argc > 2) {
        config.initialTheta = std::stod(argv[2]);
    }
    if (argc > 3) {
        config.ropeLength = std::stod(argv[3]);
    }
    if (argc > 4) {
        config.tFinal = std::stod(argv[4]);
    }

    std::cout << "========================================\n";
    std::cout << "  Closed-Loop LQR Simulation (1D Pitch)\n";
    std::cout << "========================================\n";

    pendulum::ClosedLoopSimulation sim(config);
    sim.run();
    sim.saveResults("closed_loop_results.csv");

    std::cout << "Done. Run: python3 scripts/plot_closed_loop.py closed_loop_results.csv\n";
    return 0;
}
