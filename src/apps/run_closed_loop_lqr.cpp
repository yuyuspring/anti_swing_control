#include "simulation/closed_loop_simulation.hpp"

#include <iostream>
#include <string>

void printUsage(const char* prog) {
    std::cout << "Usage: " << prog << " <mode> [initial_theta] [rope_length] [duration]\n"
              << "  mode: 0=Diagonal, 1=Coupled\n"
              << "  default: mode=0, theta=0.1rad, rope=15m, duration=60s\n";
}

int main(int argc, char* argv[]) {
    pendulum::ClosedLoopConfig config;
    pendulum::ControlMode mode = pendulum::ControlMode::kDiagonal;

    if (argc > 1) {
        int modeInt = std::stoi(argv[1]);
        if (modeInt == 1) mode = pendulum::ControlMode::kCoupled;
    }
    if (argc > 2) config.initialTheta = std::stod(argv[2]);
    if (argc > 3) config.ropeLength = std::stod(argv[3]);
    if (argc > 4) config.tFinal = std::stod(argv[4]);

    std::cout << "========================================\n";
    std::cout << "  Closed-Loop LQR Simulation (1D Pitch)\n";
    std::cout << "========================================\n";

    pendulum::ClosedLoopSimulation sim(config, mode);
    sim.run();

    std::string filename;
    switch (mode) {
        case pendulum::ControlMode::kDiagonal: filename = "closed_loop_diagonal.csv"; break;
        case pendulum::ControlMode::kCoupled: filename = "closed_loop_coupled.csv"; break;
    }
    sim.saveResults(filename);

    std::cout << "Done. Run: python3 ../scripts/plot/plot_closed_loop.py " << filename << "\n";
    return 0;
}
