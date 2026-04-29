#include "simulation/closed_loop_simulation.hpp"

#include <iostream>
#include <string>

void printUsage(const char* prog) {
    std::cout << "Usage: " << prog << " <mode> [initial_theta] [rope_length] [duration]\n"
              << "  mode: 0=Full, 1=Shortest, 2=MinSwing, 3=VelocityOmega, 4=PayloadVelocity, 5=MinEnergy, 6=SystemEnergy\n"
              << "  default: mode=0, theta=0.1rad, rope=15m, duration=60s\n";
}

int main(int argc, char* argv[]) {
    pendulum::ClosedLoopConfig config;
    pendulum::ControlMode mode = pendulum::ControlMode::kFull;

    if (argc > 1) {
        int modeInt = std::stoi(argv[1]);
        if (modeInt == 1) mode = pendulum::ControlMode::kShortest;
        else if (modeInt == 2) mode = pendulum::ControlMode::kMinSwing;
        else if (modeInt == 3) mode = pendulum::ControlMode::kVelocityOmega;
        else if (modeInt == 4) mode = pendulum::ControlMode::kPayloadVelocity;
        else if (modeInt == 5) mode = pendulum::ControlMode::kMinEnergy;
        else if (modeInt == 6) mode = pendulum::ControlMode::kSystemEnergy;
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
        case pendulum::ControlMode::kFull: filename = "closed_loop_full.csv"; break;
        case pendulum::ControlMode::kShortest: filename = "closed_loop_shortest.csv"; break;
        case pendulum::ControlMode::kMinSwing: filename = "closed_loop_minswing.csv"; break;
        case pendulum::ControlMode::kVelocityOmega: filename = "closed_loop_velomega.csv"; break;
        case pendulum::ControlMode::kPayloadVelocity: filename = "closed_loop_payload.csv"; break;
        case pendulum::ControlMode::kMinEnergy: filename = "closed_loop_minenergy.csv"; break;
        case pendulum::ControlMode::kSystemEnergy: filename = "closed_loop_systemenergy.csv"; break;
    }
    sim.saveResults(filename);

    std::cout << "Done. Run: python3 ../scripts/plot/plot_closed_loop.py " << filename << "\n";
    return 0;
}
