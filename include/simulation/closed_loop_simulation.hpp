#pragma once

#include "controller/lqr_controller.hpp"
#include "dynamics/slung_load_dynamics.hpp"
#include "observer/pendulum_observer.hpp"
#include "sensor/imu_sensor_model.hpp"
#include "utils/type_defs.hpp"

#include <string>
#include <vector>

namespace pendulum {

/**
 * @brief Configuration for closed-loop LQR simulation.
 */
struct ClosedLoopConfig {
    double dtTruth = 0.001;          ///< Ground-truth dynamics step [s]
    double dtControl = 0.02;         ///< Control / observer period [s]
    double ropeLength = 15.0;        ///< [m]
    double axMax = 3.0;              ///< Max horizontal acceleration [m/s^2]
    double tFinal = 40.0;            ///< Total simulation time [s]
    double pStart = 0.0;             ///< Initial position [m]
    double pTarget = 50.0;           ///< Target position [m]
    double initialTheta = 0.1;       ///< Initial pendulum angle [rad]
    double initialThetaDot = 0.0;    ///< Initial angular rate [rad/s]
    double dampingRatio = 0.06;      ///< Pendulum damping ratio

    // Sensor noise std-devs
    double gyroNoiseStd = 0.005;     ///< [rad/s]
    double accelNoiseStd = 0.05;     ///< [m/s^2]
    double velNoiseStd = 0.02;       ///< [m/s]
};

/**
 * @brief Closed-loop simulation engine.
 *
 * Combines non-linear slung-load dynamics, IMU sensor model,
 * pendulum observer, and LQR controller into a single simulation loop.
 */
class ClosedLoopSimulation {
public:
    explicit ClosedLoopSimulation(const ClosedLoopConfig& config);

    /**
     * @brief Run the full simulation.
     */
    void run();

    /**
     * @brief Save logged data to CSV.
     */
    void saveResults(const std::string& filename) const;

    const std::vector<ClosedLoopLogEntry>& log() const { return log_; }

private:
    ClosedLoopConfig config_;
    SlungLoadDynamics dynamics_;
    PendulumObserver observer_;
    LqrController controller_;
    ImuSensorModel sensor_;
    std::vector<ClosedLoopLogEntry> log_;

    /**
     * @brief Initialize all subsystems with the initial state.
     */
    void initializeSubsystems(SystemState& state);
};

} // namespace pendulum
