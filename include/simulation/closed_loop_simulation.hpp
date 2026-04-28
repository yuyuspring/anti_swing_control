#pragma once

#include "controller/lqr_controller.hpp"
#include "dynamics/slung_load_dynamics.hpp"
#include "observer/pendulum_observer.hpp"
#include "sensor/imu_sensor_model.hpp"
#include "utils/type_defs.hpp"

#include <string>
#include <vector>

namespace pendulum {

enum class ControlMode {
    kFull,          ///< 均衡模式
    kShortest,      ///< 最短刹车距离
    kMinSwing,      ///< 最小摆角
    kVelocityOmega,   ///< 速度+角速度联合抑制
    kPayloadVelocity, ///< payload 绝对速度抑制
    kMinEnergy,       ///< 最小能量模式
    kSystemEnergy     ///< 系统总能量最低（无人机+摆）
};
struct ClosedLoopConfig {
    double dtTruth = 0.001;          ///< Ground-truth dynamics step [s]
    double dtControl = 0.02;         ///< Control / observer period [s]
    double ropeLength = 15.0;        ///< [m]
    double axMax = 2.0;              ///< Max horizontal acceleration [m/s^2]
    double vxMax = 15.0;             ///< Max horizontal velocity [m/s]
    double jerkMax = 2.0;            ///< Max jerk [m/s^3]
    ControlMode mode = ControlMode::kFull; ///< Control strategy
    double tFinal = 60.0;            ///< Total simulation time [s]
    double pStart = 0.0;             ///< Initial position [m]
    double cruiseSpeed = 15.0;       ///< Cruise speed during constant-velocity phase [m/s]
    double brakeStartTime = 40.0;    ///< Time to start braking [s]
    double initialTheta = 0.0;       ///< Initial pendulum angle [rad]
    double initialThetaDot = 0.0;    ///< Initial angular rate [rad/s]

    // Damping parameters
    double payloadMass = 150.0;      ///< Payload mass [kg]
    double dragCoeff = 1.0;          ///< Drag coefficient Cd [-]
    double dragArea = 0.5;           ///< Reference area [m^2]
    double linearDampingCoeff = 0.15; ///< Linear damping coefficient [1/s]

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
    ClosedLoopSimulation(const ClosedLoopConfig& config, ControlMode mode);

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
    ControlMode mode_;
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
