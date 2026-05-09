#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace pendulum {

/**
 * @brief 3D vector used throughout the project.
 *
 * All components are in SI units (m, m/s, m/s^2, rad/s, etc.) unless
 * otherwise noted.
 */
struct Vector3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Vector3() = default;
    Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    double& operator[](std::size_t i) {
        return (i == 0) ? x : (i == 1) ? y : z;
    }
    const double& operator[](std::size_t i) const {
        return (i == 0) ? x : (i == 1) ? y : z;
    }
};

/**
 * @brief System state for the 1D pitch slung-load dynamics.
 */
struct SystemState {
    double time = 0.0;       ///< Simulation time [s]
    double dronePx = 0.0;    ///< Drone horizontal position [m]
    double droneVx = 0.0;    ///< Drone horizontal velocity [m/s]
    double droneAx = 0.0;    ///< Drone actual horizontal acceleration [m/s^2]
    double theta = 0.0;      ///< Pendulum angle [rad], 0 = hanging straight down
    double thetaDot = 0.0;   ///< Pendulum angular velocity [rad/s]
};

/**
 * @brief Single log entry for closed-loop simulation.
 */
struct ClosedLoopLogEntry {
    double time = 0.0;
    SystemState truth;
    double thetaEstimate = 0.0;    ///< Observed pendulum angle [rad]
    double omegaEstimate = 0.0;    ///< Observed angular rate [rad/s]
    double axCommand = 0.0;        ///< LQR commanded acceleration [m/s^2]
    double axApplied = 0.0;        ///< Actual acceleration after saturation [m/s^2]
    double vRef = 0.0;             ///< Reference velocity [m/s]
};

} // namespace pendulum
