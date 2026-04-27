#pragma once

#include "controller/lqr_gain.hpp"

namespace pendulum {

/**
 * @brief Discrete-time LQR controller for 1-D pitch brake control.
 *
 * Control objective: bring drone velocity to 0 while suppressing swing.
 *
 * State vector: x = [vx, theta, theta_dot]^T
 *   vx       = drone horizontal velocity [m/s]
 *   theta    = pendulum pitch angle [rad]  (0 = hanging down)
 *   theta_dot= pendulum pitch rate  [rad/s]
 *
 * Control input: u = ax  [m/s^2]
 *
 * Control law: u = -K * x, saturated to [-ax_limit, +ax_limit].
 *
 * Three modes:
 *   - kFull:        balance between velocity convergence and swing suppression
 *   - kShortest:    prioritize shortest brake distance (fastest velocity convergence)
 *   - kMinSwing:    prioritize minimal swing angle (gentle braking)
 */
class LqrController {
public:
    struct State {
        double vx = 0.0;      ///< m/s (target is 0)
        double theta = 0.0;   ///< rad
        double omega = 0.0;   ///< rad/s
    };

    /**
     * @param mode      Control mode (selects which LQR gain to use).
     * @param axLimit   Symmetric acceleration limit [m/s^2].
     */
    LqrController(LqrMode mode, double axLimit);

    double computeControl(const State& state) const;

private:
    double kV_ = 0.0;
    double kTheta_ = 0.0;
    double kOmega_ = 0.0;
    double axLimit_;
};

} // namespace pendulum
