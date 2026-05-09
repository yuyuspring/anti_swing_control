#pragma once

#include "controller/lqr_gain.hpp"

namespace pendulum {

/**
 * @brief Discrete-time LQI controller for 1-D pitch brake control.
 *
 * Control objective: track velocity reference while suppressing swing.
 *
 * State vector: x = [vx_err, theta, theta_dot, int_vx_err]^T
 *   vx_err     = drone horizontal velocity error [m/s]
 *   theta      = pendulum pitch angle [rad]  (0 = hanging down)
 *   theta_dot  = pendulum pitch rate  [rad/s]
 *   int_vx_err = integral of velocity error [m]
 *
 * Control input: u = ax  [m/s^2]
 *
 * Control law: u = -K * x, saturated to [-ax_limit, +ax_limit].
 */
class LqrController {
public:
    struct State {
        double vx = 0.0;        ///< m/s (error = vx - vx_ref)
        double theta = 0.0;     ///< rad
        double omega = 0.0;     ///< rad/s
        double vxIntegral = 0.0; ///< m (integral of velocity error)
    };

    /**
     * @param mode      Control mode (selects which LQI gain to use).
     * @param axLimit   Symmetric acceleration limit [m/s^2].
     */
    LqrController(LqrMode mode, double axLimit);

    /**
     * @brief Compute LQI control input.
     *
     * @param state  Current state [vx, theta, omega, vxIntegral].
     * @param vxRef  Reference velocity [m/s]. Default 0.
     * @return Saturated acceleration command [m/s^2].
     */
    double computeControl(const State& state, double vxRef = 0.0) const;

private:
    double kV_ = 0.0;
    double kTheta_ = 0.0;
    double kOmega_ = 0.0;
    double kIntegral_ = 0.0;
    double axLimit_;
};

} // namespace pendulum
