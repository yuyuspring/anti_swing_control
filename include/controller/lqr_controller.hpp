#pragma once

namespace pendulum {

/**
 * @brief Discrete-time LQR controller for 1-D pitch slung-load system.
 *
 * State vector: x = [ px_error, vx, theta, theta_dot ]^T
 *   px_error = drone_px - px_target  [m]
 *   vx       = drone horizontal velocity [m/s]
 *   theta    = pendulum pitch angle [rad]  (0 = hanging down)
 *   theta_dot= pendulum pitch rate  [rad/s]
 *
 * Control input: u = ax  [m/s^2]
 *
 * Control law: u = -K * x, saturated to [-ax_limit, +ax_limit].
 *
 * The gain vector K is generated offline by scripts/compute_lqr_gain.py
 * and stored in include/controller/lqr_gain.hpp.
 */
class LqrController {
public:
    struct Gain {
        double kP = 0.0;      ///< Position-error gain
        double kV = 0.0;      ///< Velocity gain
        double kTheta = 0.0;  ///< Angle gain
        double kOmega = 0.0;  ///< Angular-rate gain
    };

    struct State {
        double pxError = 0.0; ///< m
        double vx = 0.0;      ///< m/s
        double theta = 0.0;   ///< rad
        double omega = 0.0;   ///< rad/s
    };

    /**
     * @param gain      LQR gain vector (from lqr_gain.hpp).
     * @param axLimit   Symmetric acceleration limit [m/s^2].
     */
    LqrController(const Gain& gain, double axLimit);

    /**
     * @brief Compute the control acceleration.
     * @return Saturated control input ax [m/s^2].
     */
    double computeControl(const State& state) const;

    const Gain& gain() const { return K_; }

private:
    Gain K_;
    double axLimit_;
};

} // namespace pendulum
