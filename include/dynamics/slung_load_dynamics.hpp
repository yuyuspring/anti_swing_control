#pragma once

#include "utils/type_defs.hpp"

namespace pendulum {

/**
 * @brief Non-linear slung-load dynamics for 1-D pitch motion.
 *
 * The payload is modelled as a point mass suspended by a rigid rope of
 * fixed length L.  The drone's horizontal acceleration a_x is the control
 * input.
 *
 * Equation of motion:
 *   theta_ddot = (g*sin(theta) - a_x*cos(theta)) / L - 2*zeta*wn*theta_dot
 * where wn = sqrt(g/L).
 */
class SlungLoadDynamics {
public:
    /**
     * @param ropeLengthM   Rope length [m].  Must be > 0.
     * @param dampingRatio  Dimensionless damping ratio (zeta).
     */
    SlungLoadDynamics(double ropeLengthM, double dampingRatio);

    /**
     * @brief Advance the state by one RK4 step.
     *
     * @param[in,out] state  Current state; overwritten with next state.
     * @param axMS2          Horizontal drone acceleration [m/s^2] (control input).
     * @param dt             Integration step [s].
     */
    void step(SystemState& state, double axMS2, double dt);

    double ropeLength() const { return L_; }
    double naturalFrequency() const { return omegaN_; }

private:
    double L_;       ///< Rope length [m]
    double zeta_;    ///< Damping ratio [-]
    double omegaN_;  ///< Natural frequency sqrt(g/L) [rad/s]

    /**
     * @brief Compute the state derivative f(state, u).
     */
    void computeDerivative(const SystemState& state,
                           double axMS2,
                           double& dpx,
                           double& dvx,
                           double& dtheta,
                           double& dthetaDot) const;
};

} // namespace pendulum
