#pragma once

#include "utils/type_defs.hpp"

#include <cmath>

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
     * @param vxMax         Maximum horizontal velocity [m/s].
     */
    SlungLoadDynamics(double ropeLengthM, double vxMax = 1e6,
                      double payloadMass = 180.0,
                      double dragCoeff = 1.0,
                      double dragArea = 0.5,
                      double linearDampingCoeff = 0.15,
                      double pendulumGain = 0.6);

    /**
     * @brief Advance the state by one RK4 step.
     *
     * @param[in,out] state  Current state; overwritten with next state.
     * @param axMS2          Horizontal drone acceleration [m/s^2] (control input).
     * @param dt             Integration step [s].
     */
    void step(SystemState& state, double axMS2, double dt);

    double ropeLength() const { return L_; }
    double naturalFrequency() const { return std::sqrt(9.81 / L_); }

private:
    double L_;       ///< Rope length [m]
    double vxMax_;   ///< Max horizontal velocity [m/s]
    double payloadMass_; ///< Payload mass [kg]
    double dragCoeff_;   ///< Drag coefficient Cd [-]
    double dragArea_;    ///< Reference area [m^2]
    double airDensity_;  ///< Air density [kg/m^3]
    double dragFactor_;  ///< Pre-computed (Cd * rho * A * L) / (2 * m)
    double linearDampingCoeff_; ///< Linear damping coefficient [1/s]
    double pendulumGain_;       ///< Pendulum coupling gain (a2 = gain * g * theta) [-]

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
