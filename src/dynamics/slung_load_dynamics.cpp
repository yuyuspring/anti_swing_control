#include "dynamics/slung_load_dynamics.hpp"

#include "utils/math_utils.hpp"

#include <cassert>
#include <cmath>

namespace pendulum {

SlungLoadDynamics::SlungLoadDynamics(double ropeLengthM, double dampingRatio)
    : L_(ropeLengthM), zeta_(dampingRatio) {
    assert(L_ > 0.0 && "Rope length must be positive");
    omegaN_ = std::sqrt(kGravity / L_);
}

void SlungLoadDynamics::computeDerivative(const SystemState& state,
                                          double axMS2,
                                          double& dpx,
                                          double& dvx,
                                          double& dtheta,
                                          double& dthetaDot) const {
    dpx = state.droneVx;
    dvx = axMS2;
    dtheta = state.thetaDot;

    const double st = std::sin(state.theta);
    const double ct = std::cos(state.theta);

    // Non-linear pendulum equation with linear damping
    // Corrected sign: theta_ddot = -(g*sin(theta) + ax*cos(theta)) / L - 2*zeta*wn*theta_dot
    dthetaDot = -(kGravity * st + axMS2 * ct) / L_ - 2.0 * zeta_ * omegaN_ * state.thetaDot;
}

void SlungLoadDynamics::step(SystemState& state, double axMS2, double dt) {
    // RK4 integration
    double k1_px, k1_vx, k1_t, k1_td;
    double k2_px, k2_vx, k2_t, k2_td;
    double k3_px, k3_vx, k3_t, k3_td;
    double k4_px, k4_vx, k4_t, k4_td;

    computeDerivative(state, axMS2, k1_px, k1_vx, k1_t, k1_td);

    SystemState s2 = state;
    s2.dronePx  += k1_px * dt * 0.5;
    s2.droneVx  += k1_vx * dt * 0.5;
    s2.theta    += k1_t  * dt * 0.5;
    s2.thetaDot += k1_td * dt * 0.5;
    computeDerivative(s2, axMS2, k2_px, k2_vx, k2_t, k2_td);

    SystemState s3 = state;
    s3.dronePx  += k2_px * dt * 0.5;
    s3.droneVx  += k2_vx * dt * 0.5;
    s3.theta    += k2_t  * dt * 0.5;
    s3.thetaDot += k2_td * dt * 0.5;
    computeDerivative(s3, axMS2, k3_px, k3_vx, k3_t, k3_td);

    SystemState s4 = state;
    s4.dronePx  += k3_px * dt;
    s4.droneVx  += k3_vx * dt;
    s4.theta    += k3_t  * dt;
    s4.thetaDot += k3_td * dt;
    computeDerivative(s4, axMS2, k4_px, k4_vx, k4_t, k4_td);

    state.dronePx  += (k1_px + 2.0 * k2_px + 2.0 * k3_px + k4_px) * dt / 6.0;
    state.droneVx  += (k1_vx + 2.0 * k2_vx + 2.0 * k3_vx + k4_vx) * dt / 6.0;
    state.theta    += (k1_t  + 2.0 * k2_t  + 2.0 * k3_t  + k4_t)  * dt / 6.0;
    state.thetaDot += (k1_td + 2.0 * k2_td + 2.0 * k3_td + k4_td) * dt / 6.0;
    state.time += dt;
}

} // namespace pendulum
