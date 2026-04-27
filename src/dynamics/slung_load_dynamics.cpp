#include "dynamics/slung_load_dynamics.hpp"

#include "utils/math_utils.hpp"

#include <cassert>
#include <cmath>

namespace pendulum {

SlungLoadDynamics::SlungLoadDynamics(double ropeLengthM, double vxMax,
                                     double payloadMass, double dragCoeff,
                                     double dragArea, double linearDampingCoeff)
    : L_(ropeLengthM),
      vxMax_(vxMax),
      payloadMass_(payloadMass),
      dragCoeff_(dragCoeff),
      dragArea_(dragArea),
      airDensity_(1.225),
      linearDampingCoeff_(linearDampingCoeff) {
    assert(L_ > 0.0 && "Rope length must be positive");
    assert(payloadMass_ > 0.0 && "Payload mass must be positive");
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

    // Non-linear pendulum equation with linear + quadratic damping
    // theta_ddot = -(g*sin(theta) + ax*cos(theta)) / L - c_lin * theta_dot + drag_term
    double gravityTorque = kGravity * st;
    double inertialTorque = axMS2 * ct;
    double linearDrag = linearDampingCoeff_ * state.thetaDot;

    // === Air drag based on payload absolute velocity ===
    // Payload absolute velocity in inertial frame (NEU: x-forward, z-up)
    //   vx_payload = vx_drone + L * theta_dot * cos(theta)
    //   vz_payload = L * theta_dot * sin(theta)
    double vxPay = state.droneVx + L_ * state.thetaDot * ct;
    double vzPay = L_ * state.thetaDot * st;
    double vAbsSq = vxPay * vxPay + vzPay * vzPay;
    double vAbs = std::sqrt(vAbsSq);

    double quadraticDragThetaDDot = 0.0;
    if (vAbs > 1e-6) {
        // Drag force magnitude: 0.5 * rho * Cd * A * |v|^2
        double dragForceMag = 0.5 * airDensity_ * dragCoeff_ * dragArea_ * vAbsSq;
        // Drag vector (opposite to velocity)
        double fDragX = -dragForceMag * vxPay / vAbs;
        double fDragZ = -dragForceMag * vzPay / vAbs;
        // Tangential direction (perpendicular to rope, theta-increasing)
        // e_t = (cos(theta), sin(theta))
        double fTangential = fDragX * ct + fDragZ * st;
        // tau = f_tangential * L,  I = m * L^2  =>  thetaDDot_drag = tau / I
        quadraticDragThetaDDot = fTangential / (payloadMass_ * L_);
    }

    dthetaDot = -(gravityTorque + inertialTorque) / L_ - linearDrag + quadraticDragThetaDDot;
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

    // Velocity saturation
    state.droneVx = clamp(state.droneVx, -vxMax_, vxMax_);
}

} // namespace pendulum
