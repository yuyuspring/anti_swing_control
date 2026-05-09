#include "standalone_lqr.hpp"

#include <cmath>

namespace lqr {

// Hard-coded LQI gains (L = 10 m, dt = 0.02 s, a1 = F/M convention)
namespace gain {
    constexpr double kDiagonalV        = 3.13288806;
    constexpr double kDiagonalTheta    = -29.13243617;
    constexpr double kDiagonalOmega    = -29.50944395;
    constexpr double kDiagonalIntegral = 0.09375183;

    constexpr double kCoupledV        = 3.16856284;
    constexpr double kCoupledTheta    = -33.48320693;
    constexpr double kCoupledOmega    = -22.53165108;
    constexpr double kCoupledIntegral = 0.09445991;
}

StandaloneLqrSimulator::StandaloneLqrSimulator(const Config& config)
    : config_(config),
      pendulumGain_(config.payloadMass / (config.droneMass + config.payloadMass)) {
    switch (config_.mode) {
        case ControllerMode::kDiagonal:
            kV_ = gain::kDiagonalV;
            kTheta_ = gain::kDiagonalTheta;
            kOmega_ = gain::kDiagonalOmega;
            kIntegral_ = gain::kDiagonalIntegral;
            break;
        case ControllerMode::kCoupled:
            kV_ = gain::kCoupledV;
            kTheta_ = gain::kCoupledTheta;
            kOmega_ = gain::kCoupledOmega;
            kIntegral_ = gain::kCoupledIntegral;
            break;
    }
}

void StandaloneLqrSimulator::computeDerivative(const State& s, double a1,
                                                double& dp, double& dv,
                                                double& dth, double& dwd) const {
    const double st = std::sin(s.theta);
    const double ct = std::cos(s.theta);
    const double mu = pendulumGain_;

    double denom = 1.0 - mu * ct * ct;
    dv = ((1.0 - mu) * a1 + mu * kGravity * st * ct
          + mu * config_.ropeLength * s.omega * s.omega * st) / denom;

    dp = s.v;
    dth = s.omega;

    double gravityTorque = kGravity * st;
    double inertialTorque = dv * ct;
    double linearDrag = config_.linearDampingCoeff * s.omega;

    // Quadratic air drag on payload
    double vxPay = s.v + config_.ropeLength * s.omega * ct;
    double vzPay = config_.ropeLength * s.omega * st;
    double vAbsSq = vxPay * vxPay + vzPay * vzPay;
    double vAbs = std::sqrt(vAbsSq);

    double quadraticDrag = 0.0;
    if (vAbs > 1e-6) {
        double dragForceMag = 0.5 * config_.airDensity * config_.dragCoeff
                              * config_.dragArea * vAbsSq;
        double fDragX = -dragForceMag * vxPay / vAbs;
        double fDragZ = -dragForceMag * vzPay / vAbs;
        double fTangential = fDragX * ct + fDragZ * st;
        quadraticDrag = fTangential / (config_.payloadMass * config_.ropeLength);
    }

    dwd = -(gravityTorque + inertialTorque) / config_.ropeLength
          - linearDrag + quadraticDrag;
}

void StandaloneLqrSimulator::stepRK4(State& s, double a1, double dt) {
    double k1_p, k1_v, k1_t, k1_w;
    double k2_p, k2_v, k2_t, k2_w;
    double k3_p, k3_v, k3_t, k3_w;
    double k4_p, k4_v, k4_t, k4_w;

    computeDerivative(s, a1, k1_p, k1_v, k1_t, k1_w);

    State s2 = s;
    s2.p     += k1_p * dt * 0.5;
    s2.v     += k1_v * dt * 0.5;
    s2.theta += k1_t * dt * 0.5;
    s2.omega += k1_w * dt * 0.5;
    computeDerivative(s2, a1, k2_p, k2_v, k2_t, k2_w);

    State s3 = s;
    s3.p     += k2_p * dt * 0.5;
    s3.v     += k2_v * dt * 0.5;
    s3.theta += k2_t * dt * 0.5;
    s3.omega += k2_w * dt * 0.5;
    computeDerivative(s3, a1, k3_p, k3_v, k3_t, k3_w);

    State s4 = s;
    s4.p     += k3_p * dt;
    s4.v     += k3_v * dt;
    s4.theta += k3_t * dt;
    s4.omega += k3_w * dt;
    computeDerivative(s4, a1, k4_p, k4_v, k4_t, k4_w);

    s.p     += (k1_p + 2.0 * k2_p + 2.0 * k3_p + k4_p) * dt / 6.0;
    s.v     += (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v) * dt / 6.0;
    s.theta += (k1_t  + 2.0 * k2_t  + 2.0 * k3_t  + k4_t)  * dt / 6.0;
    s.omega += (k1_w + 2.0 * k2_w + 2.0 * k3_w + k4_w) * dt / 6.0;
    s.time  += dt;

    double dp_tmp, dv_tmp, dth_tmp, dwd_tmp;
    computeDerivative(s, a1, dp_tmp, dv_tmp, dth_tmp, dwd_tmp);
    s.a = dv_tmp;
}

double StandaloneLqrSimulator::computeControl(double v, double theta,
                                               double omega, double vRef) {
    double err = v - vRef;

    // Conditional integration: only integrate if controller would NOT saturate
    double intPreview = integral_ + err * config_.dt;
    double uPreview = -(kV_ * err + kTheta_ * theta
                        + kOmega_ * omega + kIntegral_ * intPreview);

    if (std::abs(uPreview) < config_.axMax - 1e-6) {
        integral_ = intPreview;
    }

    // Anti-windup clamp
    const double kIntMax = 4.0;
    integral_ = clamp(integral_, -kIntMax, kIntMax);

    double u = -(kV_ * err + kTheta_ * theta
                 + kOmega_ * omega + kIntegral_ * integral_);
    return saturate(u, config_.axMax);
}

double StandaloneLqrSimulator::applyJerkLimit(double target, double dt) {
    double jerk = (target - axCmd_) / dt;
    if (jerk > config_.jerkMax) {
        axCmd_ += config_.jerkMax * dt;
    } else if (jerk < -config_.jerkMax) {
        axCmd_ -= config_.jerkMax * dt;
    } else {
        axCmd_ = target;
    }
    return axCmd_;
}

void StandaloneLqrSimulator::run(const std::vector<double>& vRefSeq,
                                  std::vector<double>& pOut,
                                  std::vector<double>& vOut,
                                  std::vector<double>& aOut,
                                  std::vector<double>& thetaOut,
                                  std::vector<double>& omegaOut) {
    const size_t N = vRefSeq.size();
    pOut.resize(N);
    vOut.resize(N);
    aOut.resize(N);
    thetaOut.resize(N);
    omegaOut.resize(N);

    State state;
    state.p = config_.initialP;
    state.v = config_.initialV;
    state.theta = config_.initialTheta;
    state.omega = config_.initialOmega;

    // Reset controller state
    integral_ = 0.0;
    axCmd_ = 0.0;

    const double dtControl = 0.02;
    int controlStepsPerCall = static_cast<int>(std::round(dtControl / config_.dt));
    if (controlStepsPerCall < 1) controlStepsPerCall = 1;
    int stepCounter = 0;

    for (size_t i = 0; i < N; ++i) {
        double axTarget = axCmd_;  // keep previous if not control step
        if (stepCounter % controlStepsPerCall == 0) {
            axTarget = computeControl(state.v, state.theta, state.omega,
                                      vRefSeq[i]);
        }
        double axCmd = applyJerkLimit(axTarget, config_.dt);
        stepRK4(state, axCmd, config_.dt);

        pOut[i] = state.p;
        vOut[i] = state.v;
        aOut[i] = state.a;
        thetaOut[i] = state.theta;
        omegaOut[i] = state.omega;
        ++stepCounter;
    }
}

std::vector<double> StandaloneLqrSimulator::generateTrapezoid(double dt,
                                                               double tFinal,
                                                               double accelRate,
                                                               double cruiseSpeed,
                                                               double brakeStartTime) {
    size_t N = static_cast<size_t>(std::round(tFinal / dt)) + 1;
    std::vector<double> vref(N);
    double accelEnd = cruiseSpeed / accelRate;

    for (size_t i = 0; i < N; ++i) {
        double t = i * dt;
        if (t < accelEnd) {
            vref[i] = accelRate * t;
        } else if (t < brakeStartTime) {
            vref[i] = cruiseSpeed;
        } else {
            double decelRate = accelRate;  // symmetric
            double v = cruiseSpeed - decelRate * (t - brakeStartTime);
            vref[i] = (v < 0.0) ? 0.0 : v;
        }
    }
    return vref;
}

} // namespace lqr
