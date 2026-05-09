#include "controller/lqr_controller.hpp"

#include "utils/math_utils.hpp"

namespace pendulum {

LqrController::LqrController(LqrMode mode, double axLimit) : axLimit_(axLimit) {
    switch (mode) {
        case LqrMode::kDiagonal:
            kV_ = LqrGain::kDiagonalV;
            kTheta_ = LqrGain::kDiagonalTheta;
            kOmega_ = LqrGain::kDiagonalOmega;
            kIntegral_ = LqrGain::kDiagonalIntegral;
            break;
        case LqrMode::kCoupled:
            kV_ = LqrGain::kCoupledV;
            kTheta_ = LqrGain::kCoupledTheta;
            kOmega_ = LqrGain::kCoupledOmega;
            kIntegral_ = LqrGain::kCoupledIntegral;
            break;
    }
}

double LqrController::computeControl(const State& state, double vxRef) const {
    double u = -(kV_ * (state.vx - vxRef) +
                 kTheta_ * state.theta +
                 kOmega_ * state.omega +
                 kIntegral_ * state.vxIntegral);
    return saturate(u, axLimit_);
}

} // namespace pendulum
