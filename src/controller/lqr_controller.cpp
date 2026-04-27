#include "controller/lqr_controller.hpp"

#include "utils/math_utils.hpp"

namespace pendulum {

LqrController::LqrController(LqrMode mode, double axLimit) : axLimit_(axLimit) {
    switch (mode) {
        case LqrMode::kFull:
            kV_ = LqrGain::kFullV;
            kTheta_ = LqrGain::kFullTheta;
            kOmega_ = LqrGain::kFullOmega;
            break;
        case LqrMode::kShortest:
            kV_ = LqrGain::kShortestV;
            kTheta_ = LqrGain::kShortestTheta;
            kOmega_ = LqrGain::kShortestOmega;
            break;
        case LqrMode::kMinSwing:
            kV_ = LqrGain::kMinSwingV;
            kTheta_ = LqrGain::kMinSwingTheta;
            kOmega_ = LqrGain::kMinSwingOmega;
            break;
        case LqrMode::kVelocityOmega:
            kV_ = LqrGain::kVelocityOmegaV;
            kTheta_ = LqrGain::kVelocityOmegaTheta;
            kOmega_ = LqrGain::kVelocityOmegaOmega;
            break;
    }
}

double LqrController::computeControl(const State& state) const {
    double u = -(kV_ * state.vx +
                 kTheta_ * state.theta +
                 kOmega_ * state.omega);
    return saturate(u, axLimit_);
}

} // namespace pendulum
