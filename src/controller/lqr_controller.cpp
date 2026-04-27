#include "controller/lqr_controller.hpp"

#include "utils/math_utils.hpp"

namespace pendulum {

LqrController::LqrController(const Gain& gain, double axLimit)
    : K_(gain), axLimit_(axLimit) {}

double LqrController::computeControl(const State& state) const {
    double u = -(K_.kP * state.pxError +
                 K_.kV * state.vx +
                 K_.kTheta * state.theta +
                 K_.kOmega * state.omega);
    return saturate(u, axLimit_);
}

} // namespace pendulum
