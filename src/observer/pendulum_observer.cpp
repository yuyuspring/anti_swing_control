#include "observer/pendulum_observer.hpp"

#include "utils/math_utils.hpp"

#include <cassert>
#include <cstring>

namespace pendulum {

PendulumObserver::PendulumObserver() {
    std::memset(&obs_, 0, sizeof(obs_));
    std::memset(wLpf_, 0, sizeof(wLpf_));
}

void PendulumObserver::initialize(const Input& input) {
    float acc[3] = {
        static_cast<float>(input.accelMS2.x),
        static_cast<float>(input.accelMS2.y),
        static_cast<float>(input.accelMS2.z)
    };

    float wMeas[3] = {
        static_cast<float>(rad2deg(input.gyroRadS.x)),
        static_cast<float>(rad2deg(input.gyroRadS.y)),
        static_cast<float>(rad2deg(input.gyroRadS.z))
    };

    init_w_lpf_fob(wMeas);
    pend_observer_init(acc, wLpf_, wMeas, &obs_);
}

void PendulumObserver::update(const Input& input, double ropeLengthM) {
    // The underlying C interface expects deg/s for gyro and deg for yaw.
    float wMeas[3] = {
        static_cast<float>(rad2deg(input.gyroRadS.x)),
        static_cast<float>(rad2deg(input.gyroRadS.y)),
        static_cast<float>(rad2deg(input.gyroRadS.z))
    };

    float aMeas[3] = {
        static_cast<float>(input.accelMS2.x),
        static_cast<float>(input.accelMS2.y),
        static_cast<float>(input.accelMS2.z)
    };

    float vMeas[3] = {
        static_cast<float>(input.velocityNeu.x),
        static_cast<float>(input.velocityNeu.y),
        static_cast<float>(input.velocityNeu.z)
    };

    float ahrsInput[3] = {
        static_cast<float>(rad2deg(input.yawRad)),   // yaw [deg]
        0.0f,                                         // pitch [deg] (unused by observer init?)
        0.0f                                          // roll [deg]
    };

    pend_observer_iterate_2(&obs_, wLpf_, wMeas, aMeas, vMeas, ahrsInput,
                            static_cast<float>(ropeLengthM));
}

PendulumObserver::Output PendulumObserver::getOutput() const {
    Output out;
    // theta[0]=yaw, theta[1]=pitch, theta[2]=roll  [rad]
    out.thetaPitch = static_cast<double>(obs_.theta[1]);
    out.thetaRoll  = static_cast<double>(obs_.theta[2]);

    // w_est[0]=roll_rate, w_est[1]=pitch_rate, w_est[2]=yaw_rate  [rad/s]
    out.omegaRoll  = static_cast<double>(obs_.w_est[0]);
    out.omegaPitch = static_cast<double>(obs_.w_est[1]);

    out.vxHat = static_cast<double>(obs_.v_hat[0]);
    out.vyHat = static_cast<double>(obs_.v_hat[1]);
    out.axHat = static_cast<double>(obs_.a_hat[0]);
    out.ayHat = static_cast<double>(obs_.a_hat[1]);

    return out;
}

} // namespace pendulum
