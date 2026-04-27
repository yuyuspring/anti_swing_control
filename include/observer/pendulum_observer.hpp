#pragma once

#include "observer/pend_observer.h"
#include "utils/type_defs.hpp"

#include <array>

namespace pendulum {

/**
 * @brief C++ wrapper around the legacy C-style pendulum observer.
 *
 * This class shields callers from the unit-conversion traps of the underlying
 * C interface.  All public methods use SI units (rad, rad/s, m/s, m/s^2).
 * Internally rad/deg conversions are performed before calling
 * pend_observer_iterate_2, which expects deg/s for gyro and deg for yaw.
 */
class PendulumObserver {
public:
    struct Input {
        Vector3 gyroRadS;      ///< Body angular rate, FRD, rad/s
        Vector3 accelMS2;      ///< Specific force, FRD, m/s^2
        Vector3 velocityNeu;   ///< Navigation velocity, NEU, m/s
        double yawRad = 0.0;   ///< Heading angle, rad
    };

    struct Output {
        double thetaPitch = 0.0;   ///< Estimated pitch angle, rad
        double thetaRoll = 0.0;    ///< Estimated roll angle, rad
        double omegaPitch = 0.0;   ///< Estimated pitch rate, rad/s
        double omegaRoll = 0.0;    ///< Estimated roll rate, rad/s
        double vxHat = 0.0;        ///< Observed north velocity, m/s
        double vyHat = 0.0;        ///< Observed east velocity, m/s
        double axHat = 0.0;        ///< Observed north acceleration, m/s^2
        double ayHat = 0.0;        ///< Observed east acceleration, m/s^2
    };

    PendulumObserver();

    /**
     * @brief Initialize the observer with the first measurement.
     */
    void initialize(const Input& input);

    /**
     * @brief Single update step.
     * @param input   Sensor measurements (SI units).
     * @param ropeLengthM  Rope length in metres.
     */
    void update(const Input& input, double ropeLengthM);

    /**
     * @brief Get the latest observer output.
     */
    Output getOutput() const;

private:
    ::PendObserver obs_;
    float wLpf_[3];
};

} // namespace pendulum
