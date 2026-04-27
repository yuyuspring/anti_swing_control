#pragma once

#include "utils/type_defs.hpp"

namespace pendulum {

/**
 * @brief Generate realistic IMU measurements from ground-truth state.
 *
 * Converts the true kinematic state into FRD gyro / accel and NEU velocity
 * with additive Gaussian noise.  The gravity vector is handled according to
 * the specific-force convention (accelerometer reads +g when stationary
 * with z-axis pointing down).
 */
class ImuSensorModel {
public:
    struct Measurement {
        Vector3 gyro;      ///< Angular rate, FRD, rad/s
        Vector3 accel;     ///< Specific force, FRD, m/s^2
        Vector3 velocity;  ///< Navigation velocity, NEU, m/s
        double yaw = 0.0;  ///< Heading, rad
    };

    ImuSensorModel();

    /**
     * @brief Set the standard deviations of measurement noise.
     * @param gyroNoiseStd   Gyro noise [rad/s]
     * @param accelNoiseStd  Accelerometer noise [m/s^2]
     * @param velNoiseStd    Velocity noise [m/s]
     */
    void setNoiseStd(double gyroNoiseStd,
                     double accelNoiseStd,
                     double velNoiseStd);

    /**
     * @brief Generate a noisy measurement from ground truth.
     *
     * For the 1-D pitch case the roll/yaw channels are zero and the yaw
     * argument is passed through unchanged.
     *
     * @param truth      Ground-truth system state.
     * @param yawTruth   True yaw angle [rad] (assumed measured directly).
     * @param ropeLength Rope length [m].
     */
    Measurement generate(const SystemState& truth,
                         double yawTruth,
                         double ropeLength) const;

private:
    double gyroNoiseStd_ = 0.0;
    double accelNoiseStd_ = 0.0;
    double velNoiseStd_ = 0.0;
};

} // namespace pendulum
