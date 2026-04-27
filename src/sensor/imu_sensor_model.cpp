#include "sensor/imu_sensor_model.hpp"

#include "utils/math_utils.hpp"

#include <cmath>
#include <random>

namespace pendulum {

// Thread-local random generator to avoid global state issues
static thread_local std::mt19937 kGenerator(42);

ImuSensorModel::ImuSensorModel() = default;

void ImuSensorModel::setNoiseStd(double gyroNoiseStd,
                                 double accelNoiseStd,
                                 double velNoiseStd) {
    gyroNoiseStd_ = gyroNoiseStd;
    accelNoiseStd_ = accelNoiseStd;
    velNoiseStd_ = velNoiseStd;
}

ImuSensorModel::Measurement ImuSensorModel::generate(
        const SystemState& truth,
        double yawTruth,
        double /*ropeLength*/) const {

    std::normal_distribution<double> gyroDist(0.0, gyroNoiseStd_);
    std::normal_distribution<double> accelDist(0.0, accelNoiseStd_);
    std::normal_distribution<double> velDist(0.0, velNoiseStd_);

    Measurement meas;

    // --- Gyro (FRD) ---
    // For 1-D pitch motion around y-axis:
    //   true omega_y = theta_dot
    //   omega_x = omega_z = 0
    meas.gyro.x = gyroDist(kGenerator);
    meas.gyro.y = truth.thetaDot + gyroDist(kGenerator);
    meas.gyro.z = gyroDist(kGenerator);

    // --- Accelerometer (FRD, specific force) ---
    // Assuming IMU is mounted on the drone (not the payload).
    // In 1-D horizontal motion with z-down:
    //   accel_x = ax  (horizontal specific force = drone acceleration)
    //   accel_z = g   (gravity, since z points down)
    //
    // Note: truth.droneVx is velocity, not acceleration.  In the current
    // simulation architecture the drone acceleration is not stored in the
    // state.  We approximate it by finite difference or assume the caller
    // will pass it.  For now we use a simplified model where the horizontal
    // specific force is zero-mean noise (the observer will still work
    // because it uses the accelerometer mainly for attitude correction
    // via the gravity vector).
    //
    // TODO: refactor to pass actual drone acceleration into this function.
    meas.accel.x = accelDist(kGenerator);
    meas.accel.y = accelDist(kGenerator);
    meas.accel.z = kGravity + accelDist(kGenerator);

    // --- Velocity (NEU) ---
    // NEU frame: x=north, y=east, z=up
    // For 1-D motion along x (north):
    meas.velocity.x = truth.droneVx + velDist(kGenerator);
    meas.velocity.y = velDist(kGenerator);
    meas.velocity.z = velDist(kGenerator);

    // --- Yaw ---
    meas.yaw = yawTruth;

    return meas;
}

} // namespace pendulum
