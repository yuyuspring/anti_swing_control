#pragma once

#include <cmath>

namespace pendulum {

/**
 * @brief Mathematical constants and utility functions.
 */

constexpr double kGravity = 9.81;       ///< Gravitational acceleration [m/s^2]
constexpr double kPi = 3.14159265358979323846;

inline double deg2rad(double deg) { return deg * kPi / 180.0; }
inline double rad2deg(double rad) { return rad * 180.0 / kPi; }

/**
 * @brief Clamp a value to the range [min_val, max_val].
 */
double clamp(double value, double min_val, double max_val);

/**
 * @brief Saturate a value symmetrically to [-limit, +limit].
 */
double saturate(double value, double limit);

} // namespace pendulum
