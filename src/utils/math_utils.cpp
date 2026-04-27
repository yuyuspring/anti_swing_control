#include "utils/math_utils.hpp"

#include <algorithm>

namespace pendulum {

double clamp(double value, double min_val, double max_val) {
    return std::max(min_val, std::min(value, max_val));
}

double saturate(double value, double limit) {
    return clamp(value, -limit, limit);
}

} // namespace pendulum
