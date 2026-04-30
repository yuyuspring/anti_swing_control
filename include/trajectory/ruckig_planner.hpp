#pragma once

#include "ruckig/ruckig.hpp"
#include <array>
#include <optional>

namespace pendulum {

struct TrajectoryConstraints {
    double v_max = 15.0;
    double a_max = 2.0;
    double j_max = 2.0;
};

struct TrajectoryState {
    double position = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
};

class RuckigPlanner {
public:
    explicit RuckigPlanner(double control_cycle = 0.005);

    /**
     * Plan a time-optimal S-curve trajectory from current to target state.
     *
     * @param current   Current {position, velocity, acceleration}
     * @param target    Target {position, velocity, acceleration}
     * @param limits    {v_max, a_max, j_max}
     * @return  Trajectory duration [s], or -1.0 on failure
     */
    double plan(const TrajectoryState& current,
                const TrajectoryState& target,
                const TrajectoryConstraints& limits);

    /**
     * Sample the planned trajectory at time t.
     *
     * @param t     Time [s], 0 <= t <= duration
     * @return    State at time t. std::nullopt if no trajectory or t out of range.
     */
    std::optional<TrajectoryState> sample(double t) const;

    /** Return the planned trajectory duration [s]. */
    double duration() const;

    /** Return true if a valid trajectory is stored. */
    bool hasTrajectory() const;

private:
    double control_cycle_;
    std::optional<ruckig::Trajectory<1>> trajectory_;
};

} // namespace pendulum
