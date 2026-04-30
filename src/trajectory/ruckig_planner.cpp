#include "trajectory/ruckig_planner.hpp"
#include <iostream>

namespace pendulum {

RuckigPlanner::RuckigPlanner(double control_cycle)
    : control_cycle_(control_cycle) {}

double RuckigPlanner::plan(const TrajectoryState& current,
                           const TrajectoryState& target,
                           const TrajectoryConstraints& limits) {
    ruckig::InputParameter<1> input;
    input.control_interface = ruckig::ControlInterface::Position;

    input.current_position[0]     = current.position;
    input.current_velocity[0]     = current.velocity;
    input.current_acceleration[0] = current.acceleration;

    input.target_position[0]      = target.position;
    input.target_velocity[0]      = target.velocity;
    input.target_acceleration[0]  = target.acceleration;

    input.max_velocity[0]     = limits.v_max;
    input.max_acceleration[0] = limits.a_max;
    input.max_jerk[0]         = limits.j_max;

    // Allow slight negative velocity for numerical robustness near stop
    input.min_velocity = {-0.5};

    ruckig::Ruckig<1> ruckig(control_cycle_);
    ruckig::Trajectory<1> trajectory;
    ruckig::Result result = ruckig.calculate(input, trajectory);

    if (result == ruckig::Result::Finished || result == ruckig::Result::Working) {
        trajectory_ = trajectory;
        return trajectory.get_duration();
    }

    std::cerr << "[RuckigPlanner] Trajectory planning failed: "
              << static_cast<int>(result) << std::endl;
    trajectory_ = std::nullopt;
    return -1.0;
}

std::optional<TrajectoryState> RuckigPlanner::sample(double t) const {
    if (!trajectory_.has_value()) {
        return std::nullopt;
    }

    const auto& traj = trajectory_.value();
    if (t < 0.0 || t > traj.get_duration() + 1e-9) {
        return std::nullopt;
    }

    std::array<double, 1> p, v, a;
    traj.at_time(t, p, v, a);

    return TrajectoryState{p[0], v[0], a[0]};
}

double RuckigPlanner::duration() const {
    if (!trajectory_.has_value()) {
        return -1.0;
    }
    return trajectory_.value().get_duration();
}

bool RuckigPlanner::hasTrajectory() const {
    return trajectory_.has_value();
}

} // namespace pendulum
