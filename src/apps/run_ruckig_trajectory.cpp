#include "trajectory/ruckig_planner.hpp"
#include "utils/csv_logger.hpp"

#include <iostream>
#include <vector>
#include <iomanip>

using pendulum::RuckigPlanner;
using pendulum::TrajectoryState;
using pendulum::TrajectoryConstraints;
using pendulum::CsvLogger;

struct TrajectorySample {
    double time;
    double position;
    double velocity;
    double acceleration;
    double jerk;
};

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  Ruckig S-Curve Trajectory Generation" << std::endl;
    std::cout << "========================================" << std::endl;

    // Scenario: move from 0 m to 50 m, start/end at rest
    TrajectoryState current{0.0, 0.0, 0.0};
    TrajectoryState target{50.0, 0.0, 0.0};
    TrajectoryConstraints limits{15.0, 2.0, 2.0};

    RuckigPlanner planner(0.005); // 5 ms sampling

    double duration = planner.plan(current, target, limits);
    if (duration < 0.0) {
        std::cerr << "Trajectory planning failed." << std::endl;
        return 1;
    }

    std::cout << "Trajectory duration: " << std::fixed << std::setprecision(3)
              << duration << " s" << std::endl;

    // Sample trajectory
    double dt = 0.005;
    int n_samples = static_cast<int>(duration / dt) + 1;
    std::vector<TrajectorySample> samples;
    samples.reserve(n_samples + 1);

    for (int i = 0; i <= n_samples; ++i) {
        double t = i * dt;
        if (t > duration) t = duration;

        auto state = planner.sample(t);
        if (!state.has_value()) break;

        // Approximate jerk by finite difference
        double jerk = 0.0;
        if (i > 0 && !samples.empty()) {
            jerk = (state->acceleration - samples.back().acceleration) / dt;
        }

        samples.push_back({t, state->position, state->velocity,
                           state->acceleration, jerk});
    }

    // Save to CSV
    CsvLogger logger("results/ruckig_trajectory.csv");
    if (logger.isOpen()) {
        logger.writeHeader({"time_s", "px_m", "vx_m_s", "ax_m_s2", "jerk_m_s3"});
        for (const auto& s : samples) {
            logger.writeRow({s.time, s.position, s.velocity,
                            s.acceleration, s.jerk});
        }
        logger.flush();
        std::cout << "Saved: results/ruckig_trajectory.csv (" << samples.size()
                  << " samples)" << std::endl;
    }

    // Print a few key points
    std::cout << "\nKey samples:" << std::endl;
    for (double t : {0.0, duration * 0.25, duration * 0.5,
                     duration * 0.75, duration}) {
        auto state = planner.sample(t);
        if (state.has_value()) {
            std::cout << "  t=" << std::setw(6) << t << "s:  px=" << state->position
                      << "  vx=" << state->velocity
                      << "  ax=" << state->acceleration << std::endl;
        }
    }

    std::cout << "\nDone. Run: python3 scripts/plot/plot_trajectory.py" << std::endl;
    return 0;
}
