#include "simulation/closed_loop_simulation.hpp"

#include "controller/lqr_gain.hpp"
#include "utils/csv_logger.hpp"
#include "utils/math_utils.hpp"

#include <cmath>
#include <iostream>

namespace pendulum {

ClosedLoopSimulation::ClosedLoopSimulation(const ClosedLoopConfig& config)
    : config_(config),
      dynamics_(config.ropeLength, config.dampingRatio),
      controller_(LqrController::Gain{LqrGain::kP, LqrGain::kV,
                                       LqrGain::kTheta, LqrGain::kOmega},
                  config.axMax) {
    sensor_.setNoiseStd(config.gyroNoiseStd,
                        config.accelNoiseStd,
                        config.velNoiseStd);
}

void ClosedLoopSimulation::initializeSubsystems(SystemState& state) {
    state.time = 0.0;
    state.dronePx = config_.pStart;
    state.droneVx = 0.0;
    state.theta = config_.initialTheta;
    state.thetaDot = config_.initialThetaDot;

    // Initialize observer with the ground-truth measurement (no noise)
    // so that the observer starts from a consistent attitude.
    PendulumObserver::Input obsInput;
    obsInput.gyroRadS = Vector3(0.0, state.thetaDot, 0.0);
    obsInput.accelMS2 = Vector3(0.0, 0.0, kGravity);
    obsInput.velocityNeu = Vector3(state.droneVx, 0.0, 0.0);
    obsInput.yawRad = 0.0;

    observer_.initialize(obsInput);
}

void ClosedLoopSimulation::run() {
    SystemState state;
    initializeSubsystems(state);

    log_.clear();
    log_.reserve(static_cast<size_t>(config_.tFinal / config_.dtTruth) + 10);

    double axCmd = 0.0;
    PendulumObserver::Output est{};

    const int controlStepsPerCall =
        static_cast<int>(std::round(config_.dtControl / config_.dtTruth));
    int stepCounter = 0;

    std::cout << "[ClosedLoopSimulation] Starting simulation...\n"
              << "  Target position : " << config_.pTarget << " m\n"
              << "  Rope length     : " << config_.ropeLength << " m\n"
              << "  Max acceleration: " << config_.axMax << " m/s^2\n"
              << "  Initial theta   : " << rad2deg(config_.initialTheta) << " deg\n"
              << "  Duration        : " << config_.tFinal << " s\n"
              << "  Control period  : " << config_.dtControl << " s\n";

    while (state.time <= config_.tFinal + 1e-9) {
        // Control update at every dtControl interval
        if (stepCounter % controlStepsPerCall == 0) {
            // 1. Sensor measurement from truth
            auto meas = sensor_.generate(state, 0.0, config_.ropeLength);

            // 2. Observer update
            PendulumObserver::Input obsInput;
            obsInput.gyroRadS = meas.gyro;
            obsInput.accelMS2 = meas.accel;
            obsInput.velocityNeu = meas.velocity;
            obsInput.yawRad = meas.yaw;
            observer_.update(obsInput, config_.ropeLength);
            est = observer_.getOutput();

            // 3. LQR control
            LqrController::State lqrState;
            lqrState.pxError = state.dronePx - config_.pTarget;
            lqrState.vx = state.droneVx;
            lqrState.theta = est.thetaPitch;
            lqrState.omega = est.omegaPitch;

            axCmd = controller_.computeControl(lqrState);
        }

        // 4. Record before stepping
        ClosedLoopLogEntry entry;
        entry.time = state.time;
        entry.truth = state;
        entry.thetaEstimate = est.thetaPitch;
        entry.omegaEstimate = est.omegaPitch;
        entry.axCommand = axCmd;
        entry.axApplied = axCmd;  // No actuator dynamics for now
        entry.pxError = state.dronePx - config_.pTarget;
        log_.push_back(entry);

        // 5. Advance true dynamics
        dynamics_.step(state, axCmd, config_.dtTruth);

        ++stepCounter;
    }

    std::cout << "[ClosedLoopSimulation] Finished. Logged " << log_.size()
              << " samples.\n";
}

void ClosedLoopSimulation::saveResults(const std::string& filename) const {
    CsvLogger logger(filename);
    if (!logger.isOpen()) {
        std::cerr << "[ClosedLoopSimulation] Failed to open " << filename << "\n";
        return;
    }

    logger.writeHeader({
        "time_s",
        "px_truth_m", "vx_truth_m_s",
        "theta_truth_rad", "theta_dot_truth_rad_s",
        "theta_est_rad", "omega_est_rad_s",
        "ax_cmd_m_s2", "ax_applied_m_s2",
        "px_error_m"
    });

    for (const auto& e : log_) {
        logger.writeRow({
            e.time,
            e.truth.dronePx,
            e.truth.droneVx,
            e.truth.theta,
            e.truth.thetaDot,
            e.thetaEstimate,
            e.omegaEstimate,
            e.axCommand,
            e.axApplied,
            e.pxError
        });
    }

    logger.flush();
    std::cout << "[ClosedLoopSimulation] Results saved to " << filename << "\n";
}

} // namespace pendulum
