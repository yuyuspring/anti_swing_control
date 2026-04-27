#include "simulation/closed_loop_simulation.hpp"

#include "controller/lqr_gain.hpp"
#include "utils/csv_logger.hpp"
#include "utils/math_utils.hpp"

#include <cmath>
#include <iostream>

namespace pendulum {

ClosedLoopSimulation::ClosedLoopSimulation(const ClosedLoopConfig& config,
                                             ControlMode mode)
    : config_(config),
      mode_(mode),
      dynamics_(config.ropeLength, config.vxMax,
                config.payloadMass, config.dragCoeff, config.dragArea,
                config.linearDampingCoeff),
      controller_(static_cast<LqrMode>(mode), config.axMax) {
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

    double axTarget = 0.0;   // LQR / open-loop target acceleration
    double axCmd = 0.0;      // Actual acceleration after jerk limiting
    PendulumObserver::Output est{};

    const int controlStepsPerCall =
        static_cast<int>(std::round(config_.dtControl / config_.dtTruth));
    int stepCounter = 0;

    // Compute acceleration phase end time
    const double accelPhaseEnd = config_.cruiseSpeed / config_.axMax;

    std::cout << "[ClosedLoopSimulation] Starting simulation...\n"
              << "  Mode            : "
              << (mode_ == ControlMode::kFull ? "Full" :
                  mode_ == ControlMode::kShortest ? "Shortest" :
                  mode_ == ControlMode::kMinSwing ? "MinSwing" : "VelocityOmega")
              << "\n"
              << "  Start position  : " << config_.pStart << " m\n"
              << "  Cruise speed    : " << config_.cruiseSpeed << " m/s\n"
              << "  Accel phase     : 0 ~ " << accelPhaseEnd << " s\n"
              << "  Cruise phase    : " << accelPhaseEnd << " ~ " << config_.brakeStartTime << " s\n"
              << "  Brake phase     : " << config_.brakeStartTime << " ~ " << config_.tFinal << " s\n"
              << "  Rope length     : " << config_.ropeLength << " m\n"
              << "  Max acceleration: " << config_.axMax << " m/s^2\n"
              << "  Max velocity    : " << config_.vxMax << " m/s\n"
              << "  Max jerk        : " << config_.jerkMax << " m/s^3\n"
              << "  Initial theta   : " << rad2deg(config_.initialTheta) << " deg\n"
              << "  Duration        : " << config_.tFinal << " s\n"
              << "  Control period  : " << config_.dtControl << " s\n";

    while (state.time <= config_.tFinal + 1e-9) {
        // Observer update at every dtControl interval (runs in all phases)
        if (stepCounter % controlStepsPerCall == 0) {
            auto meas = sensor_.generate(state, 0.0, config_.ropeLength);

            PendulumObserver::Input obsInput;
            obsInput.gyroRadS = meas.gyro;
            obsInput.accelMS2 = meas.accel;
            obsInput.velocityNeu = meas.velocity;
            obsInput.yawRad = meas.yaw;
            observer_.update(obsInput, config_.ropeLength);
            est = observer_.getOutput();
        }

        // Phase-dependent control: compute target acceleration
        if (state.time < accelPhaseEnd) {
            // Phase 1: Acceleration
            axTarget = config_.axMax;
        } else if (state.time < config_.brakeStartTime) {
            // Phase 2: Constant velocity
            axTarget = 0.0;
        } else {
            // Phase 3: Braking (LQR takes over)
            if (stepCounter % controlStepsPerCall == 0) {
                LqrController::State lqrState;
                lqrState.vx = state.droneVx;
                lqrState.theta = est.thetaPitch;
                lqrState.omega = est.omegaPitch;
                axTarget = controller_.computeControl(lqrState);
            }
        }

        // Apply jerk limit: axCmd can only change by jerkMax * dt per step
        double jerk = (axTarget - axCmd) / config_.dtTruth;
        if (jerk > config_.jerkMax) {
            axCmd += config_.jerkMax * config_.dtTruth;
        } else if (jerk < -config_.jerkMax) {
            axCmd -= config_.jerkMax * config_.dtTruth;
        } else {
            axCmd = axTarget;
        }

        // Record before stepping
        ClosedLoopLogEntry entry;
        entry.time = state.time;
        entry.truth = state;
        entry.thetaEstimate = est.thetaPitch;
        entry.omegaEstimate = est.omegaPitch;
        entry.axCommand = axTarget;
        entry.axApplied = axCmd;
        log_.push_back(entry);

        // Advance true dynamics
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
        "ax_cmd_m_s2", "ax_applied_m_s2"
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
            e.axApplied
        });
    }

    logger.flush();
    std::cout << "[ClosedLoopSimulation] Results saved to " << filename << "\n";
}

} // namespace pendulum
