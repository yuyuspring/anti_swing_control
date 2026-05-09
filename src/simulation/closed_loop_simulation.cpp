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
                config.linearDampingCoeff, config.pendulumGain),
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
    double vxErrorIntegral = 0.0;  // LQI integral state
    double vxErrorPrev = 0.0;      // Previous step error for decay logic
    PendulumObserver::Output est{};

    const int controlStepsPerCall =
        static_cast<int>(std::round(config_.dtControl / config_.dtTruth));
    int stepCounter = 0;

    // Trapezoidal reference parameters
    const double accelRate = 2.0;   // reference acceleration
    const double accelPhaseEnd = config_.cruiseSpeed / accelRate;  // = 7.5 s

    std::cout << "[ClosedLoopSimulation] Starting simulation...\n"
              << "  Mode            : "
              << (mode_ == ControlMode::kDiagonal ? "Diagonal" : "Coupled")
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
              << "  Pendulum gain   : " << config_.pendulumGain << " [-]\n"
              << "  Initial theta   : " << rad2deg(config_.initialTheta) << " deg\n"
              << "  Duration        : " << config_.tFinal << " s\n"
              << "  Control period  : " << config_.dtControl << " s\n";

    while (state.time <= config_.tFinal + 1e-9) {
        // Observer update at every dtControl interval
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

        // Reference velocity: trapezoidal profile (1 m/s² accel / decel)
        double vRef = 0.0;
        if (state.time < accelPhaseEnd) {
            // Acceleration: linear ramp 0 -> cruiseSpeed
            vRef = accelRate * state.time;
        } else if (state.time < config_.brakeStartTime) {
            // Cruise
            vRef = config_.cruiseSpeed;
        } else {
            // Deceleration: linear ramp cruiseSpeed -> 0
            double decelRate = 2.0;
            vRef = config_.cruiseSpeed - decelRate * (state.time - config_.brakeStartTime);
            if (vRef < 0.0) vRef = 0.0;
        }

        // LQI closed-loop control (all phases)
        if (stepCounter % controlStepsPerCall == 0) {
            double vxError = state.droneVx - vRef;

            // Conditional integration: only integrate if controller would NOT saturate
            double intPreview = vxErrorIntegral + vxError * config_.dtControl;
            LqrController::State previewState;
            previewState.vx = state.droneVx;
            previewState.theta = est.thetaPitch;
            previewState.omega = est.omegaPitch;
            previewState.vxIntegral = intPreview;
            double uPreview = controller_.computeControl(previewState, vRef);

            if (std::abs(uPreview) < config_.axMax - 1e-6) {
                vxErrorIntegral = intPreview;
            }

            // Integral decay: when error and integral have same sign and error is shrinking,
            // decay the integral faster to reduce overshoot
            if (vxError * vxErrorIntegral > 0.0 && std::abs(vxError) < std::abs(vxErrorPrev)) {
                vxErrorIntegral *= 0.9;
            }
            vxErrorPrev = vxError;

            // Anti-windup clamp
            const double kIntMax = 3.0;
            if (vxErrorIntegral > kIntMax) vxErrorIntegral = kIntMax;
            if (vxErrorIntegral < -kIntMax) vxErrorIntegral = -kIntMax;

            LqrController::State lqrState;
            lqrState.vx = state.droneVx;
            lqrState.theta = est.thetaPitch;
            lqrState.omega = est.omegaPitch;
            lqrState.vxIntegral = vxErrorIntegral;
            axTarget = controller_.computeControl(lqrState, vRef);

            // Total acceleration limit disabled for testing
            // double a2 = config_.pendulumGain * kGravity * est.thetaPitch;
            // double a1Max = config_.axTotalMax - a2;
            // double a1Min = -config_.axTotalMax - a2;
            // if (axTarget > a1Max) axTarget = a1Max;
            // if (axTarget < a1Min) axTarget = a1Min;
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
        entry.vRef = vRef;
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
        "px_truth_m", "vx_truth_m_s", "ax_truth_m_s2",
        "theta_truth_rad", "theta_dot_truth_rad_s",
        "theta_est_rad", "omega_est_rad_s",
        "ax_cmd_m_s2", "ax_applied_m_s2",
        "v_ref_m_s"
    });

    for (const auto& e : log_) {
        logger.writeRow({
            e.time,
            e.truth.dronePx,
            e.truth.droneVx,
            e.truth.droneAx,
            e.truth.theta,
            e.truth.thetaDot,
            e.thetaEstimate,
            e.omegaEstimate,
            e.axCommand,
            e.axApplied,
            e.vRef
        });
    }

    logger.flush();
    std::cout << "[ClosedLoopSimulation] Results saved to " << filename << "\n";
}

} // namespace pendulum
