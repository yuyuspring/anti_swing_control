#pragma once

#include <string>
#include <vector>

namespace lqr {

constexpr double kGravity = 9.81;
constexpr double kPi = 3.14159265358979323846;

inline double saturate(double value, double limit) {
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

inline double clamp(double value, double min_val, double max_val) {
    if (value > max_val) return max_val;
    if (value < min_val) return min_val;
    return value;
}

/**
 * @brief Minimal state for the 1D pitch slung-load dynamics.
 */
struct State {
    double time = 0.0;    ///< Simulation time [s]
    double p = 0.0;       ///< Horizontal position [m]
    double v = 0.0;       ///< Horizontal velocity [m/s]
    double a = 0.0;       ///< Actual horizontal acceleration [m/s^2]
    double theta = 0.0;   ///< Pendulum angle [rad], 0 = hanging straight down
    double omega = 0.0;   ///< Pendulum angular velocity [rad/s]
};

enum class ControllerMode {
    kDiagonal,
    kCoupled
};

/**
 * @brief Self-contained LQR closed-loop simulator.
 *
 * Input: a velocity reference sequence.
 * Output: equal-length sequences of p, v, a, theta, omega.
 *
 * Built-in:
 *   - Full coupled slung-load dynamics (RK4)
 *   - LQI controller with conditional integration + clamp
 *   - Jerk limit on acceleration command
 */
class StandaloneLqrSimulator {
public:
    struct Config {
        double dt = 0.02;               ///< Discrete step [s]
        double ropeLength = 10.0;       ///< [m]
        double axMax = 5.0;             ///< Max nominal acceleration [m/s^2]
        double jerkMax = 10.0;          ///< Max jerk [m/s^3]
        ControllerMode mode = ControllerMode::kCoupled;

        // Mass parameters
        double payloadMass = 180.0;     ///< [kg]
        double droneMass = 120.0;       ///< [kg] (used to compute pendulumGain = m/(M+m))

        // Damping parameters
        double linearDampingCoeff = 0.15; ///< [1/s]
        double dragCoeff = 1.0;         ///< Drag coefficient Cd [-]
        double dragArea = 0.5;          ///< Reference area [m^2]
        double airDensity = 1.225;      ///< [kg/m^3]

        // Initial conditions
        double initialTheta = 0.0;      ///< [rad]
        double initialOmega = 0.0;      ///< [rad/s]
        double initialP = 0.0;          ///< [m]
        double initialV = 0.0;          ///< [m/s]
    };

    explicit StandaloneLqrSimulator(const Config& config);

    /**
     * @brief Run a batch simulation.
     *
     * @param vRefSeq  Velocity reference sequence [m/s], length = N.
     * @param pOut     Output position sequence [m], length = N.
     * @param vOut     Output velocity sequence [m/s], length = N.
     * @param aOut     Output actual acceleration sequence [m/s^2], length = N.
     * @param thetaOut Output pendulum angle sequence [rad], length = N.
     * @param omegaOut Output angular rate sequence [rad/s], length = N.
     *
     * All output vectors are resized to match vRefSeq.size().
     */
    void run(const std::vector<double>& vRefSeq,
             std::vector<double>& pOut,
             std::vector<double>& vOut,
             std::vector<double>& aOut,
             std::vector<double>& thetaOut,
             std::vector<double>& omegaOut);

    /**
     * @brief Generate a trapezoidal velocity profile.
     *
     * @param dt              Time step [s].
     * @param tFinal          Total duration [s].
     * @param accelRate       Acceleration ramp rate [m/s^2].
     * @param cruiseSpeed     Cruise speed [m/s].
     * @param brakeStartTime  Time to start braking [s].
     * @return Vector of v_ref values.
     */
    static std::vector<double> generateTrapezoid(double dt,
                                                  double tFinal,
                                                  double accelRate,
                                                  double cruiseSpeed,
                                                  double brakeStartTime);

private:
    Config config_;
    double pendulumGain_;   ///< m / (M + m)

    // LQI gains (selected by mode)
    double kV_ = 0.0;
    double kTheta_ = 0.0;
    double kOmega_ = 0.0;
    double kIntegral_ = 0.0;

    // Controller internal state
    double integral_ = 0.0;
    double axCmd_ = 0.0;    ///< Jerk-limited acceleration command

    void computeDerivative(const State& s, double a1,
                           double& dp, double& dv, double& dth, double& dwd) const;
    void stepRK4(State& s, double a1, double dt);
    double computeControl(double v, double theta, double omega, double vRef);
    double applyJerkLimit(double target, double dt);
};

} // namespace lqr
