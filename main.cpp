#include "pend_observer.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace {

constexpr float kTruthDt = 0.01f;
constexpr float kObserverDt = 0.02f;
constexpr float kCruiseSpeed = 16.0f;
constexpr float kCruiseDuration = 20.0f;
constexpr float kDecelMagnitude = 2.0f;
constexpr float kDecelDuration = kCruiseSpeed / kDecelMagnitude;
constexpr float kDecelStartTime = kCruiseDuration;
constexpr float kDecelEndTime = kDecelStartTime + kDecelDuration;
constexpr float kInitialPitchDeg = -10.0f;
constexpr float kInitialRollDeg = -10.0f;
constexpr float kTotalTime = 40.0f;
constexpr float kRopeLength = 15.0f;
constexpr float kImuDistanceFromPivot = 0.1f;
constexpr float kPlatformMass = 120.0f;
constexpr float kPayloadMass = 150.0f;
constexpr float kDampingRatio = 0.06f;
constexpr char kCsvPath[] = "simulation_results.csv";
constexpr char kPitchCsvPath[] = "simulation_results_pitch.csv";
constexpr char kRollCsvPath[] = "simulation_results_roll.csv";
constexpr char kMixedCsvPath[] = "simulation_results_mixed.csv";

struct AxisState {
    float angle;
    float angle_dot;
};

struct PendulumState {
    AxisState pitch;
    AxisState roll;
};

struct ScenarioConfig {
    const char *name;
    const char *csv_path;
    float initial_pitch_deg;
    float initial_roll_deg;
    std::array<float, 3> initial_velocity_neu;
    std::array<float, 3> decel_accel_neu;
};

struct ScenarioResult {
    std::string name;
    std::string csv_path;
    float pitch_rmse_deg;
    float roll_rmse_deg;
    float pitch_peak_deg;
    float roll_peak_deg;
    float final_pitch_true_deg;
    float final_pitch_est_deg;
    float final_roll_true_deg;
    float final_roll_est_deg;
};

struct SimulationSample {
    float time;
    float platform_position_n;
    float platform_position_e;
    float platform_velocity_n;
    float platform_velocity_e;
    float platform_accel_n;
    float platform_accel_e;
    float pitch_true;
    float pitch_dot_true;
    float pitch_ddot_true;
    float roll_true;
    float roll_dot_true;
    float roll_ddot_true;
    std::array<float, 3> bob_acc_neu;
    std::array<float, 3> acc_meas_frd;
    std::array<float, 3> gyro_meas_deg_s;
    std::array<float, 3> velocity_meas_neu;
    std::array<float, 3> ahrs_input_deg;
    float cable_tension_n;
    float platform_force_n;
};

std::array<float, 3> platform_accel(const ScenarioConfig &scenario, float time)
{
    if (time < kDecelStartTime) {
        return {0.0f, 0.0f, 0.0f};
    }
    if (time < kDecelEndTime) {
        return scenario.decel_accel_neu;
    }
    return {0.0f, 0.0f, 0.0f};
}

std::array<float, 3> platform_velocity(const ScenarioConfig &scenario, float time)
{
    if (time < kDecelStartTime) {
        return scenario.initial_velocity_neu;
    }
    if (time < kDecelEndTime) {
        float dt = time - kDecelStartTime;
        return {
            scenario.initial_velocity_neu[0] + scenario.decel_accel_neu[0] * dt,
            scenario.initial_velocity_neu[1] + scenario.decel_accel_neu[1] * dt,
            scenario.initial_velocity_neu[2] + scenario.decel_accel_neu[2] * dt,
        };
    }
    return {0.0f, 0.0f, 0.0f};
}

std::array<float, 3> platform_position(const ScenarioConfig &scenario, float time)
{
    if (time < kDecelStartTime) {
        return {
            scenario.initial_velocity_neu[0] * time,
            scenario.initial_velocity_neu[1] * time,
            scenario.initial_velocity_neu[2] * time,
        };
    }

    std::array<float, 3> cruise_distance = {
        scenario.initial_velocity_neu[0] * kCruiseDuration,
        scenario.initial_velocity_neu[1] * kCruiseDuration,
        scenario.initial_velocity_neu[2] * kCruiseDuration,
    };

    if (time < kDecelEndTime) {
        float dt = time - kDecelStartTime;
        return {
            cruise_distance[0] + scenario.initial_velocity_neu[0] * dt + 0.5f * scenario.decel_accel_neu[0] * dt * dt,
            cruise_distance[1] + scenario.initial_velocity_neu[1] * dt + 0.5f * scenario.decel_accel_neu[1] * dt * dt,
            cruise_distance[2] + scenario.initial_velocity_neu[2] * dt + 0.5f * scenario.decel_accel_neu[2] * dt * dt,
        };
    }

    return {
        cruise_distance[0] + scenario.initial_velocity_neu[0] * kDecelDuration + 0.5f * scenario.decel_accel_neu[0] * kDecelDuration * kDecelDuration,
        cruise_distance[1] + scenario.initial_velocity_neu[1] * kDecelDuration + 0.5f * scenario.decel_accel_neu[1] * kDecelDuration * kDecelDuration,
        cruise_distance[2] + scenario.initial_velocity_neu[2] * kDecelDuration + 0.5f * scenario.decel_accel_neu[2] * kDecelDuration * kDecelDuration,
    };
}

bool is_finite_vec(const float *values, int size)
{
    for (int index = 0; index < size; ++index) {
        if (!std::isfinite(values[index])) {
            return false;
        }
    }
    return true;
}

int expect_true(bool condition, const std::string &message)
{
    if (!condition) {
        std::cerr << "[FAIL] " << message << std::endl;
        return 1;
    }
    return 0;
}

float pendulum_axis_ddot(const AxisState &state, float base_accel, float accel_sign)
{
    float omega_n = std::sqrt(G / kRopeLength);
    float damping = 2.0f * kDampingRatio * omega_n;
    return -(G / kRopeLength) * std::sin(state.angle)
           - damping * state.angle_dot
           + accel_sign * (base_accel / kRopeLength) * std::cos(state.angle);
}

AxisState step_axis_rk4(const AxisState &state, float dt, float base_accel, float accel_sign)
{
    float k1_angle = state.angle_dot;
    float k1_rate = pendulum_axis_ddot(state, base_accel, accel_sign);

    AxisState state_2 = {
        state.angle + 0.5f * dt * k1_angle,
        state.angle_dot + 0.5f * dt * k1_rate,
    };
    float k2_angle = state_2.angle_dot;
    float k2_rate = pendulum_axis_ddot(state_2, base_accel, accel_sign);

    AxisState state_3 = {
        state.angle + 0.5f * dt * k2_angle,
        state.angle_dot + 0.5f * dt * k2_rate,
    };
    float k3_angle = state_3.angle_dot;
    float k3_rate = pendulum_axis_ddot(state_3, base_accel, accel_sign);

    AxisState state_4 = {
        state.angle + dt * k3_angle,
        state.angle_dot + dt * k3_rate,
    };
    float k4_angle = state_4.angle_dot;
    float k4_rate = pendulum_axis_ddot(state_4, base_accel, accel_sign);

    AxisState next = {};
    next.angle = state.angle + (dt / 6.0f) * (k1_angle + 2.0f * k2_angle + 2.0f * k3_angle + k4_angle);
    next.angle_dot = state.angle_dot + (dt / 6.0f) * (k1_rate + 2.0f * k2_rate + 2.0f * k3_rate + k4_rate);
    return next;
}

PendulumState step_truth_rk4(const ScenarioConfig &scenario, const PendulumState &state, float time, float dt)
{
    (void)time;
    std::array<float, 3> accel = platform_accel(scenario, time);
    PendulumState next = {};
    next.pitch = step_axis_rk4(state.pitch, dt, accel[0], -1.0f);
    next.roll = step_axis_rk4(state.roll, dt, accel[1], 1.0f);
    return next;
}

std::array<float, 3> rotate_nav_to_body(const std::array<float, 3> &euler_zyx, const std::array<float, 3> &vector_neu)
{
    float q[4] = {0.0f};
    float euler_for_observer[3] = {euler_zyx[0], euler_zyx[1], euler_zyx[2]};
    eul2quat(euler_for_observer, q);

    float rot_body_to_nav[3][3] = {{0.0f}};
    float rot_nav_to_body[3][3] = {{0.0f}};
    quat2rotmat(q, rot_body_to_nav);
    rotmat_transpose(rot_body_to_nav, rot_nav_to_body);

    std::array<float, 3> vector_body = {0.0f, 0.0f, 0.0f};
    for (int row = 0; row < 3; ++row) {
        vector_body[row] = rot_nav_to_body[row][0] * vector_neu[0]
                         + rot_nav_to_body[row][1] * vector_neu[1]
                         + rot_nav_to_body[row][2] * vector_neu[2];
    }
    return vector_body;
}

SimulationSample build_sample(const ScenarioConfig &scenario, float time, const PendulumState &state)
{
    SimulationSample sample = {};
    sample.time = time;
    std::array<float, 3> platform_pos = platform_position(scenario, time);
    std::array<float, 3> platform_vel = platform_velocity(scenario, time);
    std::array<float, 3> platform_acc = platform_accel(scenario, time);
    sample.platform_position_n = platform_pos[0];
    sample.platform_position_e = platform_pos[1];
    sample.platform_velocity_n = platform_vel[0];
    sample.platform_velocity_e = platform_vel[1];
    sample.platform_accel_n = platform_acc[0];
    sample.platform_accel_e = platform_acc[1];
    sample.pitch_true = state.pitch.angle;
    sample.pitch_dot_true = state.pitch.angle_dot;
    sample.pitch_ddot_true = pendulum_axis_ddot(state.pitch, platform_acc[0], -1.0f);
    sample.roll_true = state.roll.angle;
    sample.roll_dot_true = state.roll.angle_dot;
    sample.roll_ddot_true = pendulum_axis_ddot(state.roll, platform_acc[1], 1.0f);

    float sin_pitch = std::sin(state.pitch.angle);
    float cos_pitch = std::cos(state.pitch.angle);
    float pitch_omega_sq = state.pitch.angle_dot * state.pitch.angle_dot;
    float sin_roll = std::sin(state.roll.angle);
    float cos_roll = std::cos(state.roll.angle);
    float roll_omega_sq = state.roll.angle_dot * state.roll.angle_dot;
    float imu_radius = std::clamp(kImuDistanceFromPivot, 0.0f, kRopeLength);

    sample.bob_acc_neu[0] = sample.platform_accel_n + kRopeLength * (cos_pitch * sample.pitch_ddot_true - sin_pitch * pitch_omega_sq);
    sample.bob_acc_neu[1] = sample.platform_accel_e + kRopeLength * (cos_roll * sample.roll_ddot_true - sin_roll * roll_omega_sq);
    sample.bob_acc_neu[2] = kRopeLength * (sin_pitch * sample.pitch_ddot_true + cos_pitch * pitch_omega_sq)
                          + kRopeLength * (sin_roll * sample.roll_ddot_true + cos_roll * roll_omega_sq);

    std::array<float, 3> imu_acc_neu = {
        sample.platform_accel_n + imu_radius * (cos_pitch * sample.pitch_ddot_true - sin_pitch * pitch_omega_sq),
        sample.platform_accel_e + imu_radius * (cos_roll * sample.roll_ddot_true - sin_roll * roll_omega_sq),
        imu_radius * (sin_pitch * sample.pitch_ddot_true + cos_pitch * pitch_omega_sq)
            + imu_radius * (sin_roll * sample.roll_ddot_true + cos_roll * roll_omega_sq),
    };

    std::array<float, 3> gravity_neu = {0.0f, 0.0f, -G};
    std::array<float, 3> specific_force_neu = {
        imu_acc_neu[0] - gravity_neu[0],
        imu_acc_neu[1] - gravity_neu[1],
        imu_acc_neu[2] - gravity_neu[2],
    };
    std::array<float, 3> euler_payload = {0.0f, sample.pitch_true, sample.roll_true};
    std::array<float, 3> specific_force_body = rotate_nav_to_body(euler_payload, specific_force_neu);

    sample.acc_meas_frd[0] = specific_force_body[0];
    sample.acc_meas_frd[1] = specific_force_body[1];
    sample.acc_meas_frd[2] = specific_force_body[2];

    sample.gyro_meas_deg_s = {
        static_cast<float>(sample.roll_dot_true * R2D),
        static_cast<float>(sample.pitch_dot_true * R2D),
        0.0f,
    };
    sample.velocity_meas_neu = {sample.platform_velocity_n, sample.platform_velocity_e, 0.0f};
    sample.ahrs_input_deg = {0.0f, 0.0f, 0.0f};
    sample.cable_tension_n = kPayloadMass * std::sqrt(sample.acc_meas_frd[0] * sample.acc_meas_frd[0]
                                                      + sample.acc_meas_frd[1] * sample.acc_meas_frd[1]
                                                      + sample.acc_meas_frd[2] * sample.acc_meas_frd[2]);
    sample.platform_force_n = (kPlatformMass + kPayloadMass) * sample.platform_accel_n;
    return sample;
}

void write_csv_header(std::ofstream &csv)
{
    csv << "t,observer_update,platform_pos_n,platform_pos_e,platform_vel_n,platform_vel_e,platform_acc_n,platform_acc_e,";
    csv << "pitch_true_rad,pitch_dot_true_rad_s,pitch_ddot_true_rad_s2,";
    csv << "roll_true_rad,roll_dot_true_rad_s,roll_ddot_true_rad_s2,";
    csv << "bob_acc_n,bob_acc_e,bob_acc_u,";
    csv << "acc_meas_fx,acc_meas_fy,acc_meas_fz,";
    csv << "gyro_meas_x_deg_s,gyro_meas_y_deg_s,gyro_meas_z_deg_s,";
    csv << "v_meas_n,v_meas_e,v_meas_u,";
    csv << "pitch_est_rad,pitch_est_deg,w_est_pitch_rad_s,w_est_pitch_deg_s,";
    csv << "roll_est_rad,roll_est_deg,w_est_roll_rad_s,w_est_roll_deg_s,";
    csv << "pitch_error_rad,roll_error_rad,v_hat_n,v_hat_e,a_hat_n,a_hat_e,d_hat_n,d_hat_e,";
    csv << "debug_kp,debug_acc_x,debug_acc_y,debug_acc_z,";
    csv << "cable_tension_n,platform_force_n\n";
}

void write_csv_row(std::ofstream &csv,
                   const SimulationSample &sample,
                   bool observer_update,
                   const PendObserver &obs,
                   const float *debug)
{
    float pitch_est = obs.theta[1];
    float roll_est = obs.theta[2];
    float w_est_pitch = obs.w_est[1];
    float w_est_roll = obs.w_est[0];
    csv << std::fixed << std::setprecision(6)
        << sample.time << ','
        << (observer_update ? 1 : 0) << ','
        << sample.platform_position_n << ','
        << sample.platform_position_e << ','
        << sample.platform_velocity_n << ','
        << sample.platform_velocity_e << ','
        << sample.platform_accel_n << ','
        << sample.platform_accel_e << ','
        << sample.pitch_true << ','
        << sample.pitch_dot_true << ','
        << sample.pitch_ddot_true << ','
        << sample.roll_true << ','
        << sample.roll_dot_true << ','
        << sample.roll_ddot_true << ','
        << sample.bob_acc_neu[0] << ','
        << sample.bob_acc_neu[1] << ','
        << sample.bob_acc_neu[2] << ','
        << sample.acc_meas_frd[0] << ','
        << sample.acc_meas_frd[1] << ','
        << sample.acc_meas_frd[2] << ','
        << sample.gyro_meas_deg_s[0] << ','
        << sample.gyro_meas_deg_s[1] << ','
        << sample.gyro_meas_deg_s[2] << ','
        << sample.velocity_meas_neu[0] << ','
        << sample.velocity_meas_neu[1] << ','
        << sample.velocity_meas_neu[2] << ','
        << pitch_est << ','
        << pitch_est * R2D << ','
        << w_est_pitch << ','
        << w_est_pitch * R2D << ','
        << roll_est << ','
        << roll_est * R2D << ','
        << w_est_roll << ','
        << w_est_roll * R2D << ','
        << (pitch_est - sample.pitch_true) << ','
        << (roll_est - sample.roll_true) << ','
        << obs.v_hat[0] << ','
        << obs.v_hat[1] << ','
        << obs.a_hat[0] << ','
        << obs.a_hat[1] << ','
        << obs.d_hat[0] << ','
        << obs.d_hat[1] << ','
        << debug[0] << ','
        << debug[1] << ','
        << debug[2] << ','
        << debug[3] << ','
        << sample.cable_tension_n << ','
        << sample.platform_force_n << '\n';
}

ScenarioResult run_scenario(const ScenarioConfig &scenario)
{
    std::ofstream csv(scenario.csv_path);
    if (!csv.is_open()) {
        std::cerr << "无法创建 CSV 文件: " << scenario.csv_path << std::endl;
        std::exit(EXIT_FAILURE);
    }

    write_csv_header(csv);

    PendulumState truth = {
        {static_cast<float>(scenario.initial_pitch_deg / R2D), 0.0f},
        {static_cast<float>(scenario.initial_roll_deg / R2D), 0.0f},
    };
    SimulationSample initial_sample = build_sample(scenario, 0.0f, truth);

    PendObserver obs = {};
    float w_lpf[3] = {0.0f, 0.0f, 0.0f};
    float w_meas_init[3] = {
        initial_sample.gyro_meas_deg_s[0],
        initial_sample.gyro_meas_deg_s[1],
        initial_sample.gyro_meas_deg_s[2],
    };
    float acc_init[3] = {
        initial_sample.acc_meas_frd[0],
        initial_sample.acc_meas_frd[1],
        initial_sample.acc_meas_frd[2],
    };

    init_w_lpf_fob(w_meas_init);
    pend_observer_init(acc_init, w_lpf, w_meas_init, &obs);
    for (int axis = 0; axis < 3; ++axis) {
        obs.v_hat[axis] = initial_sample.velocity_meas_neu[axis];
        obs.a_hat[axis] = 0.0f;
        obs.d_hat[axis] = 0.0f;
    }

    float *debug = GetDebugdata();
    int failures = 0;
    float pitch_sum_sq_error = 0.0f;
    float roll_sum_sq_error = 0.0f;
    float pitch_peak_deg = std::fabs(initial_sample.pitch_true) * R2D;
    float roll_peak_deg = std::fabs(initial_sample.roll_true) * R2D;
    int observer_updates = 0;

    write_csv_row(csv, initial_sample, true, obs, debug);

    int observer_step_ratio = static_cast<int>(std::lround(kObserverDt / kTruthDt));
    int total_steps = static_cast<int>(std::lround(kTotalTime / kTruthDt));
    for (int step = 1; step <= total_steps; ++step) {
        float time = step * kTruthDt;
        truth = step_truth_rk4(scenario, truth, time - kTruthDt, kTruthDt);
        SimulationSample sample = build_sample(scenario, time, truth);
        pitch_peak_deg = std::max(pitch_peak_deg, static_cast<float>(std::fabs(sample.pitch_true) * R2D));
        roll_peak_deg = std::max(roll_peak_deg, static_cast<float>(std::fabs(sample.roll_true) * R2D));

        bool observer_update = (step % observer_step_ratio == 0);
        if (observer_update) {
            float w_meas[3] = {
                sample.gyro_meas_deg_s[0],
                sample.gyro_meas_deg_s[1],
                sample.gyro_meas_deg_s[2],
            };
            float a_meas[3] = {
                sample.acc_meas_frd[0],
                sample.acc_meas_frd[1],
                sample.acc_meas_frd[2],
            };
            float v_meas[3] = {
                sample.velocity_meas_neu[0],
                sample.velocity_meas_neu[1],
                sample.velocity_meas_neu[2],
            };
            float ahrs_input[3] = {
                sample.ahrs_input_deg[0],
                sample.ahrs_input_deg[1],
                sample.ahrs_input_deg[2],
            };

            pend_observer_iterate_2(&obs, w_lpf, w_meas, a_meas, v_meas, ahrs_input, kRopeLength);
            pitch_sum_sq_error += (obs.theta[1] - sample.pitch_true) * (obs.theta[1] - sample.pitch_true);
            roll_sum_sq_error += (obs.theta[2] - sample.roll_true) * (obs.theta[2] - sample.roll_true);
            observer_updates += 1;
        }

        failures += expect_true(is_finite_vec(obs.theta, 3), "观测器姿态出现非有限值");
        failures += expect_true(is_finite_vec(obs.w_est, 3), "观测器角速度出现非有限值");
        failures += expect_true(is_finite_vec(obs.v_hat, 3), "ESO 速度观测出现非有限值");
        failures += expect_true(is_finite_vec(obs.a_hat, 3), "ESO 加速度观测出现非有限值");

        write_csv_row(csv, sample, observer_update, obs, debug);
    }

    csv.close();

    float pitch_rmse_rad = 0.0f;
    float roll_rmse_rad = 0.0f;
    if (observer_updates > 0) {
        pitch_rmse_rad = std::sqrt(pitch_sum_sq_error / static_cast<float>(observer_updates));
        roll_rmse_rad = std::sqrt(roll_sum_sq_error / static_cast<float>(observer_updates));
    }

    failures += expect_true(observer_updates > 0, "观测器更新次数应大于 0");
    failures += expect_true(std::isfinite(pitch_rmse_rad), "pitch RMSE 应为有限值");
    failures += expect_true(std::isfinite(roll_rmse_rad), "roll RMSE 应为有限值");
    if (failures != 0) {
        std::cerr << scenario.name << " 场景仿真完成，但检测到 " << failures << " 个失败条件。" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    return {
        scenario.name,
        scenario.csv_path,
        static_cast<float>(pitch_rmse_rad * R2D),
        static_cast<float>(roll_rmse_rad * R2D),
        pitch_peak_deg,
        roll_peak_deg,
        static_cast<float>(truth.pitch.angle * R2D),
        static_cast<float>(obs.theta[1] * R2D),
        static_cast<float>(truth.roll.angle * R2D),
        static_cast<float>(obs.theta[2] * R2D),
    };
}

} // namespace

int main()
{
    std::vector<ScenarioConfig> scenarios = {
        {"pitch", kCsvPath, kInitialPitchDeg, 0.0f, {kCruiseSpeed, 0.0f, 0.0f}, {-kDecelMagnitude, 0.0f, 0.0f}},
        {"roll", kRollCsvPath, 0.0f, kInitialRollDeg, {0.0f, kCruiseSpeed, 0.0f}, {0.0f, -kDecelMagnitude, 0.0f}},
        {"mixed", kMixedCsvPath, kInitialPitchDeg, kInitialRollDeg, {kCruiseSpeed, kCruiseSpeed, 0.0f}, {-kDecelMagnitude, -kDecelMagnitude, 0.0f}},
    };

    std::vector<ScenarioResult> results;
    results.reserve(scenarios.size());
    for (const ScenarioConfig &scenario : scenarios) {
        results.push_back(run_scenario(scenario));
    }

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "multi-scenario pendulum validation finished" << std::endl;
    std::cout << "total_time_s: " << kTotalTime << std::endl;
    std::cout << "initial_platform_speed_m_s: " << kCruiseSpeed << std::endl;
    std::cout << "decel_start_s: " << kDecelStartTime << std::endl;
    std::cout << "decel_end_s: " << kDecelEndTime << std::endl;
    std::cout << "initial_true_pitch_deg: " << kInitialPitchDeg << std::endl;
    std::cout << "initial_true_roll_deg: " << kInitialRollDeg << std::endl;
    std::cout << "rope_length_m: " << kRopeLength << std::endl;
    std::cout << "platform_mass_kg: " << kPlatformMass << std::endl;
    std::cout << "payload_mass_kg: " << kPayloadMass << std::endl;
    for (const ScenarioResult &result : results) {
        std::cout << "scenario: " << result.name << std::endl;
        std::cout << "  csv: " << result.csv_path << std::endl;
        std::cout << "  pitch_peak_deg: " << result.pitch_peak_deg << std::endl;
        std::cout << "  roll_peak_deg: " << result.roll_peak_deg << std::endl;
        std::cout << "  pitch_rmse_deg: " << result.pitch_rmse_deg << std::endl;
        std::cout << "  roll_rmse_deg: " << result.roll_rmse_deg << std::endl;
        std::cout << "  final_true_pitch_deg: " << result.final_pitch_true_deg << std::endl;
        std::cout << "  final_est_pitch_deg: " << result.final_pitch_est_deg << std::endl;
        std::cout << "  final_true_roll_deg: " << result.final_roll_true_deg << std::endl;
        std::cout << "  final_est_roll_deg: " << result.final_roll_est_deg << std::endl;
    }

    std::cout << "仿真与导出完成。" << std::endl;
    return EXIT_SUCCESS;
}