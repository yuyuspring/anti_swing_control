#include "observer/pend_observer.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

constexpr double kReplayRopeLength = 15.0;
constexpr double kEpsilon = 1e-6;

struct CsvData {
    std::vector<std::string> header;
    std::unordered_map<std::string, std::size_t> index;
    std::vector<std::vector<double>> rows;
    // Raw string tokens for each row (used for overwrite output)
    std::vector<std::vector<std::string>> raw_tokens;
};

struct Stats {
    double sum_sq = 0.0;
    double max_abs = 0.0;
    std::size_t count = 0;

    void add(double error) {
        sum_sq += error * error;
        max_abs = std::max(max_abs, std::abs(error));
        ++count;
    }

    double rmse() const {
        if (count == 0) {
            return 0.0;
        }
        return std::sqrt(sum_sq / static_cast<double>(count));
    }
};

std::vector<std::string> split_csv_line(const std::string& line) {
    std::vector<std::string> tokens;
    std::stringstream stream(line);
    std::string token;
    while (std::getline(stream, token, ',')) {
        auto first = token.find_first_not_of(" \t\r\n");
        auto last = token.find_last_not_of(" \t\r\n");
        if (first == std::string::npos) {
            tokens.emplace_back();
        } else {
            tokens.push_back(token.substr(first, last - first + 1));
        }
    }
    return tokens;
}

double to_double(const std::string& token) {
    char* end = nullptr;
    const double value = std::strtod(token.c_str(), &end);
    if (end == token.c_str()) {
        throw std::runtime_error("CSV contains a non-numeric field: " + token);
    }
    return value;
}

CsvData load_csv(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open CSV file: " + path);
    }

    CsvData data;
    std::string line;
    if (!std::getline(file, line)) {
        throw std::runtime_error("CSV file is empty: " + path);
    }

    data.header = split_csv_line(line);
    for (std::size_t i = 0; i < data.header.size(); ++i) {
        data.index[data.header[i]] = i;
    }

    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }
        const auto tokens = split_csv_line(line);
        if (tokens.size() != data.header.size()) {
            throw std::runtime_error("CSV row width mismatch in file: " + path);
        }
        std::vector<double> row;
        row.reserve(tokens.size());
        for (const auto& token : tokens) {
            row.push_back(to_double(token));
        }
        data.rows.push_back(std::move(row));
        data.raw_tokens.push_back(tokens);
    }

    return data;
}

double get_value(const CsvData& data, const std::vector<double>& row, const std::string& key) {
    const auto it = data.index.find(key);
    if (it == data.index.end()) {
        throw std::runtime_error("Missing CSV column: " + key);
    }
    return row[it->second];
}

std::size_t find_first_valid_row(const CsvData& data) {
    for (std::size_t row_index = 0; row_index < data.rows.size(); ++row_index) {
        const auto& row = data.rows[row_index];
        double magnitude = 0.0;
        for (std::size_t column = 1; column < row.size(); ++column) {
            magnitude += std::abs(row[column]);
        }
        if (magnitude > kEpsilon) {
            return row_index;
        }
    }
    throw std::runtime_error("No valid non-zero data row found in CSV.");
}

void fill_vec3(const CsvData& data, const std::vector<double>& row, const char* x, const char* y, const char* z, float out[3]) {
    out[0] = static_cast<float>(get_value(data, row, x));
    out[1] = static_cast<float>(get_value(data, row, y));
    out[2] = static_cast<float>(get_value(data, row, z));
}

double relative_time(const CsvData& data, const std::vector<double>& row, double start_time) {
    return get_value(data, row, "time_stamp") - start_time;
}

void update_stats(Stats& stats, double replay, double recorded) {
    stats.add(replay - recorded);
}

double synthesize_forward(double north, double east, double yaw_rad) {
    return std::cos(yaw_rad) * north + std::sin(yaw_rad) * east;
}

// Overwrite observer-estimated columns in raw_tokens with replay values.
void overwrite_observer_columns(CsvData& data, std::size_t row_index, const PendObserver& obs) {
    auto& tokens = data.raw_tokens[row_index];
    auto set_col = [&](const char* key, double value) {
        auto it = data.index.find(key);
        if (it != data.index.end()) {
            std::ostringstream oss;
            oss << std::setprecision(9) << std::fixed << value;
            tokens[it->second] = oss.str();
        }
    };

    // theta (yaw, pitch, roll)
    set_col("theta[0]", obs.theta[0]);
    set_col("theta[1]", obs.theta[1]);
    set_col("theta[2]", obs.theta[2]);

    // w_est (roll_rate, pitch_rate, yaw_rate) in rad/s
    set_col("w_est[0]", obs.w_est[0]);
    set_col("w_est[1]", obs.w_est[1]);
    set_col("w_est[2]", obs.w_est[2]);

    // v_hat (north, east, down)
    set_col("v_hat[0]", obs.v_hat[0]);
    set_col("v_hat[1]", obs.v_hat[1]);
    set_col("v_hat[2]", obs.v_hat[2]);

    // a_hat (north, east, down)
    set_col("a_hat[0]", obs.a_hat[0]);
    set_col("a_hat[1]", obs.a_hat[1]);
    set_col("a_hat[2]", obs.a_hat[2]);

    // acc_modify from debug_data
    set_col("acc_modify[0]", debug_data[1]);
    set_col("acc_modify[1]", debug_data[2]);
    set_col("acc_modify[2]", debug_data[3]);

    // Rg
    set_col("Rg[0]", obs.Rg[0]);
    set_col("Rg[1]", obs.Rg[1]);
    set_col("Rg[2]", obs.Rg[2]);
}

void write_overwrite_csv(const CsvData& data, const std::string& path, std::size_t first_valid_index) {
    std::ofstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open overwrite output CSV: " + path);
    }

    // Header
    for (std::size_t i = 0; i < data.header.size(); ++i) {
        if (i > 0) file << ',';
        file << data.header[i];
    }
    file << '\n';

    // Rows before first_valid_index: keep original
    for (std::size_t r = 0; r < first_valid_index; ++r) {
        for (std::size_t c = 0; c < data.raw_tokens[r].size(); ++c) {
            if (c > 0) file << ',';
            file << data.raw_tokens[r][c];
        }
        file << '\n';
    }

    // Rows from first_valid_index onward: overwritten
    for (std::size_t r = first_valid_index; r < data.raw_tokens.size(); ++r) {
        for (std::size_t c = 0; c < data.raw_tokens[r].size(); ++c) {
            if (c > 0) file << ',';
            file << data.raw_tokens[r][c];
        }
        file << '\n';
    }
}

}  // namespace

int main(int argc, char** argv) {
    const std::string input_path = (argc > 1) ? argv[1] : "crane_imu_obs_debug.csv";
    const std::string output_path = (argc > 2) ? argv[2] : "replay_validation.csv";
    const std::string overwrite_path = (argc > 3) ? argv[3] : "crane_imu_obs_debug_1.csv";

    try {
        CsvData data = load_csv(input_path);
        const std::size_t first_valid_index = find_first_valid_row(data);
        const double start_time = get_value(data, data.rows[first_valid_index], "time_stamp");

        PendObserver obs{};
        float w_lpf[3] = {0.0f, 0.0f, 0.0f};

        float w_meas_init[3];
        float a_meas_init[3];
        fill_vec3(data, data.rows[first_valid_index], "w_meas[0]", "w_meas[1]", "w_meas[2]", w_meas_init);
        fill_vec3(data, data.rows[first_valid_index], "a_meas[0]", "a_meas[1]", "a_meas[2]", a_meas_init);

        pend_observer_init(a_meas_init, w_lpf, w_meas_init, &obs);

        std::ofstream output(output_path);
        if (!output.is_open()) {
            throw std::runtime_error("Failed to open output CSV: " + output_path);
        }

        output
            << "time_s,row_index,recorded_pitch_rad,replay_pitch_rad,pitch_error_deg,"
            << "recorded_roll_rad,replay_roll_rad,roll_error_deg,"
            << "measured_w_roll_deg_s,measured_w_pitch_deg_s,"
            << "recorded_w_roll_deg_s,replay_w_roll_deg_s,roll_rate_error_deg_s,"
            << "recorded_w_pitch_deg_s,replay_w_pitch_deg_s,pitch_rate_error_deg_s,"
            << "measured_v_forward,recorded_v_hat_forward,replay_v_hat_forward,v_hat_forward_error,"
            << "recorded_a_hat_forward,replay_a_hat_forward,a_hat_forward_error,"
            << "recorded_acc_modify_x,replay_acc_modify_x,acc_modify_x_error,"
            << "recorded_acc_modify_y,replay_acc_modify_y,acc_modify_y_error,"
            << "recorded_acc_modify_z,replay_acc_modify_z,acc_modify_z_error,"
            << "recorded_rg_x,replay_rg_x,rg_x_error,"
            << "recorded_rg_y,replay_rg_y,rg_y_error,"
            << "recorded_rg_z,replay_rg_z,rg_z_error\n";

        Stats pitch_stats;
        Stats roll_stats;
        Stats w_roll_stats;
        Stats w_pitch_stats;
        Stats v_hat_forward_stats;
        Stats a_hat_forward_stats;
        Stats acc_modify_x_stats;
        Stats acc_modify_y_stats;
        Stats acc_modify_z_stats;
        Stats rg_x_stats;
        Stats rg_y_stats;
        Stats rg_z_stats;

        for (std::size_t row_index = first_valid_index; row_index < data.rows.size(); ++row_index) {
            const auto& row = data.rows[row_index];
            const double yaw_rad = get_value(data, row, "yaw");

            float w_meas[3];
            float a_meas[3];
            float v_meas[3];
            fill_vec3(data, row, "w_meas[0]", "w_meas[1]", "w_meas[2]", w_meas);
            fill_vec3(data, row, "a_meas[0]", "a_meas[1]", "a_meas[2]", a_meas);
            fill_vec3(data, row, "v_meas[0]", "v_meas[1]", "v_meas[2]", v_meas);

            // CSV gyro is treated as rad/s, while the observer interface expects deg/s.
            w_meas[0] *= R2D;
            w_meas[1] *= R2D;
            w_meas[2] *= R2D;

            float ahrs_input[3] = {
                static_cast<float>(yaw_rad * R2D),
                0.0f,
                0.0f,
            };

            pend_observer_iterate_2(&obs, w_lpf, w_meas, a_meas, v_meas, ahrs_input, static_cast<float>(kReplayRopeLength));

            // Overwrite observer columns in raw tokens for this row
            overwrite_observer_columns(data, row_index, obs);

            const double time_s = relative_time(data, row, start_time);
            const double recorded_pitch = get_value(data, row, "theta[1]");
            const double recorded_roll = get_value(data, row, "theta[2]");
            const double recorded_w_roll = get_value(data, row, "w_est[0]") * R2D;
            const double recorded_w_pitch = get_value(data, row, "w_est[1]") * R2D;
            const double measured_w_roll = get_value(data, row, "w_meas[0]") * R2D;
            const double measured_w_pitch = get_value(data, row, "w_meas[1]") * R2D;
            const double measured_v_n = get_value(data, row, "v_meas[0]");
            const double measured_v_e = get_value(data, row, "v_meas[1]");
            const double recorded_v_hat_n = get_value(data, row, "v_hat[0]");
            const double recorded_v_hat_e = get_value(data, row, "v_hat[1]");
            const double recorded_a_hat_n = get_value(data, row, "a_hat[0]");
            const double recorded_a_hat_e = get_value(data, row, "a_hat[1]");
            const double measured_v_forward = synthesize_forward(measured_v_n, measured_v_e, yaw_rad);
            const double recorded_v_hat_forward = synthesize_forward(recorded_v_hat_n, recorded_v_hat_e, yaw_rad);
            const double replay_v_hat_forward = synthesize_forward(obs.v_hat[0], obs.v_hat[1], yaw_rad);
            const double recorded_a_hat_forward = synthesize_forward(recorded_a_hat_n, recorded_a_hat_e, yaw_rad);
            const double replay_a_hat_forward = synthesize_forward(obs.a_hat[0], obs.a_hat[1], yaw_rad);
            const double recorded_acc_modify_x = get_value(data, row, "acc_modify[0]");
            const double recorded_acc_modify_y = get_value(data, row, "acc_modify[1]");
            const double recorded_acc_modify_z = get_value(data, row, "acc_modify[2]");
            const double recorded_rg_x = get_value(data, row, "Rg[0]");
            const double recorded_rg_y = get_value(data, row, "Rg[1]");
            const double recorded_rg_z = get_value(data, row, "Rg[2]");

            update_stats(pitch_stats, obs.theta[1], recorded_pitch);
            update_stats(roll_stats, obs.theta[2], recorded_roll);
            update_stats(w_roll_stats, obs.w_est[0] * R2D, recorded_w_roll);
            update_stats(w_pitch_stats, obs.w_est[1] * R2D, recorded_w_pitch);
            update_stats(v_hat_forward_stats, replay_v_hat_forward, recorded_v_hat_forward);
            update_stats(a_hat_forward_stats, replay_a_hat_forward, recorded_a_hat_forward);
            update_stats(acc_modify_x_stats, debug_data[1], recorded_acc_modify_x);
            update_stats(acc_modify_y_stats, debug_data[2], recorded_acc_modify_y);
            update_stats(acc_modify_z_stats, debug_data[3], recorded_acc_modify_z);
            update_stats(rg_x_stats, obs.Rg[0], recorded_rg_x);
            update_stats(rg_y_stats, obs.Rg[1], recorded_rg_y);
            update_stats(rg_z_stats, obs.Rg[2], recorded_rg_z);

            output << std::fixed << std::setprecision(9)
                   << time_s << ','
                   << (row_index + 2) << ','
                   << recorded_pitch << ','
                   << obs.theta[1] << ','
                   << (obs.theta[1] - recorded_pitch) * R2D << ','
                   << recorded_roll << ','
                   << obs.theta[2] << ','
                   << (obs.theta[2] - recorded_roll) * R2D << ','
                   << measured_w_roll << ','
                   << measured_w_pitch << ','
                   << recorded_w_roll << ','
                   << obs.w_est[0] * R2D << ','
                   << obs.w_est[0] * R2D - recorded_w_roll << ','
                   << recorded_w_pitch << ','
                   << obs.w_est[1] * R2D << ','
                   << obs.w_est[1] * R2D - recorded_w_pitch << ','
                   << measured_v_forward << ','
                   << recorded_v_hat_forward << ','
                   << replay_v_hat_forward << ','
                   << replay_v_hat_forward - recorded_v_hat_forward << ','
                   << recorded_a_hat_forward << ','
                   << replay_a_hat_forward << ','
                   << replay_a_hat_forward - recorded_a_hat_forward << ','
                   << recorded_acc_modify_x << ','
                   << debug_data[1] << ','
                   << debug_data[1] - recorded_acc_modify_x << ','
                   << recorded_acc_modify_y << ','
                   << debug_data[2] << ','
                   << debug_data[2] - recorded_acc_modify_y << ','
                   << recorded_acc_modify_z << ','
                   << debug_data[3] << ','
                   << debug_data[3] - recorded_acc_modify_z << ','
                   << recorded_rg_x << ','
                   << obs.Rg[0] << ','
                   << obs.Rg[0] - recorded_rg_x << ','
                   << recorded_rg_y << ','
                   << obs.Rg[1] << ','
                   << obs.Rg[1] - recorded_rg_y << ','
                   << recorded_rg_z << ','
                   << obs.Rg[2] << ','
                   << obs.Rg[2] - recorded_rg_z << '\n';
        }

        // Write overwritten CSV with same format as input
        write_overwrite_csv(data, overwrite_path, first_valid_index);

        std::cout << std::fixed << std::setprecision(6);
        std::cout << "replay rows: " << pitch_stats.count << "\n";
        std::cout << "first valid csv line: " << (first_valid_index + 2) << "\n";
        std::cout << "pitch rmse deg: " << pitch_stats.rmse() * R2D << ", max abs deg: " << pitch_stats.max_abs * R2D << "\n";
        std::cout << "roll rmse deg: " << roll_stats.rmse() * R2D << ", max abs deg: " << roll_stats.max_abs * R2D << "\n";
        std::cout << "w_roll rmse deg/s: " << w_roll_stats.rmse() << ", max abs deg/s: " << w_roll_stats.max_abs << "\n";
        std::cout << "w_pitch rmse deg/s: " << w_pitch_stats.rmse() << ", max abs deg/s: " << w_pitch_stats.max_abs << "\n";
        std::cout << "v_hat_forward rmse: " << v_hat_forward_stats.rmse() << ", max abs: " << v_hat_forward_stats.max_abs << "\n";
        std::cout << "a_hat_forward rmse: " << a_hat_forward_stats.rmse() << ", max abs: " << a_hat_forward_stats.max_abs << "\n";
        std::cout << "acc_modify rmse xyz: "
                  << acc_modify_x_stats.rmse() << ", "
                  << acc_modify_y_stats.rmse() << ", "
                  << acc_modify_z_stats.rmse() << "\n";
        std::cout << "Rg rmse xyz: "
                  << rg_x_stats.rmse() << ", "
                  << rg_y_stats.rmse() << ", "
                  << rg_z_stats.rmse() << "\n";
        std::cout << "saved replay validation csv to " << output_path << "\n";
        std::cout << "saved overwrite csv to " << overwrite_path << "\n";

        return EXIT_SUCCESS;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
        return EXIT_FAILURE;
    }
}
