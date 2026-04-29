import csv
import math
import sys

import matplotlib.pyplot as plt


def load_csv(path):
    columns = {}
    with open(path, newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            for key, value in row.items():
                columns.setdefault(key, []).append(float(value))
    return columns


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "results/simulation_results_roll.csv"  # simulation_results
    output_path = sys.argv[2] if len(sys.argv) > 2 else "results/simulation_results.png"
    data = load_csv(csv_path)

    time = data["t"]
    pitch_true_key = "pitch_true_rad" if "pitch_true_rad" in data else "theta_true_rad"
    pitch_est_key = "pitch_est_rad" if "pitch_est_rad" in data else "theta_est_rad"
    pitch_error_key = "pitch_error_rad" if "pitch_error_rad" in data else "theta_error_rad"
    pitch_rate_true_key = "pitch_dot_true_rad_s" if "pitch_dot_true_rad_s" in data else "theta_dot_true_rad_s"
    pitch_rate_est_key = "w_est_pitch_deg_s"
    roll_true_key = "roll_true_rad" if "roll_true_rad" in data else None
    roll_est_key = "roll_est_rad" if "roll_est_rad" in data else None
    roll_error_key = "roll_error_rad" if "roll_error_rad" in data else None
    roll_rate_true_key = "roll_dot_true_rad_s" if "roll_dot_true_rad_s" in data else None
    roll_rate_est_key = "w_est_roll_deg_s" if "w_est_roll_deg_s" in data else None

    pitch_true_deg = [value * 180.0 / math.pi for value in data[pitch_true_key]]
    pitch_est_deg = [value * 180.0 / math.pi for value in data[pitch_est_key]]
    pitch_error_deg = [value * 180.0 / math.pi for value in data[pitch_error_key]]
    pitch_rate_true_deg_s = [value * 180.0 / math.pi for value in data[pitch_rate_true_key]]
    pitch_rate_est_deg_s = data[pitch_rate_est_key]

    has_roll = roll_true_key is not None and roll_est_key is not None
    if has_roll:
        roll_true_deg = [value * 180.0 / math.pi for value in data[roll_true_key]]
        roll_est_deg = [value * 180.0 / math.pi for value in data[roll_est_key]]
        roll_error_deg = [value * 180.0 / math.pi for value in data[roll_error_key]]
        roll_rate_true_deg_s = [value * 180.0 / math.pi for value in data[roll_rate_true_key]]
        roll_rate_est_deg_s = data[roll_rate_est_key]

    fig, axes = plt.subplots(4, 1, figsize=(12, 14), sharex=True)

    axes[0].plot(time, pitch_true_deg, label="true pitch")
    axes[0].plot(time, pitch_est_deg, label="estimated pitch", linestyle="--")
    if has_roll:
        axes[0].plot(time, roll_true_deg, label="true roll")
        axes[0].plot(time, roll_est_deg, label="estimated roll", linestyle=":")
    axes[0].set_ylabel("angle (deg)")
    axes[0].set_title("Pendulum Angle Truth vs Estimate")
    axes[0].grid(True)
    axes[0].legend()

    axes[1].plot(time, pitch_rate_true_deg_s, label="true pitch rate")
    axes[1].plot(time, pitch_rate_est_deg_s, label="estimated pitch rate", linestyle="--")
    if has_roll:
        axes[1].plot(time, roll_rate_true_deg_s, label="true roll rate")
        axes[1].plot(time, roll_rate_est_deg_s, label="estimated roll rate", linestyle=":")
    axes[1].set_ylabel("rate (deg/s)")
    axes[1].set_title("Angular Rate")
    axes[1].grid(True)
    axes[1].legend()

    axes[2].plot(time, data["platform_vel_n"], label="platform velocity north")
    axes[2].plot(time, data["platform_acc_n"], label="platform acceleration north")
    axes[2].plot(time, data["a_hat_n"], label="ESO accel estimate", linestyle="--")
    if "platform_vel_e" in data:
        axes[2].plot(time, data["platform_vel_e"], label="platform velocity east")
    if "platform_acc_e" in data:
        axes[2].plot(time, data["platform_acc_e"], label="platform acceleration east")
    if "a_hat_e" in data:
        axes[2].plot(time, data["a_hat_e"], label="ESO accel estimate east", linestyle=":")
    axes[2].set_ylabel("m/s, m/s^2")
    axes[2].set_title("Platform Motion and ESO Output")
    axes[2].grid(True)
    axes[2].legend()

    axes[3].plot(time, pitch_error_deg, label="pitch error")
    if has_roll:
        axes[3].plot(time, roll_error_deg, label="roll error")
    axes[3].plot(time, data["acc_meas_fx"], label="acc x body")
    axes[3].plot(time, data["acc_meas_fy"], label="acc y body")
    axes[3].plot(time, data["acc_meas_fz"], label="acc z body")
    axes[3].set_ylabel("deg / m/s^2")
    axes[3].set_xlabel("time (s)")
    axes[3].set_title("Error and Sensor Signals")
    axes[3].grid(True)
    axes[3].legend()

    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    print(f"saved plot to {output_path}")
    plt.show()


if __name__ == "__main__":
    main()