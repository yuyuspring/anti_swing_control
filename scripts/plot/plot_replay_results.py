import csv
import math
import sys

import matplotlib.pyplot as plt


def rad_to_deg_series(values):
    return [value * 180.0 / math.pi for value in values]


def load_csv(path):
    columns = {}
    with open(path, newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            for key, value in row.items():
                columns.setdefault(key, []).append(float(value))
    return columns


def rms(values):
    if not values:
        return 0.0
    return math.sqrt(sum(value * value for value in values) / len(values))


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "results/replay_validation.csv"
    output_path = sys.argv[2] if len(sys.argv) > 2 else "results/replay_validation.png"
    data = load_csv(csv_path)

    time_s = data["time_s"]
    recorded_pitch_deg = rad_to_deg_series(data["recorded_pitch_rad"])
    replay_pitch_deg = rad_to_deg_series(data["replay_pitch_rad"])
    recorded_roll_deg = rad_to_deg_series(data["recorded_roll_rad"])
    replay_roll_deg = rad_to_deg_series(data["replay_roll_rad"])
    pitch_error_deg = data["pitch_error_deg"]
    roll_error_deg = data["roll_error_deg"]
    measured_w_roll_deg_s = data["measured_w_roll_deg_s"]
    measured_w_pitch_deg_s = data["measured_w_pitch_deg_s"]
    recorded_w_roll_deg_s = data["recorded_w_roll_deg_s"]
    replay_w_roll_deg_s = data["replay_w_roll_deg_s"]
    recorded_w_pitch_deg_s = data["recorded_w_pitch_deg_s"]
    replay_w_pitch_deg_s = data["replay_w_pitch_deg_s"]
    measured_v_forward = data["measured_v_forward"]
    recorded_v_hat_forward = data["recorded_v_hat_forward"]
    replay_v_hat_forward = data["replay_v_hat_forward"]
    recorded_a_hat_forward = data["recorded_a_hat_forward"]
    replay_a_hat_forward = data["replay_a_hat_forward"]

    fig, axes = plt.subplots(4, 1, figsize=(13, 14), sharex=True)

    axes[0].plot(time_s, recorded_pitch_deg, label="recorded pitch")
    axes[0].plot(time_s, replay_pitch_deg, linestyle="--", label="replay pitch")
    axes[0].plot(time_s, recorded_roll_deg, label="recorded roll")
    axes[0].plot(time_s, replay_roll_deg, linestyle=":", label="replay roll")
    axes[0].set_ylabel("deg")
    axes[0].set_title(
        f"Angle, pitch rms={rms(pitch_error_deg):.3f} deg, roll rms={rms(roll_error_deg):.3f} deg"
    )
    axes[0].grid(True)
    axes[0].legend()

    axes[1].plot(time_s, measured_w_pitch_deg_s, label="measured pitch rate")
    axes[1].plot(time_s, recorded_w_pitch_deg_s, label="recorded pitch estimate")
    axes[1].plot(time_s, replay_w_pitch_deg_s, linestyle="--", label="replay pitch estimate")
    axes[1].plot(time_s, measured_w_roll_deg_s, label="measured roll rate")
    axes[1].plot(time_s, recorded_w_roll_deg_s, label="recorded roll estimate")
    axes[1].plot(time_s, replay_w_roll_deg_s, linestyle=":", label="replay roll estimate")
    axes[1].set_ylabel("deg/s")
    axes[1].set_title("Angular Rate Measurement vs Estimate")
    axes[1].grid(True)
    axes[1].legend()

    axes[2].plot(time_s, measured_v_forward, label="measured forward velocity")
    axes[2].plot(time_s, recorded_v_hat_forward, label="recorded forward velocity")
    axes[2].plot(time_s, replay_v_hat_forward, linestyle="--", label="replay forward velocity")
    axes[2].set_ylabel("m/s")
    axes[2].set_title("Forward Velocity Measurement vs Estimate")
    axes[2].grid(True)
    axes[2].legend()

    axes[3].plot(time_s, recorded_a_hat_forward, label="recorded forward acceleration")
    axes[3].plot(time_s, replay_a_hat_forward, linestyle="--", label="replay forward acceleration")
    axes[3].set_ylabel("m/s^2")
    axes[3].set_xlabel("time (s)")
    axes[3].set_title("Forward Acceleration Estimate")
    axes[3].grid(True)
    axes[3].legend()

    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    print(f"saved plot to {output_path}")
    plt.show()


if __name__ == "__main__":
    main()