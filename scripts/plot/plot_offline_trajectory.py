#!/usr/bin/env python3
"""Plot offline jerk-ramp trajectory simulation results.

Usage:
    python3 plot_offline_trajectory.py [csv_path]
"""

import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def plot_offline(csv_path: str, png_path: str = None):
    df = pd.read_csv(csv_path)

    if png_path is None:
        png_path = csv_path.replace(".csv", ".png")

    fig, axes = plt.subplots(5, 1, figsize=(12, 16), sharex=True)
    fig.suptitle("Offline Jerk-Ramp Trajectory Simulation", fontsize=14, fontweight="bold")

    t = df["time_s"]

    # 1. Position
    ax = axes[0]
    ax.plot(t, df["px_truth_m"], "b-", label="Drone position", linewidth=1.5)
    ax.set_ylabel("Position [m]")
    ax.legend(loc="upper left")
    ax.grid(True, alpha=0.3)
    ax.set_title("Horizontal Position")

    # 2. Velocity
    ax = axes[1]
    ax.plot(t, df["vx_truth_m_s"], "g-", label="Drone velocity", linewidth=1.5)
    ax.set_ylabel("Velocity [m/s]")
    ax.legend(loc="upper left")
    ax.grid(True, alpha=0.3)
    ax.set_title("Horizontal Velocity")

    # 3. Pendulum angle
    ax = axes[2]
    ax.plot(t, df["theta_truth_rad"] * 180 / np.pi, "b-", label="Truth", linewidth=1.5)
    ax.set_ylabel("Angle [deg]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_title("Pendulum Pitch Angle")

    # 4. Angular rate
    ax = axes[3]
    theta_dot = df["theta_dot_truth_rad_s"] * 180 / np.pi
    ax.plot(t, theta_dot, "b-", label="Truth", linewidth=1.5)
    ax.set_ylabel("Rate [deg/s]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_title("Pendulum Pitch Rate")

    # 5. Control input
    ax = axes[4]
    ax.plot(t, df["ax_applied_m_s2"], "m-", label="Applied ax", linewidth=1.5)
    ax.plot(t, df["ax_cmd_m_s2"], "k--", label="Commanded ax", linewidth=1.0, alpha=0.5)
    ax.axhline(2, color="gray", linestyle="--", alpha=0.3)
    ax.axhline(-2, color="gray", linestyle="--", alpha=0.3)
    ax.axhline(0, color="black", linestyle="-", alpha=0.2)
    ax.set_ylabel("Acceleration [m/s²]")
    ax.set_xlabel("Time [s]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_title("Control Input (Open-loop Offline)")

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.savefig(png_path, dpi=150)
    print(f"Plot saved to: {png_path}")

    # Print summary
    print("\n" + "=" * 50)
    print("Summary Statistics")
    print("=" * 50)
    brake = df[df["ax_applied_m_s2"] < -0.1]
    if len(brake) > 0:
        t_brake = brake["time_s"].iloc[0]
        brake_df = df[df["time_s"] >= t_brake].copy()
        brake_df["px_rel"] = brake_df["px_truth_m"] - brake_df["px_truth_m"].iloc[0]
        print(f"Brake start time    : {t_brake:.2f} s")
        print(f"Brake distance      : {brake_df['px_rel'].iloc[-1]:.2f} m")
        settled = brake_df[brake_df["vx_truth_m_s"].abs() < 0.1]
        if len(settled) > 0:
            print(f"Settling time       : {settled['time_s'].iloc[0] - t_brake:.2f} s")
        else:
            print(f"Final vx            : {brake_df['vx_truth_m_s'].iloc[-1]:.3f} m/s")
    print(f"Max |theta|         : {df['theta_truth_rad'].abs().max() * 180 / np.pi:.4f} deg")
    print(f"Max |theta_dot|     : {df['theta_dot_truth_rad_s'].abs().max() * 180 / np.pi:.4f} deg/s")
    print(f"Max |ax_cmd|        : {df['ax_cmd_m_s2'].abs().max():.4f} m/s²")
    print(f"Final theta         : {df['theta_truth_rad'].iloc[-1] * 180 / np.pi:.4f} deg")
    print(f"Final omega         : {df['theta_dot_truth_rad_s'].iloc[-1] * 180 / np.pi:.4f} deg/s")
    print("=" * 50)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        csv_path = "results/trajectory/offline_three_phase.csv"
    else:
        csv_path = sys.argv[1]
    plot_offline(csv_path)
