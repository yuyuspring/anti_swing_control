#!/usr/bin/env python3
"""Plot closed-loop LQR simulation results."""

import sys
import pandas as pd
import matplotlib.pyplot as plt


def plot_closed_loop(csv_path: str, png_path: str = None):
    df = pd.read_csv(csv_path)

    if png_path is None:
        png_path = csv_path.replace(".csv", ".png")

    fig, axes = plt.subplots(6, 1, figsize=(12, 18), sharex=True)
    fig.suptitle("Closed-Loop LQR Simulation (1D Pitch)", fontsize=14, fontweight="bold")

    # Compute a1, a2, a_total using full coupled model
    # a1 = nominal acceleration (controller output)
    # a_total = actual UAV acceleration (from C++ dynamics)
    # a2 = a_total - a1 (true coupling effect)
    a1 = df["ax_applied_m_s2"]
    a_total = df["ax_truth_m_s2"]
    a2 = a_total - a1

    t = df["time_s"]

    # 1. Position
    ax = axes[0]
    ax.plot(t, df["px_truth_m"], "b-", label="Drone position", linewidth=1.5)
    ax.axhline(df["px_truth_m"].iloc[-1], color="gray", linestyle="--", alpha=0.5, label="Target")
    ax.set_ylabel("Position [m]")
    ax.legend(loc="upper left")
    ax.grid(True, alpha=0.3)
    ax.set_title("Horizontal Position")

    # 2. Velocity
    ax = axes[1]
    ax.plot(t, df["vx_truth_m_s"], "g-", label="Drone velocity", linewidth=1.5)
    if "v_ref_m_s" in df.columns:
        ax.plot(t, df["v_ref_m_s"], "r--", label="Reference velocity", linewidth=1.2)
    ax.set_ylabel("Velocity [m/s]")
    ax.legend(loc="upper left")
    ax.grid(True, alpha=0.3)
    ax.set_title("Horizontal Velocity")

    # 3. Pendulum angle (truth vs estimate)
    ax = axes[2]
    ax.plot(t, df["theta_truth_rad"] * 180 / 3.14159, "b-", label="Truth", linewidth=1.5)
    ax.plot(t, df["theta_est_rad"] * 180 / 3.14159, "r--", label="Observer estimate", linewidth=1.2)
    ax.set_ylabel("Angle [deg]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_title("Pendulum Pitch Angle")

    # 4. Angular rate (truth vs estimate)
    ax = axes[3]
    # Compute truth theta_dot from CSV
    theta_dot_truth = df["theta_dot_truth_rad_s"] * 180 / 3.14159
    ax.plot(t, theta_dot_truth, "b-", label="Truth", linewidth=1.5)
    ax.plot(t, df["omega_est_rad_s"] * 180 / 3.14159, "r--", label="Observer estimate", linewidth=1.2)
    ax.set_ylabel("Rate [deg/s]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_title("Pendulum Pitch Rate")

    # 5. Acceleration breakdown (a1, a2, a_total)
    ax = axes[4]
    ax.plot(t, a1, "b-", label="a1 (nominal)", linewidth=1.5)
    ax.plot(t, a2, "r--", label="a2 (coupling)", linewidth=1.2)
    ax.plot(t, a_total, "g-", label="a = a1 + a2", linewidth=1.5)
    ax.axhline(0, color="gray", linestyle="-", alpha=0.3)
    ax.set_ylabel("Acceleration [m/s²]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_title("Acceleration Breakdown")

    # 6. Control input
    ax = axes[5]
    ax.plot(t, df["ax_cmd_m_s2"], "m-", label="Commanded ax", linewidth=1.5)
    ax.plot(t, df["ax_applied_m_s2"], "k--", label="Applied ax", linewidth=1.0)
    ax.axhline(0, color="gray", linestyle="-", alpha=0.3)
    ax.set_ylabel("Acceleration [m/s²]")
    ax.set_xlabel("Time [s]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_title("Control Input (LQR)")

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.savefig(png_path, dpi=150)
    print(f"Plot saved to: {png_path}")
    plt.show()

    # Print summary statistics
    print("\n" + "=" * 50)
    print("Summary Statistics")
    print("=" * 50)
    print(f"Final position error : {df['px_error_m'].iloc[-1]:.4f} m")
    print(f"Max |theta|          : {df['theta_truth_rad'].abs().max() * 180 / 3.14159:.4f} deg")
    print(f"Max |theta_dot|      : {df['theta_dot_truth_rad_s'].abs().max() * 180 / 3.14159:.4f} deg/s")
    print(f"Max |ax_cmd|         : {df['ax_cmd_m_s2'].abs().max():.4f} m/s²")
    print(f"RMSE theta           : {(df['theta_truth_rad'] - df['theta_est_rad']).pow(2).mean()**0.5 * 180 / 3.14159:.4f} deg")
    print("=" * 50)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 plot_closed_loop.py <closed_loop_results.csv>")
        sys.exit(1)
    plot_closed_loop(sys.argv[1])
