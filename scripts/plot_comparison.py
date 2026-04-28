#!/usr/bin/env python3
"""Compare five control modes: Full, Shortest, MinSwing, VelocityOmega, PayloadVelocity."""

import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def load_and_label(path, label):
    df = pd.read_csv(path)
    df["mode"] = label
    return df


def detect_brake_start(df, ax_threshold=-0.1):
    """Detect when braking starts by finding first significant negative acceleration."""
    braking = df[df["ax_applied_m_s2"] < ax_threshold]
    if len(braking) == 0:
        return None, None
    t_brake = braking["time_s"].iloc[0]
    idx = (df["time_s"] - t_brake).abs().idxmin()
    px_brake = df.loc[idx, "px_truth_m"]
    return t_brake, px_brake


def plot_comparison(files, labels, output_path="comparison.png"):
    if len(files) != len(labels):
        print("Error: files and labels must have same length")
        sys.exit(1)

    data = [load_and_label(f, l) for f, l in zip(files, labels)]
    colors = {"Full": "blue", "Shortest": "green", "MinSwing": "red", "VelocityOmega": "purple", "PayloadVelocity": "orange", "MinEnergy": "brown", "SystemEnergy": "cyan"}

    # Auto-detect brake start from first dataset (all should be same)
    t_brake, px_brake = detect_brake_start(data[0])
    if t_brake is None:
        t_brake = 15.0
        px_brake = 0.0

    fig, axes = plt.subplots(5, 1, figsize=(12, 16), sharex=True)
    fig.suptitle("Control Mode Comparison (1D Pitch)", fontsize=14, fontweight="bold")

    # 1. Position
    ax = axes[0]
    for df in data:
        c = colors.get(df["mode"].iloc[0], "black")
        ax.plot(df["time_s"], df["px_truth_m"], color=c, label=df["mode"].iloc[0], linewidth=1.5)
    ax.axvline(t_brake, color="black", linestyle="--", alpha=0.4)
    ax.axhline(px_brake, color="black", linestyle="--", alpha=0.4,
               label=f"brake start (t={t_brake:.1f}s, x={px_brake:.1f}m)")
    ax.set_ylabel("Position [m]")
    ax.legend(loc="upper left")
    ax.grid(True, alpha=0.3)
    ax.set_title("Horizontal Position")

    # 2. Velocity
    ax = axes[1]
    for df in data:
        c = colors.get(df["mode"].iloc[0], "black")
        ax.plot(df["time_s"], df["vx_truth_m_s"], color=c, label=df["mode"].iloc[0], linewidth=1.5)
    ax.axhline(15, color="gray", linestyle="--", alpha=0.3, label="vx_limit")
    ax.axhline(-15, color="gray", linestyle="--", alpha=0.3)
    ax.axhline(0, color="black", linestyle="-", alpha=0.2)
    ax.axvline(t_brake, color="black", linestyle="--", alpha=0.4)
    ax.set_ylabel("Velocity [m/s]")
    ax.legend(loc="upper left")
    ax.grid(True, alpha=0.3)
    ax.set_title("Horizontal Velocity")

    # 3. Payload Absolute Horizontal Velocity
    ax = axes[2]
    for df in data:
        c = colors.get(df["mode"].iloc[0], "black")
        # v_payload_x = vx + L * omega * cos(theta)
        rope_len = 15.0
        v_abs = df["vx_truth_m_s"] + df["theta_dot_truth_rad_s"] * rope_len * np.cos(df["theta_truth_rad"])
        ax.plot(df["time_s"], v_abs, color=c, label=df["mode"].iloc[0], linewidth=1.5)
    ax.axhline(0, color="black", linestyle="-", alpha=0.2)
    ax.axvline(t_brake, color="black", linestyle="--", alpha=0.4)
    ax.set_ylabel("Abs Velocity [m/s]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_title("Payload Absolute Horizontal Velocity")

    # 4. Pendulum angle
    ax = axes[3]
    for df in data:
        c = colors.get(df["mode"].iloc[0], "black")
        ax.plot(df["time_s"], df["theta_truth_rad"] * 180 / 3.14159,
                color=c, label=df["mode"].iloc[0], linewidth=1.5)
    ax.axhline(0, color="black", linestyle="-", alpha=0.2)
    ax.axvline(t_brake, color="black", linestyle="--", alpha=0.4)
    ax.set_ylabel("Angle [deg]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_title("Pendulum Pitch Angle")

    # 5. Control input
    ax = axes[4]
    for df in data:
        c = colors.get(df["mode"].iloc[0], "black")
        ax.plot(df["time_s"], df["ax_applied_m_s2"], color=c, label=df["mode"].iloc[0], linewidth=1.5)
    ax.axhline(3, color="gray", linestyle="--", alpha=0.3)
    ax.axhline(-3, color="gray", linestyle="--", alpha=0.3)
    ax.axhline(0, color="black", linestyle="-", alpha=0.2)
    ax.axvline(t_brake, color="black", linestyle="--", alpha=0.4)
    ax.set_ylabel("Acceleration [m/s²]")
    ax.set_xlabel("Time [s]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_title("Control Input")

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.savefig(output_path, dpi=150)
    print(f"Comparison plot saved to: {output_path}")
    # plt.show()  # Uncomment for interactive display

    # Print summary
    print("\n" + "=" * 60)
    print("Summary Statistics")
    print("=" * 60)
    for df in data:
        mode = df["mode"].iloc[0]
        print(f"\n{mode}:")
        print(f"  Final position  : {df['px_truth_m'].iloc[-1]:.4f} m")
        print(f"  Max |theta|     : {df['theta_truth_rad'].abs().max() * 180 / 3.14159:.2f} deg")
        print(f"  Max |vx|        : {df['vx_truth_m_s'].abs().max():.2f} m/s")
        print(f"  Max |ax|        : {df['ax_applied_m_s2'].abs().max():.2f} m/s²")
    print("=" * 60)


if __name__ == "__main__":
    if len(sys.argv) < 8:
        print("Usage: python3 plot_comparison.py <full.csv> <shortest.csv> <minswing.csv> <velomega.csv> <payload.csv> <minenergy.csv> <systemenergy.csv>")
        sys.exit(1)
    plot_comparison(
        [sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6], sys.argv[7]],
        ["Full", "Shortest", "MinSwing", "VelocityOmega", "PayloadVelocity", "MinEnergy", "SystemEnergy"]
    )
