#!/usr/bin/env python3
"""Compare two control modes: Diagonal (diagonal Q) vs Coupled (non-diagonal Q)."""

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
    colors = {"Diagonal": "#D62728", "Coupled": "#1F77B4"}

    # Brake start is fixed at t=40s (matching the reference command)
    t_brake = 40.0
    idx = (data[0]["time_s"] - t_brake).abs().idxmin()
    px_brake = data[0].loc[idx, "px_truth_m"]

    fig, axes = plt.subplots(5, 1, figsize=(12, 16), sharex=True)
    fig.suptitle("Control Mode Comparison (1D Pitch)", fontsize=14, fontweight="bold")

    G = 9.81
    PEND_GAIN = 0.6
    rope_len = 10.0

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

    # 2. Velocity: drone vx + payload abs velocity + v_ref
    ax = axes[1]
    # Plot v_ref once (same for both modes)
    ax.plot(data[0]["time_s"], data[0]["v_ref_m_s"], color="gray", linestyle="-",
            linewidth=1.0, alpha=0.6, label="v_ref")
    for df in data:
        c = colors.get(df["mode"].iloc[0], "black")
        mode_name = df["mode"].iloc[0]
        # Drone velocity
        ax.plot(df["time_s"], df["vx_truth_m_s"], color=c, label=f"{mode_name} drone vx", linewidth=1.5)
        # Payload absolute horizontal velocity
        v_payload = df["vx_truth_m_s"] + df["theta_dot_truth_rad_s"] * rope_len * np.cos(df["theta_truth_rad"])
        ax.plot(df["time_s"], v_payload, color=c, linestyle="--", linewidth=1.2,
                label=f"{mode_name} payload vx")
    ax.axhline(15, color="gray", linestyle="--", alpha=0.3)
    ax.axhline(-15, color="gray", linestyle="--", alpha=0.3)
    ax.axhline(0, color="black", linestyle="-", alpha=0.2)
    ax.axvline(t_brake, color="black", linestyle="--", alpha=0.4)
    ax.set_ylabel("Velocity [m/s]")
    ax.set_ylim(-5, 20)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_title("Horizontal Velocity (Drone / Payload / Reference)")

    # 3. Pendulum angle
    ax = axes[2]
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

    # 4. Angular velocity
    ax = axes[3]
    for df in data:
        c = colors.get(df["mode"].iloc[0], "black")
        ax.plot(df["time_s"], df["theta_dot_truth_rad_s"] * 180 / 3.14159,
                color=c, label=df["mode"].iloc[0], linewidth=1.5)
    ax.axhline(0, color="black", linestyle="-", alpha=0.2)
    ax.axvline(t_brake, color="black", linestyle="--", alpha=0.4)
    ax.set_ylabel("Angular Rate [deg/s]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_title("Pendulum Pitch Angular Rate")

    # 5. Acceleration breakdown: a_total + a1 + a2
    ax = axes[4]
    for df in data:
        c = colors.get(df["mode"].iloc[0], "black")
        mode_name = df["mode"].iloc[0]
        a1 = df["ax_applied_m_s2"]
        a_total = df["ax_truth_m_s2"]
        a2 = a_total - a1
        ax.plot(df["time_s"], a_total, color=c, linestyle="-", linewidth=1.5,
                label=f"{mode_name} a_total")
        ax.plot(df["time_s"], a1, color=c, linestyle="--", linewidth=1.2,
                label=f"{mode_name} a1")
        ax.plot(df["time_s"], a2, color=c, linestyle=":", linewidth=1.0,
                label=f"{mode_name} a2")
    ax.axhline(2, color="gray", linestyle="--", alpha=0.3)
    ax.axhline(-2, color="gray", linestyle="--", alpha=0.3)
    ax.axhline(0, color="black", linestyle="-", alpha=0.2)
    ax.axvline(t_brake, color="black", linestyle="--", alpha=0.4)
    ax.set_ylabel("Acceleration [m/s²]")
    ax.set_xlabel("Time [s]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_title("Acceleration Breakdown (a_total = a1 + a2)")

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
        print(f"  Max |omega|     : {df['theta_dot_truth_rad_s'].abs().max() * 180 / 3.14159:.2f} deg/s")
        print(f"  Max |vx|        : {df['vx_truth_m_s'].abs().max():.2f} m/s")
        print(f"  Max |ax|        : {df['ax_applied_m_s2'].abs().max():.2f} m/s²")
    print("=" * 60)


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 plot_comparison.py <diagonal.csv> <coupled.csv>")
        sys.exit(1)
    plot_comparison(
        [sys.argv[1], sys.argv[2]],
        ["Diagonal", "Coupled"]
    )
