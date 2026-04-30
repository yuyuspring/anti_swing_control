#!/usr/bin/env python3
"""对比 t0_mode = theta_max vs omega_max (simple 模型, jerk=0.3)"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

MODES = ["theta_max", "omega_max"]
LABELS = ["t0 = θ_max (omega=0)", "t0 = omega_max (ddot=0)"]
COLORS = ["C0", "C1"]


def load(mode):
    return pd.read_csv(f"results/trajectory/offline_simple_{mode}.csv")


def plot_comparison():
    fig, axes = plt.subplots(5, 1, figsize=(12, 16), sharex=True)
    fig.suptitle("t0 Mode Comparison — Simple Model (jerk=0.3)\n"
                 "theta_max: t0 when θ_dot=0  |  omega_max: t0 when θ_ddot=0",
                 fontsize=14, fontweight="bold")

    stats = []

    for mode, label, color in zip(MODES, LABELS, COLORS):
        df = load(mode)
        braking = df[df["ax_applied_m_s2"] < -0.1]
        t_brake = braking["time_s"].iloc[0]
        idx = (df["time_s"] - t_brake).abs().idxmin()
        px_brake = df.loc[idx, "px_truth_m"]

        sub = df[df["time_s"] >= t_brake].copy()
        sub["time_rel"] = sub["time_s"] - t_brake
        sub["px_rel"] = sub["px_truth_m"] - px_brake

        axes[0].plot(sub["time_rel"], sub["px_rel"], color=color, label=label, linewidth=1.5)
        axes[1].plot(sub["time_rel"], sub["vx_truth_m_s"], color=color, linewidth=1.5)
        axes[2].plot(sub["time_rel"], sub["theta_truth_rad"] * 180 / np.pi, color=color, linewidth=1.5)
        axes[3].plot(sub["time_rel"], sub["theta_dot_truth_rad_s"] * 180 / np.pi, color=color, linewidth=1.5)
        axes[4].plot(sub["time_rel"], sub["ax_applied_m_s2"], color=color, linewidth=1.5)

        stats.append({
            "mode": mode,
            "label": label,
            "brake_dist": sub["px_rel"].iloc[-1],
            "max_theta": sub["theta_truth_rad"].abs().max() * 180 / np.pi,
            "max_omega": sub["theta_dot_truth_rad_s"].abs().max() * 180 / np.pi,
            "final_vx": sub["vx_truth_m_s"].iloc[-1],
        })

    axes[0].set_ylabel("Brake Distance [m]")
    axes[0].set_title("Relative Horizontal Position")
    axes[0].legend(loc="upper left")
    axes[0].grid(True, alpha=0.3)

    axes[1].set_ylabel("Velocity [m/s]")
    axes[1].set_title("Horizontal Velocity")
    axes[1].axhline(0, color="black", linestyle="-", alpha=0.2)
    axes[1].grid(True, alpha=0.3)

    axes[2].set_ylabel("Angle [deg]")
    axes[2].set_title("Pendulum Pitch Angle")
    axes[2].grid(True, alpha=0.3)

    axes[3].set_ylabel("Rate [deg/s]")
    axes[3].set_title("Pendulum Pitch Rate")
    axes[3].axhline(0, color="black", linestyle="-", alpha=0.2)
    axes[3].grid(True, alpha=0.3)

    axes[4].set_ylabel("Acceleration [m/s²]")
    axes[4].set_title("Applied Acceleration")
    axes[4].axhline(0, color="black", linestyle="-", alpha=0.2)
    axes[4].grid(True, alpha=0.3)
    axes[4].set_xlabel("Time since brake start [s]")

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    out = "results/trajectory/t0mode_comparison.png"
    plt.savefig(out, dpi=150)
    print(f"Comparison saved to: {out}")

    print("\n" + "=" * 70)
    print("t0 Mode Comparison Summary (jerk=0.3)")
    print("=" * 70)
    for s in stats:
        print(f"  {s['label']}")
        print(f"    Brake distance : {s['brake_dist']:.2f} m")
        print(f"    Max |theta|    : {s['max_theta']:.2f}°")
        print(f"    Max |omega|    : {s['max_omega']:.2f}°/s")
        print(f"    Final vx       : {s['final_vx']:.2f} m/s")
        print()
    print("=" * 70)


if __name__ == "__main__":
    plot_comparison()
