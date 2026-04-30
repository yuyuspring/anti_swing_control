#!/usr/bin/env python3
"""Plot Ruckig S-curve trajectory results."""

import sys
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "results/ruckig_trajectory.csv"
    png_path = sys.argv[2] if len(sys.argv) > 2 else "results/ruckig_trajectory.png"

    df = pd.read_csv(csv_path)

    fig, axes = plt.subplots(4, 1, figsize=(12, 14), sharex=True)
    fig.suptitle("Ruckig S-Curve Trajectory", fontsize=14, fontweight="bold")

    t = df["time_s"]

    # 1. Position
    ax = axes[0]
    ax.plot(t, df["px_m"], "b-", lw=1.5)
    ax.set_ylabel("Position [m]")
    ax.grid(True, alpha=0.3)
    ax.set_title("Position")

    # 2. Velocity
    ax = axes[1]
    ax.plot(t, df["vx_m_s"], "g-", lw=1.5)
    ax.axhline(15, color="gray", ls="--", alpha=0.3, label="v_max")
    ax.set_ylabel("Velocity [m/s]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_title("Velocity")

    # 3. Acceleration
    ax = axes[2]
    ax.plot(t, df["ax_m_s2"], "r-", lw=1.5)
    ax.axhline(2, color="gray", ls="--", alpha=0.3, label="a_max")
    ax.axhline(-2, color="gray", ls="--", alpha=0.3)
    ax.set_ylabel("Acceleration [m/s²]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_title("Acceleration")

    # 4. Jerk
    ax = axes[3]
    ax.plot(t, df["jerk_m_s3"], "m-", lw=1.5)
    ax.axhline(2, color="gray", ls="--", alpha=0.3, label="j_max")
    ax.axhline(-2, color="gray", ls="--", alpha=0.3)
    ax.set_ylabel("Jerk [m/s³]")
    ax.set_xlabel("Time [s]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_title("Jerk (finite difference)")

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(png_path, dpi=150)
    print(f"Plot saved to: {png_path}")


if __name__ == "__main__":
    main()
