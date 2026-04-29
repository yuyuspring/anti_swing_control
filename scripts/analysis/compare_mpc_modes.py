#!/usr/bin/env python3
"""
MPC 两种模式对比：最小摆速度 vs 最小系统能量

也支持叠加对应的 LQR 模式作为参考。
"""

import sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

L = 15.0

def load_brake_phase(csv_path):
    df = pd.read_csv(csv_path)
    brake = df[df["time_s"] >= 40.0].copy()
    brake["t_rel"] = brake["time_s"] - 40.0
    brake["px_rel"] = brake["px_truth_m"] - brake["px_truth_m"].iloc[0]
    return brake


def metrics(df):
    brake = df[df["time_s"] > 40.0]
    done = brake[brake["vx_truth_m_s"] < 0.1].index
    accel_dist = 0.5 * 2.0 * (7.5)**2
    cruise_dist = 15.0 * (40.0 - 7.5)
    brake_dist = (df.loc[done[0], "px_truth_m"] if len(done) > 0 else df["px_truth_m"].iloc[-1]) - accel_dist - cruise_dist
    settling = df.loc[done[0], "time_s"] - 40.0 if len(done) > 0 else np.nan
    max_theta = abs(brake["theta_truth_rad"]).max() * 180 / np.pi
    max_ax = abs(brake["ax_applied_m_s2"]).max()
    return brake_dist, settling, max_theta, max_ax


def main():
    show = "--show" in sys.argv
    if show:
        sys.argv.remove("--show")
        matplotlib.use("TkAgg")

    results_dir = "results/mpc"

    # MPC data
    b_mw = load_brake_phase(f"{results_dir}/closed_loop_mpc_minomega.csv")
    b_se = load_brake_phase(f"{results_dir}/closed_loop_mpc_minsysenergy.csv")

    # Optional LQR reference
    try:
        b_lqr_vo = load_brake_phase(f"results/lqr/closed_loop_velomega.csv")
        b_lqr_se = load_brake_phase(f"results/lqr/closed_loop_systemenergy.csv")
        has_lqr = True
    except Exception:
        has_lqr = False

    m_mw = metrics(pd.read_csv(f"{results_dir}/closed_loop_mpc_minomega.csv"))
    m_se = metrics(pd.read_csv(f"{results_dir}/closed_loop_mpc_minsysenergy.csv"))
    if has_lqr:
        m_lvo = metrics(pd.read_csv(f"results/lqr/closed_loop_velomega.csv"))
        m_lse = metrics(pd.read_csv(f"results/lqr/closed_loop_systemenergy.csv"))

    fig, axes = plt.subplots(5, 1, figsize=(12, 16), sharex=True)

    # 1. Position
    ax = axes[0]
    ax.plot(b_mw["t_rel"], b_mw["px_rel"], "r-", lw=2, label="MPC min |omega|")
    ax.plot(b_se["t_rel"], b_se["px_rel"], "b-", lw=2, label="MPC min system energy")
    if has_lqr:
        ax.plot(b_lqr_vo["t_rel"], b_lqr_vo["px_rel"], "r--", lw=1.5, alpha=0.5, label="LQR VelOmega")
        ax.plot(b_lqr_se["t_rel"], b_lqr_se["px_rel"], "b--", lw=1.5, alpha=0.5, label="LQR SysEnergy")
    ax.set_ylabel("Brake Distance [m]")
    ax.set_title("MPC Mode Comparison (N=20, dt=0.1 s, axLimit = ±2 m/s²)")
    ax.legend(loc="right")
    ax.grid(True, alpha=0.3)

    # 2. Velocity
    ax = axes[1]
    ax.plot(b_mw["t_rel"], b_mw["vx_truth_m_s"], "r-", lw=2, label="MPC min |omega|")
    ax.plot(b_se["t_rel"], b_se["vx_truth_m_s"], "b-", lw=2, label="MPC min sys energy")
    if has_lqr:
        ax.plot(b_lqr_vo["t_rel"], b_lqr_vo["vx_truth_m_s"], "r--", lw=1.5, alpha=0.5)
        ax.plot(b_lqr_se["t_rel"], b_lqr_se["vx_truth_m_s"], "b--", lw=1.5, alpha=0.5)
    ax.set_ylabel("Velocity [m/s]")
    ax.legend(loc="right")
    ax.grid(True, alpha=0.3)

    # 3. Payload abs velocity
    ax = axes[2]
    for b, c, lab in [(b_mw, "r", "MPC min |omega|"), (b_se, "b", "MPC min sys energy")]:
        vx = b["vx_truth_m_s"].values
        th = b["theta_truth_rad"].values
        w = b["theta_dot_truth_rad_s"].values
        v_pay = vx + L * w * np.cos(th)
        ax.plot(b["t_rel"], v_pay, color=c, lw=2, label=lab)
    if has_lqr:
        for b, c in [(b_lqr_vo, "r"), (b_lqr_se, "b")]:
            vx = b["vx_truth_m_s"].values
            th = b["theta_truth_rad"].values
            w = b["theta_dot_truth_rad_s"].values
            v_pay = vx + L * w * np.cos(th)
            ax.plot(b["t_rel"], v_pay, color=c, lw=1.5, ls="--", alpha=0.5)
    ax.set_ylabel("Payload |vx| [m/s]")
    ax.legend(loc="right")
    ax.grid(True, alpha=0.3)

    # 4. Angle
    ax = axes[3]
    ax.plot(b_mw["t_rel"], np.degrees(b_mw["theta_truth_rad"]), "r-", lw=2, label="MPC min |omega|")
    ax.plot(b_se["t_rel"], np.degrees(b_se["theta_truth_rad"]), "b-", lw=2, label="MPC min sys energy")
    if has_lqr:
        ax.plot(b_lqr_vo["t_rel"], np.degrees(b_lqr_vo["theta_truth_rad"]), "r--", lw=1.5, alpha=0.5)
        ax.plot(b_lqr_se["t_rel"], np.degrees(b_lqr_se["theta_truth_rad"]), "b--", lw=1.5, alpha=0.5)
    ax.set_ylabel("Pitch Angle [deg]")
    ax.legend(loc="right")
    ax.grid(True, alpha=0.3)

    # 5. Control input
    ax = axes[4]
    ax.plot(b_mw["t_rel"], b_mw["ax_applied_m_s2"], "r-", lw=2, label="MPC min |omega|")
    ax.plot(b_se["t_rel"], b_se["ax_applied_m_s2"], "b-", lw=2, label="MPC min sys energy")
    if has_lqr:
        ax.plot(b_lqr_vo["t_rel"], b_lqr_vo["ax_applied_m_s2"], "r--", lw=1.5, alpha=0.5)
        ax.plot(b_lqr_se["t_rel"], b_lqr_se["ax_applied_m_s2"], "b--", lw=1.5, alpha=0.5)
    ax.axhline(-2, color="gray", ls="--", alpha=0.3)
    ax.axhline(2, color="gray", ls="--", alpha=0.3)
    ax.set_ylabel("Acceleration [m/s²]")
    ax.set_xlabel("Time since brake start [s]")
    ax.legend(loc="right")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out_name = f"{results_dir}/mpc_mode_comparison.png"
    fig.savefig(out_name, dpi=150)
    print(f"Saved: {out_name}")

    if show:
        print("Showing plot window (close manually)...")
        plt.show()

    print("\n" + "=" * 70)
    print("Brake Phase Metrics")
    print("=" * 70)
    print(f"{'Metric':<25} {'MPC min|w|':>12} {'MPC sysE':>12}", end="")
    if has_lqr:
        print(f" {'LQR VelO':>12} {'LQR SysE':>12}")
    else:
        print()
    print("-" * 70)
    labels = ["Brake distance [m]", "Settling time [s]", "Max |theta| [deg]", "Max |ax| [m/s²]"]
    for i, lab in enumerate(labels):
        print(f"{lab:<25} {m_mw[i]:>12.2f} {m_se[i]:>12.2f}", end="")
        if has_lqr:
            print(f" {m_lvo[i]:>12.2f} {m_lse[i]:>12.2f}")
        else:
            print()
    print("=" * 70)


if __name__ == "__main__":
    main()
