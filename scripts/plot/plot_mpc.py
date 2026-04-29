#!/usr/bin/env python3
"""
MPC 绘图脚本

用法:
    python3 plot_mpc.py [OPTION]

Options:
    --single        只绘制 MPC min |omega| 单组曲线
    --energy        只绘制 MPC min sys energy 单组曲线
    --compare       对比 MPC min |omega| vs LQR VelocityOmega
    --all           全部对比（MPC 两种 + LQR 两种），默认模式
    --show          弹窗显示（matplotlib 交互后端）

输出:
    - results/mpc/mpc_single_minomega.png      （单图）
    - results/mpc/mpc_single_minsysenergy.png  （单图）
    - results/mpc/mpc_compare_minomega.png     （对比图）
    - results/mpc/mpc_all_comparison.png       （全部对比图）
"""

import sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import argparse

L = 15.0


def load_brake_phase(csv_path):
    df = pd.read_csv(csv_path)
    brake = df[df["time_s"] >= 40.0].copy()
    brake["t_rel"] = brake["time_s"] - 40.0
    brake["px_rel"] = brake["px_truth_m"] - brake["px_truth_m"].iloc[0]
    return brake


def plot_single(axs, brake, color, label, linestyle="-"):
    """在一组 axs 上绘制单条曲线"""
    # 1. 位置
    axs[0].plot(brake["t_rel"], brake["px_rel"], color=color, lw=2, ls=linestyle, label=label)
    # 2. 速度
    axs[1].plot(brake["t_rel"], brake["vx_truth_m_s"], color=color, lw=2, ls=linestyle, label=label)
    # 3. Payload 速度
    vx = brake["vx_truth_m_s"].values
    th = brake["theta_truth_rad"].values
    w = brake["theta_dot_truth_rad_s"].values
    v_pay = vx + L * w * np.cos(th)
    axs[2].plot(brake["t_rel"], v_pay, color=color, lw=2, ls=linestyle, label=label)
    # 4. 摆角
    axs[3].plot(brake["t_rel"], np.degrees(brake["theta_truth_rad"]), color=color, lw=2, ls=linestyle, label=label)
    # 5. 控制输入
    axs[4].plot(brake["t_rel"], brake["ax_applied_m_s2"], color=color, lw=2, ls=linestyle, label=label)


def setup_axes(title_suffix=""):
    """创建 5×1 子图并返回 fig, axs"""
    fig, axs = plt.subplots(5, 1, figsize=(12, 16), sharex=True)
    titles = ["Brake Distance", "Velocity", "Payload |vx|", "Pitch Angle", "Acceleration"]
    ylabels = ["Distance [m]", "Velocity [m/s]", "Abs Velocity [m/s]", "Angle [deg]", "Acceleration [m/s²]"]
    for ax, t, ylab in zip(axs, titles, ylabels):
        ax.set_ylabel(ylab)
        ax.grid(True, alpha=0.3)
    axs[-1].set_xlabel("Time since brake start [s]")
    axs[-1].axhline(-2, color="gray", ls="--", alpha=0.3)
    axs[-1].axhline(2, color="gray", ls="--", alpha=0.3)
    fig.suptitle(f"MPC {title_suffix}" if title_suffix else "MPC Comparison", fontsize=14)
    return fig, axs


def draw_single_mode(brake_df, color, label, title, out_name, show=False):
    """绘制单模式 5 子图"""
    fig, axs = setup_axes(title)
    plot_single(axs, brake_df, color, label)
    for ax in axs:
        ax.legend(loc="right")
    plt.tight_layout()
    fig.savefig(out_name, dpi=150)
    print(f"Saved: {out_name}")
    if show:
        plt.show()
    plt.close(fig)


def draw_compare(mpc_brake, lqr_brake, mpc_color, lqr_color, mpc_label, lqr_label, out_name, show=False):
    """绘制 MPC vs LQR 对比 5 子图"""
    fig, axs = setup_axes(f"MPC vs LQR ({mpc_label})")
    plot_single(axs, mpc_brake, mpc_color, f"MPC {mpc_label}", linestyle="-")
    plot_single(axs, lqr_brake, lqr_color, f"LQR {lqr_label}", linestyle="--")
    for ax in axs:
        ax.legend(loc="right")
    plt.tight_layout()
    fig.savefig(out_name, dpi=150)
    print(f"Saved: {out_name}")
    if show:
        plt.show()
    plt.close(fig)


def draw_all(mpc_mw, mpc_se, lqr_vo, lqr_se, out_name, show=False):
    """绘制全部 4 组对比"""
    fig, axs = setup_axes("Mode Comparison (N=20, dt=0.1 s)")
    plot_single(axs, mpc_mw, "r", "MPC min |omega|", linestyle="-")
    plot_single(axs, mpc_se, "b", "MPC min sys energy", linestyle="-")
    plot_single(axs, lqr_vo, "r", "LQR VelOmega", linestyle="--")
    plot_single(axs, lqr_se, "b", "LQR SysEnergy", linestyle="--")
    for ax in axs:
        ax.legend(loc="right")
    plt.tight_layout()
    fig.savefig(out_name, dpi=150)
    print(f"Saved: {out_name}")
    if show:
        plt.show()
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="MPC plotting script")
    parser.add_argument("--single", action="store_true", help="Plot MPC min |omega| only")
    parser.add_argument("--energy", action="store_true", help="Plot MPC min sys energy only")
    parser.add_argument("--compare", action="store_true", help="Compare MPC min |omega| vs LQR VelocityOmega")
    parser.add_argument("--all", action="store_true", help="Plot all modes (default)")
    parser.add_argument("--show", action="store_true", help="Show plot window")
    args = parser.parse_args()

    if args.show:
        matplotlib.use("TkAgg")

    results_dir = "results/mpc"
    b_mpc_mw = load_brake_phase(f"{results_dir}/closed_loop_mpc_minomega.csv")
    b_mpc_se = load_brake_phase(f"{results_dir}/closed_loop_mpc_minsysenergy.csv")
    try:
        b_lqr_vo = load_brake_phase(f"results/lqr/closed_loop_velomega.csv")
        b_lqr_se = load_brake_phase(f"results/lqr/closed_loop_systemenergy.csv")
        has_lqr = True
    except Exception:
        has_lqr = False
        print("Warning: LQR CSV files not found, skipping LQR curves")

    # Default: --all if no mode specified
    if not any([args.single, args.energy, args.compare]):
        args.all = True

    if args.single:
        draw_single_mode(b_mpc_mw, "r", "MPC min |omega|", "min |omega|",
                         f"{results_dir}/mpc_single_minomega.png", args.show)

    if args.energy:
        draw_single_mode(b_mpc_se, "b", "MPC min sys energy", "min sys energy",
                         f"{results_dir}/mpc_single_minsysenergy.png", args.show)

    if args.compare and has_lqr:
        draw_compare(b_mpc_mw, b_lqr_vo, "r", "b", "min |omega|", "VelOmega",
                     f"{results_dir}/mpc_compare_minomega.png", args.show)

    if args.all:
        if has_lqr:
            draw_all(b_mpc_mw, b_mpc_se, b_lqr_vo, b_lqr_se,
                     f"{results_dir}/mpc_all_comparison.png", args.show)
        else:
            # Only MPC modes
            fig, axs = setup_axes("MPC Mode Comparison")
            plot_single(axs, b_mpc_mw, "r", "MPC min |omega|", linestyle="-")
            plot_single(axs, b_mpc_se, "b", "MPC min sys energy", linestyle="-")
            for ax in axs:
                ax.legend(loc="right")
            plt.tight_layout()
            fig.savefig(f"{results_dir}/mpc_all_comparison.png", dpi=150)
            print(f"Saved: {results_dir}/mpc_all_comparison.png")
            if args.show:
                plt.show()
            plt.close(fig)


if __name__ == "__main__":
    main()
