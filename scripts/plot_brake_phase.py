#!/usr/bin/env python3
"""绘制刹车段（brake phase）的详细对比图。

特点：
- 时间轴以刹车开始时刻为 0
- 位置轴以刹车开始位置为 0（显示相对位移 / 刹车距离）
"""

import sys
import pandas as pd
import matplotlib.pyplot as plt


def load_and_label(path, label):
    df = pd.read_csv(path)
    df["mode"] = label
    return df


def detect_brake_start(df, ax_threshold=-0.1):
    """从控制输入中检测刹车开始时刻。"""
    braking = df[df["ax_applied_m_s2"] < ax_threshold]
    if len(braking) == 0:
        return None, None
    t_brake = braking["time_s"].iloc[0]
    idx = (df["time_s"] - t_brake).abs().idxmin()
    px_brake = df.loc[idx, "px_truth_m"]
    return t_brake, px_brake


def plot_brake_phase(files, labels, output_path="brake_phase.png"):
    if len(files) != len(labels):
        print("Error: files and labels must have same length")
        sys.exit(1)

    data = [load_and_label(f, l) for f, l in zip(files, labels)]
    colors = {"Full": "blue", "Shortest": "green", "MinSwing": "red", "VelocityOmega": "purple"}

    # 自动检测刹车开始时间（以第一个文件为准）
    t_brake, px_brake = detect_brake_start(data[0])
    if t_brake is None:
        t_brake = 15.0
        px_brake = 0.0

    # 筛选刹车段数据，并做归一化
    brake_data = []
    for df in data:
        sub = df[df["time_s"] >= t_brake].copy()
        sub["time_rel"] = sub["time_s"] - t_brake
        sub["px_rel"] = sub["px_truth_m"] - px_brake
        brake_data.append(sub)

    fig, axes = plt.subplots(5, 1, figsize=(12, 16), sharex=True)
    fig.suptitle("Brake Phase Comparison", fontsize=14, fontweight="bold")

    # 1. Position（相对位移 = 刹车距离）
    ax = axes[0]
    for df in brake_data:
        c = colors.get(df["mode"].iloc[0], "black")
        ax.plot(df["time_rel"], df["px_rel"], color=c, label=df["mode"].iloc[0], linewidth=1.5)
    ax.set_ylabel("Brake Distance [m]")
    ax.legend(loc="lower right")
    ax.grid(True, alpha=0.3)
    ax.set_title("Relative Horizontal Position (start = 0)")

    # 2. Velocity
    ax = axes[1]
    for df in brake_data:
        c = colors.get(df["mode"].iloc[0], "black")
        ax.plot(df["time_rel"], df["vx_truth_m_s"], color=c, label=df["mode"].iloc[0], linewidth=1.5)
    ax.axhline(0, color="black", linestyle="-", alpha=0.2)
    ax.set_ylabel("Velocity [m/s]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_title("Horizontal Velocity")

    # 3. Swing Velocity (L * omega)
    ax = axes[2]
    rope_len = 15.0
    for df in brake_data:
        c = colors.get(df["mode"].iloc[0], "black")
        v_swing = df["theta_dot_truth_rad_s"] * rope_len
        ax.plot(df["time_rel"], v_swing, color=c, label=df["mode"].iloc[0], linewidth=1.5)
    ax.axhline(0, color="black", linestyle="-", alpha=0.2)
    ax.set_ylabel("Swing Velocity [m/s]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_title("Payload Swing Velocity (L·ω)")

    # 4. Pendulum angle
    ax = axes[3]
    for df in brake_data:
        c = colors.get(df["mode"].iloc[0], "black")
        ax.plot(df["time_rel"], df["theta_truth_rad"] * 180 / 3.14159,
                color=c, label=df["mode"].iloc[0], linewidth=1.5)
    ax.axhline(0, color="black", linestyle="-", alpha=0.2)
    ax.set_ylabel("Angle [deg]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_title("Pendulum Pitch Angle")

    # 5. Control input
    ax = axes[4]
    for df in brake_data:
        c = colors.get(df["mode"].iloc[0], "black")
        ax.plot(df["time_rel"], df["ax_applied_m_s2"], color=c, label=df["mode"].iloc[0], linewidth=1.5)
    ax.axhline(3, color="gray", linestyle="--", alpha=0.3)
    ax.axhline(-3, color="gray", linestyle="--", alpha=0.3)
    ax.axhline(0, color="black", linestyle="-", alpha=0.2)
    ax.set_ylabel("Acceleration [m/s²]")
    ax.set_xlabel("Time since brake start [s]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_title("Control Input")

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.savefig(output_path, dpi=150)
    print(f"Brake phase plot saved to: {output_path}")
    plt.show()

    # 打印刹车段统计
    print("\n" + "=" * 60)
    print("Brake Phase Summary")
    print("=" * 60)
    for df in brake_data:
        mode = df["mode"].iloc[0]
        print(f"\n{mode}:")
        print(f"  Brake distance  : {df['px_rel'].iloc[-1]:.2f} m")
        print(f"  Max |theta|     : {df['theta_truth_rad'].abs().max() * 180 / 3.14159:.2f} deg")
        print(f"  Max |vx|        : {df['vx_truth_m_s'].abs().max():.2f} m/s")
        settled = df[df["vx_truth_m_s"].abs() < 0.1]
        print(f"  Settling time   : {settled['time_rel'].iloc[0]:.2f} s (|vx| < 0.1)" if len(settled) > 0 else "  Not settled")
    print("=" * 60)


if __name__ == "__main__":
    if len(sys.argv) < 5:
        print("Usage: python3 plot_brake_phase.py <full.csv> <shortest.csv> <minswing.csv> <velomega.csv>")
        sys.exit(1)
    plot_brake_phase(
        [sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]],
        ["Full", "Shortest", "MinSwing", "VelocityOmega"]
    )
