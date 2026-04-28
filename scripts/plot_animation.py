#!/usr/bin/env python3
"""
Generate animation of LQR closed-loop simulation results.

Usage:
    python3 plot_animation.py <full.csv> <shortest.csv> <minswing.csv> <velomega.csv> <payload.csv>
    python3 plot_animation.py <full.csv> <shortest.csv> <minswing.csv> <velomega.csv> <payload.csv> --phase brake
    python3 plot_animation.py <full.csv> <shortest.csv> <minswing.csv> <velomega.csv> <payload.csv> --phase full --output lqr_animation.mp4
"""

import sys
import argparse
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter, PillowWriter
from matplotlib.patches import Rectangle, FancyBboxPatch


def load_and_label(path, label):
    df = pd.read_csv(path)
    df["mode"] = label
    return df


def detect_brake_start(df, ax_threshold=-0.1):
    braking = df[df["ax_applied_m_s2"] < ax_threshold]
    if len(braking) == 0:
        return None
    return braking["time_s"].iloc[0]


def create_animation(files, labels, output_path="lqr_animation.mp4", phase="brake",
                     fps=30, duration_s=8.0, rope_len=15.0, drone_width=4.0, drone_height=1.5):
    """
    phase: "brake" -> animate brake phase only (t-5 to t+15 around brake start)
           "full"  -> animate full trajectory (0 to t_final)
    """
    data = [load_and_label(f, l) for f, l in zip(files, labels)]
    colors = {"Full": "blue", "Shortest": "green", "MinSwing": "red",
              "VelocityOmega": "purple", "PayloadVelocity": "orange"}

    t_brake = detect_brake_start(data[0])
    if t_brake is None:
        t_brake = 40.0

    # Determine time window
    if phase == "brake":
        t_start = max(0.0, t_brake - 1.0)
        t_end = min(data[0]["time_s"].iloc[-1], t_brake + 18.0)
    else:
        t_start = 0.0
        t_end = data[0]["time_s"].iloc[-1]

    # Subsample for animation frames
    total_frames = int(fps * duration_s)
    frame_times = np.linspace(t_start, t_end, total_frames)

    # Compute px at brake start for each mode (for relative positioning)
    px_brake_list = []
    for df in data:
        t = df["time_s"].values
        px = df["px_truth_m"].values
        px_brake = np.interp(t_brake, t, px)
        px_brake_list.append(px_brake)

    # Precompute interpolated states for all modes
    interp_states = []
    for i, df in enumerate(data):
        t = df["time_s"].values
        px = df["px_truth_m"].values
        theta = df["theta_truth_rad"].values
        vx = df["vx_truth_m_s"].values
        ax = df["ax_applied_m_s2"].values

        px_f = np.interp(frame_times, t, px) - px_brake_list[i]
        theta_f = np.interp(frame_times, t, theta)
        vx_f = np.interp(frame_times, t, vx)
        ax_f = np.interp(frame_times, t, ax)
        interp_states.append({
            "label": df["mode"].iloc[0],
            "px": px_f,
            "theta": theta_f,
            "vx": vx_f,
            "ax": ax_f,
        })

    # Setup figure: taller for better swing visibility
    fig, ax = plt.subplots(figsize=(12, 10))
    ax.set_xlabel("Relative Horizontal Position [m]", fontsize=12)
    ax.set_ylabel("Height [m]", fontsize=12)
    ax.grid(True, alpha=0.3)

    # Y limits: extend downward for longer rope visibility
    ax.set_ylim(-rope_len - 15, drone_height + 2)

    # Determine X limits based on phase
    if phase == "brake":
        # Relative px: all start near 0 at brake start
        ax.set_xlim(-8, 58)
    else:
        px_max_all = max(df["px_truth_m"].max() for df in data)
        ax.set_xlim(-5, px_max_all + 10)

    # Title
    title = ax.set_title("", fontsize=14, fontweight="bold")

    # Create artists for each mode
    artists = []
    for state in interp_states:
        c = colors.get(state["label"], "black")
        # Drone body (rectangle)
        drone_rect = Rectangle((0, 0), drone_width, drone_height,
                               facecolor=c, edgecolor="black", alpha=0.7, linewidth=1.5)
        ax.add_patch(drone_rect)
        # Rope (line)
        rope_line, = ax.plot([], [], color=c, linewidth=2, solid_capstyle="round")
        # Payload (circle marker)
        payload_dot, = ax.plot([], [], "o", color=c, markersize=10, markeredgecolor="black", markeredgewidth=1)
        # Label text near drone
        label_text = ax.text(0, 0, state["label"], color=c, fontsize=10, fontweight="bold",
                             ha="center", va="bottom")
        # Speed indicator bar (mini bar on drone)
        speed_bar, = ax.plot([], [], color="white", linewidth=3, solid_capstyle="round")

        artists.append({
            "drone_rect": drone_rect,
            "rope_line": rope_line,
            "payload_dot": payload_dot,
            "label_text": label_text,
            "speed_bar": speed_bar,
            "color": c,
        })

    # Info text box (bottom left)
    info_text = ax.text(0.02, 0.02, "", transform=ax.transAxes, fontsize=11,
                        verticalalignment="bottom", fontfamily="monospace",
                        bbox=dict(boxstyle="round,pad=0.5", facecolor="white", edgecolor="gray", alpha=0.9))

    # Legend
    ax.legend([a["payload_dot"] for a in artists],
              [s["label"] for s in interp_states],
              loc="lower right", fontsize=10)

    def init():
        for art in artists:
            art["rope_line"].set_data([], [])
            art["payload_dot"].set_data([], [])
            art["speed_bar"].set_data([], [])
        info_text.set_text("")
        title.set_text("")
        return [info_text, title] + [item for art in artists for item in
                                     [art["rope_line"], art["payload_dot"], art["speed_bar"], art["label_text"]]]

    def update(frame):
        t = frame_times[frame]
        info_lines = [f"Time: {t:.2f} s"]

        for i, (state, art) in enumerate(zip(interp_states, artists)):
            px = state["px"][frame]
            th = state["theta"][frame]
            vx = state["vx"][frame]
            ax_val = state["ax"][frame]

            # Drone center position (all modes at same x, stacked visually by color)
            drone_cx = px
            drone_cy_base = 0.0
            drone_cy = drone_cy_base + drone_height / 2.0

            # Update drone rectangle
            art["drone_rect"].set_xy((drone_cx - drone_width / 2.0, drone_cy_base))

            # Payload position
            pay_x = drone_cx + rope_len * np.sin(th)
            pay_y = drone_cy_base - rope_len * np.cos(th)

            # Update rope
            art["rope_line"].set_data([drone_cx, pay_x], [drone_cy_base, pay_y])

            # Update payload
            art["payload_dot"].set_data([pay_x], [pay_y])

            # Update label (above drone)
            art["label_text"].set_position((drone_cx, drone_height + 0.5))

            # Speed indicator (small bar inside drone showing vx direction/magnitude)
            bar_len = vx / 20.0 * (drone_width / 2.0)  # scale: 20 m/s = half width
            art["speed_bar"].set_data([drone_cx, drone_cx + bar_len], [drone_height / 2.0, drone_height / 2.0])

            info_lines.append(f"{state['label'][:8]:8s}  x={px:6.1f}  vx={vx:6.2f}  th={np.rad2deg(th):6.2f}°  ax={ax_val:5.2f}")

        info_text.set_text("\n".join(info_lines))

        if phase == "brake":
            title.set_text(f"LQR Brake Phase Animation  (t - t_brake = {t - t_brake:+.2f} s)")
        else:
            title.set_text("LQR Full Trajectory Animation")

        return [info_text, title] + [item for art in artists for item in
                                     [art["rope_line"], art["payload_dot"], art["speed_bar"], art["label_text"]]]

    ani = matplotlib.animation.FuncAnimation(fig, update, frames=total_frames,
                                              init_func=init, blit=False, interval=1000 // fps)

    # Save
    if output_path.endswith(".gif"):
        writer = PillowWriter(fps=fps)
    else:
        writer = FFMpegWriter(fps=fps, metadata=dict(artist="pend_observer_test"),
                              bitrate=5000)

    ani.save(output_path, writer=writer)
    plt.close(fig)
    print(f"Animation saved to: {output_path}")
    print(f"  Phase: {phase}, Duration: {duration_s}s, FPS: {fps}, Frames: {total_frames}")


def main():
    parser = argparse.ArgumentParser(description="Generate LQR simulation animation")
    parser.add_argument("files", nargs="+", help="Input CSV files")
    parser.add_argument("--labels", nargs="+", default=None, help="Labels for each file")
    parser.add_argument("--phase", choices=["brake", "full"], default="brake",
                        help="Animation phase: brake=brake phase only, full=full trajectory")
    parser.add_argument("--output", default="lqr_animation.mp4", help="Output file path")
    parser.add_argument("--fps", type=int, default=30, help="Frames per second")
    parser.add_argument("--duration", type=float, default=8.0, help="Animation duration in seconds")
    args = parser.parse_args()

    files = args.files
    if args.labels is None:
        labels = [f"Mode{i}" for i in range(len(files))]
    else:
        labels = args.labels
        if len(labels) != len(files):
            print("Error: number of labels must match number of files")
            sys.exit(1)

    create_animation(files, labels, output_path=args.output, phase=args.phase,
                     fps=args.fps, duration_s=args.duration)


if __name__ == "__main__":
    import os
    build_dir = os.path.join(os.path.dirname(__file__), "..", "build")
    default_files = [
        os.path.join(build_dir, "closed_loop_full.csv"),
        os.path.join(build_dir, "closed_loop_shortest.csv"),
        os.path.join(build_dir, "closed_loop_minswing.csv"),
        os.path.join(build_dir, "closed_loop_velomega.csv"),
        os.path.join(build_dir, "closed_loop_payload.csv"),
    ]
    if len(sys.argv) < 6 and all(os.path.exists(f) for f in default_files):
        # Inject default files before argparse sees the args
        extra_args = []
        if "--" in sys.argv:
            dash_idx = sys.argv.index("--")
            extra_args = sys.argv[dash_idx:]
            sys.argv = sys.argv[:dash_idx]
        # Find where options end
        opt_idx = 1
        while opt_idx < len(sys.argv) and sys.argv[opt_idx].startswith("-"):
            opt_idx += 1
        sys.argv = sys.argv[:opt_idx] + default_files + sys.argv[opt_idx:] + extra_args
    main()
