#!/bin/bash
# 一键执行完整流程：生成LQR增益 -> 编译 -> 仿真 -> 绘图
#
# 用法:
#   ./run.sh        执行全部（仿真+两组图+动画）
#   ./run.sh 1      执行全部（仿真+两组图+动画）
#   ./run.sh 2      只绘制全段对比图（comparison.png）
#   ./run.sh 3      只绘制刹车段对比图（brake_phase.png）
#   ./run.sh 4      只生成刹车段动画（lqr_brake_animation.mp4）

set -e  # 遇到错误立即退出

# 解析参数
PLOT_MODE="${1:-1}"  # 默认模式1

# 参数校验
if [[ "$PLOT_MODE" != "1" && "$PLOT_MODE" != "2" && "$PLOT_MODE" != "3" && "$PLOT_MODE" != "4" ]]; then
    echo "Usage: ./run.sh [1|2|3|4]"
    echo "  1 (default): 仿真 + 绘制两组图 + 生成动画"
    echo "  2          : 仿真 + 只绘制全段对比图"
    echo "  3          : 仿真 + 只绘制刹车段对比图"
    echo "  4          : 仿真 + 只生成刹车段动画"
    exit 1
fi

echo "========================================"
echo "Step 1: Compute LQR gains"
echo "========================================"
python3 ./scripts/compute_lqr_gain.py

echo ""
echo "========================================"
echo "Step 2: Build & Run Simulations"
echo "========================================"
cd build
make -j$(nproc)

./run_closed_loop_lqr 0
./run_closed_loop_lqr 1
./run_closed_loop_lqr 2
./run_closed_loop_lqr 3
./run_closed_loop_lqr 4

echo ""
echo "========================================"
echo "Step 3: Generate Plots (mode=$PLOT_MODE)"
echo "========================================"

if [[ "$PLOT_MODE" == "1" || "$PLOT_MODE" == "2" ]]; then
    echo "--> Generating comparison.png ..."
    python3 ../scripts/plot_comparison.py closed_loop_full.csv closed_loop_shortest.csv closed_loop_minswing.csv closed_loop_velomega.csv closed_loop_payload.csv
fi

if [[ "$PLOT_MODE" == "1" || "$PLOT_MODE" == "3" ]]; then
    echo "--> Generating brake_phase.png ..."
    python3 ../scripts/plot_brake_phase.py closed_loop_full.csv closed_loop_shortest.csv closed_loop_minswing.csv closed_loop_velomega.csv closed_loop_payload.csv
fi

if [[ "$PLOT_MODE" == "1" || "$PLOT_MODE" == "4" ]]; then
    echo "--> Generating lqr_brake_animation.mp4 ..."
    python3 ../scripts/plot_animation.py \
        closed_loop_full.csv closed_loop_shortest.csv closed_loop_minswing.csv closed_loop_velomega.csv closed_loop_payload.csv \
        --labels Full Shortest MinSwing VelocityOmega PayloadVelocity \
        --phase brake --output lqr_brake_animation.mp4 --fps 30 --duration 10
fi

cd ..

echo ""
echo "========================================"
echo "All done!"
echo "========================================"

if [[ "$PLOT_MODE" == "1" || "$PLOT_MODE" == "2" ]]; then
    echo "  - build/comparison.png"
fi
if [[ "$PLOT_MODE" == "1" || "$PLOT_MODE" == "3" ]]; then
    echo "  - build/brake_phase.png"
fi
if [[ "$PLOT_MODE" == "1" || "$PLOT_MODE" == "4" ]]; then
    echo "  - build/lqr_brake_animation.mp4"
fi
echo "  - include/controller/lqr_gain.hpp"
