#!/bin/bash
# 一键执行完整流程：生成LQR增益 -> 编译 -> 仿真 -> 绘图
#
# 用法:
#   ./run.sh        执行全部（仿真+两组图）
#   ./run.sh 1      执行全部（仿真+两组图）
#   ./run.sh 2      只绘制全段对比图（comparison.png）
#   ./run.sh 3      只绘制刹车段对比图（brake_phase.png）

set -e  # 遇到错误立即退出

# 解析参数
PLOT_MODE="${1:-1}"  # 默认模式1

# 参数校验
if [[ "$PLOT_MODE" != "1" && "$PLOT_MODE" != "2" && "$PLOT_MODE" != "3" ]]; then
    echo "Usage: ./run.sh [1|2|3]"
    echo "  1 (default): 仿真 + 绘制两组图"
    echo "  2          : 仿真 + 只绘制全段对比图"
    echo "  3          : 仿真 + 只绘制刹车段对比图"
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

echo ""
echo "========================================"
echo "Step 3: Generate Plots (mode=$PLOT_MODE)"
echo "========================================"

if [[ "$PLOT_MODE" == "1" || "$PLOT_MODE" == "2" ]]; then
    echo "--> Generating comparison.png ..."
    python3 ../scripts/plot_comparison.py closed_loop_full.csv closed_loop_shortest.csv closed_loop_minswing.csv closed_loop_velomega.csv
fi

if [[ "$PLOT_MODE" == "1" || "$PLOT_MODE" == "3" ]]; then
    echo "--> Generating brake_phase.png ..."
    python3 ../scripts/plot_brake_phase.py closed_loop_full.csv closed_loop_shortest.csv closed_loop_minswing.csv closed_loop_velomega.csv
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
echo "  - include/controller/lqr_gain.hpp"
