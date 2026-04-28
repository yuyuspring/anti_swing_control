#!/bin/bash
# 一键执行完整流程：生成LQR增益 -> 编译 -> 仿真 -> 绘图/动画
#
# 用法:
#   ./run.sh                      执行全部（仿真+对比图+刹车图+动画）
#   ./run.sh --all                同上
#   ./run.sh --comparison  (-c)   只绘制全段对比图（comparison.png）
#   ./run.sh --brake       (-b)   只绘制刹车段对比图（brake_phase.png）
#   ./run.sh --animation   (-a)   只生成刹车段动画（lqr_brake_animation.mp4）
#   ./run.sh --help        (-h)   显示帮助

set -e

print_usage() {
    cat << EOF
Usage: ./run.sh [OPTION]

Options:
  (none)           执行全部流程：仿真 + 对比图 + 刹车图 + 动画
  --all, -a        同上
  --comparison, -c 只生成 comparison.png（全段对比）
  --brake, -b      只生成 brake_phase.png（刹车段对比）
  --animation, -m  只生成 lqr_brake_animation.mp4（刹车段动画）
  --help, -h       显示此帮助信息

Examples:
  ./run.sh              # 完整流程
  ./run.sh -c           # 仅生成全段对比图
  ./run.sh -b           # 仅生成刹车段对比图
  ./run.sh -m           # 仅生成动画
EOF
}

# Parse arguments
MODE="all"
if [[ $# -ge 1 ]]; then
    case "$1" in
        --all|-a)
            MODE="all"
            ;;
        --comparison|-c)
            MODE="comparison"
            ;;
        --brake|-b)
            MODE="brake"
            ;;
        --animation|-m)
            MODE="animation"
            ;;
        --help|-h)
            print_usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            print_usage
            exit 1
            ;;
    esac
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
echo "Step 3: Generate Outputs (mode=$MODE)"
echo "========================================"

if [[ "$MODE" == "all" || "$MODE" == "comparison" ]]; then
    echo "--> Generating comparison.png ..."
    python3 ../scripts/plot_comparison.py \
        closed_loop_full.csv closed_loop_shortest.csv \
        closed_loop_minswing.csv closed_loop_velomega.csv closed_loop_payload.csv
fi

if [[ "$MODE" == "all" || "$MODE" == "brake" ]]; then
    echo "--> Generating brake_phase.png ..."
    python3 ../scripts/plot_brake_phase.py \
        closed_loop_full.csv closed_loop_shortest.csv \
        closed_loop_minswing.csv closed_loop_velomega.csv closed_loop_payload.csv
fi

if [[ "$MODE" == "all" || "$MODE" == "animation" ]]; then
    echo "--> Generating lqr_brake_animation.mp4 ..."
    python3 ../scripts/plot_animation.py \
        closed_loop_full.csv closed_loop_shortest.csv \
        closed_loop_minswing.csv closed_loop_velomega.csv closed_loop_payload.csv \
        --labels Full Shortest MinSwing VelocityOmega PayloadVelocity \
        --phase brake --output lqr_brake_animation.mp4 --fps 30 --duration 10
fi

cd ..

echo ""
echo "========================================"
echo "All done!"
echo "========================================"

if [[ "$MODE" == "all" || "$MODE" == "comparison" ]]; then
    echo "  - build/comparison.png"
fi
if [[ "$MODE" == "all" || "$MODE" == "brake" ]]; then
    echo "  - build/brake_phase.png"
fi
if [[ "$MODE" == "all" || "$MODE" == "animation" ]]; then
    echo "  - build/lqr_brake_animation.mp4"
fi
echo "  - include/controller/lqr_gain.hpp"
