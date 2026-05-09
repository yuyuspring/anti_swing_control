#!/bin/bash
# 一键执行完整流程：生成LQR增益 -> 编译 -> 仿真 -> 绘图/动画
#
# 用法:
#   ./run_lqr.sh                      执行全部（仿真+对比图+刹车图+动画）
#   ./run_lqr.sh --all                同上
#   ./run_lqr.sh --comparison  (-c)   只绘制全段对比图（comparison.png）
#   ./run_lqr.sh --brake       (-b)   只绘制刹车段对比图（brake_phase.png）
#   ./run_lqr.sh --animation   (-m)   只生成刹车段动画（lqr_brake_animation.mp4）
#   ./run_lqr.sh --single <n>  (-s)   只运行模式 n 并生成单张图
#   ./run_lqr.sh --truth       (-t)   使用真值（ground truth）代替观测器估计
#   ./run_lqr.sh --help        (-h)   显示帮助
#
# 模式编号:
#   0=Diagonal (对角Q), 1=Coupled (非对角Q)

set -e

print_usage() {
    cat << EOF
Usage: ./run_lqr.sh [OPTION]

Options:
  (none)           执行全部流程：仿真 + 对比图 + 刹车图 + 动画
  --all, -a        同上
  --comparison, -c 只生成 comparison.png（全段对比）
  --brake, -b      只生成 brake_phase.png（刹车段对比）
  --animation, -m  只生成 lqr_brake_animation.mp4（刹车段动画）
  --single <n>, -s <n>  只运行模式 n 并生成单张详细图
  --truth, -t          使用真值（ground truth）控制，不用观测器
  --help, -h           显示此帮助信息

模式编号:
  0=Diagonal (对角Q), 1=Coupled (非对角Q)

Examples:
  ./run_lqr.sh              # 完整流程（观测器）
  ./run_lqr.sh -c           # 仅生成全段对比图
  ./run_lqr.sh -s 1         # 仅运行 Coupled 模式 + 单张图
  ./run_lqr.sh -t           # 使用真值控制（对比用）
EOF
}

# Map mode number to CSV filename
mode_to_file() {
    case "$1" in
        0) echo "closed_loop_diagonal.csv" ;;
        1) echo "closed_loop_coupled.csv" ;;
        *) echo "unknown" ;;
    esac
}

# Parse arguments
MODE="all"
SINGLE_MODE=""
USE_OBSERVER=1

while [[ $# -gt 0 ]]; do
    case "$1" in
        --all|-a)
            MODE="all"
            shift
            ;;
        --comparison|-c)
            MODE="comparison"
            shift
            ;;
        --brake|-b)
            MODE="brake"
            shift
            ;;
        --animation|-m)
            MODE="animation"
            shift
            ;;
        --single|-s)
            if [[ -z "$2" || "$2" =~ ^- ]]; then
                echo "Error: --single requires a mode number (0-1)"
                print_usage
                exit 1
            fi
            SINGLE_MODE="$2"
            shift 2
            ;;
        --truth|-t)
            USE_OBSERVER=0
            shift
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
done

echo "========================================"
echo "Step 1: Compute LQR gains"
echo "========================================"
python3 ./scripts/design/compute_lqr_gain.py --rope-length 10.0 --pendulum-gain 0.6

echo ""
echo "========================================"
echo "Step 2: Build & Run Simulations"
echo "========================================"
cd build
make -j$(nproc)
cd ..

# Run LQR simulations
mkdir -p results/lqr
cd results/lqr

if [[ -n "$SINGLE_MODE" ]]; then
    echo "--> Running single mode: $SINGLE_MODE (observer=$USE_OBSERVER) ..."
    ../../build/run_closed_loop_lqr "$SINGLE_MODE" 0 10 60 "$USE_OBSERVER"
else
    echo "--> Running both modes (observer=$USE_OBSERVER) ..."
    ../../build/run_closed_loop_lqr 0 0 10 60 "$USE_OBSERVER"
    ../../build/run_closed_loop_lqr 1 0 10 60 "$USE_OBSERVER"
fi

echo ""
echo "========================================"
echo "Step 3: Generate Outputs (mode=$MODE)"
echo "========================================"

if [[ -n "$SINGLE_MODE" ]]; then
    CSV_FILE=$(mode_to_file "$SINGLE_MODE")
    echo "--> Generating single-mode plot for $CSV_FILE ..."
    python3 ../../scripts/plot/plot_closed_loop.py "$CSV_FILE"
fi

if [[ -z "$SINGLE_MODE" && ( "$MODE" == "all" || "$MODE" == "comparison" ) ]]; then
    echo "--> Generating comparison.png ..."
    python3 ../../scripts/plot/plot_comparison.py \
        closed_loop_diagonal.csv closed_loop_coupled.csv
fi

if [[ -z "$SINGLE_MODE" && ( "$MODE" == "all" || "$MODE" == "brake" ) ]]; then
    echo "--> Generating brake_phase.png ..."
    python3 ../../scripts/plot/plot_brake_phase.py \
        closed_loop_diagonal.csv closed_loop_coupled.csv
fi

if [[ -z "$SINGLE_MODE" && ( "$MODE" == "all" || "$MODE" == "animation" ) ]]; then
    echo "--> Generating lqr_brake_animation.mp4 ..."
    python3 ../../scripts/plot/plot_animation.py \
        closed_loop_diagonal.csv closed_loop_coupled.csv \
        --labels Diagonal Coupled \
        --phase brake --output lqr_brake_animation.mp4 --fps 60 --duration 10
fi

cd ../..

echo ""
echo "========================================"
echo "All done!"
echo "========================================"

if [[ -n "$SINGLE_MODE" ]]; then
    CSV_FILE=$(mode_to_file "$SINGLE_MODE")
    echo "  - results/lqr/${CSV_FILE%.csv}.png"
fi
if [[ -z "$SINGLE_MODE" && ( "$MODE" == "all" || "$MODE" == "comparison" ) ]]; then
    echo "  - results/lqr/comparison.png"
fi
if [[ -z "$SINGLE_MODE" && ( "$MODE" == "all" || "$MODE" == "brake" ) ]]; then
    echo "  - results/lqr/brake_phase.png"
fi
if [[ -z "$SINGLE_MODE" && ( "$MODE" == "all" || "$MODE" == "animation" ) ]]; then
    echo "  - results/lqr/lqr_brake_animation.mp4"
fi
echo "  - include/controller/lqr_gain.hpp"
