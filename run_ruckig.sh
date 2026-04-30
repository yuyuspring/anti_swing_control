#!/bin/bash
# 一键生成 Ruckig S-curve 轨迹并绘图
#
# 用法:
#   ./run_ruckig.sh              生成轨迹 + 绘图
#   ./run_ruckig.sh --plot-only  跳过轨迹生成，只绘图
#   ./run_ruckig.sh --help       显示帮助

set -e

PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"
RESULTS_DIR="$PROJECT_ROOT/results"

cd "$PROJECT_ROOT"

print_usage() {
    cat << EOF
Usage: ./run_ruckig.sh [OPTION]

Options:
  (none)            编译 + 生成轨迹 + 绘图
  --plot-only, -p   跳过编译和轨迹生成，只绘图
  --help, -h        显示此帮助信息

输出:
  - results/ruckig_trajectory.csv
  - results/ruckig_trajectory.png
EOF
}

PLOT_ONLY=false
if [[ $# -ge 1 ]]; then
    case "$1" in
        --plot-only|-p)
            PLOT_ONLY=true
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

if [[ "$PLOT_ONLY" == false ]]; then
    echo "========================================"
    echo "Step 1: Build"
    echo "========================================"
    cd build
    make -j$(nproc)
    cd ..

    echo ""
    echo "========================================"
    echo "Step 2: Generate S-curve trajectory"
    echo "========================================"
    ./build/run_ruckig_trajectory
fi

echo ""
echo "========================================"
echo "Step 3: Plot trajectory"
echo "========================================"
python3 scripts/plot/plot_trajectory.py

echo ""
echo "========================================"
echo "Done!"
echo "========================================"
echo "  - results/ruckig_trajectory.csv"
echo "  - results/ruckig_trajectory.png"
