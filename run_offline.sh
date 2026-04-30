#!/bin/bash
# 一键运行 Jerk-Ramp 离线轨迹仿真 + 绘图
#
# 用法:
#   ./run_offline.sh                  运行仿真 (默认 jerk=0.5) + 画图
#   ./run_offline.sh --jerk 0.3       指定 jerk 大小
#   ./run_offline.sh --brake-start 30 指定刹车开始时间
#   ./run_offline.sh --plot-only      不运行仿真，只画图
#   ./run_offline.sh --help           显示帮助

set -e

PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"
RESULTS_DIR="$PROJECT_ROOT/results/trajectory"

cd "$PROJECT_ROOT"

print_usage() {
    cat << EOF
Usage: ./run_offline.sh [OPTION]

Options:
  --jerk J           第0段 jerk 大小 (m/s³), 默认 0.3
  --brake-start T    刹车开始时间 (s), 默认 40.0
  --plot-only, -p    跳过仿真，直接绘图
  --help, -h         显示此帮助信息

示例:
  ./run_offline.sh --jerk 0.3          # 用小 jerk 获得低摆动
  ./run_offline.sh --jerk 0.17         # 极小 jerk，几乎无残余摆动
  ./run_offline.sh --plot-only         # 只重新画图

输出:
  - results/trajectory/offline_three_phase.csv
  - results/trajectory/offline_three_phase.png
  - results/trajectory/offline_three_phase_brake.png

参数说明:
  jerk 越小 → Max θ 越小 → 残余摆动越小 → 但刹车时间和距离越长
  jerk 越大 → 刹车越快 → 但激发摆动越强
EOF
}

JERK=""
BRAKE_START=""
PLOT_ONLY=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --jerk)
            JERK="--jerk $2"
            shift 2
            ;;
        --brake-start)
            BRAKE_START="--brake-start $2"
            shift 2
            ;;
        --plot-only|-p)
            PLOT_ONLY=true
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

# 检查虚拟环境
if [[ ! -d "$PROJECT_ROOT/.venv" ]]; then
    echo "Error: .venv/ not found. Please create and activate the virtual environment."
    exit 1
fi

source "$PROJECT_ROOT/.venv/bin/activate"

# Step 1: Run simulation (unless --plot-only)
if [[ "$PLOT_ONLY" == false ]]; then
    echo "========================================"
    echo "Step 1: Running offline trajectory simulation ..."
    echo "========================================"
    python3 "$PROJECT_ROOT/scripts/simulation/run_offline_trajectory.py" $JERK $BRAKE_START
fi

# Step 2: Run plotting
echo ""
echo "========================================"
echo "Step 2: Generating plots ..."
echo "========================================"

python3 "$PROJECT_ROOT/scripts/plot/plot_offline_trajectory.py"
python3 "$PROJECT_ROOT/scripts/plot/plot_offline_brake_phase.py"

cd "$PROJECT_ROOT"

echo ""
echo "========================================"
echo "Done!"
echo "========================================"
ls -lh "$RESULTS_DIR"/offline_three_phase*.png 2>/dev/null || true
