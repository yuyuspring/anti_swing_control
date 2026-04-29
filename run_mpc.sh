#!/bin/bash
# 一键运行 MPC 仿真 + 绘图
#
# 用法:
#   ./run_mpc.sh              运行 MPC 仿真 + 绘制全部对比图
#   ./run_mpc.sh --single     运行 MPC 仿真 + 只绘制 MPC min |omega|
#   ./run_mpc.sh --energy     运行 MPC 仿真 + 只绘制 MPC min sys energy
#   ./run_mpc.sh --compare    运行 MPC 仿真 + MPC vs LQR 对比
#   ./run_mpc.sh --plot-only  不运行仿真，只绘图
#   ./run_mpc.sh --help       显示帮助

set -e

PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"
RESULTS_DIR="$PROJECT_ROOT/results/mpc"

cd "$PROJECT_ROOT"

print_usage() {
    cat << EOF
Usage: ./run_mpc.sh [OPTION]

Options:
  (none)            运行 MPC 仿真，并绘制全部对比图（默认）
  --single, -s      只绘制 MPC min |omega| 单组曲线
  --energy, -e      只绘制 MPC min sys energy 单组曲线
  --compare, -c     绘制 MPC min |omega| vs LQR VelocityOmega 对比
  --plot-only, -p   跳过仿真，直接绘图
  --help, -h        显示此帮助信息

输出:
  - results/mpc/closed_loop_mpc_minomega.csv
  - results/mpc/closed_loop_mpc_minsysenergy.csv
  - results/mpc/mpc_*.png

环境要求:
  - Python 虚拟环境 .venv/ 需已安装 cvxpy, osqp, pandas, matplotlib
EOF
}

MODE="all"
PLOT_ONLY=false
while [[ $# -gt 0 ]]; do
    case "$1" in
        --single|-s)
            MODE="single"
            shift
            ;;
        --energy|-e)
            MODE="energy"
            shift
            ;;
        --compare|-c)
            MODE="compare"
            shift
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

# Step 1: Run MPC simulation (unless --plot-only)
if [[ "$PLOT_ONLY" == false ]]; then
    echo "========================================"
    echo "Step 1: Running MPC closed-loop simulation ..."
    echo "========================================"
    python3 "$PROJECT_ROOT/scripts/simulation/run_mpc_simulation.py" min_omega
    python3 "$PROJECT_ROOT/scripts/simulation/run_mpc_simulation.py" min_system_energy
fi

# Step 2: Run plotting
echo ""
echo "========================================"
echo "Step 2: Generating plot ..."
echo "========================================"

case "$MODE" in
    all)
        python3 "$PROJECT_ROOT/scripts/plot/plot_mpc.py" --all
        ;;
    single)
        python3 "$PROJECT_ROOT/scripts/plot/plot_mpc.py" --single
        ;;
    energy)
        python3 "$PROJECT_ROOT/scripts/plot/plot_mpc.py" --energy
        ;;
    compare)
        python3 "$PROJECT_ROOT/scripts/plot/plot_mpc.py" --compare
        ;;
esac

cd "$PROJECT_ROOT"

echo ""
echo "========================================"
echo "Done!"
echo "========================================"
ls -lh "$RESULTS_DIR"/mpc_*.png 2>/dev/null || true
