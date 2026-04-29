#!/bin/bash
# 一键执行 CSV replay 验证流程
#
# 用法:
#   ./run_replay.sh                  默认输入: data/crane_imu_obs_debug.csv
#   ./run_replay.sh my_data.csv      指定自定义输入
#   ./run_replay.sh --help           显示帮助

set -e

print_usage() {
    cat << EOF
Usage: ./run_replay.sh [INPUT_CSV]

Options:
  (none)              使用默认输入 data/crane_imu_obs_debug.csv
  <file.csv>          指定自定义 CSV 输入文件
  --help, -h          显示此帮助信息

输出:
  - results/replay_validation.csv
  - results/replay_validation.png
EOF
}

# Parse arguments
INPUT_CSV="data/crane_imu_obs_debug.csv"
if [[ $# -ge 1 ]]; then
    case "$1" in
        --help|-h)
            print_usage
            exit 0
            ;;
        *)
            INPUT_CSV="$1"
            ;;
    esac
fi

VALIDATION_CSV="results/replay_validation.csv"
PLOT_PNG="results/replay_validation.png"

echo "========================================"
echo "CSV Replay Validation"
echo "========================================"
echo "Input : $INPUT_CSV"
echo "Output: $VALIDATION_CSV"
echo "Plot  : $PLOT_PNG"
echo ""

# Build
cd build
make -j$(nproc)

# Run replay
./run_csv_replay "../$INPUT_CSV" "../$VALIDATION_CSV"

cd ..

# Plot results
echo ""
echo "========================================"
echo "Generating plot ..."
echo "========================================"
python3 scripts/plot/plot_replay_results.py "$VALIDATION_CSV" "$PLOT_PNG"

echo ""
echo "========================================"
echo "Done!"
echo "========================================"
echo "  - $VALIDATION_CSV"
echo "  - $PLOT_PNG"
