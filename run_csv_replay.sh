#!/bin/bash
# 一键运行 CSV replay 验证 + 绘图
#
# 用法:
#   ./run_csv_replay.sh                    使用默认输入文件
#   ./run_csv_replay.sh <input.csv>        指定输入 CSV

set -e

INPUT_CSV="${1:-data/crane_imu_obs_debug.csv}"
VALIDATION_CSV="results/replay_validation.csv"
PLOT_PNG="results/replay_validation.png"

echo "========================================"
echo "Step 1: Build"
echo "========================================"
cd build
make -j$(nproc) run_csv_replay

echo ""
echo "========================================"
echo "Step 2: Run CSV Replay"
echo "========================================"
echo "  Input : ${INPUT_CSV}"
echo "  Output: ${VALIDATION_CSV}"
./run_csv_replay "../${INPUT_CSV}" "../${VALIDATION_CSV}"

echo ""
echo "========================================"
echo "Step 3: Plot Validation Results"
echo "========================================"
python3 ../scripts/plot_replay_results.py "../${VALIDATION_CSV}" "../${PLOT_PNG}"

cd ..

echo ""
echo "========================================"
echo "All done!"
echo "========================================"
echo "  - ${VALIDATION_CSV}"
echo "  - ${PLOT_PNG}"
