# 项目框架结构

## 1. 项目概述

| 项目 | 说明 |
|------|------|
| **名称** | Anti-Swing Control |
| **目标** | 吊重系统摆角观测与防摆控制 |
| **控制方法** | LQR（7种模式）+ MPC（2种模式） |
| **语言** | C++17（核心仿真）+ Python（离线设计、可视化） |
| **构建** | CMake |

---

## 2. 目录结构总览

```
pend_observer_test/
│
├── CMakeLists.txt              # C++ 构建配置
├── README.md                   # 项目文档（Quick Start）
├── PROJECT_STRUCTURE.md        # 本文件
│
├── 一键入口脚本（根目录）
│   ├── run_lqr.sh              # LQR 闭环仿真全流程
│   ├── run_mpc.sh              # MPC 闭环仿真 + 绘图
│   └── run_replay.sh           # 观测器 CSV 回放验证
│
├── include/                    # C++ 公共头文件
│   ├── controller/             # 控制器（LQR、增益表）
│   ├── dynamics/               # 吊重动力学
│   ├── observer/               # 摆角观测器
│   ├── sensor/                 # IMU 传感器模型
│   ├── simulation/             # 仿真引擎
│   └── utils/                  # 工具类
│
├── src/                        # C++ 实现
│   ├── apps/                   # 可执行程序入口
│   ├── controller/
│   ├── dynamics/
│   ├── observer/
│   ├── sensor/
│   ├── simulation/
│   └── utils/
│
├── scripts/                    # Python 脚本（按功能分组）
│   ├── design/                 # 离线设计
│   ├── simulation/             # MPC 仿真
│   ├── plot/                   # 绘图脚本
│   └── analysis/               # 对比分析
│
├── data/                       # 输入数据（IMU 记录）
├── results/                    # 输出结果（按类型分组）
│   ├── lqr/                    # LQR 仿真结果
│   ├── mpc/                    # MPC 仿真结果
│   └── replay/                 # CSV 回放结果
├── docs/                       # 设计文档
└── build/                      # CMake 构建输出（仅编译产物）
```

---

## 3. C++ 核心库（include/ + src/）

### 3.1 模块划分

| 模块 | 头文件 | 实现 | 职责 |
|------|--------|------|------|
| **observer** | `pend_observer.h` | `pend_observer.cpp` | Legacy C 观测器核心（ESO + 四元数） |
| | `pendulum_observer.hpp` | `pendulum_observer.cpp` | C++ 封装层（单位转换接口） |
| **dynamics** | `slung_load_dynamics.hpp` | `slung_load_dynamics.cpp` | 吊重非线性动力学（RK4 积分） |
| **controller** | `lqr_controller.hpp` | `lqr_controller.cpp` | LQR 状态反馈控制器 |
| | `lqr_gain.hpp` | — | 7种模式增益表（Python 自动生成） |
| **sensor** | `imu_sensor_model.hpp` | `imu_sensor_model.cpp` | IMU 传感器模型与噪声 |
| **simulation** | `closed_loop_simulation.hpp` | `closed_loop_simulation.cpp` | 闭环仿真引擎（LQR/MPC） |
| | `open_loop_simulation.hpp` | `open_loop_simulation.cpp` | 开环仿真引擎 |
| **utils** | `csv_logger.hpp` | `csv_logger.cpp` | CSV 数据记录 |
| | `math_utils.hpp` | `math_utils.cpp` | 数学工具 |
| | `type_defs.hpp` | — | 类型定义与结构体 |

### 3.2 可执行程序（src/apps/）

| 可执行文件 | 入口文件 | 功能 |
|-----------|---------|------|
| `run_open_loop_sim` | `run_open_loop_sim.cpp` | 开环仿真（3 种场景：pitch / roll / mixed） |
| `run_csv_replay` | `run_csv_replay.cpp` | CSV 数据回放验证 |
| `run_closed_loop_lqr` | `run_closed_loop_lqr.cpp` | 闭环 LQR 仿真（7 种模式） |

### 3.3 静态库

`libpendulum.a` — 包含 observer、dynamics、controller、sensor、simulation、utils 全部核心代码。

---

## 4. Python 脚本（scripts/）

### 4.1 离线设计（scripts/design/）

| 脚本 | 功能 | 输出 |
|------|------|------|
| `compute_lqr_gain.py` | 求解 DARE → 生成 7 种 LQR 增益 + 稳定性检查 | `include/controller/lqr_gain.hpp` |

### 4.2 仿真（scripts/simulation/）

| 脚本 | 功能 | 输出 |
|------|------|------|
| `run_mpc_simulation.py` | MPC 闭环仿真（min_omega / min_system_energy） | `results/mpc/closed_loop_mpc_*.csv` |

### 4.3 绘图（scripts/plot/）

| 脚本 | 功能 | 输入 |
|------|------|------|
| `plot_mpc.py` | MPC 单图 / 对比图（参数控制） | `results/mpc/closed_loop_mpc_*.csv` |
| `plot_comparison.py` | LQR 7 模式全段对比 | `results/lqr/closed_loop_*.csv` |
| `plot_brake_phase.py` | LQR 7 模式刹车段对比 | `results/lqr/closed_loop_*.csv` |
| `plot_closed_loop.py` | 单组 LQR 闭环结果 | `results/lqr/closed_loop_*.csv` |
| `plot_animation.py` | LQR 动画生成 | `results/lqr/closed_loop_*.csv` |
| `plot_results.py` | 开环仿真结果 | `results/simulation_results*.csv` |
| `plot_replay_results.py` | CSV 回放验证图 | `results/replay_validation.csv` |

### 4.4 对比分析（scripts/analysis/）

| 脚本 | 功能 |
|------|------|
| `compare_mpc_lqr.py` | MPC vs LQR 刹车段对比 + 指标表格 |
| `compare_mpc_modes.py` | MPC 两种模式对比 + 指标表格 |

---

## 5. 数据流

### 5.1 LQR 设计流

```
scripts/design/compute_lqr_gain.py
        ↓
include/controller/lqr_gain.hpp
        ↓
CMake 编译 → build/run_closed_loop_lqr
        ↓
results/lqr/closed_loop_*.csv
        ↓
scripts/plot/*.py / scripts/plot/plot_animation.py
```

### 5.2 MPC 仿真流

```
scripts/simulation/run_mpc_simulation.py
        ↓
results/mpc/closed_loop_mpc_*.csv
        ↓
scripts/plot/plot_mpc.py
scripts/analysis/compare_mpc_lqr.py
scripts/analysis/compare_mpc_modes.py
```

### 5.3 一键入口映射

| 入口脚本 | 调用链 |
|---------|--------|
| `./run_lqr.sh` | `scripts/design/compute_lqr_gain.py` → `make` → `results/lqr/` 运行仿真 → `scripts/plot/plot_comparison.py` / `scripts/plot/plot_brake_phase.py` / `scripts/plot/plot_animation.py` |
| `./run_mpc.sh` | `scripts/simulation/run_mpc_simulation.py` (×2) → `scripts/plot/plot_mpc.py` |
| `./run_replay.sh` | `make` → `build/run_csv_replay` → `scripts/plot/plot_replay_results.py` |

---

## 6. 输入 / 输出

### 6.1 输入数据（data/）

| 文件 | 来源 | 用途 |
|------|------|------|
| `crane_imu_obs.csv` | 起重机 IMU 实测 | CSV 回放验证 |
| `crane_imu_obs_debug.csv` | 调试用记录 | CSV 回放验证（默认） |
| `crane_imu_obs_debug_1.csv` | 调试用记录 | CSV 回放验证 |

### 6.2 构建输出（build/）

**仅编译产物，不存放仿真数据。**

| 类型 | 文件 |
|------|------|
| 可执行程序 | `run_open_loop_sim`, `run_csv_replay`, `run_closed_loop_lqr` |
| 静态库 | `libpendulum.a` |
| CMake 产物 | `Makefile`, `CMakeCache.txt`, `CMakeFiles/`, `cmake_install.cmake` |

### 6.3 仿真结果（results/）

| 目录 | 内容 |
|------|------|
| `results/lqr/` | LQR 闭环数据：`closed_loop_*.csv`、对比图、动画 |
| `results/mpc/` | MPC 闭环数据：`closed_loop_mpc_*.csv`、对比图 |
| `results/` (根) | 开环仿真：`simulation_results*.csv`、回放验证：`replay_validation.*` |

---

## 7. 设计文档（docs/）

| 文档 | 内容 |
|------|------|
| `lqr_controller_design.md` | LQR 控制器设计原理、DARE 求解、7 种模式 Q/R 整定、稳定性分析 |
| `CSV_REPLAY_REFERENCE.md` | CSV 回放字段定义、坐标系约定、单位转换规则 |
