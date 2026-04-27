# LQR 控制器设计文档

> 本文档描述闭环仿真中 LQR 制动控制器的设计原理、参数整定与实现细节。  
> 对应代码：`scripts/compute_lqr_gain.py`、`include/controller/lqr_gain.hpp`、`src/controller/lqr_controller.cpp`

---

## 1. 控制目标

**任务**：在刹车阶段将无人机水平速度降到 0，同时抑制吊重摆动。

- 控制对象：1-D 俯仰平面内的吊重-无人机系统
- 被控量：无人机水平加速度 $a_x$（作为控制输入 $u$）
- 状态反馈来源：IMU 传感器 → 吊重观测器 → LQR 控制器

---

## 2. 状态空间定义

选取线性化状态向量：

$$
x = \begin{bmatrix} v_x \\ \theta \\ \dot{\theta} \end{bmatrix}
$$

| 状态 | 符号 | 物理含义 | 平衡目标 |
|------|------|---------|---------|
| $v_x$ | `vx` | 无人机水平速度 | $0\ \text{m/s}$ |
| $\theta$ | `theta` | 吊重俯仰角（0 = 竖直向下） | $0\ \text{rad}$ |
| $\dot{\theta}$ | `omega` | 吊重角速度 | $0\ \text{rad/s}$ |

控制输入：

$$
u = a_x \quad [\text{m/s}^2]
$$

---

## 3. 连续时间状态空间模型

对**加速支点单摆**做小角度线性化：

- 无人机被强制驱动，加速度为 $a_x$
- 在随无人机加速的非惯性参考系中，吊重受到向后的惯性力 $-m a_x$
- 单摆线性化方程：$\ddot{\theta} = -\dfrac{g}{L}\theta - \dfrac{1}{L}a_x$

得到连续时间状态空间：

$$
\dot{x} = A_c x + B_c u
$$

其中：

$$
A_c = \begin{bmatrix}
0 & 0 & 0 \\
0 & 0 & 1 \\
0 & -g/L & 0
\end{bmatrix},
\quad
B_c = \begin{bmatrix}
1 \\
0 \\
-1/L
\end{bmatrix}
$$

### 物理意义拆解

| 行 | 方程 | 说明 |
|----|------|------|
| 第 1 行 | $\dot{v}_x = a_x$ | 无人机速度直接由控制输入决定 |
| 第 2 行 | $\dot{\theta} = \dot{\theta}$ | 角速度定义 |
| 第 3 行 | $\ddot{\theta} = -\dfrac{g}{L}\theta - \dfrac{1}{L}a_x$ | 重力恢复力矩 + 惯性力矩 |

> 代码参考：`scripts/compute_lqr_gain.py` 第 18–44 行

---

## 4. 离散化（ZOH）

控制器以固定周期 $T_s = 0.02\ \text{s}$（50 Hz）运行，需将连续模型离散化。

采用**零阶保持（Zero-Order Hold, ZOH）**方法：

$$
A_d = e^{A_c T_s}, \quad
B_d = \int_0^{T_s} e^{A_c \tau} B_c \, d\tau
$$

实现上通过矩阵指数一次性计算：

```python
M = [[A_c, B_c],
     [  0,   0]]
M_exp = expm(M * Ts)
Ad = M_exp[:3, :3]
Bd = M_exp[:3, 3:]
```

> 代码参考：`scripts/compute_lqr_gain.py` 第 47–58 行

---

## 5. 离散代数 Riccati 方程（DARE）

离散 LQR 性能指标：

$$
J = \sum_{k=0}^{\infty} \left( x_k^{\top} Q x_k + u_k^{\top} R u_k \right)
$$

求解离散代数 Riccati 方程得代价矩阵 $P$：

$$
P = A_d^{\top} P A_d - A_d^{\top} P B_d (R + B_d^{\top} P B_d)^{-1} B_d^{\top} P A_d + Q
$$

最优状态反馈增益：

$$
K = (R + B_d^{\top} P B_d)^{-1} B_d^{\top} P A_d
$$

控制律：

$$
u_k = -K x_k
$$

> 代码参考：`scripts/compute_lqr_gain.py` 第 61–66 行，使用 `scipy.linalg.solve_discrete_are`

---

## 6. 三种控制模式的 Q/R 整定

通过调整权重矩阵 $Q = \text{diag}(Q_{v_x}, Q_{\theta}, Q_{\dot{\theta}})$ 和 $R$，得到三种不同控制性格的 LQR。

### 6.1 权重配置

| 模式 | $Q_{v_x}$ | $Q_{\theta}$ | $Q_{\dot{\theta}}$ | $R$ | 设计意图 |
|------|----------|-------------|-------------------|-----|---------|
| **Full** | 1 | 20 | 5 | 2 | **均衡**：速度与摆动兼顾 |
| **Shortest** | 20 | $10^{-6}$ | $10^{-6}$ | 0.5 | **最短距离**：只关心速度收敛 |
| **MinSwing** | 1 | 100 | 50 | 5 | **最小摆动**：大幅惩罚摆角与角速度 |

### 6.2 增益结果

| 模式 | $K_{v_x}$ | $K_{\theta}$ | $K_{\dot{\theta}}$ | 闭环最大特征值 |
|------|----------|-------------|-------------------|--------------|
| **Full** | `0.700706` | `-2.327973` | `-3.030386` | $< 1$（稳定） |
| **Shortest** | `5.937191` | `-0.001692` | `-0.000285` | $< 1$（稳定） |
| **MinSwing** | `0.443665` | `-2.869639` | `-5.229539` | $< 1$（稳定） |

### 6.3 物理直觉

#### Shortest 模式
- $K_{v_x} \approx 5.94$ 很大，$K_{\theta} \approx K_{\dot{\theta}} \approx 0$
- 速度还有 $10\ \text{m/s}$ 时，理论输出 $u \approx -59.4\ \text{m/s}^2$
- **直接饱和到 $-3\ \text{m/s}^2$，全力减速，几乎不抑制摆动**

#### MinSwing 模式
- $K_{\theta} \approx -2.87$、$K_{\dot{\theta}} \approx -5.23$ 很大
- 会主动用加速度抵消摆动（例如摆角为正时施加负加速度拉回来）
- $K_{v_x} = 0.44$ 较小，减速更温柔

#### Full 模式
- 三者均显著参与，速度与摆动权重折中

> 代码参考：`scripts/compute_lqr_gain.py` 第 78–85 行；生成的 `include/controller/lqr_gain.hpp`

---

## 7. 在线控制律（C++ 实现）

### 7.1 增益查表

三种增益在编译期硬编码于 `lqr_gain.hpp`：

```cpp
struct LqrGain {
    // Full: Q=[1,20,5], R=2
    static constexpr double kFullV     = 0.70070557;
    static constexpr double kFullTheta = -2.32797311;
    static constexpr double kFullOmega = -3.03038608;

    // Shortest: Q=[20,0,0], R=0.5
    static constexpr double kShortestV     = 5.93719068;
    static constexpr double kShortestTheta = -0.00169174;
    static constexpr double kShortestOmega = -0.00028525;

    // MinSwing: Q=[1,100,50], R=5
    static constexpr double kMinSwingV     = 0.44366486;
    static constexpr double kMinSwingTheta = -2.86963865;
    static constexpr double kMinSwingOmega = -5.22953917;
};
```

### 7.2 控制量计算

```cpp
double LqrController::computeControl(const State& state) const {
    double u = -(kV_ * state.vx +
                 kTheta_ * state.theta +
                 kOmega_ * state.omega);
    return saturate(u, axLimit_);  // 限制在 [-3, +3] m/s²
}
```

**关键细节**：
- `state.vx`、`state.theta`、`state.omega` 来自**观测器输出**（含噪声 IMU 经卡尔曼滤波后的估计值），而非仿真真值
- 控制更新频率 50 Hz，与动力学积分 1000 Hz 解耦
- 输出的 $a_x$ 在 `SlungLoadDynamics::step()` 中进一步经过 jerk limit（$2\ \text{m/s}^3$）平滑

> 代码参考：`src/controller/lqr_controller.cpp` 第 27–31 行

---

## 8. 设计流程图

```
┌─────────────────┐     ┌──────────────┐     ┌─────────────┐
│  连续模型 Ac,Bc  │ ──→ │ ZOH 离散化    │ ──→ │ 求解 DARE   │
│  (小角度线性化)  │     │ Ts = 0.02 s  │     │ 得 K        │
└─────────────────┘     └──────────────┘     └─────────────┘
                                                    │
                       ┌────────────────────────────┘
                       ▼
              ┌─────────────────┐
              │  lqr_gain.hpp   │  ← 硬编码三种 K（编译期嵌入）
              └─────────────────┘
                       │
         ┌─────────────┼─────────────┐
         ▼             ▼             ▼
    ┌─────────┐  ┌─────────┐  ┌─────────┐
    │  Full   │  │ Shortest│  │ MinSwing│
    │ K=[...] │  │ K=[...] │  │ K=[...] │
    └─────────┘  └─────────┘  └─────────┘
```

---

## 9. 当前设计的局限性

| 局限 | 说明 | 可能的改进 |
|------|------|-----------|
| **线性化假设** | 增益基于 $\theta \approx 0$ 设计，大摆角（> 30°）时模型失准 | 可考虑 LPV 或非线性 MPC |
| **固定绳长** | 增益与 $L$ 绑定，换绳长需重新运行 Python 脚本 | 可改为在线增益调度（gain scheduling） |
| **无积分项** | 纯状态反馈，对稳态误差（如传感器 bias、风扰）无消除能力 | 加入积分状态 $\int v_x \, dt$ |
| **无位置控制** | 3 状态 LQR 只把速度控到 0，不控制最终停靠位置 | 扩展为 4 状态 $[p, v, \theta, \dot{\theta}]^{\top}$ |
| **Q/R 凭经验** | 权重靠手动试凑，未做系统性优化 | 可用遗传算法或基于仿真数据的自动调参 |

---

## 10. 相关文件索引

| 文件 | 作用 |
|------|------|
| `scripts/compute_lqr_gain.py` | Python 离线设计脚本：建模型 → 离散化 → 解 DARE → 生成 C++ 头文件 |
| `include/controller/lqr_gain.hpp` | 自动生成的增益头文件（硬编码三种模式的 K） |
| `include/controller/lqr_controller.hpp` | C++ LQR 控制器类接口定义 |
| `src/controller/lqr_controller.cpp` | 在线控制律实现（查表 + 状态反馈 + 饱和） |
