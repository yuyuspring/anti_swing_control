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

> 代码参考：`scripts/compute_lqr_gain.py` 第 23–64 行

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

> 代码参考：`scripts/compute_lqr_gain.py` 第 67–104 行

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

> 代码参考：`scripts/compute_lqr_gain.py` 第 107–135 行，使用 `scipy.linalg.solve_discrete_are`

---

## 6. 五种控制模式的 Q/R 整定

通过调整权重矩阵 $Q$ 和 $R$，得到五种不同控制性格的 LQR。

### 6.1 权重配置

| 模式 | $Q$ 形式 | $R$ | 设计意图 |
|------|---------|-----|---------|
| **Full** | $\text{diag}(2, 30, 10)$ | 2 | **均衡**：速度与摆动兼顾 |
| **Shortest** | $\text{diag}(8, 2, 1)$ | 3 | **最短距离**：优先速度收敛 |
| **MinSwing** | $\text{diag}(1, 100, 50)$ | 2 | **最小摆动**：大幅惩罚摆角与角速度 |
| **VelocityOmega** | $\text{diag}(10, 1, 100)$ | 2 | **速度+角速度**：同时抑制速度和平抑角速度 |
| **PayloadVelocity** | 非对角矩阵（见 6.4） | 2 | **Payload 绝对速度**：直接惩罚吊重水平绝对速度 |

### 6.2 增益结果

| 模式 | $K_{v_x}$ | $K_{\theta}$ | $K_{\dot{\theta}}$ | 闭环最大特征值 | 状态 |
|------|----------|-------------|-------------------|--------------|------|
| **Full** | `0.987930` | `-3.330799` | `-3.209652` | 0.9978 | ✓ 稳定 |
| **Shortest** | `1.605990` | `-0.825129` | `-0.517009` | 0.9997 | ⚠ 收敛极慢 |
| **MinSwing** | `0.698830` | `-5.699921` | `-7.029508` | 0.9948 | ✓ 稳定 |
| **VelocityOmega** | `2.182318` | `-5.096300` | `-2.939453` | 0.9984 | ✓ 稳定 |
| **PayloadVelocity** | `1.392986` | `-16.870620` | `-1.619842` | 0.9925 | ✓ 稳定 |

### 6.3 物理直觉

#### Full 模式
- 三者均显著参与，速度与摆动权重折中
- 适合一般工况，无明显偏向

#### Shortest 模式
- $K_{v_x} \approx 1.61$ 较大，$K_{\theta} \approx -0.83$、$K_{\dot{\theta}} \approx -0.52$ 较小
- 优先快速减速，对摆动容忍度较高
- 闭环极点接近单位圆（`max|eig| = 0.9997`），收敛较慢

#### MinSwing 模式
- $K_{\theta} \approx -5.70$、$K_{\dot{\theta}} \approx -7.03$ 很大
- 主动用加速度抵消摆动（例如摆角为正时施加负加速度拉回来）
- $K_{v_x} = 0.70$ 较小，减速更温柔

#### VelocityOmega 模式
- $Q_{\dot{\theta}} = 100$ 对角速度施加强惩罚
- $K_{v_x} = 2.18$ 较大，减速积极；$K_{\dot{\theta}} = -2.94$ 抑制摆速
- 刹车距离最短（46.25 m），但摆角抑制不如 MinSwing

#### PayloadVelocity 模式
- **唯一使用非对角 $Q$ 矩阵的模式**
- 惩罚项为 $q_{\text{pay}} \cdot (v_x + L\dot{\theta})^2$，即吊重的水平绝对速度
- $K_{\theta} = -16.87$ 很大，对摆角极其敏感
- 刹车距离 49.70 m，摆角抑制介于 Full 和 MinSwing 之间

### 6.4 PayloadVelocity 的非对角 Q 矩阵

PayloadVelocity 模式不采用对角 $Q$，而是通过交叉项直接惩罚 $v_x + L\dot{\theta}$：

$$
Q = \begin{bmatrix}
q_{\text{pay}} & 0 & q_{\text{pay}} \cdot L \\
0 & q_{\theta} & 0 \\
q_{\text{pay}} \cdot L & 0 & q_{\text{pay}} \cdot L^2 + q_{\omega}
\end{bmatrix}
$$

当前参数（$L = 15\ \text{m}$）：

```python
q_pay = 4.0
q_theta = 1.0
q_omega_extra = 1.0
Q = [[4,    0,   60],
     [0,    1,    0],
     [60,   0,  901]]
```

> ⚠ **关键约束**：增大 $q_{\text{pay}}$ 时需警惕 $K_{\dot{\theta}}$ 变正。当 $q_{\text{pay}} \gtrsim 5.5$ 时，$K_{\dot{\theta}}$ 从负变正，会在大角度下引入正反馈，导致非线性发散。

> 代码参考：`scripts/compute_lqr_gain.py` 第 176–200 行；生成的 `include/controller/lqr_gain.hpp`

---

## 7. 工程稳定性检查

脚本在输出增益后执行两层稳定性评估：

### 7.1 线性稳定性（特征值判据）

计算闭环矩阵 $A_{cl} = A_d - B_d K$ 的特征值，检查是否全部位于单位圆内：

```python
max_eig = max(abs(eigvals(Acl)))
if max_eig < 1.0:
    # 线性渐近稳定
```

### 7.2 工程稳定性（K_omega 符号检查）

线性稳定 ≠ 实际仿真稳定。对于大角度非线性系统，**$K_{\dot{\theta}}$ 的符号至关重要**：

| $K_{\dot{\theta}}$ 符号 | 物理效应 | 大角度稳定性 |
|------------------------|---------|------------|
| **负** ($K_{\dot{\theta}} < 0$) | 负反馈：$\dot{\theta} > 0$ 时施加正加速度，抑制摆动 | ✅ 稳定 |
| **正** ($K_{\dot{\theta}} > 0$) | 正反馈：$\dot{\theta} > 0$ 时施加负加速度，加剧摆动 | ⚠️ 可能发散 |

当 $K_{\dot{\theta}} > 0$ 时，脚本输出警告：

```
⚠ K_omega>0，大角度可能发散
```

### 7.3 收敛速度检查

当 `max|eig| > 0.999` 时，标注：

```
⚠ 收敛极慢
```

> 代码参考：`scripts/compute_lqr_gain.py` 第 208–227 行

---

## 8. 在线控制律（C++ 实现）

### 8.1 增益查表

五种增益在编译期硬编码于 `lqr_gain.hpp`：

```cpp
struct LqrGain {
    // Full: Q=[2,30,10], R=2
    static constexpr double kFullV     = 0.98793039;
    static constexpr double kFullTheta = -3.33079858;
    static constexpr double kFullOmega = -3.20965162;

    // Shortest: Q=[8,2,1], R=3
    static constexpr double kShortestV     = 1.60599035;
    static constexpr double kShortestTheta = -0.82512925;
    static constexpr double kShortestOmega = -0.51700885;

    // MinSwing: Q=[1,100,50], R=2
    static constexpr double kMinSwingV     = 0.69883014;
    static constexpr double kMinSwingTheta = -5.69992072;
    static constexpr double kMinSwingOmega = -7.02950842;

    // VelocityOmega: Q=[10,1,100], R=2
    static constexpr double kVelocityOmegaV     = 2.18231813;
    static constexpr double kVelocityOmegaTheta = -5.09629988;
    static constexpr double kVelocityOmegaOmega = -2.93945274;

    // PayloadVelocity: Q penalizes (vx + L*omega)^2, R=2
    static constexpr double kPayloadVelocityV     = 1.39298637;
    static constexpr double kPayloadVelocityTheta = -16.87062001;
    static constexpr double kPayloadVelocityOmega = -1.61984222;
};
```

### 8.2 控制量计算

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

> 代码参考：`src/controller/lqr_controller.cpp`

---

## 9. 设计流程图

```
┌─────────────────┐     ┌──────────────┐     ┌─────────────┐
│  连续模型 Ac,Bc  │ ──→ │ ZOH 离散化    │ ──→ │ 求解 DARE   │
│  (小角度线性化)  │     │ Ts = 0.02 s  │     │ 得 K        │
└─────────────────┘     └──────────────┘     └─────────────┘
                                                    │
                       ┌────────────────────────────┘
                       ▼
              ┌─────────────────┐
              │  lqr_gain.hpp   │  ← 硬编码五种 K（编译期嵌入）
              └─────────────────┘
                       │
         ┌─────────────┼─────────────┬─────────────┬─────────────┐
         ▼             ▼             ▼             ▼             ▼
    ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐
    │  Full   │  │ Shortest│  │ MinSwing│  │VelOmega │  │ PayLoad │
    │ K=[...] │  │ K=[...] │  │ K=[...] │  │ K=[...] │  │ K=[...] │
    └─────────┘  └─────────┘  └─────────┘  └─────────┘  └─────────┘
```

---

## 10. 当前设计的局限性

| 局限 | 说明 | 可能的改进 |
|------|------|-----------|
| **线性化假设** | 增益基于 $\theta \approx 0$ 设计，大摆角（> 30°）时模型失准 | 可考虑 LPV 或非线性 MPC |
| **固定绳长** | 增益与 $L$ 绑定，换绳长需重新运行 Python 脚本 | 可改为在线增益调度（gain scheduling） |
| **无积分项** | 纯状态反馈，对稳态误差（如传感器 bias、风扰）无消除能力 | 加入积分状态 $\int v_x \, dt$ |
| **无位置控制** | 3 状态 LQR 只把速度控到 0，不控制最终停靠位置 | 扩展为 4 状态 $[p, v, \theta, \dot{\theta}]^{\top}$ |
| **Q/R 凭经验** | 权重靠手动试凑，未做系统性优化 | 可用遗传算法或基于仿真数据的自动调参 |
| **非对角 Q 的物理风险** | PayloadVelocity 使用非对角 $Q$，参数空间存在 $K_{\dot{\theta}} > 0$ 的危险区域 | 脚本已增加符号检查，可进一步做非线性仿真预验证 |

---

## 11. 相关文件索引

| 文件 | 作用 |
|------|------|
| `scripts/compute_lqr_gain.py` | Python 离线设计脚本：建模型 → 离散化 → 解 DARE → 工程稳定性检查 → 生成 C++ 头文件 |
| `include/controller/lqr_gain.hpp` | 自动生成的增益头文件（硬编码五种模式的 K） |
| `include/controller/lqr_controller.hpp` | C++ LQR 控制器类接口定义 |
| `src/controller/lqr_controller.cpp` | 在线控制律实现（查表 + 状态反馈 + 饱和） |
