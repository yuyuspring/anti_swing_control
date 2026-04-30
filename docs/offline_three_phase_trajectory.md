# 三段式离线轨迹规划（梯形加速度）

## 1. 轨迹定义

刹车阶段采用**梯形加速度**曲线，分为三段：

| 段 | 时间区间 | 加速度 | 说明 |
|---|---------|--------|------|
| 第1段 | $[0, t_{\text{jerk}}]$ | $a_x(t) = -a_{\max} \cdot \frac{t}{t_{\text{jerk}}}$ | 最大 jerk ramp |
| 第2段 | $[t_{\text{jerk}}, t_1]$ | $a_x(t) = -a_{\max}$ | 恒定减速至 $\theta_{\max}$ |
| 第3段 | $[t_1, t_1+t_2]$ | $a_x(t) = -a_{\max} \cdot (1 - \frac{t-t_1}{t_2})$ | 线性释放至 0 |

其中 $t_1$ 由**事件触发**（摆角达到最大值），$t_2$ 由**速度约束**（vx 归 0）确定。

---

## 2. 物理参数

| 符号 | 数值 | 含义 |
|------|------|------|
| $g$ | 9.81 m/s² | 重力加速度 |
| $L$ | 15.0 m | 绳长 |
| $a_{\max}$ | 2.0 m/s² | 最大加速度 |
| $t_{\text{jerk}}$ | 1.0 s | jerk ramp 时间 |
| $\omega_n = \sqrt{g/L}$ | 0.8087 rad/s | 单摆固有频率 |
| $\alpha = a_{\max}/g$ | 0.2039 rad | 归一化加速度 |

---

## 3. $t_1$ 的解析计算（$\theta_{\max}$ 时刻）

### 3.1 小角度近似

单摆运动方程（小角度 $\sin\theta \approx \theta$）：

$$
\ddot{\theta} + \omega_n^2 \theta = \frac{a_x(t)}{L} = \frac{a_{\max}}{g} \cdot \omega_n^2 \cdot f(t)
$$

其中 $f(t)$ 为无量纲加速度形状函数。

### 3.2 第1段解析解

第1段：$a_x(t) = -a_{\max} \cdot t/t_{\text{jerk}}$，即 $f(t) = -t/t_{\text{jerk}}$

方程：

$$
\ddot{\theta} + \omega_n^2 \theta = -\alpha \omega_n^2 \cdot \frac{t}{t_{\text{jerk}}}
$$

初始条件 $\theta(0)=0$, $\dot{\theta}(0)=0$，解为：

$$
\theta(t) = \alpha \left[ \frac{t}{t_{\text{jerk}}} - \frac{\sin(\omega_n t)}{\omega_n t_{\text{jerk}}} \right]
$$

$$
\dot{\theta}(t) = \frac{\alpha}{t_{\text{jerk}}} \left[ 1 - \cos(\omega_n t) \right]
$$

在 $t = t_{\text{jerk}}$ 时：

$$
\theta_j = \alpha \left( 1 - \frac{\sin(\omega_n t_{\text{jerk}})}{\omega_n t_{\text{jerk}}} \right) = 0.0215 \text{ rad} = 1.23°
$$

$$
\dot{\theta}_j = \frac{\alpha}{t_{\text{jerk}}} \left( 1 - \cos(\omega_n t_{\text{jerk}}) \right) = 0.0631 \text{ rad/s} = 3.62°/\text{s}
$$

### 3.3 第2段解析解

第2段：$a_x(t) = -a_{\max}$，即 $f(t) = -1$

方程：

$$
\ddot{\theta} + \omega_n^2 \theta = \alpha \omega_n^2
$$

齐次解 + 特解，令 $\Delta t = t - t_{\text{jerk}}$：

$$
\theta(\Delta t) = C \cos(\omega_n \Delta t) + D \sin(\omega_n \Delta t) + \alpha
$$

由初始条件 $\theta(0) = \theta_j$, $\dot{\theta}(0) = \dot{\theta}_j$：

$$
C = \theta_j - \alpha = -0.1824 \text{ rad}
$$

$$
D = \frac{\dot{\theta}_j}{\omega_n} = 0.0780 \text{ rad}
$$

### 3.4 $\theta_{\max}$ 条件

$\theta$ 达到最大值时 $\dot{\theta} = 0$：

$$
\dot{\theta}(\Delta t) = -\omega_n C \sin(\omega_n \Delta t) + \omega_n D \cos(\omega_n \Delta t) = 0
$$

$$
\tan(\omega_n \Delta t) = \frac{D}{C} = -0.4279
$$

由于 $C < 0$, $D > 0$，解在**第二象限**：

$$
\omega_n \Delta t = \arctan2(D, C) = 2.737 \text{ rad} = 156.83°
$$

$$
\Delta t = \frac{2.737}{0.8087} = 3.385 \text{ s}
$$

**结果**：

$$
\boxed{t_1 = t_{\text{jerk}} + \Delta t = 1.0 + 3.385 = 4.385 \text{ s}}
$$

$$
\theta_{\max} = C \cos(2.737) + D \sin(2.737) + \alpha = 0.402 \text{ rad} = 23.05°
$$

### 3.5 数值验证

| | 解析解 | 非线性数值仿真 | 误差 |
|--|--------|--------------|------|
| $t_1$ | 4.385 s | 4.357 s | 0.028 s (0.6%) |
| $\theta_{\max}$ | 23.05° | 22.73° | 0.32° (1.4%) |

---

## 4. $t_2$ 的解析计算（vx = 0）

### 4.1 速度约束

无人机速度变化等于加速度曲线下的面积：

$$
\Delta v = \int_0^{t_1+t_2} a_x(t) \, dt = -v_0
$$

其中 $v_0 = 15$ m/s 为初始巡航速度。

### 4.2 分段积分

| 段 | 面积贡献 |
|---|---------|
| 第1段（三角形） | $-\frac{1}{2} a_{\max} t_{\text{jerk}}$ |
| 第2段（矩形） | $-a_{\max} (t_1 - t_{\text{jerk}})$ |
| 第3段（三角形） | $-\frac{1}{2} a_{\max} t_2$ |

总速度变化：

$$
\Delta v = -\frac{1}{2} a_{\max} t_{\text{jerk}} - a_{\max}(t_1 - t_{\text{jerk}}) - \frac{1}{2} a_{\max} t_2 = -v_0
$$

解出 $t_2$：

$$
\boxed{t_2 = \frac{v_0 - \frac{1}{2} a_{\max} t_{\text{jerk}} - a_{\max}(t_1 - t_{\text{jerk}})}{\frac{1}{2} a_{\max}}}
$$

代入数值：

$$
t_2 = \frac{15 - 1.0 - 2.0 \times 3.385}{1.0} = \frac{15 - 1.0 - 6.77}{1.0} = 7.23 \text{ s}
$$

### 4.3 关键观察

$t_2 = 7.23$ s 时，vx 确实归 0，但此时 $\theta \neq 0$。这是因为第3段中 $\theta$ 的恢复运动和 vx 的减速运动通过同一个 $a_x(t)$ 耦合，但两者需要的"时间节奏"不同步。

| $t_2$ 取值 | $\theta(t_1+t_2)$ | $\omega(t_1+t_2)$ | $v_x(t_1+t_2)$ |
|-----------|-------------------|-------------------|----------------|
| 5.55 s（$\theta=0$） | $\approx 0°$ | 7.5°/s | 1.15 m/s |
| **7.23 s（$v_x=0$）** | **9.53°** | **3.35°/s** | **$\approx$ 0** |
| 9.55 s（$\theta=0$ 二次） | $\approx 0°$ | -10.3°/s | -2.64 m/s |

**结论**：在梯形加速度约束下，$\theta=0$ 和 $v_x=0$ **无法同时满足**。

---

## 5. 完整仿真结果

采用与 LQR/MPC 一致的仿真框架（加速→巡航→刹车，全程考虑 jerk 限制）：

| 指标 | 数值 |
|------|------|
| 刹车开始时间 | 40.0 s |
| 总刹车时间 | 11.62 s |
| 制动距离 | 67.3 m |
| 稳定时间 ($\|v_x\| < 0.1$) | 10.72 s |
| Max $\|\theta\|$ | 18.7°（刹车段）/ 24.0°（全程）|
| Max $\|\dot{\theta}\|$ | 8.21°/s（刹车段）/ 11.8°/s（全程）|
| Final $\theta$ | 1.87° |
| Final $\omega$ | -0.16°/s |

---

## 6. 代码实现

```python
import numpy as np

# 物理参数
g = 9.81
L = 15.0
a_max = 2.0
t_jerk = 1.0
omega_n = np.sqrt(g / L)
alpha = a_max / g

# ---- t1: theta_max ----
theta_j = alpha * (1 - np.sin(omega_n * t_jerk) / (omega_n * t_jerk))
omega_j = (alpha / t_jerk) * (1 - np.cos(omega_n * t_jerk))
C = theta_j - alpha
D = omega_j / omega_n
omega_dt = np.arctan2(D, C)
if omega_dt < np.pi / 2:
    omega_dt += np.pi
t1 = t_jerk + omega_dt / omega_n

# ---- t2: vx = 0 ----
v0 = 15.0
t2 = (v0 - 0.5 * a_max * t_jerk - a_max * (t1 - t_jerk)) / (0.5 * a_max)
```

---

## 7. 附录：$\theta(t_1+t_2)=0$ 的解析方程

若强行要求 $\theta(t_1+t_2)=0$，可推导 $t_2$ 需满足的超越方程：

$$
(\theta_{\max} - \alpha) \cos(\omega_n t_2) + \frac{\alpha}{\omega_n t_2} \sin(\omega_n t_2) = 0
$$

该方程的解为 $t_2 \approx 5.55$ s, 9.55 s, 13.48 s ... 均不满足 $v_x=0$。
