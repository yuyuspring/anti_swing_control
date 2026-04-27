#ifndef PEND_OBSERVER_H
#define PEND_OBSERVER_H

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

//   #ifdef __cplusplus
//   extern "C" {
//   #endif

// 常量定义
#define G           9.81f    // 重力加速度 (m/s^2)
#define Ts           0.02f    // 时间间隔 (s)
#define K           (2*M_PI*Ts*0.6f)  // 更新部分比例增益
#define SW          1        // 单摆使能
#define KP_ACC      (2*M_PI*Ts*1.0f)   // 预测部分加速度修正增益
#define R2D         (180.0f/M_PI)     // 弧度转角度

#define KP_ACC_2      (2*M_PI*Ts*0.03f) // 加速度叉乘修正增益的上限
#define KP_ACC_2_MIN      (2*M_PI*Ts*0.005*0.03f) // 加速度叉乘修正增益的下限
#define K_2           (2*M_PI*Ts*0.1f)  // 更新部分比例增益

// ESO 参数定义
#define ESO_OMEGA_O 3.1415926f
#define ESO_K1      (3.0f * ESO_OMEGA_O)
#define ESO_K2      (3.0f * ESO_OMEGA_O * ESO_OMEGA_O)
#define ESO_K3      (ESO_OMEGA_O * ESO_OMEGA_O * ESO_OMEGA_O)

static float debug_data[4] = {0.0f, 0.0f, 0.0f, 0.0f};

static float obs_data[50] = {0.0f};

// 观测器状态数据结构
typedef struct {
    // 角速度（rad/s）
    float w_est[3];    // 观测值 [x y z] rad/s
    float w_meas[3];   // 带噪声测量值 [x y z] rad/s
    // 加速度（m/s^2）
    float a_meas[3];   // 带噪声测量值 [x y z] m/s^2
    // 姿态相关
    float q[4];        // 四元数 [q0, q1, q2, q3]
    float theta[3];    // 观测器姿态角（偏航、俯仰、滚转，rad）

    float Rg[3];       // R(q)*g 重力转换项
    float e1[3];       // 加速度叉乘误差项
    float pend[3];     // 单摆角速度增量项


    // ESO观测器状态
    float v_hat[3];    // 观测速度 (m/s)
    float a_hat[3];    // 观测加速度 (m/s^2)
    float d_hat[3];    // 观测扰动
} PendObserver;

struct FOB_t
{
    float den[2]; ///< denominator gains
    float num[2]; ///< numerator gains
    float input[1]; ///< input history
    float output[1]; ///< output history
};

void init_w_lpf_fob(const float w_meas[3]);

// 初始化函数
void pend_observer_init(const float acc[3], float *w_lpf, const float w_meas[3], PendObserver *obs);

// 观测迭代函数（单次步长更新）
void pend_observer_iterate(PendObserver *obs, float *w_lpf, float *w_meas, float *a_meas,float length_rope);

// 观测迭代函数
// ahrs_input: [yaw, pitch, roll] in degrees
void pend_observer_iterate_2(PendObserver *obs, float *w_lpf, float *w_meas, float *a_meas, float *v_meas,const float *ahrs_input, float length_rope);

// 辅助函数：四元数乘法（q1 ⊗ q2）
void quat_mult(const float *q1, const float *q2, float *q_out);

// 辅助函数：四元数转旋转矩阵
void quat2rotmat(const float *q, float R[3][3]);

// 辅助函数：四元数转欧拉角
void quat2eul(const float q[4], float euler[3]);

// 辅助函数：旋转矩阵转欧拉角（Z-Y-X 顺序）
void rotmat2euler(const float R[3][3], float *euler);

// 辅助函数：向量叉乘
void cross_product(const float *v1, const float *v2, float *out);

void rotmat_transpose(const float src[3][3], float dst[3][3]);

// 辅助函数：欧拉角转四元数 (Z-Y-X 顺序: Input [yaw, pitch, roll])
void eul2quat(const float eul[3], float q[4]);

void InitFOBF(struct FOB_t *filter,float f_cut,float f_s,float value);
float UpdateFOBF(struct FOB_t *filter, float value);

void PushObsvthetaToRingbuffer(float obsv_euler[3]);
void GetDelayedObsvthetaFromRingbuffer(int16_t index_need,float output[3]);

float* GetDebugdata();

float* GetObsDebugdata();

//   #ifdef __cplusplus
//   }
//   #endif

#endif // PEND_OBSERVER_H
/** END OF FILE ******************************************************************************** */