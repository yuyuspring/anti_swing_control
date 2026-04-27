#include "observer/pend_observer.h"

typedef struct
{
    float x;
    float y;
    float z;
} vector3_test;


typedef struct
{
    vector3_test a;
    vector3_test b;
    vector3_test c;
} matrix3_test;

void get_acc_tbfn(const float* euler_rad, const float* acc_n, float *acc_b);

static struct FOB_t w_lpf_fob[3];
static float ring_buffer_yaw[10] = {0};
static float ring_buffer_pitch[10] = {0};
static float ring_buffer_roll[10] = {0};

static int16_t index_obsv_theta = 0;
const int16_t size_ring_buffer = 10;

void init_w_lpf_fob(const float w_meas[3])
{
    for(int i=0;i<3;i++)
    {
        InitFOBF(&w_lpf_fob[i],0.3f,50.0f,w_meas[i]/R2D);
    }
}

// 初始化观测器状态
void pend_observer_init(const float acc[3], float *w_lpf, const float w_meas[3], PendObserver *obs)
{    
    obs->w_est[0] = w_meas[0]/R2D;
    obs->w_est[1] = w_meas[1]/R2D;
    obs->w_est[2] = w_meas[2]/R2D;

    obs->w_meas[0] = w_meas[0]/R2D;
    obs->w_meas[1] = w_meas[1]/R2D;
    obs->w_meas[2] = w_meas[2]/R2D;

    w_lpf[0] = UpdateFOBF(&w_lpf_fob[0],obs->w_meas[0]);
    w_lpf[1] = UpdateFOBF(&w_lpf_fob[1],obs->w_meas[1]);
    w_lpf[2] = UpdateFOBF(&w_lpf_fob[2],obs->w_meas[2]);

    // 计算重力模长（理论值≈9.81m/s²，用于验证数据有效性）
    float gravity_sq = acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2];
    float gravity = sqrtf(gravity_sq);
    // 异常值判断（模长过小时直接返回错误）
    if (gravity < 1.0f || gravity > 20.0f) 
    {
        // 正常重力范围1~20m/s²
        return;
    }
    // 计算俯仰角 (Pitch)：绕Y轴旋转
    // Pitch = arctan(-ax / sqrt(ay² + az²))
    float pitch_den = sqrtf(acc[1]*acc[1] + acc[2]*acc[2]);
    if (pitch_den <= 1e-6f) 
    { 
        // 避免除零         
        obs->theta[1] = 0.0f;
    } 
    else 
    {         
        obs->theta[1] = atan2f(-acc[0], pitch_den);
    }
    // 计算滚转角 (Roll)：绕X轴旋转
    // Roll = arctan(ay / az)
    if (fabsf(acc[2]) <= 1e-6f) 
    { 
        // 避免除零         
        obs->theta[2] = (acc[1] > 0) ? M_PI_2 : -M_PI_2;
    } 
    else 
    {         
        obs->theta[2] = atan2f(acc[1], acc[2]);
    }
    // 偏航角无法通过加速度计解算，保持0     
    obs->theta[0] = 0.0f;

    eul2quat(obs->theta, obs->q);

    for (int i = 0; i < 3; i++) {
        obs->v_hat[i] = 0.0f;
        obs->a_hat[i] = 0.0f;
        obs->d_hat[i] = 0.0f;
    }

    PushObsvthetaToRingbuffer(obs->theta);
}

// 观测器单次迭代（核心逻辑）
void pend_observer_iterate(PendObserver *obs, float *w_lpf, float *w_meas, float *a_meas,float length_rope) {
    memcpy(obs->a_meas, a_meas, 3*sizeof(float));

    obs->w_meas[0] = w_meas[0]/R2D;
    obs->w_meas[1] = w_meas[1]/R2D;
    obs->w_meas[2] = w_meas[2]/R2D;

    w_lpf[0] = UpdateFOBF(&w_lpf_fob[0],obs->w_meas[0]);
    w_lpf[1] = UpdateFOBF(&w_lpf_fob[1],obs->w_meas[1]);
    w_lpf[2] = UpdateFOBF(&w_lpf_fob[2],obs->w_meas[2]);

    // ---------------------- 预测部分 ----------------------
    // obs->pend 滚转 俯仰 偏航
    // obs->theta 偏航 俯仰 滚转
    // 1. 角速度预测：w(k) = w(k-1) - (g/l)*Ts*sin(theta(k))
    obs->pend[0] = -(G/length_rope)*Ts*sin(obs->theta[0]); // 偏航
    obs->pend[1] = -(G/length_rope)*Ts*sin(obs->theta[1]); // 俯仰
    obs->pend[2] = -(G/length_rope)*Ts*sin(obs->theta[2]); // 滚转

    obs->w_est[0] += obs->pend[2] * SW; // 滚转
    obs->w_est[1] += obs->pend[1] * SW; // 俯仰
    obs->w_est[2] += obs->pend[0] * SW; // 偏航
    
    // 2. 四元数更新：dq = 0.5*q ⊗ [0; w_est*Ts]
    float w_half_dt[4] = {0.0f, 
                          0.5f*obs->w_est[0]*Ts, 
                          0.5f*obs->w_est[1]*Ts, 
                          0.5f*obs->w_est[2]*Ts};
    float dq[4];
    quat_mult(obs->q, w_half_dt, dq);
    
    // 四元数积分 + 归一化
    float q_temp[4];
    for (int i = 0; i < 4; i++) {
        q_temp[i] = obs->q[i] + dq[i];
    }
    float q_norm = sqrt(q_temp[0]*q_temp[0] + q_temp[1]*q_temp[1] + 
                        q_temp[2]*q_temp[2] + q_temp[3]*q_temp[3]);
    for (int i = 0; i < 4; i++) {
        obs->q[i] = q_temp[i] / q_norm;
    }    
    
    // 3. 加速度修正：e1 = (R(q)*g) × a / g²
    float R_nb[3][3];
    quat2rotmat(obs->q, R_nb);

    float R_bn[3][3];
    rotmat_transpose(R_nb, R_bn);

    float g_n[3] = {0.0f, 0.0f, G};
    // 计算 R(q)*g
    for (int i = 0; i < 3; i++) {
        obs->Rg[i] = R_bn[i][0]*g_n[0] + R_bn[i][1]*g_n[1] + R_bn[i][2]*g_n[2];
    }
    // 叉乘 + 增益修正
    cross_product(obs->Rg, obs->a_meas, obs->e1);
    obs->e1[0] = -obs->e1[0];
    obs->e1[1] = -obs->e1[1];
    obs->e1[2] = -obs->e1[2];
    for (int i = 0; i < 3; i++) {
        obs->e1[i] /= (G*G);
        obs->w_est[i] += KP_ACC * obs->e1[i];
    }
    
    // 4. 姿态角更新（旋转矩阵转欧拉角）
    rotmat2euler(R_nb, obs->theta);
    
    // ---------------------- 更新部分 ----------------------
    // 1. 角速度误差：e2 = w_meas - w_est
    // 2. 角速度更新：w_est += k*e2
    for (int i = 0; i < 3; i++) {
        float e2 = obs->w_meas[i] - obs->w_est[i];
        obs->w_est[i] += K * e2;
    }
}

void pend_observer_iterate_2(PendObserver *obs, float *w_lpf, float *w_meas, float *a_meas, float *v_meas,const float *ahrs_input, float length_rope) 
{
    memcpy(obs->a_meas, a_meas, 3*sizeof(float));

    obs->w_meas[0] = w_meas[0]/R2D;
    obs->w_meas[1] = w_meas[1]/R2D;
    obs->w_meas[2] = w_meas[2]/R2D;

    obs_data[0] = obs->w_meas[0];
    obs_data[1] = obs->w_meas[1];
    obs_data[2] = obs->w_meas[2];

    w_lpf[0] = UpdateFOBF(&w_lpf_fob[0],obs->w_meas[0]);
    w_lpf[1] = UpdateFOBF(&w_lpf_fob[1],obs->w_meas[1]);
    w_lpf[2] = UpdateFOBF(&w_lpf_fob[2],obs->w_meas[2]);

    float ahrs_fus[3] = {0.0f, 0.0f, 0.0f};
    ahrs_fus[0] = ahrs_input[0] / R2D; // 偏航角测量值（rad）
    ahrs_fus[1] = ahrs_input[1] / R2D; // 俯仰角测量值（rad）
    ahrs_fus[2] = ahrs_input[2] / R2D; // 滚转角测量值（rad）

    obs_data[3] = ahrs_fus[0];

    // ---------------------- ESO观测器更新 ----------------------
    for(int i = 0; i < 3; i++) {
        float e_v = v_meas[i] - obs->v_hat[i];
        obs->d_hat[i] += ESO_K3 * Ts * e_v;
        obs->a_hat[i] += (ESO_K2 * e_v + obs->d_hat[i]) * Ts;
        obs->v_hat[i] += (ESO_K1 * e_v + obs->a_hat[i]) * Ts;
    }

    obs_data[4] = obs->a_hat[0];
    obs_data[5] = obs->a_hat[1];
    obs_data[6] = obs->a_hat[2];

    obs_data[7] = obs->v_hat[0];
    obs_data[8] = obs->v_hat[1];
    obs_data[9] = obs->v_hat[2];

    // ---------------------- 预测部分 ----------------------
    float cos_hdg = cosf(ahrs_fus[0]);
    float sin_hdg = sinf(ahrs_fus[0]);
    float R_yaw[3][3] = {
        { cos_hdg, sin_hdg, 0.0f},
        {-sin_hdg, cos_hdg, 0.0f},
        { 0.0f,    0.0f,    1.0f}
    };

    float a_pend[3];
    for (int i = 0; i < 3; i++) {
              a_pend[i] = R_yaw[i][0] * obs->a_hat[0] +
                          R_yaw[i][1] * obs->a_hat[1] +
                          R_yaw[i][2] * obs->a_hat[2];
    }

    // obs->theta 偏航 俯仰 滚转
    float delayed_obsv_euler[3] = {0};
    GetDelayedObsvthetaFromRingbuffer(0,delayed_obsv_euler);
    float delayed_yaw = delayed_obsv_euler[0];
    float delayed_pitch = delayed_obsv_euler[1];
    float delayed_roll = delayed_obsv_euler[2];

    obs_data[25] = delayed_yaw;
    obs_data[26] = delayed_pitch;
    obs_data[27] = delayed_roll;

    // obs->pend[0] = -(G/length_rope)*Ts*sinf(delayed_yaw) - (a_pend[2]/length_rope)*Ts*cosf(delayed_yaw);        // 偏航
    obs->pend[0] = 0;        // 偏航
    obs->pend[1] = -(G/length_rope)*Ts*sinf(delayed_pitch) - (a_pend[0]/length_rope)*Ts*cosf(delayed_pitch);    // 俯仰
    obs->pend[2] = -(G/length_rope)*Ts*sinf(delayed_roll) + (a_pend[1]/length_rope)*Ts*cosf(delayed_roll);      // 滚转

    obs_data[31] = -(G/length_rope)*Ts*sinf(delayed_roll);
    obs_data[32] = -(a_pend[1]/length_rope)*Ts*cosf(delayed_roll);

    obs->w_est[0] += obs->pend[2]; // 滚转
    obs->w_est[1] += obs->pend[1]; // 俯仰
    obs->w_est[2] = 0; // 偏航

    obs_data[10] = v_meas[0];
    obs_data[11] = v_meas[1];
    obs_data[12] = v_meas[2];    
    obs_data[13] = a_pend[0];
    obs_data[14] = a_pend[1];
    obs_data[15] = a_pend[2];

    obs_data[16] = obs->pend[0];
    obs_data[17] = obs->pend[1];
    obs_data[18] = obs->pend[2];

    obs_data[19] = obs->w_est[0];
    obs_data[20] = obs->w_est[1];
    obs_data[21] = obs->w_est[2];
    
    // 2. 四元数更新：dq = 0.5*q ⊗ [0; w_est*Ts]
    float w_half_dt[4] = {0.0f, 
                          0.5f*obs->w_est[0]*Ts, 
                          0.5f*obs->w_est[1]*Ts, 
                          0.5f*obs->w_est[2]*Ts};
    float dq[4];
    quat_mult(obs->q, w_half_dt, dq);
    
    // 四元数积分 + 归一化
    float q_temp[4];
    for (int i = 0; i < 4; i++) {
        q_temp[i] = obs->q[i] + dq[i];
    }
    float q_norm = sqrt(q_temp[0]*q_temp[0] + q_temp[1]*q_temp[1] + 
                        q_temp[2]*q_temp[2] + q_temp[3]*q_temp[3]);
    for (int i = 0; i < 4; i++) {
        obs->q[i] = q_temp[i] / q_norm;
    }

    // 姿态角更新
    quat2eul(obs->q,obs->theta);
    obs->theta[0] = ahrs_fus[0]; // modify yaw manually
    eul2quat(obs->theta, obs->q);

    obs_data[28] = obs->theta[0];
    obs_data[29] = obs->theta[1];
    obs_data[30] = obs->theta[2];

    // 加速度修正：e1 = (R(q)*g) × a / g²
    float R_np[3][3];
    quat2rotmat(obs->q, R_np);

    float R_pn[3][3];
    rotmat_transpose(R_np, R_pn);

    float acc_modify[3] = {0.0f, 0.0f, 0.0f};
    float g_n[3] = {0.0f, 0.0f, G};
    // 计算 R(q)*g
    for (int i = 0; i < 3; i++) {
        obs->Rg[i] = R_pn[i][0]*g_n[0] + R_pn[i][1]*g_n[1] + R_pn[i][2]*g_n[2];
        acc_modify[i] = obs->a_meas[i] - (R_pn[i][0]*obs->a_hat[0] + R_pn[i][1]*obs->a_hat[1] + R_pn[i][2]*obs->a_hat[2]);
    }
    obs_data[36] = a_meas[0];
    obs_data[37] = a_meas[1];
    obs_data[38] = a_meas[2];
    obs_data[39] = obs->Rg[0];
    obs_data[40] = obs->Rg[1];
    obs_data[41] = obs->Rg[2];
    obs_data[42] = (R_pn[0][0]*obs->a_hat[0] + R_pn[0][1]*obs->a_hat[1] + R_pn[0][2]*obs->a_hat[2]);
    obs_data[43] = (R_pn[1][0]*obs->a_hat[0] + R_pn[1][1]*obs->a_hat[1] + R_pn[1][2]*obs->a_hat[2]);
    obs_data[44] = (R_pn[2][0]*obs->a_hat[0] + R_pn[2][1]*obs->a_hat[1] + R_pn[2][2]*obs->a_hat[2]);
    
    float acc_mod_norm = sqrtf(acc_modify[0]*acc_modify[0] + 
                               acc_modify[1]*acc_modify[1] + 
                               acc_modify[2]*acc_modify[2]);
    
    float acc_err = fabsf(acc_mod_norm - G);
    float err_g = acc_err / G; 

    float kp_adaptive = KP_ACC_2;
    float limit_low = 0.1f;              
    float limit_high = 0.5f;

    if (err_g > limit_high) {
        kp_adaptive = KP_ACC_2_MIN;
    } else if (err_g > limit_low) {
        float ratio = (limit_high - err_g) / (limit_high - limit_low);
        kp_adaptive = KP_ACC_2_MIN + (KP_ACC_2 - KP_ACC_2_MIN) * ratio;
    }

    debug_data[0] = kp_adaptive;
    debug_data[1] = acc_modify[0];
    debug_data[2] = acc_modify[1];
    debug_data[3] = acc_modify[2];

    obs_data[33] = acc_modify[0];
    obs_data[34] = acc_modify[1];
    obs_data[35] = acc_modify[2];

    // 叉乘 + 增益修正
    cross_product(obs->Rg, acc_modify, obs->e1);
    obs->e1[0] = -obs->e1[0];
    obs->e1[1] = -obs->e1[1];
    obs->e1[2] = -obs->e1[2];
    for (int i = 0; i < 3; i++) {
       obs->e1[i] /= (G*G);
       obs->w_est[i] += KP_ACC_2 * obs->e1[i];
    //    obs->w_est[i] += kp_adaptive * obs->e1[i];
    }
    
    // ---------------------- 更新部分 ----------------------
    // 1. 角速度误差：e2 = w_meas - w_est
    // 2. 角速度更新：w_est += k*e2
    for (int i = 0; i < 3; i++) {
        float e2 = obs->w_meas[i] - obs->w_est[i];
        obs->w_est[i] += K_2 * e2;
    }
    obs->w_est[2] = 0;

    obs_data[22] = obs->w_est[0];
    obs_data[23] = obs->w_est[1];
    obs_data[24] = obs->w_est[2];

    PushObsvthetaToRingbuffer(obs->theta);
}

// 辅助函数：四元数乘法 q1⊗q2
void quat_mult(const float *q1, const float *q2, float *q_out) {
    float q0_1 = q1[0], q1_1 = q1[1], q2_1 = q1[2], q3_1 = q1[3];
    float q0_2 = q2[0], q1_2 = q2[1], q2_2 = q2[2], q3_2 = q2[3];
    
    q_out[0] = q0_1*q0_2 - q1_1*q1_2 - q2_1*q2_2 - q3_1*q3_2;
    q_out[1] = q0_1*q1_2 + q1_1*q0_2 + q2_1*q3_2 - q3_1*q2_2;
    q_out[2] = q0_1*q2_2 - q1_1*q3_2 + q2_1*q0_2 + q3_1*q1_2;
    q_out[3] = q0_1*q3_2 + q1_1*q2_2 - q2_1*q1_2 + q3_1*q0_2;
}

// 辅助函数：四元数转旋转矩阵
// R_eb = C(q^b_e)
void quat2rotmat(const float *q, float R[3][3]) {
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    
    R[0][0] = 1 - 2*q2*q2 - 2*q3*q3;
    R[0][1] = 2*q1*q2 - 2*q0*q3;
    R[0][2] = 2*q1*q3 + 2*q0*q2;
    
    R[1][0] = 2*q1*q2 + 2*q0*q3;
    R[1][1] = 1 - 2*q1*q1 - 2*q3*q3;
    R[1][2] = 2*q2*q3 - 2*q0*q1;
    
    R[2][0] = 2*q1*q3 - 2*q0*q2;
    R[2][1] = 2*q2*q3 + 2*q0*q1;
    R[2][2] = 1 - 2*q1*q1 - 2*q2*q2;
}

// 辅助函数：旋转矩阵转欧拉角（Z-Y-X 顺序）
// R_nb的欧拉角
void rotmat2euler(const float R[3][3], float *euler) {
    // 偏航角 (psi)
    if(fabsf(R[0][0]) <= 1e-6f)
    {
        euler[0] = (R[1][0] > 0) ? M_PI_2 : -M_PI_2;
    }
    else
    {
        euler[0] = atan2f(R[1][0], R[0][0]);
    }

    // 俯仰角 (theta)
    float sin_theta = -R[2][0];
    if(sin_theta >1.0f)
    {
        sin_theta = 1.0f;
    }else if(sin_theta < -1.0f)
    {
        sin_theta = -1.0f;
    }
    euler[1] = asinf(sin_theta);

    // 滚转角 (phi)
    if(fabsf(R[2][2]) <= 1e-6f)
    {
        euler[2] = (R[2][1] > 0) ? M_PI_2 : -M_PI_2;
    }
    else
    {
        euler[2] = atan2f(R[2][1], R[2][2]);
    }
}

// 辅助函数：向量叉乘 v1×v2
void cross_product(const float *v1, const float *v2, float *out) {
    out[0] = v1[1]*v2[2] - v1[2]*v2[1];
    out[1] = v1[2]*v2[0] - v1[0]*v2[2];
    out[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

// 辅助函数：欧拉角转四元数 (Z-Y-X 顺序: Input [yaw, pitch, roll])
// eul[0]: Yaw (Z), eul[1]: Pitch (Y), eul[2]: Roll (X)
// q: [w, x, y, z]
void eul2quat(const float eul[3], float q[4]) {
    float cy = cosf(eul[0] * 0.5f);
    float sy = sinf(eul[0] * 0.5f);
    float cp = cosf(eul[1] * 0.5f);
    float sp = sinf(eul[1] * 0.5f);
    float cr = cosf(eul[2] * 0.5f);
    float sr = sinf(eul[2] * 0.5f);

    q[0] = cy * cp * cr + sy * sp * sr;  // w
    q[1] = cy * cp * sr - sy * sp * cr;  // x
    q[2] = sy * cp * sr + cy * sp * cr;  // y
    q[3] = sy * cp * cr - cy * sp * sr;  // z
}

// 辅助函数：3x3矩阵转置
void rotmat_transpose(const float src[3][3], float dst[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            dst[i][j] = src[j][i];
        }
    }
}

void InitFOBF(struct FOB_t *filter,float f_cut,float f_s,float value)
{
    double T = 1.0f/f_s;
    double K_tmp = 2.0f*M_PI*f_cut*T;

    filter->num[0] = K_tmp/(2.0f+K_tmp);
    filter->num[1] = filter->num[0];
    filter->den[0] = 1.0f;
    filter->den[1] = (-2.0+K_tmp)/(2.0f+K_tmp);

    filter->input[0] = value;
    filter->output[0] = value;
}

float UpdateFOBF(struct FOB_t *filter, float value)
{
    float out = filter->num[0] * value
               + filter->num[1] * filter->input[0]
               - filter->den[1] * filter->output[0];

    filter->input[0] = value;
    filter->output[0] = out;

    return out;
}

// unit:rad
// euler_rad[0]:roll 
// euler_rad[1]:pitch 
// euler_rad[2]:yaw 
void get_acc_tbfn(const float* euler_rad, const float* acc_n, float *acc_b)
{
    matrix3_test C_bn = {{1.0,0.0,0.0}, {0.0,1.0,0.0}, {0.0,0.0,1.0}};

    float roll_rad_tmp = euler_rad[0];
    float pitch_rad_tmp = euler_rad[1];
    float yaw_rad_tmp = euler_rad[2];

    float sr = sinf(roll_rad_tmp);
    float cr = cosf(roll_rad_tmp);
    float cp = cosf(pitch_rad_tmp);
    float sp = sinf(pitch_rad_tmp);
    float sy = sinf(yaw_rad_tmp);
    float cy = cosf(yaw_rad_tmp);

    // DCM rotation from earth to body.
    C_bn.a.x = cp * cy; // 1 1
    C_bn.a.y = cp * sy; // 1 2
    C_bn.a.z = -sp; // 1 3

    C_bn.b.x = (sr * sp * cy) - (cr * sy); // 2 1
    C_bn.b.y = (sr * sp * sy) + (cr * cy); // 2 2
    C_bn.b.z = sr * cp; // 2 3

    C_bn.c.x = (cr * sp * cy) + (sr * sy); // 3 1
    C_bn.c.y = (cr * sp * sy) - (sr * cy); // 3 2
    C_bn.c.z = cr * cp; // 3 3

     acc_b[0] = C_bn.a.x*acc_n[0] + C_bn.a.y*acc_n[1] + C_bn.a.z*acc_n[2];
     acc_b[1] = C_bn.b.x*acc_n[0] + C_bn.b.y*acc_n[1] + C_bn.b.z*acc_n[2];
     acc_b[2] = C_bn.c.x*acc_n[0] + C_bn.c.y*acc_n[1] + C_bn.c.z*acc_n[2];
}

/**
 * @brief 将四元数转换为欧拉角 (ZYX顺序: 偏航, 俯仰, 滚转)
 *
 * @param q     输入的四元数数组 [w, x, y, z]
 * @param euler 输出的欧拉角数组 [yaw(Z), pitch(Y), roll(X)]，单位为弧度
 */
void quat2eul(const float q[4], float euler[3]) {
    float qw = q[0];
    float qx = q[1];
    float qy = q[2];
    float qz = q[3];

    // 计算 asin 的输入值，同 matlab 一样限幅在 [-1, 1] 以内，防止浮点误差导致越界
    float aSinInput = -2.0f * (qx * qz - qw * qy);
    if (aSinInput > 1.0f) {
        aSinInput = 1.0f;
    } else if (aSinInput < -1.0f) {
        aSinInput = -1.0f;
    }

    // qw.^2 + qx.^2 - qy.^2 - qz.^2 等平方项准备
    float qw2 = qw * qw;
    float qx2 = qx * qx;
    float qy2 = qy * qy;
    float qz2 = qz * qz;

    // euler[0]: Z轴旋转 (偏航角 Yaw)
    euler[0] = atan2f(2.0f * (qx * qy + qw * qz), qw2 + qx2 - qy2 - qz2);

    // euler[1]: Y轴旋转 (俯仰角 Pitch)
    euler[1] = asinf(aSinInput);

    // euler[2]: X轴旋转 (滚转角 Roll)
    euler[2] = atan2f(2.0f * (qy * qz + qw * qx), qw2 - qx2 - qy2 + qz2);
}

// yaw:obsv_euler[0]
// pitch:obsv_euler[1]
// roll:obsv_euler[2]
// unit:rad
void PushObsvthetaToRingbuffer(float obsv_euler[3])
{
    index_obsv_theta = (index_obsv_theta+1) % size_ring_buffer;
    ring_buffer_yaw[index_obsv_theta] = obsv_euler[0];
    ring_buffer_pitch[index_obsv_theta] = obsv_euler[1];
    ring_buffer_roll[index_obsv_theta] = obsv_euler[2];
}

// yaw:output[0]
// pitch:output[1]
// roll:output[2]
// unit:rad
void GetDelayedObsvthetaFromRingbuffer(int16_t index_need,float output[3])
{
    int index_search = index_obsv_theta - index_need;
    index_search = index_search < 0 ? (index_search+size_ring_buffer) : index_search;

    output[0] = ring_buffer_yaw[index_search];
    output[1] = ring_buffer_pitch[index_search];
    output[2] = ring_buffer_roll[index_search];
}

float* GetDebugdata(){
    return debug_data;
}

float* GetObsDebugdata(){
    return obs_data;
}

/** END OF FILE ******************************************************************************** */