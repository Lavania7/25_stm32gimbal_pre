#include "Kalman.h"

/**************************************************************************
Function: Simple Kalman filter for X-axis
Input   : Acceleration (angle from accelerometer), Angular velocity (from gyro)
Output  : Filtered angle
功能：X轴卡尔曼滤波器
输入：加速度计获取的角度，陀螺仪获取的角速度
返回值：X轴滤波后的角度
**************************************************************************/
float dt = 0.005;  // 采样间隔：5ms
float Kalman_Filter_x(float Accel, float Gyro)
{
    static float angle_dot;  // 角速度
    static float angle;      // 角度
    float Q_angle = 0.001;   // 过程噪声协方差（角度）
    float Q_gyro = 0.003;    // 过程噪声协方差（陀螺仪漂移）
    float R_angle = 0.5;     // 测量噪声协方差
    char C_0 = 1;            // 观测矩阵系数
    
    static float Q_bias;     // 陀螺仪偏置
    static float Angle_err;  // 角度误差
    static float PCt_0, PCt_1, E;
    static float K_0, K_1, t_0, t_1;
    static float Pdot[4] = {0, 0, 0, 0};  // 协方差矩阵导数
    static float PP[2][2] = {{1, 0}, {0, 1}};  // 协方差矩阵
    
    // 预测步骤：状态更新
    angle += (Gyro - Q_bias) * dt;  // 角度预测
    
    // 预测步骤：协方差更新
    Pdot[0] = Q_angle - PP[0][1] - PP[1][0];  // P00导数
    Pdot[1] = -PP[1][1];  // P01导数
    Pdot[2] = -PP[1][1];  // P10导数
    Pdot[3] = Q_gyro;     // P11导数
    
    PP[0][0] += Pdot[0] * dt;  // 更新P00
    PP[0][1] += Pdot[1] * dt;  // 更新P01
    PP[1][0] += Pdot[2] * dt;  // 更新P10
    PP[1][1] += Pdot[3] * dt;  // 更新P11
    
    // 更新步骤：计算残差
    Angle_err = Accel - angle;  // 测量值与预测值之差
    
    // 更新步骤：计算卡尔曼增益
    PCt_0 = C_0 * PP[0][0];  // P00 * C
    PCt_1 = C_0 * PP[1][0];  // P10 * C
    
    E = R_angle + C_0 * PCt_0;  // 残差协方差
    
    K_0 = PCt_0 / E;  // 卡尔曼增益K0
    K_1 = PCt_1 / E;  // 卡尔曼增益K1
    
    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];
    
    // 更新步骤：更新协方差矩阵
    PP[0][0] -= K_0 * t_0;  // 更新P00
    PP[0][1] -= K_0 * t_1;  // 更新P01
    PP[1][0] -= K_1 * t_0;  // 更新P10
    PP[1][1] -= K_1 * t_1;  // 更新P11
    
    // 更新状态估计
    angle += K_0 * Angle_err;   // 更新角度估计
    Q_bias += K_1 * Angle_err;  // 更新陀螺仪偏置估计
    angle_dot = Gyro - Q_bias;  // 计算角速度
    
    return angle;
}

/**************************************************************************
Function: First order complementary filter for X-axis
Input   : Angle from accelerometer, Angular velocity from gyro
Output  : Filtered angle
功能：X轴一阶互补滤波器
输入：加速度计获取的角度，陀螺仪获取的角速度
返回值：X轴滤波后的角度
**************************************************************************/
float Complementary_Filter_x(float angle_m, float gyro_m)
{
    static float angle;  // 当前角度
    float K1 = 0.02;     // 滤波器增益
    
    // 互补滤波公式：
    // 角度 = K1 * 加速度计角度 + (1 - K1) * (上一时刻角度 + 陀螺仪角速度 * dt)
    angle = K1 * angle_m + (1 - K1) * (angle + gyro_m * dt);
    return angle;
}

/**************************************************************************
Function: Simple Kalman filter for Y-axis
Input   : Acceleration (angle from accelerometer), Angular velocity (from gyro)
Output  : Filtered angle
功能：Y轴卡尔曼滤波器
输入：加速度计获取的角度，陀螺仪获取的角速度
返回值：Y轴滤波后的角度
**************************************************************************/
float Kalman_Filter_y(float Accel, float Gyro)
{
    static float angle_dot;  // 角速度
    static float angle;      // 角度
    float Q_angle = 0.001;   // 过程噪声协方差（角度）
    float Q_gyro = 0.003;    // 过程噪声协方差（陀螺仪漂移）
    float R_angle = 0.5;     // 测量噪声协方差
    char C_0 = 1;            // 观测矩阵系数
    
    static float Q_bias;     // 陀螺仪偏置
    static float Angle_err;  // 角度误差
    static float PCt_0, PCt_1, E;
    static float K_0, K_1, t_0, t_1;
    static float Pdot[4] = {0, 0, 0, 0};  // 协方差矩阵导数
    static float PP[2][2] = {{1, 0}, {0, 1}};  // 协方差矩阵
    
    // 预测步骤：状态更新
    angle += (Gyro - Q_bias) * dt;  // 角度预测
    
    // 预测步骤：协方差更新
    Pdot[0] = Q_angle - PP[0][1] - PP[1][0];  // P00导数
    Pdot[1] = -PP[1][1];  // P01导数
    Pdot[2] = -PP[1][1];  // P10导数
    Pdot[3] = Q_gyro;     // P11导数
    
    PP[0][0] += Pdot[0] * dt;  // 更新P00
    PP[0][1] += Pdot[1] * dt;  // 更新P01
    PP[1][0] += Pdot[2] * dt;  // 更新P10
    PP[1][1] += Pdot[3] * dt;  // 更新P11
    
    // 更新步骤：计算残差
    Angle_err = Accel - angle;  // 测量值与预测值之差
    
    // 更新步骤：计算卡尔曼增益
    PCt_0 = C_0 * PP[0][0];  // P00 * C
    PCt_1 = C_0 * PP[1][0];  // P10 * C
    
    E = R_angle + C_0 * PCt_0;  // 残差协方差
    
    K_0 = PCt_0 / E;  // 卡尔曼增益K0
    K_1 = PCt_1 / E;  // 卡尔曼增益K1
    
    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];
    
    // 更新步骤：更新协方差矩阵
    PP[0][0] -= K_0 * t_0;  // 更新P00
    PP[0][1] -= K_0 * t_1;  // 更新P01
    PP[1][0] -= K_1 * t_0;  // 更新P10
    PP[1][1] -= K_1 * t_1;  // 更新P11
    
    // 更新状态估计
    angle += K_0 * Angle_err;   // 更新角度估计
    Q_bias += K_1 * Angle_err;  // 更新陀螺仪偏置估计
    angle_dot = Gyro - Q_bias;  // 计算角速度
    
    return angle;
}

/**************************************************************************
Function: First order complementary filter for Y-axis
Input   : Angle from accelerometer, Angular velocity from gyro
Output  : Filtered angle
功能：Y轴一阶互补滤波器
输入：加速度计获取的角度，陀螺仪获取的角速度
返回值：Y轴滤波后的角度
**************************************************************************/
float Complementary_Filter_y(float angle_m, float gyro_m)
{
    static float angle;  // 当前角度
    float K1 = 0.02;     // 滤波器增益
    
    // 互补滤波公式：
    // 角度 = K1 * 加速度计角度 + (1 - K1) * (上一时刻角度 + 陀螺仪角速度 * dt)
    angle = K1 * angle_m + (1 - K1) * (angle + gyro_m * dt);
    return angle;
}
