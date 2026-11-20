#include <stdio.h>
#include <math.h>
#include "Attitude.h"
#include "DM_IMU_L1_V1.0.h"
#include "Nav.h"

Attitude_t g_att;

static uint8_t s_div = 0;

/**
  * @brief  角度归一化到 [-pi, pi]
  */
float Attitude_WrapPi(float a)
{
    while (a >   ATT_PI) a -= ATT_2PI;
    while (a <  -ATT_PI) a += ATT_2PI;
    return a;
}

void Attitude_Init(void)
{
    g_att.yaw      = 0.0f;
    g_att.yaw_rate = 0.0f;
    g_att.gz_bias  = 0.0f;
    g_att.euler[0] = 0.0f;  g_att.euler[1] = 0.0f;  g_att.euler[2] = 0.0f;
    g_att.gyro[0]  = 0.0f;  g_att.gyro[1]  = 0.0f;  g_att.gyro[2]  = 0.0f;
    g_att.acc[0]   = 0.0f;  g_att.acc[1]   = 0.0f;  g_att.acc[2]   = 0.0f;
    g_att.online   = 0;
}

/**
  * @brief  静止标定陀螺 z 轴零偏 (调用期间车必须完全静止)
  * @param  ms 采样时长 ms
  */
void Attitude_Calibrate(uint16_t ms)
{
    float sum = 0.0f;
    uint16_t n, i;

    if (ms < 50) ms = 50;
    n = (uint16_t)(ms / 5);

    for (i = 0; i < n; i++)
    {
        Red_DM_IMU_GyroData();
        HAL_Delay(5);
        sum += DM_Motor_IMU_StructData.Gyro_f_raw[2];
    }
    g_att.gz_bias = sum / (float)n;

    printf("# gyro z bias = %.6f rad/s (%d samples)\r\n", g_att.gz_bias, n);
}

/**
  * @brief  把当前朝向置为 0
  */
void Attitude_ZeroYaw(void)
{
    g_att.yaw = 0.0f;
}

/**
  * @brief  姿态周期任务 (5 ms)
  * @note   每 10 ms 请求一次欧拉角与角速度, 每 5 ms 用最新角速度积分 yaw
  */
void Attitude_Update(void)
{
    float gz;

    /* ---- 每 10 ms 请求一帧欧拉角 + 一帧角速度 ---- */
    if ((s_div & 0x01u) == 0)
    {
        Red_DM_IMU_EulerData();
        Red_DM_IMU_GyroData();
        g_att.online = 1;
    }
    s_div++;

    /* ---- 采样 ---- */
    g_att.euler[0] = DM_Motor_IMU_StructData.Euler_f_pitch;
    g_att.euler[1] = DM_Motor_IMU_StructData.Euler_f_yaw;
    g_att.euler[2] = DM_Motor_IMU_StructData.Euler_f_roll;
    g_att.gyro[0]  = DM_Motor_IMU_StructData.Gyro_f_raw[0];
    g_att.gyro[1]  = DM_Motor_IMU_StructData.Gyro_f_raw[1];
    g_att.gyro[2]  = DM_Motor_IMU_StructData.Gyro_f_raw[2];
    g_att.acc[0]   = DM_Motor_IMU_StructData.acc_f_raw[0];
    g_att.acc[1]   = DM_Motor_IMU_StructData.acc_f_raw[1];
    g_att.acc[2]   = DM_Motor_IMU_StructData.acc_f_raw[2];

    /* ---- 角速度低通 ---- */
    gz = g_att.gyro[2] - g_att.gz_bias;
    g_att.yaw_rate += ATT_LPF * (gz - g_att.yaw_rate);

    /* ---- 积分 ---- */
    g_att.yaw = Attitude_WrapPi(g_att.yaw + g_att.yaw_rate * 0.005f);

    /* ---- 静止时在线修正零偏: 空闲且无真实转动时缓慢学习 ---- */
    if (g_nav_state == NAV_IDLE && fabsf(g_att.gyro[2] - g_att.gz_bias) < 0.03f)
        g_att.gz_bias += 0.005f * (g_att.gyro[2] - g_att.gz_bias);

#if ATT_USE_EULER
    /* 慢速修正: 仅在几乎静止时拉向欧拉角, 避免积分长期漂移 */
    if (fabsf(g_att.yaw_rate) < 0.05f)
    {
        g_att.yaw = Attitude_WrapPi(g_att.yaw + 0.02f * Attitude_WrapPi(g_att.euler[ATT_EULER_FIELD] * ATT_PI / 180.0f - g_att.yaw));
    }
#endif
}

/**
  * @brief  打印 IMU 诊断值, 用于确认哪个欧拉字段是绕 z 轴的 yaw
  * @note   把车绕竖直轴逆转 90 度, 看哪个字段变化 +90
  */
void Attitude_PrintDiag(void)
{
    printf("# IMU euler[0]/[1]/[2] = %.2f / %.2f / %.2f deg  (pitch/yaw/roll)\r\n",
           g_att.euler[0], g_att.euler[1], g_att.euler[2]);
    printf("# gyro x/y/z = %.4f / %.4f / %.4f rad/s   bias z = %.5f\r\n",
           g_att.gyro[0], g_att.gyro[1], g_att.gyro[2], g_att.gz_bias);
    printf("# acc  x/y/z = %.3f / %.3f / %.3f m/s2\r\n",
           g_att.acc[0], g_att.acc[1], g_att.acc[2]);
    printf("# yaw = %.3f deg  yawrate = %.4f rad/s\r\n",
           g_att.yaw * 57.29578f, g_att.yaw_rate);
}
