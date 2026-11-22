#include <stdio.h>
#include <math.h>
#include "Attitude.h"
#include "DM_IMU_L1_V1.0.h"
#include "Nav.h"

Attitude_t g_att;


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
    /* 静止时零偏应在 0.005 以内; 超过 0.02 说明标定那 400ms 车被碰/在晃.
       该值会让 yaw 每秒漂 gz_bias 弧度, 是「有时好有时差」的元凶. */
    if (fabsf(g_att.gz_bias) > 0.02f)
    {
        printf("# !!! 标定期间车在动: 该零偏会让 yaw 每秒漂 %.1f 度 !!!\r\n",
               g_att.gz_bias * 57.29578f);
        printf("# !!! 请让车完全静止后重新上电 !!!\r\n");
    }
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
volatile uint32_t g_att_calls = 0;   /* Attitude_Update 被调用次数 */
volatile uint32_t g_imu_rej   = 0;   /* 被判为坏帧而丢弃的角速度采样数 */
volatile uint8_t  g_imu_dead  = 0;   /* 1 = IMU 已失效, yaw 不可信 */
/* 陀螺刻度修正: 转 360 度实际只转了 (360+E) 度时, 本值 = (360+E)/360.
   例: 转一圈少 6 度 -> 354/360 = 0.983 */
float g_gyro_scale = 0.9975f;   /* 实测: 转 8 次 90 度(720) 偏 1.8 度 -> 718.2/720 */
static uint16_t   s_zero_cnt  = 0;   /* 三轴原始值全等于中点的连续次数 */
static uint16_t   s_norx_cnt  = 0;   /* 角速度帧不更新的连续次数 */
static uint32_t   s_rx_snap   = 0;

void Attitude_Update(void)
{
    float gz;

    g_att_calls++;

    /* ---- 每 5ms 请求一帧欧拉角 + 一帧角速度 ----
       原为隔一次请求(10ms=100Hz), 航向反馈延迟 = 采样 10ms + 低通 ~10ms;
       改成每拍请求(5ms=200Hz)可把延迟减半, 航向环能跟得更紧、转向后回正更快。
       FDCAN3 1Mbps 下两帧只占约 0.2ms, 负载无忧。 */
    Red_DM_IMU_EulerData();
    Red_DM_IMU_GyroData();
    g_att.online = 1;

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

    /* ---- IMU 健康检查 ----
       失效表现(实测): 帧照常回, 但三轴原始值恒为编码中点 0x7FFF,
       欧拉角也全 0 -- 掉电复位后卡在零输出, 只有断电才能恢复.
       此时 yaw 永久冻结 -> 航向环永远达不到目标 -> 车会一直原地转. */
    {
        const uint8_t *g = DM_Motor_IMU_StructData.Gyro_raw;
        if (g[2] == 0xFF && g[3] == 0x7F && g[4] == 0xFF &&
            g[5] == 0x7F && g[6] == 0xFF && g[7] == 0x7F)
            s_zero_cnt++;
        else
            s_zero_cnt = 0;

        if (g_imu_rx_gyro == s_rx_snap)
            s_norx_cnt++;
        else
        {
            s_norx_cnt = 0;
            s_rx_snap  = g_imu_rx_gyro;
        }

        /* 1s 恒零 或 0.5s 无新帧 -> 判失效 (一旦置位不再自动清除, 需重启) */
        if (s_zero_cnt > 200u || s_norx_cnt > 100u)
            g_imu_dead = 1;
    }

    /* ---- 角速度低通 ---- */
    gz = g_att.gyro[2] - g_att.gz_bias;
    /* 坏帧防护: 整车不可能转这么快(12 rad/s = 688 deg/s), 超限判为坏帧丢弃.
       实测出现过 payload 全 0 的帧 -> 解码成 -34.88 rad/s(编码下限),
       仅 17ms 就能让 yaw 崩掉 33 度, 跑图时遇到一次就全废. */
    if (gz > 12.0f || gz < -12.0f)
    {
        g_imu_rej++;
        gz = 0.0f;
    }
    gz *= g_gyro_scale;            /* 刻度修正 */
    g_att.yaw_rate += ATT_LPF * (gz - g_att.yaw_rate);

    /* ---- 积分 ---- */
    g_att.yaw = Attitude_WrapPi(g_att.yaw + g_att.yaw_rate * 0.005f);

    /* ---- 在线修正零偏: 只要角速度接近零就学, 不再要求 NAV_IDLE ----
       原条件带 g_nav_state == NAV_IDLE, 导致行驶中零偏无法收敛 ->
       实测 500mm 直行期间 yaw 以 0.078 deg/s 漂移(70 秒 5.5 度).
       幅度门 0.03 rad/s(1.7 deg/s) 已足以排除真实转向, 故去掉状态限制. ---- */
    if (fabsf(g_att.gyro[2] - g_att.gz_bias) < 0.03f)
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
    printf("# yaw = %.3f deg  yawrate = %.4f rad/s  gyro_scale = %.4f\r\n",
           g_att.yaw * 57.29578f, g_att.yaw_rate, g_gyro_scale);
    /*---- 链路诊断: 判断 yaw 冻结的根因 ----*/
    printf("# att_upd=%lu  imu rx total=%lu  gyro=%lu  tx_err=%lu  rej=%lu\r\n",
           (unsigned long)g_att_calls, (unsigned long)g_imu_rx_total,
           (unsigned long)g_imu_rx_gyro, (unsigned long)g_imu_tx_err,
           (unsigned long)g_imu_rej);
    printf("# last gyro frame: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
           g_imu_last_g[0], g_imu_last_g[1], g_imu_last_g[2], g_imu_last_g[3],
           g_imu_last_g[4], g_imu_last_g[5], g_imu_last_g[6], g_imu_last_g[7]);
    printf("# fdcan3 PSR=0x%08lX ECR=0x%08lX\r\n",
           (unsigned long)DM_Motor_IMU_StructData.hcan->Instance->PSR,
           (unsigned long)DM_Motor_IMU_StructData.hcan->Instance->ECR);
}
