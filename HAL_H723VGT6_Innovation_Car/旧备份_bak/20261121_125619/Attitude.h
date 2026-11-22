#ifndef __ATTITUDE_H__
#define __ATTITUDE_H__

#include "main.h"

/*==========================================================================
 * 姿态: 达妙 DM-IMU-L1  (模块 x=车体左, y=车体后, z=车体上)
 * 车辆的 yaw = 绕 z 轴, 俯视逆时针为正, 与底盘 wz 同号
 * 上电后把当前朝向置为 0, 之后靠陀螺 z 轴积分维持
 *==========================================================================*/

#define ATT_PI            3.14159265f
#define ATT_2PI           6.28318531f
#define ATT_LPF           0.50f         /* 角速度一阶低通系数 (200Hz 下约 20Hz) */
#define ATT_USE_EULER     0           /* 1 = 用欧拉角慢速修正积分漂移 (先实测哪个字段是 yaw) */
#define ATT_EULER_FIELD   2           /* 0/1/2 对应 Euler_f_pitch / Euler_f_yaw / Euler_f_roll */

typedef struct
{
    float yaw;          /* rad, 上电零点, 逆时针正 */
    float yaw_rate;     /* rad/s, 低通后 */
    float gz_bias;      /* 陀螺 z 零偏 */
    float euler[3];     /* 原始三轴角度, 供诊断 */
    float gyro[3];      /* 原始三轴角速度 */
    float acc[3];       /* 原始三轴加速度 */
    uint8_t online;
} Attitude_t;

extern Attitude_t g_att;

/* IMU 健康状态: 1 = 已失效(回帧但数值恒零, 或长时间无回帧), yaw 不可信 */
extern volatile uint8_t g_imu_dead;

void  Attitude_Init(void);
void  Attitude_Update(void);              /* 5 ms 调用 */
void  Attitude_Calibrate(uint16_t ms);    /* 静止零偏标定, 阻塞 */
void  Attitude_ZeroYaw(void);
float Attitude_WrapPi(float a);
void  Attitude_PrintDiag(void);

#endif
