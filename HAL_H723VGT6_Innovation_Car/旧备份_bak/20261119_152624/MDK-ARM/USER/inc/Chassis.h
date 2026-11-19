#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "main.h"

/*==========================================================================
 * 麦轮底盘运动学 (X 型布置, A 轮 = 左前/右后, B 轮 = 右前/左后)
 * 车体系: x 向前,  y 向左,  wz 逆时针为正
 * 世界系: x 向右(地图 col 方向), y 向上(地图 row 递减方向)
 * 角度单位 rad, 线速度 m/s, 电机指令为减速器输出端 rad/s
 *==========================================================================*/

/*---------------- 机械参数 ----------------*/
#define CH_WHEEL_D        0.097f        /* 麦轮外径 m */
#define CH_WHEEL_R        0.0485f       /* 麦轮半径 m */
#define CH_WHEEL_BASE     0.180f        /* 轴距 L m (前后轮中心距) */
#define CH_WHEEL_TRACK    0.235f        /* 轮距 W m (左右轮中心距) */
#define CH_K              (((CH_WHEEL_BASE) + (CH_WHEEL_TRACK)) * 0.5f)   /* 0.2075 m */

/*---------------- 限幅与斜坡 ----------------*/
#define CH_RAD_MAX        30.0f         /* 电机 VMAX rad/s */
#define CH_RAD_USE        12.0f         /* 底盘实际使用上限 rad/s 约 0.58 m/s */
#define CH_VXY_MAX        0.60f         /* 车体线速度上限 m/s */
#define CH_WZ_MAX         2.50f         /* 车体角速度上限 rad/s */
#define CH_ACC_V          1.20f         /* 线加速度 m/s^2 */
#define CH_ACC_W          8.00f         /* 角加速度 rad/s^2 */

/*---------------- 轮序索引 ----------------*/
#define CH_FL   0                       /* 0x01 左前 A */
#define CH_FR   1                       /* 0x02 右前 B */
#define CH_RL   2                       /* 0x03 左后 B */
#define CH_RR   3                       /* 0x04 右后 A */

typedef struct
{
    float vx,  vy,  wz;         /* 目标车体速度 */
    float cvx, cvy, cwz;        /* 斜坡后当前值 */
    float w_rad[4];             /* 下发轮角速度 rad/s (已含方向系数) */
    float fb_rad[4];            /* 反馈轮角速度 rad/s */
    float fb_vx, fb_vy, fb_wz;  /* 反馈正解车体速度 */
    float dir[4];               /* 方向标定系数 +1 / -1 */
    uint8_t single_mode;        /* 1 = 单轮标定模式 */
    uint8_t single_idx;
    float   single_rad;
} Chassis_t;

extern Chassis_t g_chassis;

void    Chassis_Init(void);
void    Chassis_SetBodyVel(float vx, float vy, float wz);
void    Chassis_Update(void);           /* 5 ms 调用: 斜坡 + 逆解 + 下发 */
void    Chassis_Hold(void);             /* 目标速度清零(带斜坡) */
void    Chassis_SetDir(uint8_t idx, float d);
void    Chassis_Single(uint8_t idx, float rad);
void    Chassis_SingleExit(void);
void    Chassis_GetBodyVel(float *vx, float *vy, float *wz);

#endif
