#ifndef __NAV_H__
#define __NAV_H__

#include "main.h"

/*==========================================================================
 * 导航: 位姿推算 + A* 路径 + 车头跟随转向
 *   - 定位只有轮速里程计 + IMU yaw
 *   - 初始车头朝地图上方(+y, 即 row 减小方向)
 *   - 控制策略: 车头对准当前段方向前进, 走到拐点先停车原地转向,
 *     对准下一段后再前进 (原地转向注意窄道扫壁风险)
 *   - 位置环: 车体系 P + 速度反馈阻尼; 航向环: P + 角速度阻尼
 *==========================================================================*/

#define NAV_WP_MAX        64
#define NAV_ARRIVE_MM     14.0f     /* 中间航点到达判据 */
#define NAV_FINAL_MM      5.0f      /* 终点到达判据 (8->5, 实测原来总差 7.5mm) */
#define NAV_PASS_MM       25.0f     /* 越过判据(沿段方向投影) */
#define NAV_TURN_GATE    0.0873f   /* 段航向偏差>5度: 先原地转向再平移
                                      (25->10->5: 门槛越松, 转向后斜行越久) */
#define NAV_YAW_INIT     1.570796f /* 初始车头朝向: 地图上方(+y)=90度 */
#define NAV_CELL_MM       MAP_CELL_MM

/*---- 车体 275x275, 半宽 137.5, 余量 15 -> 152.5mm -> 4 格 ----*/
#define NAV_CAR_SIZE_MM   275.0f
#define NAV_CAR_HALF_MM   137.5f
#define NAV_CLEAR_MM      152.5f
#define NAV_CLEAR_CELLS   4

/*---- 状态 ----*/
#define NAV_IDLE          0
#define NAV_RUN           1
#define NAV_DONE          2
#define NAV_FAIL          3
#define NAV_MANUAL        4
#define NAV_ALIGN         5     /* 到点后对准指定朝向 */

typedef struct
{
    float x;        /* mm, 地图 x, 向右为正 */
    float y;        /* mm, 地图 y, 向上为正 (row 47 底 -> row 0 顶) */
    float yaw;      /* rad, 逆时针为正 */
} Pose_t;

extern Pose_t  g_pose;
extern float   g_odom_scale;        /* 里程计比例标定, 默认 1.0 */
extern uint8_t g_nav_state;
extern uint8_t g_nav_wp_total;
extern uint8_t g_nav_wp_index;

void    Nav_Init(void);
void    Nav_Update(void);                    /* 5 ms, 中断里跑 */
void    Odom_Update(void);                   /* 5 ms, 中断里跑 */
void    Nav_ResetPose(uint8_t row, uint8_t col);
void    Nav_ResetPoseTo(uint8_t row, uint8_t col, float yaw_deg);  /* 指定真实车头角 */
void    Nav_SetPoseMm(float x_mm, float y_mm, float yaw_deg);      /* 直接按 mm 设位姿 */
void    Nav_RotateTo(float deg);                                   /* 原地转到指定角度 */
void    Nav_RotateBy(float deg);                                   /* 相对当前角度再转 deg */
uint8_t Nav_PlanToCell(uint8_t row, uint8_t col);
uint8_t Nav_PlanToMm(float x_mm, float y_mm);
uint8_t Nav_PlanToZone(uint8_t kind, uint8_t idx);
void    Nav_Stop(void);
void    Nav_SetGains(float kp, float kd, float kpy, float kdy);
void    Nav_SetDockYaw(float deg);           /* 到点朝向, 999=取消 */
void    Nav_ManualKick(void);
void    Nav_SetManualWd(uint8_t on);
extern uint8_t g_nav_manual_wd;
void    Nav_GetGains(float *kp, float *kd, float *kpy, float *kdy);
uint8_t Nav_DockCell(uint8_t kind, uint8_t idx, uint8_t *row, uint8_t *col);
void    Nav_PrintInfo(void);

/* 内置路线: <<R 1>> / <<R 2>> 一键跑图, <<R 0>> 停止 */
/* 转向策略: 0=锁车头永不转 1=只在大角度(>100度)转 2=默认,车头跟随段方向
   3=纯追踪圆弧过渡(不停车、不原地转, 前瞻点引导, 转角大时自动降速) */
extern uint8_t g_yaw_mode;

/* 失败原因: 1=IMU失效 2=转向超时 3=规划失败 4=起点/终点不可规划 */
extern volatile uint8_t g_nav_fail_why;

void    Nav_RouteStart(uint8_t on);
void    Nav_RoutePoll(void);        /* 主循环调用: 到点后推进下一点 */
uint8_t Nav_RouteBusy(void);
uint8_t Nav_RouteIndex(void);

#endif
