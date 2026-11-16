#ifndef __NAV_H__
#define __NAV_H__

#include "main.h"

/*==========================================================================
 * 导航: 位姿推算 + A* 路径 + 航向锁定跟随
 *   - 定位只有轮速里程计 + IMU yaw
 *   - 控制策略: 车头航向锁定(上电朝向置 0)，靠 vx/vy 全向平移走完路径，
 *     全程不在通道里原地转向 (275mm 车在 450mm 通道里转不开)
 *   - 位置环: 车体系 P + 速度反馈阻尼; 航向环: PI-ish (P + 角速度阻尼)
 *==========================================================================*/

#define NAV_WP_MAX        64
#define NAV_ARRIVE_MM     14.0f     /* 中间航点到达判据 */
#define NAV_FINAL_MM      8.0f      /* 终点到达判据 */
#define NAV_PASS_MM       25.0f     /* 越过判据(沿段方向投影) */
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
uint8_t Nav_PlanToCell(uint8_t row, uint8_t col);
uint8_t Nav_PlanToMm(float x_mm, float y_mm);
uint8_t Nav_PlanToZone(uint8_t kind, uint8_t idx);
void    Nav_Stop(void);
void    Nav_SetGains(float kp, float kd, float kpy, float kdy);
void    Nav_ManualKick(void);
void    Nav_SetManualWd(uint8_t on);
extern uint8_t g_nav_manual_wd;
void    Nav_GetGains(float *kp, float *kd, float *kpy, float *kdy);
uint8_t Nav_DockCell(uint8_t kind, uint8_t idx, uint8_t *row, uint8_t *col);
void    Nav_PrintInfo(void);

#endif
