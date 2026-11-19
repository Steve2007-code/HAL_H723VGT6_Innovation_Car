/******************************************************************************
 * link_config.h  --  板间链路参数集中处（与底盘侧协议对齐，改这里即可）
 *
 * 单位：pos = 0.001 rad，vel = 0.001 rad/s，均为有符号整数。
 * 本期 yaw 目标只留接口 + 编译期默认值，不接外部输入、不做视觉粗调。
 ******************************************************************************/
#ifndef __LINK_CONFIG_H
#define __LINK_CONFIG_H

#define LINK_UART_BAUD          115200
#define LINK_PERIOD_MS          10        /* 与 TIM3 10ms 同拍，Y 帧 100Hz */

/* F103 侧同样限幅（H723 侧还会再硬钳一次） */
#define LINK_POS_MIN_MRAD       0       /* 4340 yaw 下边界: 0 度 */
#define LINK_POS_MAX_MRAD       3142    /* 4340 yaw 上边界: +pi = 180 度 */
#define LINK_VEL_MAX_MRADPS     3000

/* yaw 目标：本期只留接口 + 默认值，待用户确定工位逻辑后再填算法 */
#define LINK_YAW_DEFAULT_MRAD   0
#define LINK_YAW_VEL_MRADPS     500       /* 默认限速 0.5 rad/s */

/* 断链与启动握手 */
#define LINK_LOST_MS            1000      /* >1s 无 A/K -> link_lost，停发 Y */
#define LINK_E_RETRY_MS         100       /* 上电前 1s 每 100ms 补发一次 E,1 */
#define LINK_E_RETRY_SPAN_MS    1000

/* K 帧事件码（底盘侧发，只存不解释） */
#define LINK_K_READY            1
#define LINK_K_ENABLED          2
#define LINK_K_BAD_FRAME        3
#define LINK_K_CLAMPED          4
#define LINK_K_LOST             5

/* ---------- yaw 视觉跟随（摄像头俯视：yaw 追极角，A 轴追 dx） ---------- */
#define YAW_SIGN                (1)     /* 方向符号，目标在正右方时跑反了就改 -1 */
#define YAW_KP_Q12              2800    /* 比例项 2800/4096 约 68% (有超前量限幅兜底) */
#define YAW_KD_Q12              5000    /* 微分阻尼(像素/帧), 随 KP 上调压抖 */
#define YAW_ANGLE_DEAD          5       /* 像素死区 (px), 判定目标已居中的容差 */
#define YAW_RATE_MAX_MRAD       200     /* 每帧最大角度增量 mrad (30fps x 200 = 6000 mrad/s) */
#define YAW_LPF_Q12             2048    /* 一阶低通系数 Q12，0.5 */
#define YAW_VEL_MRADPS          6000    /* 发给 4340 的限速 6.0 rad/s (需 H723 LINK_VEL_LIM 同步放开) */
#define YAW_LEAD_MAX            500     /* 目标角允许领先实际上限 mrad (150->350: 这才是速度上限的真正瓶颈) */

#endif
