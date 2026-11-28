/******************************************************************************
 * track_config.h  --  视觉跟随可调参数（集中在这里，标定只改本文件）
 *
 * 说明：A 轴是水平轴，跟随摄像头 X 方向像素误差；B 轴是上下轴，不参与 PID。
 ******************************************************************************/
#ifndef __TRACK_CONFIG_H
#define __TRACK_CONFIG_H

#include "stepper.h"

/* ---------- 串口 ---------- */
#define TRACK_UART_BAUD        115200
#define TRACK_TIMEOUT_MS       250        /* 超过这么久没收到有效帧就判丢失 */

/* ---------- 控制周期 ---------- */
#define TRACK_PERIOD_MS        10         /* PID 周期，TIM3 定时置标志 */

/* ---------- 方向（需要现场标定 1） ---------- */
/* dx > 0 表示目标在画面右侧。若电机跑反了，把 -1 改成 +1 即可 */
#define TRACK_SIGN             (1)       /* 已实车标定: A 轴跟 dy 时 +1 为正确方向 */

/* ---------- PID（需要现场标定 2） ---------- */
#define TRACK_KP               72.0f     /* 单位 pps/px (36 -> 72, 小/中误差响应再翻倍) */
#define TRACK_KI               0.0f
#define TRACK_KD               0.0f
#define TRACK_DEAD_PX          1         /* 死区(5->1): 对准要求 ±2px, 死区必须更小才压得进去 */
#define TRACK_OUT_MAX_PPS      12000      /* 输出限幅 (6000 -> 12000, 与 MAX_PPS 同步) */
#define TRACK_MIN_PPS          600       /* 小误差保底速度 (300 -> 600, 微小误差也快速贴合) */
#define TRACK_INTEG_MAX        200.0f    /* 积分抗饱和 */

/* ---------- 执行层 ---------- */
#define TRACK_AXIS             AXIS_A
#define TRACK_ACC_PPS_PER_MS   24        /* 速度斜坡 (16 -> 24, 到 12000pps 约 500ms; 丢步降回 16) */
#define TRACK_LPF_ALPHA        0.5f      /* dy 一阶低通，1.0 表示不滤波 */

/* ---------- 协议 ---------- */
#define LOST_SENTINEL          9999      /* 未识别到目标时 x、y 的默认值 */

#endif
