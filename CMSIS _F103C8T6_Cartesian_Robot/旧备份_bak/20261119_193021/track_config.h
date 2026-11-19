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
#define TRACK_KP               9.0f      /* 单位 pps/px (3.0 -> 6.0 -> 9.0, 再快 50%) */
#define TRACK_KI               0.0f
#define TRACK_KD               0.0f
#define TRACK_DEAD_PX          5         /* 死区，抑制像素抖动 */
#define TRACK_OUT_MAX_PPS      3000      /* 输出限幅 (1200 -> 3000, 上限 MAX_PPS=4000) */
#define TRACK_INTEG_MAX        200.0f    /* 积分抗饱和 */

/* ---------- 执行层 ---------- */
#define TRACK_AXIS             AXIS_A
#define TRACK_ACC_PPS_PER_MS   12        /* 速度斜坡 (8 -> 12, 加速更快; 若丢步降回 8) */
#define TRACK_LPF_ALPHA        0.5f      /* dy 一阶低通，1.0 表示不滤波 */

/* ---------- 协议 ---------- */
#define LOST_SENTINEL          9999      /* 未识别到目标时 x、y 的默认值 */

#endif
