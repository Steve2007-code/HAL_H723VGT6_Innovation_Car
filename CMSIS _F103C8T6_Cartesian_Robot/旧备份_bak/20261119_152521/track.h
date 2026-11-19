/******************************************************************************
 * track.h  --  跟随调度层：10ms 时基 + PID 更新
 ******************************************************************************/
#ifndef __TRACK_H
#define __TRACK_H

#include "stm32f10x.h"

/* 全局时基（track.c 定义，中断里更新） */
extern volatile uint32_t g_ms;
extern volatile uint32_t g_last_frame_ms;
extern volatile uint8_t  g_tick_10ms;
extern volatile uint8_t  g_track_valid;      /* 1 = 目标有效，可用于点 LED */

void Track_Timer_Init(void);                  /* TIM3，10ms 中断只置标志 */
void Track_Init(void);                        /* PID 与状态初始化 */
void Track_Update(void);                      /* 主循环每 10ms 调一次 */

#endif
