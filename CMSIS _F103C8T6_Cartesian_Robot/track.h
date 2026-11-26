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

/* 手动位置控制：on=0 暂停视觉 PID（A 轴可被 MoveTo 接管），on=1 恢复跟随 */
void    Track_SetRun(uint8_t on);
uint8_t Track_GetRun(void);

/* 只读访问器：供 yaw 极角跟随取最近一帧误差 */
int16_t Track_GetDx(void);
int16_t Track_GetDy(void);
uint8_t Track_GetValid(void);

#endif
