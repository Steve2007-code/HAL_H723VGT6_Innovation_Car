#ifndef __STEPPER_H
#define __STEPPER_H

#include "stm32f10x.h"

/* 轴号 */
#define AXIS_A   0u
#define AXIS_B   1u

/* ---------- 软件限位（单位：微步，闭区间） ---------- */
#define AXIS_A_POS_MIN    0         /* A 轴上电在行程 0 位 */
#define AXIS_A_POS_MAX    6500      /* 行程 0~6500 步 */
#define AXIS_B_POS_MIN    0
#define AXIS_B_POS_MAX    20000

/* 1/8 细分时 200 x 8 = 1600 步/圈（拨码 SW1=ON, SW2=OFF, SW3=OFF） */
#define STEPPER_STEPS_PER_REV   1600

/* ---------- 初始化 / 使能 ---------- */
void     Stepper_Init(void);
void     Stepper_Enable(uint8_t axis, uint8_t en);      /* en=1 使能(有保持力矩), 0 释放 */

/* ---------- 1. 连续运行：匀加速 -> 匀速，可匀减速停止 ---------- */
/* dir: 1 正转 / 0 反转；speed_pps: 目标速度(步/秒)；accel_ms: 从静止加到该速度用多少毫秒 */
void     Stepper_Jog(uint8_t axis, int8_t dir, uint16_t speed_pps, uint16_t accel_ms);
/* 匀减速停止：decel_ms 毫秒内速度线性降到 0（decel_ms=0 立即停） */
void     Stepper_Stop(uint8_t axis, uint16_t decel_ms);
/* 急停（不减速，直接断脉冲） */
void     Stepper_StopNow(uint8_t axis);

/* ---------- 3. 连续速度跟随（视觉 PID 专用，非阻塞） ---------- */
/* 有符号速度，单位 步/秒；正数 = pos 增大方向。可每个控制周期反复调用，
   内部只改目标速度，不重启加减速斜坡，因此连续跟随不会抖动、不丢步。 */
void     Stepper_SetVel(uint8_t axis, int32_t vel_pps);
/* 进入/退出跟随模式；on=1 时该轴进入 MODE_TRACK，on=0 时停机 */
void     Stepper_TrackOn(uint8_t axis, uint8_t on);

/* ---------- 2. 位置控制：自动加速 -> 匀速 -> 到点前自动减速 ---------- */
/* 走到绝对位置 target_steps（单位：微步），正数=正转方向 */
void     Stepper_MoveTo(uint8_t axis, int32_t target_steps, uint16_t speed_pps, uint16_t accel_ms);
/* 相对当前位置走 steps 步（可正可负） */
void     Stepper_MoveSteps(uint8_t axis, int32_t steps, uint16_t speed_pps, uint16_t accel_ms);

/* ---------- 状态查询 ---------- */
uint8_t  Stepper_IsBusy(uint8_t axis);       /* 1 = 还在动 */
int32_t  Stepper_GetPos(uint8_t axis);       /* 当前位置（微步） */
void     Stepper_SetPos(uint8_t axis, int32_t steps);   /* 把当前位置设成任意值（配合 MoveTo 用） */
uint16_t Stepper_GetSpeed(uint8_t axis);     /* 当前实际速度（步/秒） */
int32_t  Stepper_GetPosMin(uint8_t axis);    /* 软件限位下限（微步） */
int32_t  Stepper_GetPosMax(uint8_t axis);    /* 软件限位上限（微步） */

#endif
