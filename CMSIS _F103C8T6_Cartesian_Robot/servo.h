#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f10x.h"

/* ==================================================================
 * MG995 / MG90S 舵机驱动（TIM4_CH1 -> PB6，50Hz）
 *
 * 用法：
 *     Servo_Init();            // 上电初始化（先不出脉冲，舵机不乱甩）
 *     Servo_SetAngle(90);      // 立即到 90 度
 *     Servo_SlewTo(180);       // 平滑走过去（每 20ms 帧走一步）
 *     Servo_Tick();            // 主循环每 10ms 调用一次（内部 2 分频到 20ms）
 *
 * 资源占用：TIM4（只做 PWM 输出，不使能任何中断），
 *           与 TIM2(步进脉冲 50kHz) / TIM3(10ms 节拍) 完全不冲突。
 * 接线：信号 -> PB6，舵机电源单独 5~6V，必须与 STM32 共地。
 * ================================================================== */

/* ---------------- 脉宽标定 ----------------
 * 500us / 2500us 是理论极限，很多 MG995 批次实际到不了，
 * 按极限输出会在两端"顶死"（嗡嗡响 + 大电流）。
 * 默认窗口 600 ~ 2400us；两端不响不抖后再以 50us 为步长往外放。
 */
#define SERVO_MIN_US        600     /* 0 度对应的高电平时间(us) */
#define SERVO_MAX_US        2400    /* 180 度对应的高电平时间(us) */

/* 平滑速度：每个 PWM 周期(20ms)最多变化多少 us
 * 15us/帧 约 1.35 度/帧 约 67 度/s
 * 更快：30(约 135 度/s)；更稳：8(约 36 度/s)
 */
#define SERVO_SLEW_US       15

#define SERVO_ANGLE_MIN     0.0f
#define SERVO_ANGLE_MAX     180.0f

/* ---------------- 初始化 / 使能 ---------------- */
void     Servo_Init(void);
void     Servo_Enable(void);
void     Servo_Disable(void);

/* ---------------- 帧同步推进 ---------------- */
/* 主循环每 10ms 调用一次；内部计数 2 次才推进一步，即严格 20ms 一帧。
   所有 CCR 写入都发生在帧起点，保证每个波形都是完整脉宽，舵机不抖。 */
void     Servo_Tick(void);

/* ---------------- 运动控制 ---------------- */
void     Servo_SetAngle(float Angle);       /* 立即到位 */
void     Servo_SlewTo(float Angle);         /* 平滑趋近，不阻塞 */

/* ---------------- 参数 / 状态 ---------------- */
void     Servo_SetSlewUs(uint16_t UsPerFrame);
void     Servo_SetPulseRange(uint16_t MinUs, uint16_t MaxUs);
void     Servo_SetPulse(uint16_t PulseUs);  /* 直接给脉宽，立即生效 */
uint16_t Servo_GetPulse(void);              /* 当前脉宽(us)，0 = 无输出 */
float    Servo_GetAngle(void);              /* 当前角度 0.0 ~ 180.0 */
uint8_t  Servo_IsBusy(void);                /* 1 = 还在动  0 = 已到位 */

#endif
