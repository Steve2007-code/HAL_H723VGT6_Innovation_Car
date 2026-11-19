/******************************************************************************
 * pid.h  --  位置式 PID，输入像素误差，输出有符号速度(pps)
 ******************************************************************************/
#ifndef __PID_H
#define __PID_H

#include "stm32f10x.h"

typedef struct {
    float   kp, ki, kd;
    float   integ;          /* 积分累积 */
    float   prev;           /* 上次误差，用于微分 */
    int16_t dead_px;        /* 死区 */
    int32_t out_max;        /* 输出限幅，pps */
    float   integ_max;      /* 积分抗饱和上限 */
} PID_t;

void    PID_Init(PID_t *p, float kp, float ki, float kd);
void    PID_Reset(PID_t *p);                              /* 丢目标时清积分 */
int32_t PID_Calc(PID_t *p, int16_t err, float dt);       /* 返回有符号 pps */

#endif
