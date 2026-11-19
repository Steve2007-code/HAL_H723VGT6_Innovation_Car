/******************************************************************************
 * pid.c  --  死区 + 积分限幅 + 输出限幅
 *
 * F103 无硬件 FPU，float 走软件库；10ms 调用一次，开销可忽略。
 ******************************************************************************/
#include "pid.h"

void PID_Init(PID_t *p, float kp, float ki, float kd)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->integ = 0.0f;
    p->prev  = 0.0f;
    p->dead_px   = 0;
    p->out_max   = 1000;
    p->integ_max = 100.0f;
}

void PID_Reset(PID_t *p)
{
    p->integ = 0.0f;
    p->prev  = 0.0f;
}

int32_t PID_Calc(PID_t *p, int16_t err, float dt)
{
    float e, d, out;

    if (err > -p->dead_px && err < p->dead_px) e = 0.0f;
    else e = (float)err;

    p->integ += e * dt;
    if (p->integ >  p->integ_max) p->integ =  p->integ_max;
    if (p->integ < -p->integ_max) p->integ = -p->integ_max;

    d = (dt > 0.0f) ? (e - p->prev) / dt : 0.0f;
    p->prev = e;

    out = p->kp * e + p->ki * p->integ + p->kd * d;

    if (out >  (float)p->out_max) out =  (float)p->out_max;
    if (out < -(float)p->out_max) out = -(float)p->out_max;

    return (int32_t)out;
}
