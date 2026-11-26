/******************************************************************************
 * servo.c  --  MG995 舵机驱动：TIM4_CH1 -> PB6，50Hz PWM
 *
 * 时基：TIM4 挂 APB1，PCLK1 = 36MHz；APB1 有 2 分频时定时器时钟 = 72MHz。
 *       PSC = 72-1  -> 1MHz，1 个计数 = 1us
 *       ARR = 20000-1 -> 20ms 周期 = 50Hz
 *       CCR1 = 高电平 us 数（600~2400）
 *
 * 为什么不用定时器中断：本工程的 TIM2 已被步进脉冲占满（50kHz），
 * TIM3 被 10ms 节拍占用。舵机只需要 20ms 一次推进，放进主循环 10ms 任务里
 * 做 2 分频即可，不新增任何中断。
 ******************************************************************************/

#include "servo.h"

typedef struct
{
    volatile uint16_t cur_us;       /* 当前实际输出脉宽，0 表示没输出 */
    volatile uint16_t tgt_us;       /* 目标脉宽 */
    volatile uint16_t step_us;      /* 每帧最多变化多少 us */
    uint16_t min_us;                /* 0 度对应脉宽 */
    uint16_t max_us;                /* 180 度对应脉宽 */
    uint8_t  tick_div;              /* 10ms -> 20ms 分频计数 */
} Servo_t;

static Servo_t s_servo =
{
    0,
    0,
    SERVO_SLEW_US,
    SERVO_MIN_US,
    SERVO_MAX_US,
    0
};

/* ---------------- 内部工具 ---------------- */
static float Servo_ClampAngle(float Angle)
{
    if (Angle < SERVO_ANGLE_MIN) return SERVO_ANGLE_MIN;
    if (Angle > SERVO_ANGLE_MAX) return SERVO_ANGLE_MAX;
    return Angle;
}

static uint16_t Servo_AngleToPulse(float Angle)
{
    uint16_t Span = (uint16_t)(s_servo.max_us - s_servo.min_us);
    return (uint16_t)(s_servo.min_us + (float)Span * Angle / SERVO_ANGLE_MAX + 0.5f);
}

static float Servo_PulseToAngle(uint16_t Pulse)
{
    uint16_t Span = (uint16_t)(s_servo.max_us - s_servo.min_us);
    if (Pulse <= s_servo.min_us || Span == 0) return SERVO_ANGLE_MIN;
    return (float)(Pulse - s_servo.min_us) * SERVO_ANGLE_MAX / (float)Span;
}

/* ---------------- 初始化 ---------------- */
void Servo_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    /* PB6 = TIM4_CH1，复用推挽 50MHz（CRL[27:24] = 0xB） */
    GPIOB->CRL &= ~(0xFU << 24);
    GPIOB->CRL |=  (0xBU << 24);

    TIM4->PSC  = 72 - 1;            /* 72MHz -> 1MHz */
    TIM4->ARR  = 20000 - 1;         /* 20ms */
    TIM4->CCR1 = 0;

    TIM4->CCMR1 &= ~(0x7U << 4);
    TIM4->CCMR1 |=  (0x6U << 4);    /* OC1M = 110 PWM1 */
    TIM4->CCMR1 |=  (1U << 3);      /* OC1PE = 1，CCR 预装载 */

    TIM4->CCER &= ~(1U << 1);       /* CC1P = 0，高电平有效 */
    TIM4->CCER &= ~(1U << 0);       /* 先不使能输出，避免上电乱甩 */

    TIM4->CNT = 0;
    TIM4->EGR = 1U;                 /* UG：立即装载 PSC / ARR */
    TIM4->CR1 |= 1U;                /* CEN */

    s_servo.cur_us   = 0;
    s_servo.tgt_us   = 0;
    s_servo.step_us  = SERVO_SLEW_US;
    s_servo.min_us   = SERVO_MIN_US;
    s_servo.max_us   = SERVO_MAX_US;
    s_servo.tick_div = 0;
}

/* ---------------- 使能 / 释放 ---------------- */
void Servo_Enable(void)
{
    TIM4->CCER |= (1U << 0);
}

void Servo_Disable(void)
{
    TIM4->CCER &= ~(1U << 0);
    s_servo.cur_us = 0;
    s_servo.tgt_us = 0;
}

/* ---------------- 帧同步推进（10ms 调用） ---------------- */
void Servo_Tick(void)
{
    uint16_t Cur, Tgt, Step;

    if (++s_servo.tick_div < 2) return;     /* 10ms x 2 = 20ms */
    s_servo.tick_div = 0;

    if (s_servo.cur_us == 0) return;        /* 还没有输出，不动 */

    Cur  = s_servo.cur_us;
    Tgt  = s_servo.tgt_us;
    Step = s_servo.step_us;

    if (Cur < Tgt)
    {
        Cur = ((uint16_t)(Tgt - Cur) > Step) ? (uint16_t)(Cur + Step) : Tgt;
    }
    else if (Cur > Tgt)
    {
        Cur = ((uint16_t)(Cur - Tgt) > Step) ? (uint16_t)(Cur - Step) : Tgt;
    }

    if (Cur != s_servo.cur_us)
    {
        s_servo.cur_us = Cur;
        TIM4->CCR1 = Cur;
    }
}

/* ---------------- 运动控制 ---------------- */
void Servo_SetAngle(float Angle)
{
    Servo_SetPulse(Servo_AngleToPulse(Servo_ClampAngle(Angle)));
}

void Servo_SlewTo(float Angle)
{
    if (s_servo.cur_us == 0)            /* 还没输出过，直接给目标，避免从 0 度扫过来 */
    {
        Servo_SetAngle(Angle);
        return;
    }

    Servo_Enable();
    s_servo.tgt_us = Servo_AngleToPulse(Servo_ClampAngle(Angle));
}

uint8_t Servo_IsBusy(void)
{
    return (s_servo.cur_us != s_servo.tgt_us);
}

/* ---------------- 参数 / 状态 ---------------- */
void Servo_SetSlewUs(uint16_t UsPerFrame)
{
    if (UsPerFrame == 0) UsPerFrame = 1;
    s_servo.step_us = UsPerFrame;
}

void Servo_SetPulseRange(uint16_t MinUs, uint16_t MaxUs)
{
    if (MinUs >= MaxUs) return;
    s_servo.min_us = MinUs;
    s_servo.max_us = MaxUs;
}

void Servo_SetPulse(uint16_t PulseUs)
{
    if (PulseUs < s_servo.min_us) PulseUs = s_servo.min_us;
    if (PulseUs > s_servo.max_us) PulseUs = s_servo.max_us;

    /* 已经在这个位置就不再写寄存器 */
    if (PulseUs == s_servo.cur_us && PulseUs == s_servo.tgt_us) return;

    Servo_Enable();
    TIM4->CCR1 = PulseUs;
    s_servo.cur_us = PulseUs;
    s_servo.tgt_us = PulseUs;
}

uint16_t Servo_GetPulse(void)
{
    return s_servo.cur_us;
}

float Servo_GetAngle(void)
{
    return Servo_PulseToAngle(s_servo.cur_us);
}
