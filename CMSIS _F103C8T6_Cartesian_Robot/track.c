/******************************************************************************
 * track.c  --  跟随调度：取最新帧 -> 低通 -> PID -> 连续速度指令
 *
 * A 轴跟随 dy（y 像素误差）：前后伸缩把目标拉到画面水平中线；
 * dx 只提供给 yaw_track.c 做 yaw 闭环（两轴解耦，互不干扰）。
 *
 * 全程非阻塞：串口收字节在 USART1 中断，发脉冲在 TIM2 中断，
 * 这里只在主循环里被 10ms 标志触发，不做任何忙等。
 ******************************************************************************/
#include "track.h"
#include "track_config.h"
#include "vision.h"
#include "pid.h"
#include "stepper.h"

volatile uint32_t g_ms;
volatile uint32_t g_last_frame_ms;
volatile uint8_t  g_tick_10ms;
volatile uint8_t  g_track_valid;

static PID_t   s_pid;
static int16_t s_dx;
static int16_t s_dy;
static uint8_t s_valid;
static float   s_dy_f;
static uint8_t s_run = 1;          /* 0 = 手动位置控制期间暂停 PID */

void Track_Timer_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->PSC = (uint16_t)(SystemCoreClock / 10000U - 1U);   /* 10kHz */
    TIM3->ARR = 100U - 1U;                                    /* 10ms */
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CNT = 0;
    TIM3->SR  = 0;
    TIM3->DIER = TIM_DIER_UIE;
    TIM3->CR1  = TIM_CR1_CEN | TIM_CR1_URS;

    NVIC_SetPriority(TIM3_IRQn, 2);        /* 低于 TIM2 脉冲中断 */
    NVIC_EnableIRQ(TIM3_IRQn);
}

void TIM3_IRQHandler(void)
{
    if (TIM3->SR & TIM_SR_UIF) {
        TIM3->SR = (uint16_t)~TIM_SR_UIF;
        g_ms += TRACK_PERIOD_MS;
        g_tick_10ms = 1;                   /* 只置标志，PID 不进中断 */
    }
}

void Track_Init(void)
{
    PID_Init(&s_pid, TRACK_KP, TRACK_KI, TRACK_KD);
    s_pid.dead_px   = TRACK_DEAD_PX;
    s_pid.out_max   = TRACK_OUT_MAX_PPS;
    s_pid.integ_max = TRACK_INTEG_MAX;

    s_dx     = 0;
    s_dy     = 0;
    s_valid  = 0;
    s_dy_f   = 0.0f;
    g_last_frame_ms = 0;
    g_track_valid   = 0;
}

void Track_Update(void)
{
    VisionFrame_t f;
    int32_t v;

    /* 把缓冲里所有帧读完，取最后一帧（最新值） */
    while (Vision_Poll(&f)) {
        s_dx    = f.dx;
        s_dy    = f.dy;
        s_valid = f.valid;
    }

    /* 手动位置控制期间不跑 PID（帧照常消费以清空缓冲，避免环形缓冲溢出） */
    if (!s_run) {
        g_track_valid = 0;
        return;
    }

    /* 丢失目标：9999 哨兵或通信超时 */
    if (!s_valid || Vision_AgeMs() > TRACK_TIMEOUT_MS) {
        PID_Reset(&s_pid);
        s_dy_f = 0.0f;
        g_track_valid = 0;
        Stepper_SetVel(TRACK_AXIS, 0);     /* 平滑减速停住，不硬停 */
        return;
    }

    g_track_valid = 1;

    /* 一阶低通抑制像素抖动 (A 轴跟 dy) */
    s_dy_f += TRACK_LPF_ALPHA * ((float)s_dy - s_dy_f);

    v = PID_Calc(&s_pid, (int16_t)s_dy_f, (float)TRACK_PERIOD_MS / 1000.0f);

    /* 小误差保底速度: 纯比例项在误差很小时输出接近于 0, 表现为蹭着走很慢。
       死区外强制至少 TRACK_MIN_PPS, 由速度斜坡平滑加速, 不会突变。 */
    if (v > 0 && v <  TRACK_MIN_PPS) v =  TRACK_MIN_PPS;
    if (v < 0 && v > -TRACK_MIN_PPS) v = -TRACK_MIN_PPS;

    Stepper_SetVel(TRACK_AXIS, (int32_t)TRACK_SIGN * v);
}

/* ---------- 手动/自动切换：手动位置控制时暂停视觉 PID ---------- */
void Track_SetRun(uint8_t on)
{
    s_run = on;
    if (!on) {
        PID_Reset(&s_pid);
        s_dy_f = 0.0f;
        Stepper_SetVel(TRACK_AXIS, 0);
        g_track_valid = 0;
    }
}

uint8_t Track_GetRun(void) { return s_run; }

/* ---------- 只读访问器：供 yaw 跟随取最近一帧误差 ---------- */
int16_t Track_GetDx(void) { return s_dx; }
int16_t Track_GetDy(void) { return s_dy; }

/* 与 Track_Update 的丢失判定保持一致：帧有效且未超时 */
uint8_t Track_GetValid(void)
{
    if (!s_valid) return 0;
    if (Vision_AgeMs() > TRACK_TIMEOUT_MS) return 0;
    return 1;
}
