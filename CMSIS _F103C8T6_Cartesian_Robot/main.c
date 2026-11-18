/******************************************************************************
 * main.c  --  直角坐标机械臂 A 轴视觉跟随（全非阻塞）
 *
 * 数据流：摄像头串口 <<X,dx,Y,dy>> -> USART1 中断收字节 -> 主循环解析帧
 *         -> TIM3 每 10ms 触发 PID -> Stepper_SetVel(AXIS_A, v)
 *         -> TIM2 50kHz 中断速度斜坡 + 发脉冲
 *
 * A 轴 = 水平轴，跟随像素 X 误差；B 轴 = 上下轴，只使能不参与 PID。
 * 目标丢失（9999 或 250ms 无帧）时平滑减速停住保持。
 *
 * LED：PC13 低电平亮，识别到目标时常亮。
 * 标定项集中在 track_config.h：TRACK_SIGN（方向）、TRACK_KP（增益）。
 ******************************************************************************/

#include "stm32f10x.h"
#include "stepper.h"
#include "track_config.h"
#include "track.h"
#include "vision.h"

/* 粗延时：72MHz 下约 1ms，仅用于上电等驱动器建立电流 */
static void delay_ms(uint32_t ms)
{
    uint32_t i;
    volatile uint32_t j;

    for (i = 0; i < ms; i++) {
        for (j = 0; j < 8000; j++) {
        }
    }
}

static void led_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    GPIOC->CRH &= ~(0xFU << 20);        /* PC13 */
    GPIOC->CRH |=  (0x2U << 20);        /* 2MHz 推挽输出 */
    GPIOC->BSRR = (1U << 13);           /* 熄灭 */
}

static void led_on(void)  { GPIOC->BRR  = (1U << 13); }
static void led_off(void) { GPIOC->BSRR = (1U << 13); }

int main(void)
{
    Stepper_Init();
    led_init();

    Vision_Init(TRACK_UART_BAUD);
    Track_Timer_Init();
    Track_Init();

    Stepper_Enable(AXIS_A, 1);
    Stepper_Enable(AXIS_B, 1);          /* B 轴给保持力矩，不参与跟随 */
    delay_ms(300);

    Stepper_TrackOn(TRACK_AXIS, 1);     /* A 轴进入连续速度跟随模式 */

    while (1) {
        if (g_tick_10ms) {
            g_tick_10ms = 0;
            Track_Update();             /* 解析帧 + PID + 下发速度 */
        }

        if (g_track_valid) led_on();    /* 识别到目标常亮 */
        else               led_off();   /* 丢失目标熄灭 */

        /* 无任何忙等，后续可在此叠加串口指令、状态上报等任务 */
    }
}
