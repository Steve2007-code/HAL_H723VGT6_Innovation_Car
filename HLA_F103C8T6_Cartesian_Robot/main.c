/******************************************************************************
 * main.c  --  演示两个功能函数
 *
 * 1) 连续运行 + 匀减速停止：
 *      Stepper_Jog(AXIS_A, 1, 500, 200);   // 200ms 内匀加速到 500 步/秒，然后匀速一直跑
 *      Stepper_Stop(AXIS_A, 300);          // 300ms 内匀减速停止
 *
 * 2) 指定位置：
 *      Stepper_MoveTo(AXIS_A, 1600, 500, 200);     // 走到 1600 步(=1圈)，自动加速-匀速-到位前减速
 *      Stepper_MoveSteps(AXIS_A, -400, 500, 200);  // 相对走 -400 步
 *
 * 单位说明：1/8 细分，1600 步 = 1 圈。
 *
 * LED：PC13，低电平亮。A 路动作时亮，B 路动作时灭。
 ******************************************************************************/

#include "stm32f10x.h"
#include "stepper.h"

/* 粗延时：72MHz 下约 1ms */
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

/* 等某轴动完 */
static void wait_axis(uint8_t axis)
{
    while (Stepper_IsBusy(axis)) {
    }
}

int main(void)
{
    Stepper_Init();
    led_init();

    Stepper_Enable(AXIS_A, 1);
    Stepper_Enable(AXIS_B, 1);
    delay_ms(300);                      /* 等驱动器把电流建立起来 */

    while (1) {
//        /* ================= A 路 ================= */
        led_on();

        /* --- 位置模式：正转 1 圈（1600 步），500 步/秒，200ms 加速 --- */
        Stepper_MoveTo(AXIS_A, 7000, 4000, 200);
        wait_axis(AXIS_A);
        delay_ms(500);

//        /* --- 点动模式：加速到 500 步/秒反转，跑 1.5 秒，再 300ms 匀减速停 --- */
//        Stepper_Jog(AXIS_A, 0, 1000, 200);       /* dir: 1 正转 / 0 反转 */
//        delay_ms(1500);
//        Stepper_Stop(AXIS_A, 300);
//        wait_axis(AXIS_A);
//        delay_ms(500);

//        /* --- 回原点（绝对位置 0），慢一点，300 步/秒 --- */
//        Stepper_MoveTo(AXIS_A, 0, 300, 150);
//        wait_axis(AXIS_A);
//        delay_ms(500);

//        /* ================= B 路（同样流程） ================= */
//        led_off();

//        Stepper_MoveTo(AXIS_B, 20000, 4000, 900);
//        wait_axis(AXIS_B);
//        delay_ms(50);
//				Stepper_MoveTo(AXIS_B, 8000, 4000, 900);
//        wait_axis(AXIS_B);
//        delay_ms(50);
    }
}
