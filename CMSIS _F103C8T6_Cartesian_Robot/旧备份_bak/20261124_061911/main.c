/******************************************************************************
 * main.c  --  直角坐标机械臂 A 轴视觉跟随（全非阻塞）
 *
 * 数据流：摄像头串口 <<X,dx,Y,dy>> -> USART1 中断收字节 -> 主循环解析帧
 *         -> TIM3 每 10ms 触发 PID -> Stepper_SetVel(AXIS_A, v)
 *         -> TIM2 50kHz 中断速度斜坡 + 发脉冲
 *
 * 板间：USART2 <<Y,pos,vel>> 100Hz 上报 yaw 目标给底盘 H723（UART7），
 *       收 <<A/K>> 只存状态；成帧层 frame.c 两路独立实例，互不串帧。
 *
 * A 轴 = 水平轴，跟随像素 X 误差；B 轴 = 上下轴，只使能不参与 PID。
 * yaw = 底盘 4340：摄像头俯视时追画面极角 atan2(dx,-dy)，让目标转回正上方。
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
#include "link_config.h"
#include "mcu_link.h"
#include "yaw_track.h"
#include "servo.h"
#include "backlight.h"
#include "lcd.h"

/* 舵机两个工位角度（实测） */
#define SERVO_ANGLE_CLOSE   58.0f
#define SERVO_ANGLE_OPEN    130.0f

/* 屏幕上的两行中文标签（索引见 cnfont.h：11正 12确 13抓 14取 15数 16放 17置 10冒号） */
static const uint8_t LBL_GRAB[6]  = { 11, 12, 13, 14, 15, 10 };
static const uint8_t LBL_PLACE[6] = { 11, 12, 16, 17, 15, 10 };

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

/* ---------------- 屏幕两页 ----------------
 * 第 0 页：任务码（大字铺满），同时点亮补光
 * 5 秒后切第 1 页：正确抓取数 / 正确放置数，补光熄灭
 * 用 10ms 计数做非阻塞延时，5 秒内主循环照常跑，不影响板间通信。
 */
static uint8_t  s_page      = 0;
static uint16_t s_page_tick = 0;

static void page_tick(void)
{
    if (s_page != 0) return;
    if (++s_page_tick < 500) return;          /* 10ms x 500 = 5 秒 */

    LCD_Fill(LCD_BLACK);
    LCD_DrawCNLine(64, 64,  LBL_GRAB,  6, 0, LCD_WHITE, LCD_BLACK);
    LCD_DrawCNLine(64, 152, LBL_PLACE, 6, 0, LCD_WHITE, LCD_BLACK);
    Backlight_Set(0);                          /* 任务码页结束，补光灭 */
    s_page = 1;
}

int main(void)
{
    Stepper_Init();
    led_init();
    Backlight_Init();
    Servo_Init();

    LCD_Init();
    LCD_Fill(LCD_BLACK);

    /* 第 0 页：任务码，横 8 倍(宽 40px) 纵 12 倍(高 84px = 14.95mm)，两排铺满 */
    LCD_DrawString(2, 24,  "123+321", LCD_WHITE, LCD_BLACK, 8, 12, 6);
    LCD_DrawString(2, 132, "456+654", LCD_WHITE, LCD_BLACK, 8, 12, 6);
    Backlight_Set(1);                          /* 任务码页点亮补光 */
    s_page      = 0;
    s_page_tick = 0;

    Vision_Init(TRACK_UART_BAUD);
    Track_Timer_Init();
    Track_Init();
    Link_Init(LINK_UART_BAUD);
    YawTrack_Init();

    Stepper_Enable(AXIS_A, 1);
    Stepper_Enable(AXIS_B, 1);          /* B 轴给保持力矩，不参与跟随 */
    delay_ms(300);

    Stepper_TrackOn(TRACK_AXIS, 1);     /* A 轴进入连续速度跟随模式 */

    /* 上电工作状态：夹爪停在关位（补光由屏幕页面控制） */
    Servo_SetAngle(SERVO_ANGLE_CLOSE);      /* 夹爪默认闭合 */

    while (1) {
        if (g_tick_10ms) {
            g_tick_10ms = 0;
            Link_PollRx();              /* 1) 先收 A/K，更新链路状态 */
            Track_Update();             /* 2) 视觉 PID + 下发 A 轴速度，不阻塞、不发帧 */
            YawTrack_Update(Track_GetDx(), Track_GetDy(), Track_GetValid());
                                        /* 3) yaw 极角闭环 + 更新 yaw 目标角 */
            Link_TxTick();              /* 3)+4) 取 yaw 目标 + 组包投递 TX 缓冲 */
            Servo_Tick();               /* 舵机 20ms 帧同步推进（内部 2 分频） */
            page_tick();                /* 屏幕页面：任务码显示 5 秒后转统计页 */
        }

        if (g_track_valid) led_on();    /* 识别到目标常亮 */
        else               led_off();   /* 丢失目标熄灭 */

        /* 无任何忙等；Link_TxTick 只投递缓冲，实际发送由 USART2 TXE 中断完成 */
    }
}
