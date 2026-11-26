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
#include "qr_scan.h"
#include "link_config.h"
#include "mcu_link.h"
#include "yaw_track.h"
#include "servo.h"
#include "backlight.h"
#include "lcd.h"

/* 舵机两个工位角度（实测） */
#define SERVO_ANGLE_CLOSE   58.0f    /* 闭合 */
#define SERVO_ANGLE_OPEN    140.0f   /* 打开 */

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
static uint8_t  s_qr_shown  = 0;       /* 1 = 已显示任务码，开始计 5 秒 */

/* 把 "132+312+123+231" 拆成两排 7 字符显示 */
static void show_task_code(const char *code)
{
    char l1[8], l2[8];
    uint8_t i;

    for (i = 0; i < 7U; i++) l1[i] = code[i];
    l1[7] = 0;
    for (i = 0; i < 7U; i++) l2[i] = code[8U + i];
    l2[7] = 0;

    LCD_Fill(LCD_BLACK);
    LCD_DrawString(2, 24,  l1, LCD_WHITE, LCD_BLACK, 8, 12, 6);
    LCD_DrawString(2, 132, l2, LCD_WHITE, LCD_BLACK, 8, 12, 6);
}

static void page_tick(void)
{
    if (s_page != 0) return;

    /* 第 0 页：先等二维码，收到后刷新任务码并开始计时 */
    if (!s_qr_shown)
    {
        if (!g_qr_ready) return;
        g_qr_ready  = 0;
        show_task_code(g_qr_code);
        s_qr_shown  = 1;
        s_page_tick = 0;
        return;
    }

    /* 已显示任务码满 5 秒 -> 切统计页 */
    if (++s_page_tick < 500) return;          /* 10ms x 500 = 5 秒 */

    LCD_Fill(LCD_BLACK);
    /* 统计页铺满：8 字符 x 40px = 320 正好满宽，高 60px（两排各 60，三处间隙 40） */
    LCD_DrawCNLineScaled(0, 40,  LBL_GRAB,  6, 0, 40, 60, LCD_WHITE, LCD_BLACK);
    LCD_DrawCNLineScaled(0, 140, LBL_PLACE, 6, 0, 40, 60, LCD_WHITE, LCD_BLACK);
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

    /* 第 0 页：先显示占位，收到二维码后由 show_task_code() 覆盖 */
    LCD_DrawString(2, 24,  "000+000", LCD_WHITE, LCD_BLACK, 8, 12, 6);
    LCD_DrawString(2, 132, "000+000", LCD_WHITE, LCD_BLACK, 8, 12, 6);
    s_page      = 0;
    s_page_tick = 0;
    s_qr_shown  = 0;

    Vision_Init(TRACK_UART_BAUD);      /* USART1: 视觉 dx/dy */
    QR_Init(TRACK_UART_BAUD);          /* USART3: 二维码任务码 */
    Track_Timer_Init();
    Track_Init();
    Link_Init(LINK_UART_BAUD);
    YawTrack_Init();

    Stepper_Enable(AXIS_A, 1);
    Stepper_Enable(AXIS_B, 1);          /* B 轴给保持力矩，不参与跟随 */
    delay_ms(300);

    /* B 轴上升到 2000 脉冲：非阻塞，只设目标，TIM2 中断里后台自己走完 */
    Stepper_MoveTo(AXIS_B, 2000, 800, 300);

    Stepper_TrackOn(TRACK_AXIS, 1);     /* A 轴进入连续速度跟随模式 */

    /* 上电工作状态：夹爪打开到 140 度（补光由屏幕页面控制） */
    Servo_SetAngle(SERVO_ANGLE_OPEN);       /* 夹爪上电张开 */

    while (1) {
        if (g_tick_10ms) {
            g_tick_10ms = 0;
            Link_PollRx();              /* 1) 先收 A/K，更新链路状态 */
            QR_Poll();                  /* 二维码帧解析（USART3） */
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
