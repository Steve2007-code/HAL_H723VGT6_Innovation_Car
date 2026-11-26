/******************************************************************************
 * vision.h  --  摄像头串口数据帧解析（非阻塞）
 *
 * 数据包：包头 <<、包尾 >>，格式 <<X,x像素差值,Y,y像素差值>>
 *   识别到目标：<<X,-24,Y,35>>
 *   丢失目标：  <<X,9999,Y,9999>>
 ******************************************************************************/
#ifndef __VISION_H
#define __VISION_H

#include "stm32f10x.h"

typedef struct {
    int16_t  dx;             /* X 方向像素差值 */
    int16_t  dy;             /* Y 方向像素差值（本项目 B 轴不用，仅保留） */
    uint8_t  valid;          /* 1 = 识别到目标，0 = 丢失 */
    uint32_t stamp_ms;       /* 收帧时刻 */
} VisionFrame_t;

/* ---------------- 二维码任务码 ----------------
 * 摄像头识别到二维码后发: <<flag,132+312+123+231>>
 * 固定 15 字符 = 4 组 3 位数字 + 3 个 '+'，收到后置 g_qr_ready=1。
 */
#define QR_CODE_MAX     20
extern char              g_qr_code[QR_CODE_MAX];   /* 例如 "132+312+123+231" */
extern volatile uint8_t  g_qr_ready;               /* 1 = 收到一帧新任务码 */

void     Vision_Init(uint32_t baud);        /* USART1 PA9/PA10，RXNE 中断 + 环形缓冲 */
uint8_t  Vision_Poll(VisionFrame_t *f);     /* 非阻塞：解析到完整帧返回 1 */
uint32_t Vision_AgeMs(void);                /* 距最近一帧的时间，用于超时判丢 */

#endif
