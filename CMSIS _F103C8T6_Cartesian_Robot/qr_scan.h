#ifndef __QR_SCAN_H
#define __QR_SCAN_H

#include "stm32f10x.h"

/* ==================================================================
 * 第二路摄像头（二维码）—— USART3，全双工
 *
 * 接线（交叉 + 共地）：
 *   F103 PB10 (USART3_TX) ──→ 摄像头 RX
 *   F103 PB11 (USART3_RX) ←── 摄像头 TX
 *   GND ──────────────────── GND
 *
 * 协议：摄像头识别到二维码后发
 *   <<flag,132+312+123+231>>
 * 固定 15 字符 = 4 组 3 位数字 + 下标 3/7/11 处 3 个 '+'。
 * 收齐后 g_qr_ready 置 1，主循环取走后自行清零。
 *
 * 说明：USART3 挂 APB1(36MHz)，BRR = 36MHz / baud。
 *       中断优先级 4，与 USART2 相同，低于 TIM2/TIM3/USART1。
 * ================================================================== */

#define QR_CODE_MAX     20

extern char              g_qr_code[QR_CODE_MAX];   /* 例如 "132+312+123+231" */
extern volatile uint8_t  g_qr_ready;               /* 1 = 收到一帧新任务码 */

void QR_Init(uint32_t baud);        /* USART3: PB10=TX, PB11=RX */
void QR_Poll(void);                 /* 主循环调用，解析 <<flag,...>> */

/* 向摄像头发送（TX 方向），用于触发扫码等 */
void QR_SendByte(uint8_t b);
void QR_SendStr(const char *s);

#endif
