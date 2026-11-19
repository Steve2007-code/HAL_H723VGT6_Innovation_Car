/******************************************************************************
 * frame.h  --  通用 <<...>> 成帧层，供 USART1(摄像头) 与 USART2(板间) 各自实例化
 *
 * FrameRx_t 必须每个数据流一个实例，禁止共用静态状态机，否则会串帧。
 ******************************************************************************/
#ifndef __FRAME_H
#define __FRAME_H

#include "stm32f10x.h"

#define FR_RX_SZ     128        /* 必须是 2 的幂 */
#define FR_BODY_SZ   32
#define FR_MAX_VALS  4

typedef struct {
    volatile uint8_t  rx[FR_RX_SZ];
    volatile uint16_t wr;
    uint16_t          rd;
    uint8_t           st;
    uint8_t           n;
    char              body[FR_BODY_SZ];
} FrameRx_t;

void    Frame_Init(FrameRx_t *q);
void    Frame_FeedByte(FrameRx_t *q, uint8_t c);                 /* 仅 RX 中断调用 */
uint8_t Frame_Poll(FrameRx_t *q, char *out, uint8_t cap);        /* 主循环调用，1=取到一帧 */
int     parse_i(const char **pp, int32_t *v);
uint8_t parse_cmd_ints(const char *body, char *cmd, int32_t *vals, uint8_t max_vals);

#endif
