/******************************************************************************
 * mcu_link.h  --  USART2 板间链路（F103 侧），全非阻塞
 *
 * 主循环 10ms 任务内必须按序调用：
 *   Link_PollRx();  Track_Update();  Link_TxTick();
 * ISR 只收字节/发送字节，不做组包、不做协议栈，禁止忙等。
 ******************************************************************************/
#ifndef __MCU_LINK_H
#define __MCU_LINK_H

#include "stm32f10x.h"

void    Link_Init(uint32_t baud);
void    Link_PollRx(void);      /* 1) 先收 A/K，更新链路状态 */
void    Link_TxTick(void);      /* 3)+4) 取 yaw 目标 + 组包投递 TX 缓冲（非阻塞） */

void    Link_SetYawTarget(int32_t pos_mrad, int32_t vel_mradps);
int32_t Link_GetYawTarget(void);
uint8_t Link_IsLost(void);
uint8_t Link_GetKCode(void);

#endif
