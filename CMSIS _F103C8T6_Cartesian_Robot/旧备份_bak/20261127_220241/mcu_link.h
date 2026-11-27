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
/* 回传机械臂状态给 H723: <<S,apos,bpos,ytgt,yact,trun,yman>> */
void    Link_SendStatus(int32_t apos, int32_t bpos, int32_t ytgt, int32_t yact,
                        int32_t trun, int32_t yman, int32_t step,
                        int32_t dx, int32_t dy);
int32_t Link_GetYawTarget(void);
int32_t Link_GetYawPos(void);      /* 4340 实际角度 mrad (来自 A 帧) */
int32_t Link_GetYawVel(void);      /* 4340 实际角速度 mrad/s (来自 A 帧, 用于判断停稳) */
uint8_t Link_IsLost(void);
uint8_t Link_GetKCode(void);

#endif
