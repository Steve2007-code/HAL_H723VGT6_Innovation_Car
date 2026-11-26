#ifndef __MCU_CMD_H
#define __MCU_CMD_H

#include "stm32f10x.h"

/* ==================================================================
 * USART3 调试命令通道（与二维码帧 <<flag,...>> 共存，帧内不能含换行）
 *
 *   <<A,<pos>>             A 轴到绝对位置(微步)，默认 3000pps
 *   <<A,<pos>,<pps>>       A 轴到绝对位置 + 指定速度（自动暂停视觉跟随）
 *   <<B,<pos>>             B 轴到绝对位置(微步)
 *   <<B,<pos>,<pps>>
 *   <<Y,<mrad>>            4340 yaw 目标角(0.001rad, -1745~2269 即 -100~130 度)
 *   <<Y,<mrad>,<vel>>      同上 + 限速(0.001rad/s)
 *   <<T,1>> / <<T,0>>      A 轴 恢复 / 暂停 视觉跟随
 *   <<H>>                  yaw 恢复视觉跟随
 *   <<Q>>                  查询 -> 回 <<Q,apos,bpos,ytgt,yact,trun,yman>>
 *   <<G,1>> / <<G,0>>      夹爪 张开(160) / 闭合(50)
 *   <<L,1>> / <<L,0>>      补光 开 / 关
 * ================================================================== */

void Cmd_Exec(const char *body);       /* USART3 字符命令，由 QR_Poll 分派 */

/* 板间 M 帧执行入口（H723 -> F103 下行命令），v/n 为 sub 之后的参数
   sub: 1=A轴 2=B轴 3=yaw 4=A轴跟随 5=yaw回视觉 6=夹爪 7=补光 9=查询 */
void Cmd_ExecSub(int32_t sub, const int32_t *v, uint8_t n);

#endif
