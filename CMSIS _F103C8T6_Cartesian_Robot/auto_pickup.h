#ifndef __AUTO_PICKUP_H
#define __AUTO_PICKUP_H

#include "stm32f10x.h"

/* ==================================================================
 * 任务码驱动的第一批物料夹取（3 个物料依次抓上车上的 1/2/3 号盘）
 *
 *   AutoPickup_Start()   开始（板间命令 <<M C>>）
 *   AutoPickup_Stop()    中止
 *   AutoPickup_Tick()    主循环 10ms 任务里调用
 *
 * 任务码来源: USART3 二维码摄像头发来的 <<flag,156+123+516+231>>
 *   第 1 组 3 位 = 第一批三个物料的颜色与搬运顺序
 *   本模块只解第 1 组; 第 2~4 组暂不处理。
 *
 * 与摄像头的交互(USART1):
 *   F103 -> 摄像头  <<color N>>   N=颜色码, 0=停止追踪
 *   摄像头 -> F103  <<X,dx,Y,dy>> 目标像素误差(原有格式)
 *
 * 颜色码: 1=红  2=黄  3=蓝  4=绿  5=黑  6=浅蓝  0=停止追踪
 *
 * 记住每个料盘上的颜色, 供后续第二组解算使用: AutoPickup_GetSlotColor()
 * ================================================================== */

#define PU_SLOT_NUM   3

void     AutoPickup_Start(void);
void     AutoPickup_Stop(void);
void     AutoPickup_Tick(void);
uint8_t  AutoPickup_IsRunning(void);
uint16_t AutoPickup_GetStep(void);          /* 1~47, 0=未运行 */

/* 视觉跟随测试(不跑流程): color=1~6 发色+开灯+启动跟随; color=0 停止 */
void     AutoPickup_VisionTest(uint8_t color);

/* 第 idx 号料盘(0~2)上放的物料颜色码; 0=空 */
uint8_t  AutoPickup_GetSlotColor(uint8_t idx);
/* 第一批三个物料的颜色码(搬运顺序) */
uint8_t  AutoPickup_GetOrderColor(uint8_t idx);

#endif
