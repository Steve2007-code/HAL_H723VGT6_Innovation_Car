#ifndef __AUTO_PLACE_H
#define __AUTO_PLACE_H

#include "stm32f10x.h"

/* ==================================================================
 * 自动放料流程：依次夹取 3 个物料，放到车上 1/2/3 号料盘
 *
 *   AutoPlace_Start()   开始（板间命令 <<M P>>）
 *   AutoPlace_Stop()    中止（<<M S>>，或任何单轴手动命令）
 *   AutoPlace_Tick()    主循环 10ms 任务里调用，推进状态机
 *
 * 每步都要等 A/B 停稳、yaw 到容差内才进入下一步。
 * 流程结束停在 A4000 / B20000 / Y0，yaw 保持手动（<<M H>> 才回视觉）。
 * ================================================================== */

void     AutoPlace_Start(void);
void     AutoPlace_Stop(void);
void     AutoPlace_Tick(void);
uint8_t  AutoPlace_IsRunning(void);
uint16_t AutoPlace_GetStep(void);      /* 1~27 正在第几步, 0 = 未运行 */

#endif
