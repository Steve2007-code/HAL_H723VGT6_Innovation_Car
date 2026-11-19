#ifndef __LINK_H__
#define __LINK_H__

#include "main.h"

/* 双机板间链路参数 (与机械臂侧 link_config.h 对应, 单位/限幅必须逐字一致) */
#define LINK_WD_MS          400     /* H723 yaw 命令看门狗: 400ms 无 Y 停发 0x105 */
#define LINK_A_PERIOD_MS    50      /* A 帧上报周期 20Hz (10~20Hz) */
#define LINK_CAN_MIN_MS     10      /* 0x105 最小下发间隔, 防粘包瞬时连发 */
#define LINK_POS_LIM        3142    /* 0.001 rad, +/-pi */
#define LINK_VEL_LIM        3000    /* 0.001 rad/s, +/-3 */
#define LINK_K_COOL_MS      500     /* K,3 / K,4 冷却, 防噪声刷屏 */

/* UART7 句柄与外设由用户配置 (main.c), 此处仅 extern */
extern UART_HandleTypeDef huart7;

void Link_Init(void);               /* 上电初始化协议层, 发一次 K,1 */
void Link_Poll(void);               /* 主循环调用, 与 Debug_Task() 并列 */
void Link_FeedByte(uint8_t c);      /* 仅 UART7 RX 完成回调里调用, 只写环形缓冲 */

#endif
