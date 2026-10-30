#ifndef __CAN_H__
#define __CAN_H__

#include "main.h"

/*外部句柄声明*/
extern FDCAN_HandleTypeDef hfdcan1;

/*外部可调用函数声明*/
HAL_StatusTypeDef  CAN_Send_Data_Frame(FDCAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint8_t len);

#endif


