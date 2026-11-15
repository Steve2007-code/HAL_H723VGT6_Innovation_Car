#include "CAN.h"
#include "DM_BLDC_Motor.h"
#include "DM_IMU_L1_V1.0.h"
#include "stdio.h"

/**
  * @brief  CAN发送一帧报文
  * @param  id-目标设备的id data-发送的数据 len-发送的数据长度
  * @note		无
  * @retval 无
  */
HAL_StatusTypeDef  CAN_Send_Data_Frame(FDCAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef txHeader;
    txHeader.Identifier = id;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    if(len == 0) txHeader.DataLength = FDCAN_DLC_BYTES_0;
    else if(len == 1) txHeader.DataLength = FDCAN_DLC_BYTES_1;
    else if(len == 2) txHeader.DataLength = FDCAN_DLC_BYTES_2;
    else if(len == 3) txHeader.DataLength = FDCAN_DLC_BYTES_3;
    else if(len == 4) txHeader.DataLength = FDCAN_DLC_BYTES_4;
    else if(len == 5) txHeader.DataLength = FDCAN_DLC_BYTES_5;
    else if(len == 6) txHeader.DataLength = FDCAN_DLC_BYTES_6;
    else if(len == 7) txHeader.DataLength = FDCAN_DLC_BYTES_7;
    else txHeader.DataLength = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;

    return HAL_FDCAN_AddMessageToTxFifoQ(hcan, &txHeader, data);
}

/**
  * @brief  CAN回调中断
  * @param  无
  * @note		无
  * @retval 无
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    // 读出收到的报文
//    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData);
		while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0)
	{
		if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
		{
			if(hfdcan->Instance == FDCAN1)
			{
				//把读出来的电机回传数据放进解析函数里面解析回传数据
				Motor_UpdateData(rxData, rxHeader.Identifier);
			}
			else if(hfdcan->Instance == FDCAN3)
			{
				//把读出来的IMU数据放进解析函数里面解析回传数据
				IMU_UpdateData(rxData);
			}
		}
	}
    // rxHeader.Identifier 是CAN ID，rxData[0]~rxData[7]是8字节数据
  }
}



