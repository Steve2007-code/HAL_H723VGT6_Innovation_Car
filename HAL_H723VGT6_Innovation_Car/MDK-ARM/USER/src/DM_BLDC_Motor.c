#include <string.h>
#include <stdio.h>
#include "DM_BLDC_Motor.h"
#include "can.h"

/*CAN数据帧定义*/
uint8_t DM_Mototr_Enable[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};//电机使能
uint8_t DM_Mototr_Disable[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};//电机失能

/*达妙电机MIT模式下的结构体定义*/
DM_Motor_Data DM_Motor_4310_StructData;//4310参数定义结构体
DM_Motor_Data DM_Motor_4310P_StructData;//4340参数定义结构体
DM_Motor_Data DM_Motor_4310_Next_StructData;//4310next参数定义结构体
DM_Motor_Data DM_Motor_4310P_Next_StructData;//4340next参数定义结构体
DM_Motor_Data DM_Motor_4340P_Yaw_StructData;//4340Yaw轴参数定义结构体

/*
  * @brief  DM电机结构体初始化
  * @param  无
  * @note		代码初始化时务必进行初始化
  * @retval 无
  */
void DM_CAN_Motor_Init(void)
{
	/*4310数据初始化*/
	DM_Motor_4310_StructData.hcan = &hfdcan1;
	DM_Motor_4310_StructData.ID = DM_Motor_4310_ID;
	DM_Motor_4310_StructData.Kd = 2;
	DM_Motor_4310_StructData.Kd_max = 5;
	DM_Motor_4310_StructData.Kd_min = 0;
	DM_Motor_4310_StructData.Kp = 20;
	DM_Motor_4310_StructData.Kp_max = 500;
	DM_Motor_4310_StructData.Kp_min = 0;
	DM_Motor_4310_StructData.Position = 0;
	DM_Motor_4310_StructData.Position_max = 300;
	DM_Motor_4310_StructData.Position_min = -300;
	DM_Motor_4310_StructData.Torque = 0;
	DM_Motor_4310_StructData.Torque_max = 10;
	DM_Motor_4310_StructData.Torque_min = -10;
	DM_Motor_4310_StructData.Velocity = 0;
	DM_Motor_4310_StructData.Velocity_max = 30;
	DM_Motor_4310_StructData.Velocity_min = -30;
	
	DM_Motor_4310_StructData.BackID = DM_Motor_4310_BackID;
	
	/*4310P数据初始化*/
	DM_Motor_4310P_StructData.hcan = &hfdcan1;
	DM_Motor_4310P_StructData.ID = DM_Motor_4310P_ID;
	DM_Motor_4310P_StructData.Kd = 2;
	DM_Motor_4310P_StructData.Kd_max = 5;
	DM_Motor_4310P_StructData.Kd_min = 0;
	DM_Motor_4310P_StructData.Kp = 20;
	DM_Motor_4310P_StructData.Kp_max = 500;
	DM_Motor_4310P_StructData.Kp_min = 0;
	DM_Motor_4310P_StructData.Position = 0;
	DM_Motor_4310P_StructData.Position_max = 300;
	DM_Motor_4310P_StructData.Position_min = -300;
	DM_Motor_4310P_StructData.Torque = 0;
	DM_Motor_4310P_StructData.Torque_max = 10;
	DM_Motor_4310P_StructData.Torque_min = -10;
	DM_Motor_4310P_StructData.Velocity = 0;
	DM_Motor_4310P_StructData.Velocity_max = 30;
	DM_Motor_4310P_StructData.Velocity_min = -30;
	
	DM_Motor_4310P_StructData.BackID = DM_Motor_4310P_BackID;
	
	/*4310_Next数据初始化*/
	DM_Motor_4310_Next_StructData.hcan = &hfdcan1;
	DM_Motor_4310_Next_StructData.ID = DM_Motor_4310_Next_ID;
	DM_Motor_4310_Next_StructData.Kd = 2;
	DM_Motor_4310_Next_StructData.Kd_max = 5;
	DM_Motor_4310_Next_StructData.Kd_min = 0;
	DM_Motor_4310_Next_StructData.Kp = 20;
	DM_Motor_4310_Next_StructData.Kp_max = 500;
	DM_Motor_4310_Next_StructData.Kp_min = 0;
	DM_Motor_4310_Next_StructData.Position = 0;
	DM_Motor_4310_Next_StructData.Position_max = 300;
	DM_Motor_4310_Next_StructData.Position_min = -300;
	DM_Motor_4310_Next_StructData.Torque = 0;
	DM_Motor_4310_Next_StructData.Torque_max = 10;
	DM_Motor_4310_Next_StructData.Torque_min = -10;
	DM_Motor_4310_Next_StructData.Velocity = 0;
	DM_Motor_4310_Next_StructData.Velocity_max = 30;
	DM_Motor_4310_Next_StructData.Velocity_min = -30;
	
	DM_Motor_4310_Next_StructData.BackID = DM_Motor_4310_Next_BackID;
	
	/*4310P_Next数据初始化*/
	DM_Motor_4310P_Next_StructData.hcan = &hfdcan1;
	DM_Motor_4310P_Next_StructData.ID = DM_Motor_4310P_Next_ID;
	DM_Motor_4310P_Next_StructData.Kd = 2;
	DM_Motor_4310P_Next_StructData.Kd_max = 5;
	DM_Motor_4310P_Next_StructData.Kd_min = 0;
	DM_Motor_4310P_Next_StructData.Kp = 20;
	DM_Motor_4310P_Next_StructData.Kp_max = 500;
	DM_Motor_4310P_Next_StructData.Kp_min = 0;
	DM_Motor_4310P_Next_StructData.Position = 0;
	DM_Motor_4310P_Next_StructData.Position_max = 300;
	DM_Motor_4310P_Next_StructData.Position_min = -300;
	DM_Motor_4310P_Next_StructData.Torque = 0;
	DM_Motor_4310P_Next_StructData.Torque_max = 10;
	DM_Motor_4310P_Next_StructData.Torque_min = -10;
	DM_Motor_4310P_Next_StructData.Velocity = 0;
	DM_Motor_4310P_Next_StructData.Velocity_max = 30;
	DM_Motor_4310P_Next_StructData.Velocity_min = -30;
	
	DM_Motor_4310P_Next_StructData.BackID = DM_Motor_4310P_Next_BackID;
	
	/*4340Yaw轴数据初始化*/
	DM_Motor_4340P_Yaw_StructData.hcan = &hfdcan1;
	DM_Motor_4340P_Yaw_StructData.ID = DM_Motor_4340_Yaw_ID;
	DM_Motor_4340P_Yaw_StructData.Kd = 2;
	DM_Motor_4340P_Yaw_StructData.Kd_max = 5;
	DM_Motor_4340P_Yaw_StructData.Kd_min = 0;
	DM_Motor_4340P_Yaw_StructData.Kp = 20;
	DM_Motor_4340P_Yaw_StructData.Kp_max = 500;
	DM_Motor_4340P_Yaw_StructData.Kp_min = 0;
	DM_Motor_4340P_Yaw_StructData.Position = 0;
	DM_Motor_4340P_Yaw_StructData.Position_max = 12;
	DM_Motor_4340P_Yaw_StructData.Position_min = -12;
	DM_Motor_4340P_Yaw_StructData.Torque = 0;
	DM_Motor_4340P_Yaw_StructData.Torque_max = 10;
	DM_Motor_4340P_Yaw_StructData.Torque_min = -10;
	DM_Motor_4340P_Yaw_StructData.Velocity = 0;
	DM_Motor_4340P_Yaw_StructData.Velocity_max = 30;
	DM_Motor_4340P_Yaw_StructData.Velocity_min = -30;
	
	DM_Motor_4340P_Yaw_StructData.BackID = DM_Motor_4340_Yaw_BackID;
}

/*
  * @brief  CAN发送一帧数据给电机
  * @param  *hcan CAN外设句柄 - id 电机id 可以是DM_Motor_4340或DM_Motor_4310 - *data:8位数据帧 - len 数据帧长度
  * @note		无
  * @retval HAL返回的CAN发送状态
  */
HAL_StatusTypeDef DM_CAN_Motor_Send_Data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint8_t len)
{
	return CAN_Send_Data_Frame(hcan, id, data, len);
}

/*
  * @brief  CAM发送电机使/失能
  * @param  id 电机id，可以是 DM_Motor_4340_ID 或 DM_Motor_4310_ID DATA DM_Mototr_Enable DM_Mototr_Disable
  * @note		无
  * @retval HAL返回的CAN发送状态
  */
HAL_StatusTypeDef DM_CAN_Motor_SetEnable(uint16_t id, uint8_t *data)
{
	if(data[7] == 0xFC)//使能
	{
		return CAN_Send_Data_Frame(&hfdcan1, id, DM_Mototr_Enable, 8);
	}
	else if(data[7] == 0xFD)//失能
	{
		return CAN_Send_Data_Frame(&hfdcan1, id, DM_Mototr_Disable, 8);
	}
	return HAL_ERROR;
}

/*
  * @brief  int16转换16位无符号数
  * @param  x 目标值 xmin 可调整的最小值 xmax 可调整的最大值
  * @note		字段映射为整数
  * @retval 无
  */
static uint16_t DM_Motor_16_Data_Frame(float x, float xmin, float xmax)
{
	if(x <= xmin){x = xmin;}
	if(x >= xmax){x = xmax;}
	uint16_t value = (uint16_t)((x - xmin) / (xmax - xmin) * 65535.0f + 0.5f);
	return value;
}
/*
  * @brief  int16转换12位无符号数
  * @param  x 目标值 xmin 可调整的最小值 xmax 可调整的最大值
  * @note		字段映射为整数
  * @retval 无
  */
static uint16_t DM_Motor_12_Data_Frame(float x, float xmin, float xmax)
{
	if(x <= xmin){x = xmin;}
	if(x >= xmax){x = xmax;}
	uint16_t value = (uint16_t)((x - xmin) / (xmax - xmin) * 4095.0f + 0.5f);
	return value;
}


static float DM_Raw_To_Value(uint16_t raw, float xmin, float xmax, uint8_t bits)
{
    float full = (float)(((uint32_t)1u << bits) - 1u);
    if ((float)raw > full) raw = (uint16_t)full;
    return (float)raw / full * (xmax - xmin) + xmin;
}

/*
  * @brief  CAN发送电机旋转数据帧(弧度制)(定位置控制)(TIM报文)
  * @param  id 电机id，可以是 DM_Motor_4310_StructData 或 DM_Motor_4340_StructData
	* @param	Position 目标位置 Velocity 目标速度 Kp 比例 Kd 微分 Torque 扭矩
  * @note		无
  * @retval HAL返回的CAN发送状态
  */
HAL_StatusTypeDef DM_CAN_Motor_Rotation_Rad(DM_Motor_Data *ax, float Position)
{
	uint8_t Data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	ax ->Position = Position;
	uint16_t Position_raw = DM_Motor_16_Data_Frame(ax ->Position, ax ->Position_min, ax ->Position_max);
	uint16_t Velocity_raw = DM_Motor_12_Data_Frame(ax ->Velocity, ax ->Velocity_min, ax ->Velocity_max);
	uint16_t Kp_raw = DM_Motor_12_Data_Frame(ax ->Kp, ax ->Kp_min, ax ->Kp_max);
	uint16_t Kd_raw = DM_Motor_12_Data_Frame(ax ->Kd, ax ->Kd_min, ax ->Kd_max);
	uint16_t Torque_raw = DM_Motor_12_Data_Frame(ax ->Torque, ax ->Torque_min, ax ->Torque_max);
	
	Data[0] = (uint8_t)(Position_raw >> 8);
  Data[1] = (uint8_t)(Position_raw & 0xFF);
  Data[2] = (uint8_t)(Velocity_raw >> 4);
  Data[3] = (uint8_t)(((Velocity_raw & 0x0F) << 4) | (Kp_raw >> 8));
  Data[4] = (uint8_t)(Kp_raw & 0xFF);
  Data[5] = (uint8_t)(Kd_raw >> 4);
  Data[6] = (uint8_t)(((Kd_raw & 0x0F) << 4) | (Torque_raw >> 8));
  Data[7] = (uint8_t)(Torque_raw & 0xFF);
	
	return CAN_Send_Data_Frame(ax ->hcan, ax ->ID, Data, 8);
}

/*
  * @brief  CAN发送电机旋转数据帧(角度制)(定位置控制)(TIM报文)
  * @param  ax 电机结构体指针 Position目标位置
	* @param	Position 目标位置 Velocity 目标速度 Kp 比例 Kd 微分 Torque 扭矩
  * @note		无
  * @retval HAL返回的CAN发送状态
  */
HAL_StatusTypeDef DM_CAN_Motor_Rotation_deg(DM_Motor_Data *ax, float Position)
{
	uint8_t Data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	ax ->Position = Position * PI / 180.0f;
	uint16_t Position_raw = DM_Motor_16_Data_Frame(ax ->Position, ax ->Position_min, ax ->Position_max);
	uint16_t Velocity_raw = DM_Motor_12_Data_Frame(ax ->Velocity, ax ->Velocity_min, ax ->Velocity_max);
	uint16_t Kp_raw = DM_Motor_12_Data_Frame(ax ->Kp, ax ->Kp_min, ax ->Kp_max);
	uint16_t Kd_raw = DM_Motor_12_Data_Frame(ax ->Kd, ax ->Kd_min, ax ->Kd_max);
	uint16_t Torque_raw = DM_Motor_12_Data_Frame(ax ->Torque, ax ->Torque_min, ax ->Torque_max);
	
	Data[0] = (uint8_t)(Position_raw >> 8);
  Data[1] = (uint8_t)(Position_raw & 0xFF);
  Data[2] = (uint8_t)(Velocity_raw >> 4);
  Data[3] = (uint8_t)(((Velocity_raw & 0x0F) << 4) | (Kp_raw >> 8));
  Data[4] = (uint8_t)(Kp_raw & 0xFF);
  Data[5] = (uint8_t)(Kd_raw >> 4);
  Data[6] = (uint8_t)(((Kd_raw & 0x0F) << 4) | (Torque_raw >> 8));
  Data[7] = (uint8_t)(Torque_raw & 0xFF);
	
	return CAN_Send_Data_Frame(ax ->hcan, ax ->ID, Data, 8);
}

/*
  * @brief  CAN发送电机旋转数据帧(弧度制)(定速度控制)(速度模式报文)
  * @param  ax 电机结构体指针 vel 目标角速度
  * @note		无
  * @retval HAL返回的CAN发送状态
  */
HAL_StatusTypeDef DM_CAN_Motor_Velocity_Rad(DM_Motor_Data *ax, float vel)
{
	uint8_t  Data[4] = {0x00, 0x00, 0x00, 0x00};
	uint32_t *p_vel = (uint32_t *)&vel;

	ax->Velocity = vel;

	Data[0] = (*p_vel >> 0)  & 0xFF;
	Data[1] = (*p_vel >> 8)  & 0xFF;
	Data[2] = (*p_vel >> 16) & 0xFF;
	Data[3] = (*p_vel >> 24) & 0xFF;

	return CAN_Send_Data_Frame(ax->hcan, (0x200 + ax->ID), Data, 4);
}


/*
  * @brief  CAN发送电机MIT(TIM报文)
  * @param  ax 电机结构体指针 vel 目标角速度 pos 电机位置 - kp - ki - kd - tor 转矩
  * @note		无
  * @retval HAL返回的CAN发送状态
  */
HAL_StatusTypeDef DM_CAN_Motor_MIT(DM_Motor_Data *ax, float vel, float pos, float kp, float kd, float tor)
{
	uint8_t  Data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint16_t Position_raw, Velocity_raw, Kp_raw, Kd_raw, Torque_raw;

	ax->Velocity = vel;//速度
	ax->Position = pos;//位置
	ax->Torque = tor;//转矩
	ax->Kp = kp;
	ax->Kd = kd;

	Position_raw = DM_Motor_16_Data_Frame(ax->Position, ax->Position_min, ax->Position_max);
	Velocity_raw = DM_Motor_12_Data_Frame(ax->Velocity, ax->Velocity_min, ax->Velocity_max);
	Kp_raw = DM_Motor_12_Data_Frame(ax->Kp, ax->Kp_min, ax->Kp_max);
	Kd_raw = DM_Motor_12_Data_Frame(ax->Kd, ax->Kd_min, ax->Kd_max);
	Torque_raw = DM_Motor_12_Data_Frame(ax->Torque, ax->Torque_min, ax->Torque_max);

	Data[0] = (uint8_t)(Position_raw >> 8);
	Data[1] = (uint8_t)(Position_raw & 0xFF);
	Data[2] = (uint8_t)(Velocity_raw >> 4);
	Data[3] = (uint8_t)(((Velocity_raw & 0x0F) << 4) | (Kp_raw >> 8));
	Data[4] = (uint8_t)(Kp_raw & 0xFF);
	Data[5] = (uint8_t)(Kd_raw >> 4);
	Data[6] = (uint8_t)(((Kd_raw & 0x0F) << 4) | (Torque_raw >> 8));
	Data[7] = (uint8_t)(Torque_raw & 0xFF);

	return CAN_Send_Data_Frame(ax->hcan, ax->ID, Data, 8);
}

/*
  * @brief  位置速度模式控制电机
  * @param  ax 电机结构体指针 vel 目标角速度 pos 电机位置
  * @note		无
  * @retval HAL返回的CAN发送状态
  */
HAL_StatusTypeDef DM_CAN_Motor_Position_Speed(DM_Motor_Data *ax, float vel, float pos)
{
	uint8_t  Data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint32_t *p_pos = (uint32_t *)&pos;
  uint32_t *p_vel = (uint32_t *)&vel;

	ax->Velocity = vel;//速度
	ax->Position = pos;//位置
	
	// p_des 4字节 float，小端 D0 D1 D2 D3
  Data[0] = (*p_pos >> 0)  & 0xFF;
  Data[1] = (*p_pos >> 8)  & 0xFF;
  Data[2] = (*p_pos >> 16) & 0xFF;
  Data[3] = (*p_pos >> 24) & 0xFF;

  // v_des 4字节 float，小端 D4 D5 D6 D7
  Data[4] = (*p_vel >> 0)  & 0xFF;
  Data[5] = (*p_vel >> 8)  & 0xFF;
  Data[6] = (*p_vel >> 16) & 0xFF;
  Data[7] = (*p_vel >> 24) & 0xFF;

	return CAN_Send_Data_Frame(ax->hcan, (0x100 + ax->ID), Data, 8);
}

/**
  * @brief  4310电机回传数据解析
  * @param  *ax IMU数据结构体
  * @note		无
  * @retval 无
  */
void DM_IMU_Update4310P(DM_Motor_Data *pData)
{
	pData->ID_or_ERR = pData->UpData[0];
	pData->POS      = DM_Raw_To_Value(((uint16_t)pData->UpData[1] << 8) | pData->UpData[2],
																		pData->Position_min, pData->Position_max, 16);
	pData->VEL      = DM_Raw_To_Value(((uint16_t)pData->UpData[3] << 4) | ((uint16_t)pData->UpData[4] >> 4),
																		pData->Velocity_min, pData->Velocity_max, 12);
	pData->T        = DM_Raw_To_Value((((uint16_t)pData->UpData[4] & 0x0FU) << 8) | pData->UpData[5],
																		pData->Torque_min, pData->Torque_max, 12);
	pData->T_MOS    = (float)(int8_t)pData->UpData[6];
	pData->TRotorl  = (float)(int8_t)pData->UpData[7];
}

/**
  * @brief  printf输出电机数据
  * @param  * pData 要输出的电机结构体 值可以是 DM_Motor_4310P_StructData DM_Motor_4310_StructData
	* @param															DM_Motor_4310_Next_StructData DM_Motor_4310P_Next_StructData
  * @note		无
  * @retval 无
  */
void Motor_Printf_UpdateData(DM_Motor_Data* pData)
{
	printf("ID:%#X,位置:%f,速度:%f,扭矩:%f,MOS温度:%f,线圈温度:%f\r\n",
																			pData ->ID_or_ERR,pData ->POS
																			,pData ->VEL,pData ->T,
																			pData ->T_MOS,pData ->TRotorl);
}

/**
  * @brief  printf输出4电机数据
  * @param  无
  * @note		无
  * @retval 无
  */
void Motor_Printf_Dour_UpdateData(void)
{
	HAL_Delay(1);
	printf("ID:%#X,位置:%f,速度:%f,扭矩:%f,MOS温度:%f,线圈温度:%f\r\n",
					DM_Motor_4310P_Next_StructData.ID_or_ERR,DM_Motor_4310P_Next_StructData.POS
					,DM_Motor_4310P_Next_StructData.VEL,DM_Motor_4310P_Next_StructData.T,
					DM_Motor_4310P_Next_StructData.T_MOS,DM_Motor_4310P_Next_StructData.TRotorl);
	printf("ID:%#X,位置:%f,速度:%f,扭矩:%f,MOS温度:%f,线圈温度:%f\r\n",
					DM_Motor_4310P_StructData.ID_or_ERR,DM_Motor_4310P_StructData.POS
					,DM_Motor_4310P_StructData.VEL,DM_Motor_4310P_StructData.T,
					DM_Motor_4310P_StructData.T_MOS,DM_Motor_4310P_StructData.TRotorl);
	printf("ID:%#X,位置:%f,速度:%f,扭矩:%f,MOS温度:%f,线圈温度:%f\r\n",
					DM_Motor_4310_StructData.ID_or_ERR,DM_Motor_4310_StructData.POS
					,DM_Motor_4310_StructData.VEL,DM_Motor_4310_StructData.T,
					DM_Motor_4310_StructData.T_MOS,DM_Motor_4310_StructData.TRotorl);
	printf("ID:%#X,位置:%f,速度:%f,扭矩:%f,MOS温度:%f,线圈温度:%f\r\n",
					DM_Motor_4310_Next_StructData.ID_or_ERR,DM_Motor_4310_Next_StructData.POS
					,DM_Motor_4310_Next_StructData.VEL,DM_Motor_4310_Next_StructData.T,
					DM_Motor_4310_Next_StructData.T_MOS,DM_Motor_4310_Next_StructData.TRotorl);
}

/**
  * @brief  电机回传一键测试函数
  * @param  无
  * @note		无
  * @retval 无
  */
void Motor_BackerData_Text(void)
{
		DM_CAN_Motor_Rotation_deg(&DM_Motor_4310_StructData,10.0f);
		DM_CAN_Motor_Rotation_deg(&DM_Motor_4310P_StructData,10.0f);
		DM_CAN_Motor_Rotation_deg(&DM_Motor_4310_Next_StructData,10.0f);
		DM_CAN_Motor_Rotation_deg(&DM_Motor_4310P_Next_StructData,10.0f);
		HAL_Delay(1);
		Motor_Printf_UpdateData(&DM_Motor_4310_StructData);
		Motor_Printf_UpdateData(&DM_Motor_4310P_StructData);
		Motor_Printf_UpdateData(&DM_Motor_4310_Next_StructData);
		Motor_Printf_UpdateData(&DM_Motor_4310P_Next_StructData);
		printf("---------------------------------\r\n");
		HAL_Delay(1000);
		DM_CAN_Motor_Rotation_deg(&DM_Motor_4310_StructData,0.0f);
		DM_CAN_Motor_Rotation_deg(&DM_Motor_4310P_StructData,0.0f);
		DM_CAN_Motor_Rotation_deg(&DM_Motor_4310_Next_StructData,0.0f);
		DM_CAN_Motor_Rotation_deg(&DM_Motor_4310P_Next_StructData,0.0f);
		HAL_Delay(1);
		Motor_Printf_UpdateData(&DM_Motor_4310_StructData);
		Motor_Printf_UpdateData(&DM_Motor_4310P_StructData);
		Motor_Printf_UpdateData(&DM_Motor_4310_Next_StructData);
		Motor_Printf_UpdateData(&DM_Motor_4310P_Next_StructData);
		printf("---------------------------------\r\n");
		HAL_Delay(1000);
}

/**
  * @brief  解收电机数据之后下发到对应的解析函数
  * @param  *pData CAN接收到的数据
  * @note		无
  * @retval 无
  */
void Motor_UpdateData(uint8_t* pData, uint8_t Back_ID)
{
	//电机回传数据解析分配函数

	switch(pData[0])
	{
		case DM_Motor_4310_BackID:
			memcpy(DM_Motor_4310_StructData.UpData, pData, 8);
			DM_IMU_Update4310P(&DM_Motor_4310_StructData);
			break;
		case DM_Motor_4310P_BackID:
			memcpy(DM_Motor_4310P_StructData.UpData, pData, 8);
			DM_IMU_Update4310P(&DM_Motor_4310P_StructData);
			break;
		case DM_Motor_4310_Next_BackID:
			memcpy(DM_Motor_4310_Next_StructData.UpData, pData, 8);
			DM_IMU_Update4310P(&DM_Motor_4310_Next_StructData);
			break;
		case DM_Motor_4310P_Next_BackID:
			memcpy(DM_Motor_4310P_Next_StructData.UpData, pData, 8);
			DM_IMU_Update4310P(&DM_Motor_4310P_Next_StructData);
			break;
		case DM_Motor_4340_Yaw_BackID:
			memcpy(DM_Motor_4340P_Yaw_StructData.UpData, pData, 8);
			DM_IMU_Update4310P(&DM_Motor_4340P_Yaw_StructData);
			break;
	}
	//IMU回传数据解析分配函数
	if(Back_ID == 0xFF)
	{
    ;
	}
}
