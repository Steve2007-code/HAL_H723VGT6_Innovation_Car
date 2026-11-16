#include "FourMotor_Coordination.h"

//FORWARD            // 前进
//BACKWARD          // 后退
//STRAFE_LEFT       // 左横移（左移）
//STRAFE_RIGHT      // 右横移（右移）
//TURN_LEFT_IN_PLACE    // 原地左转掉头
//TURN_RIGHT_IN_PLACE   // 原地右转掉头
//FORWARD_LEFT      // 左前斜（左前）
//FORWARD_RIGHT     // 右前斜（右前）
//BACKWARD_LEFT     // 左后斜（左后）
//BACKWARD_RIGHT    // 右后斜（右后）
//PIVOT_FRONT_LEFT      // 绕前轴中心左转
//PIVOT_FRONT_RIGHT     // 绕前轴中心右转
//PIVOT_REAR_LEFT       // 绕后轴中心左转
//PIVOT_REAR_RIGHT      // 绕后轴中心右转

/*----------------------------------------------------------*/
//											     	车头
//							左前                           右前      
//              0x01                           0x02              
//      DM_Motor_4310P_Next_ID           DM_Motor_4310P_ID                                      
//  DM_Motor_4310P_Next_StructData    DM_Motor_4310P_StructData  



//              左后                           右后              
//              0x03                           0x04             
//        DM_Motor_4310_ID              DM_Motor_4310_Next_ID  
//    DM_Motor_4310_StructData      DM_Motor_4310_Next_StructData
/*----------------------------------------------------------*/

/**
  * @brief  底盘以指定速度   前进   指定距离
  * @param  vel-电机速度 dist-电机距离
  * @note		FORWARD-前进
  * @retval 无  
	*/
void FMC_FORWARD(float vel, float dist)
{
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310P_Next_StructData,vel,dist);
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310P_StructData,vel,-dist);
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310_StructData,vel,dist);
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310_Next_StructData,vel,-dist);
}

/**
  * @brief  底盘以指定速度   后退   指定距离
  * @param  vel-电机速度 dist-电机距离
  * @note		FORWARD-后退
  * @retval 无
  */
void FMC_BACKWARD(float vel, float dist)
{
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310P_Next_StructData,vel,-dist);
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310P_StructData,vel,dist);
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310_StructData,vel,-dist);
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310_Next_StructData,vel,dist);
}

/**
  * @brief  底盘以指定速度   左横移   指定距离
  * @param  vel-电机速度 dist-电机距离
  * @note		STRAFE_LEFT-左横移
  * @retval 无
  */
void FMC_STRAFE_LEFT(float vel, float dist)
{
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310P_Next_StructData,vel,-dist);
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310P_StructData,vel,-dist);
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310_StructData,vel,dist);
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310_Next_StructData,vel,dist);
}

/**
  * @brief  底盘以指定速度   右横移   指定距离
  * @param  vel-电机速度 dist-电机距离
  * @note		STRAFE_RIGHT-右横移
  * @retval 无
  */
void FMC_STRAFE_RIGHT(float vel, float dist)
{
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310P_Next_StructData,vel,dist);
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310P_StructData,vel,dist);
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310_StructData,vel,-dist);
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310_Next_StructData,vel,-dist);
}

/**
  * @brief  底盘以指定速度   左转掉头   指定距离
  * @param  vel-电机速度 dist-电机距离
  * @note		TURN_LEFT_IN_PLACE-左转掉头
  * @retval 无
  */
void FMC_TURN_LEFT_IN_PLACE(float vel, float dist)
{
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310P_Next_StructData,vel,-dist);
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310P_StructData,vel,-dist);
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310_StructData,vel,-dist);
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310_Next_StructData,vel,-dist);
}

/**
  * @brief  底盘以指定速度   右转掉头   指定距离
  * @param  vel-电机速度 dist-电机距离
  * @note		TURN_RIGHT_IN_PLACE-右转掉头
  * @retval 无
  */
void FMC_TURN_RIGHT_IN_PLACE(float vel, float dist)
{
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310P_Next_StructData,vel,dist);
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310P_StructData,vel,dist);
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310_StructData,vel,dist);
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310_Next_StructData,vel,dist);
}

/**
  * @brief  底盘以指定速度指定距离   左前斜   指定距离
  * @param  vel-电机速度 dist-电机距离
  * @note		FORWARD_LEFT-左前斜
  * @retval 无
  */
void FMC_FORWARD_LEFT(float vel, float dist)
{
//	DM_CAN_Motor_Position_Speed(&DM_Motor_4310P_Next_StructData,vel,dist);
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310P_StructData,vel,-dist);
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310_StructData,vel,dist);
//	DM_CAN_Motor_Position_Speed(&DM_Motor_4310_Next_StructData,vel,dist);
}

/**
  * @brief  底盘以指定速度指定距离   右前斜   指定距离
  * @param  vel-电机速度 dist-电机距离
  * @note		FORWARD_RIGHT-右前斜
  * @retval 无
  */
void FMC_FORWARD_RIGHT(float vel, float dist)
{
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310P_Next_StructData,vel,-dist);
//	DM_CAN_Motor_Position_Speed(&DM_Motor_4310P_StructData,vel,dist);
//	DM_CAN_Motor_Position_Speed(&DM_Motor_4310_StructData,vel,dist);
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310_Next_StructData,vel,dist);
}

/**
  * @brief  底盘以指定速度指定距离   左后斜   指定距离
  * @param  vel-电机速度 dist-电机距离
  * @note		BACKWARD_LEFT-左后斜
  * @retval 无
  */
void FMC_BACKWARD_LEFT(float vel, float dist)
{
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310P_Next_StructData,vel,-dist);
//	DM_CAN_Motor_Position_Speed(&DM_Motor_4310P_StructData,vel,dist);
//	DM_CAN_Motor_Position_Speed(&DM_Motor_4310_StructData,vel,dist);
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310_Next_StructData,vel,dist);
}

/**
  * @brief  底盘以指定速度指定距离   右后斜   指定距离
  * @param  vel-电机速度 dist-电机距离
  * @note		BACKWARD_RIGHT-右后斜
  * @retval 无
  */
void FMC_BACKWARD_RIGHT(float vel, float dist)
{
//	DM_CAN_Motor_Position_Speed(&DM_Motor_4310P_Next_StructData,vel,dist);
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310P_StructData,vel,-dist);
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310_StructData,vel,dist);
//	DM_CAN_Motor_Position_Speed(&DM_Motor_4310_Next_StructData,vel,dist);
}

/**
  * @brief  底盘以指定速度指定距离   绕前轴中心左转   指定距离
  * @param  vel-电机速度 dist-电机距离
  * @note		PIVOT_FRONT_LEFT-绕前轴中心左转
  * @retval 无
  */
void FMC_PIVOT_FRONT_LEFT(float vel, float dist)
{
//	DM_CAN_Motor_Position_Speed(&DM_Motor_4310P_Next_StructData,vel,dist);
//	DM_CAN_Motor_Position_Speed(&DM_Motor_4310P_StructData,vel,dist);
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310_StructData,vel,dist);
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310_Next_StructData,vel,dist);
}

/**
  * @brief  底盘以指定速度指定距离   绕前轴中心右转   指定距离
  * @param  vel-电机速度 dist-电机距离
  * @note		PIVOT_FRONT_RIGHT-绕前轴中心右转
  * @retval 无
  */
void FMC_PIVOT_FRONT_RIGHT(float vel, float dist)
{
//	DM_CAN_Motor_Position_Speed(&DM_Motor_4310P_Next_StructData,vel,dist);
//	DM_CAN_Motor_Position_Speed(&DM_Motor_4310P_StructData,vel,dist);
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310_StructData,vel,dist);
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310_Next_StructData,vel,dist);
}

/**
  * @brief  底盘以指定速度指定距离   绕后轴中心左转   指定距离
  * @param  vel-电机速度 dist-电机距离
  * @note		PIVOT_REAR_LEFT-绕后轴中心左转
  * @retval 无
  */
void FMC_PIVOT_REAR_LEFT(float vel, float dist)
{
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310P_Next_StructData,vel,dist);
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310P_StructData,vel,dist);
//	DM_CAN_Motor_Position_Speed(&DM_Motor_4310_StructData,vel,dist);
//	DM_CAN_Motor_Position_Speed(&DM_Motor_4310_Next_StructData,vel,dist);
}

/**
  * @brief  底盘以指定速度指定距离   绕后轴中心右转   指定距离
  * @param  vel-电机速度 dist-电机距离
  * @note		PIVOT_REAR_RIGHT-绕后轴中心右转
  * @retval 无
  */
void FMC_PIVOT_REAR_RIGHT(float vel, float dist)
{
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310P_Next_StructData,vel,dist);
	DM_CAN_Motor_Position_Speed(&DM_Motor_4310P_StructData,vel,dist);
//	DM_CAN_Motor_Position_Speed(&DM_Motor_4310_StructData,vel,dist);
//	DM_CAN_Motor_Position_Speed(&DM_Motor_4310_Next_StructData,vel,dist);
}

