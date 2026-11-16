#include "Time.h"
#include "Attitude.h"
#include "Chassis.h"
#include "Nav.h"

/**
  * @brief  定时器初始化
  * @param  无
  * @note		无
  * @retval 无
  */
void Time_Init(void)
{
	HAL_TIM_Base_Start_IT(&htim4);
}


/**
  * @brief  定时器中断函数
  * @param  *htim 触发中断的定时器
  * @note		无
  * @retval 无
  */
uint32_t Tick = 0;
uint8_t flag = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM4)
    {
			Tick++;

			/* ---- 5ms 控制节拍 (200Hz): 姿态 -> 里程计 -> 导航 -> 底盘下发 ---- */
			if((Tick % 5) == 0)
			{
				Attitude_Update();
				Odom_Update();
				Nav_Update();
				Chassis_Update();
			}

			/* ---- 1s 心跳灯 ---- */
			if(Tick % 1000 == 0 && flag == 0)
			{
				WS2812_SetColor_DMA(255,0,0);
				flag = 1;
			}
			else if(Tick % 1000 == 0 && flag == 1)
			{
				WS2812_SetColor_DMA(0,255,0);
				flag = 0;
			}
    }
}
