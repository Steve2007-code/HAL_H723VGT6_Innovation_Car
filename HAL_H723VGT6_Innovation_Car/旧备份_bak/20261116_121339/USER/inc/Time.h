#ifndef __TIME_H__
#define __TIME_H__

#include "main.h"
#include "2812LED.h"

/*定时器句柄声明*/
extern TIM_HandleTypeDef htim4;

/*外部可调用变量声明*/
extern uint32_t Tick;

//---------------外部可调用函数声明---------------//
/*定时器初始化*/
void Time_Init(void);


#endif

