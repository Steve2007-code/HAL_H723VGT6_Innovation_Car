#ifndef __2812LED_H__
#define __2812LED_H__

#include "main.h"
#include "DMA.h"

extern SPI_HandleTypeDef hspi6;

/*2812所用数据定义*/
#define BIT0 0xC0
#define BIT1 0xF8//SPI-DMA发送忙标志位 0-空闲 1-忙

///*句柄别名*/
//#define LED2812_SPI &hspi6

/*2812外部颜色存储变量的外部可调用声明*/
__attribute__((at(0x38000000)))
extern uint8_t LED2812_Tx_Bus[28];//24颜色 +4复位
extern uint8_t LED2812_Tx_Busy;//设置颜色业务忙标志位

/*2812颜色设置函数外部声明*/
void WS2812_SetColor_DMA(uint8_t g, uint8_t r, uint8_t b);

#endif
