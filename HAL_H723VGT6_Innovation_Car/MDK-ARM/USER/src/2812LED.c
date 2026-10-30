#include "2812LED.h"

uint8_t LED2812_Tx_Bus[28];//24颜色 +4复位
uint8_t LED2812_Tx_Busy = 0;//设置颜色业务忙标志位

/**
  * @brief  设置颜色函数
  * @param  g-绿色通道 r-红色通道 b-蓝色通道
  * @note		无
  * @retval 无
  */
void WS2812_SetColor_DMA(uint8_t g, uint8_t r, uint8_t b)
{
  if(LED2812_Tx_Busy) return;
  LED2812_Tx_Busy = 1;

  uint8_t color[3] = {g, r, b};
  uint16_t idx = 0;
  for(uint8_t i = 0; i < 3; i++)
  {
    for(int8_t bit = 7; bit >= 0; bit--)
    {
      if(color[i] & (1 << bit))
        LED2812_Tx_Bus[idx++] = BIT1;
      else
        LED2812_Tx_Bus[idx++] = BIT0;
    }
  }
  // 4字节复位
  LED2812_Tx_Bus[idx++] = 0x00;
  LED2812_Tx_Bus[idx++] = 0x00;
  LED2812_Tx_Bus[idx++] = 0x00;
  LED2812_Tx_Bus[idx++] = 0x00;

  HAL_SPI_Transmit_DMA(&hspi6, LED2812_Tx_Bus, 28);
}
