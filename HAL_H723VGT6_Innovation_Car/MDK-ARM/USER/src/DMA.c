#include "DMA.h"
#include "2812LED.h"

/*DMA回调函数*/
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	/*如果是来自与SPI6的回调*/
  if(hspi->Instance == SPI6)
  {
    LED2812_Tx_Busy = 0;//SPI-DMA发送忙标志位清零
  }
}

