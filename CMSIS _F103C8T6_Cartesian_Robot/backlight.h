#ifndef __BACKLIGHT_H
#define __BACKLIGHT_H

#include "stm32f10x.h"

/* ==================================================================
 * LED 补光开关（PB12，推挽输出，高电平亮）
 *
 * 注意：GPIO 只提供电平，不要用它直接带 LED 大电流。
 *       实际接线应为  PB12 -> 1k 电阻 -> NPN/MOSFET 栅极，
 *       LED 由 5V/12V 独立电源供电，MOSFET 漏极串在 LED 负极回路里，
 *       整个系统必须共地。
 * ================================================================== */

void    Backlight_Init(void);
void    Backlight_Set(uint8_t on);      /* 1 = 点亮，0 = 熄灭 */
void    Backlight_Toggle(void);
uint8_t Backlight_Get(void);

#endif
