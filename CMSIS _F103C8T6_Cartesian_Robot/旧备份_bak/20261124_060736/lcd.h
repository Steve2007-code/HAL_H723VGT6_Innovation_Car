#ifndef __LCD_H
#define __LCD_H

#include "stm32f10x.h"

/* ==================================================================
 * ILI9341 2.8 寸 SPI 屏驱动（320x240 横屏）
 *
 * 硬件连接（全部为 F103C8T6 上当前空闲的引脚）：
 *   SCK   -> PB13   (SPI2_SCK)
 *   MISO  -> PB14   (SPI2_MISO，本驱动只写不读，可不接)
 *   MOSI  -> PB15   (SPI2_MOSI)
 *   CS    -> PA8
 *   DC    -> PA0    (高=数据 低=命令)
 *   RESET -> PA1    (低有效)
 *   LED   -> PA11   (高=背光开)
 *   VCC   -> 3.3V
 *   GND   -> GND
 *
 * SPI2 挂 APB1(36MHz)，4 分频 -> 9MHz，ILI9341 写周期 66ns 完全够。
 * ================================================================== */

/* ---------------- SPI2 ---------------- */
#define LCD_SPI                 SPI2

/* ---------------- 控制脚 ---------------- */
#define LCD_CS_PORT             GPIOA
#define LCD_CS_PIN              8
#define LCD_DC_PORT             GPIOA
#define LCD_DC_PIN              0
#define LCD_RST_PORT            GPIOA
#define LCD_RST_PIN             1
#define LCD_BL_PORT             GPIOA
#define LCD_BL_PIN              11

/* ---------------- 常用颜色（RGB565） ---------------- */
#define LCD_BLACK               0x0000
#define LCD_WHITE               0xFFFF
#define LCD_RED                 0xF800
#define LCD_GREEN               0x07E0
#define LCD_BLUE                0x001F
#define LCD_YELLOW              0xFFE0
#define LCD_CYAN                0x07FF
#define LCD_MAGENTA             0xF81F
#define LCD_GRAY                0x8410

/* ---------------- API ---------------- */
void LCD_Init(void);
void LCD_Backlight(uint8_t on);
void LCD_Fill(uint16_t Color);
void LCD_FillRect(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, uint16_t Color);
void LCD_SetWindow(uint16_t X0, uint16_t Y0, uint16_t X1, uint16_t Y1);
void LCD_DrawChar(uint16_t X, uint16_t Y, char C, uint16_t Fg, uint16_t Bg, uint8_t Scale);
void LCD_DrawString(uint16_t X, uint16_t Y, const char *Str, uint16_t Fg, uint16_t Bg, uint8_t Scale);

#endif
