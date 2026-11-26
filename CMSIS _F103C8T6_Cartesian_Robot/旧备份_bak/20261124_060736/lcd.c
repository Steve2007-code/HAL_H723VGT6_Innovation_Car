/******************************************************************************
 * lcd.c  --  ILI9341 2.8 寸 SPI 屏（SPI2 硬件 SPI，320x240 横屏）
 *
 * 引脚：SCK=PB13  MISO=PB14  MOSI=PB15  CS=PA8  DC=PA0  RST=PA1  BL=PA11
 * 只写不读，MISO 可以不接。
 *
 * 字符：内置 5x7 点阵（0~9 与 +），按 Scale 倍放大绘制，
 *       整块区域只开一次窗口，连续写像素，速度快。
 ******************************************************************************/

#include "lcd.h"

/* ---------------- 引脚操作 ---------------- */
#define CS_L()      (LCD_CS_PORT->BRR  = (1U << LCD_CS_PIN))
#define CS_H()      (LCD_CS_PORT->BSRR = (1U << LCD_CS_PIN))
#define DC_CMD()    (LCD_DC_PORT->BRR  = (1U << LCD_DC_PIN))
#define DC_DATA()   (LCD_DC_PORT->BSRR = (1U << LCD_DC_PIN))
#define RST_L()     (LCD_RST_PORT->BRR  = (1U << LCD_RST_PIN))
#define RST_H()     (LCD_RST_PORT->BSRR = (1U << LCD_RST_PIN))
#define BL_ON()     (LCD_BL_PORT->BSRR = (1U << LCD_BL_PIN))
#define BL_OFF()    (LCD_BL_PORT->BRR  = (1U << LCD_BL_PIN))

/* ---------------- 5x7 字库：0~9 与 + ---------------- */
static const uint8_t FONT5X7[11][5] =
{
    {0x3E,0x51,0x49,0x45,0x3E},   /* 0 */
    {0x00,0x42,0x7F,0x40,0x00},   /* 1 */
    {0x42,0x61,0x51,0x49,0x46},   /* 2 */
    {0x21,0x41,0x45,0x4B,0x31},   /* 3 */
    {0x18,0x14,0x12,0x7F,0x10},   /* 4 */
    {0x27,0x45,0x45,0x45,0x39},   /* 5 */
    {0x3C,0x4A,0x49,0x49,0x30},   /* 6 */
    {0x01,0x71,0x09,0x05,0x03},   /* 7 */
    {0x36,0x49,0x49,0x49,0x36},   /* 8 */
    {0x06,0x49,0x49,0x29,0x1E},   /* 9 */
    {0x08,0x08,0x3E,0x08,0x08}    /* + */
};

/* ---------------- 内部工具 ---------------- */
static void lcd_delay_ms(uint32_t Ms)
{
    volatile uint32_t i;
    while (Ms--)
    {
        for (i = 0; i < 8000; i++)
        {
        }
    }
}

static void lcd_wr_byte(uint8_t D)
{
    while (!(LCD_SPI->SR & SPI_SR_TXE))
    {
    }
    *(volatile uint8_t *)&LCD_SPI->DR = D;
    while (!(LCD_SPI->SR & SPI_SR_RXNE))
    {
    }
    (void)(*(volatile uint8_t *)&LCD_SPI->DR);
}

static void lcd_wr16(uint16_t D)
{
    lcd_wr_byte((uint8_t)(D >> 8));
    lcd_wr_byte((uint8_t)(D & 0xFF));
}

static void LCD_WriteCmd(uint8_t Cmd)
{
    CS_L();
    DC_CMD();
    lcd_wr_byte(Cmd);
    CS_H();
}

static void LCD_WriteData(uint8_t Data)
{
    CS_L();
    DC_DATA();
    lcd_wr_byte(Data);
    CS_H();
}

/* ---------------- 引脚与 SPI 初始化 ---------------- */
static void lcd_gpio_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;

    /* PB13(SCK) PB15(MOSI)：复用推挽 50MHz；PB14(MISO)：输入浮空 */
    GPIOB->CRH &= ~(0xFU << 20);            /* PB13 */
    GPIOB->CRH |=  (0xBU << 20);
    GPIOB->CRH &= ~(0xFU << 24);            /* PB14 */
    GPIOB->CRH |=  (0x4U << 24);
    GPIOB->CRH &= ~(0xFU << 28);            /* PB15 */
    GPIOB->CRH |=  (0xBU << 28);

    /* PA0(DC) PA1(RST)：推挽输出 2MHz */
    GPIOA->CRL &= ~(0xFU << 0);
    GPIOA->CRL |=  (0x2U << 0);
    GPIOA->CRL &= ~(0xFU << 4);
    GPIOA->CRL |=  (0x2U << 4);

    /* PA8(CS) PA11(BL)：推挽输出 2MHz */
    GPIOA->CRH &= ~(0xFU << 0);
    GPIOA->CRH |=  (0x2U << 0);
    GPIOA->CRH &= ~(0xFU << 12);
    GPIOA->CRH |=  (0x2U << 12);

    CS_H();
    DC_DATA();
    RST_H();
    BL_OFF();
}

static void lcd_spi_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    /* MSTR=1, SSM=1, SSI=1, BR=001(36M/4=9MHz), CPOL=0, CPHA=0, 8bit, MSB first */
    LCD_SPI->CR1 = (1U << 9) | (1U << 8) | (1U << 2) | (1U << 3);
    LCD_SPI->CR1 |= (1U << 6);              /* SPE */
}

/* ---------------- 公共 API ---------------- */
void LCD_Backlight(uint8_t On)
{
    if (On) BL_ON();
    else    BL_OFF();
}

void LCD_Init(void)
{
    lcd_gpio_init();
    lcd_spi_init();

    /* 硬复位 */
    RST_L();
    lcd_delay_ms(20);
    RST_H();
    lcd_delay_ms(120);

    /* 软复位 */
    LCD_WriteCmd(0x01);
    lcd_delay_ms(150);

    LCD_WriteCmd(0x11);                     /* Sleep Out */
    lcd_delay_ms(120);

    LCD_WriteCmd(0x36);                     /* MADCTL：MV|BGR -> 横屏 320x240 */
    LCD_WriteData(0x28);

    LCD_WriteCmd(0x3A);                     /* 像素格式：16bit RGB565 */
    LCD_WriteData(0x55);

    LCD_WriteCmd(0x21);                     /* 打开反显（MSP2807 这类 IPS 屏需要） */
    LCD_WriteCmd(0x13);                     /* 普通显示模式 */
    LCD_WriteCmd(0x29);                     /* Display ON */
    lcd_delay_ms(50);

    BL_ON();
}

void LCD_SetWindow(uint16_t X0, uint16_t Y0, uint16_t X1, uint16_t Y1)
{
    LCD_WriteCmd(0x2A);                     /* Column Address Set */
    LCD_WriteData((uint8_t)(X0 >> 8));
    LCD_WriteData((uint8_t)(X0 & 0xFF));
    LCD_WriteData((uint8_t)(X1 >> 8));
    LCD_WriteData((uint8_t)(X1 & 0xFF));

    LCD_WriteCmd(0x2B);                     /* Row Address Set */
    LCD_WriteData((uint8_t)(Y0 >> 8));
    LCD_WriteData((uint8_t)(Y0 & 0xFF));
    LCD_WriteData((uint8_t)(Y1 >> 8));
    LCD_WriteData((uint8_t)(Y1 & 0xFF));

    LCD_WriteCmd(0x2C);                     /* Memory Write */
}

void LCD_FillRect(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, uint16_t Color)
{
    uint32_t N;

    if (W == 0 || H == 0) return;

    LCD_SetWindow(X, Y, (uint16_t)(X + W - 1), (uint16_t)(Y + H - 1));

    CS_L();
    DC_DATA();
    N = (uint32_t)W * (uint32_t)H;
    while (N--)
    {
        lcd_wr16(Color);
    }
    CS_H();
}

void LCD_Fill(uint16_t Color)
{
    LCD_FillRect(0, 0, 320, 240, Color);
}

void LCD_DrawChar(uint16_t X, uint16_t Y, char C, uint16_t Fg, uint16_t Bg, uint8_t Scale)
{
    const uint8_t *p;
    uint8_t Idx = 0xFF;
    uint16_t Row, Col;

    if (Scale == 0) Scale = 1;

    if (C >= '0' && C <= '9')      Idx = (uint8_t)(C - '0');
    else if (C == '+')             Idx = 10;
    else                           return;      /* 不支持的字符直接跳过 */

    p = FONT5X7[Idx];

    LCD_SetWindow(X, Y,
                  (uint16_t)(X + 5U * Scale - 1U),
                  (uint16_t)(Y + 7U * Scale - 1U));

    CS_L();
    DC_DATA();
    for (Row = 0; Row < (uint16_t)(7U * Scale); Row++)
    {
        uint8_t Bits = p[0];
        for (Col = 0; Col < (uint16_t)(5U * Scale); Col++)
        {
            Bits = p[Col / Scale];
            lcd_wr16(((Bits >> (Row / Scale)) & 1U) ? Fg : Bg);
        }
    }
    CS_H();
}

void LCD_DrawString(uint16_t X, uint16_t Y, const char *Str, uint16_t Fg, uint16_t Bg, uint8_t Scale)
{
    if (Scale == 0) Scale = 1;

    while (*Str)
    {
        LCD_DrawChar(X, Y, *Str, Fg, Bg, Scale);
        X = (uint16_t)(X + 5U * Scale);         /* 字形最右列本身留空, 不再额外加间距 */
        Str++;
    }
}
