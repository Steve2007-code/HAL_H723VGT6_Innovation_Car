/******************************************************************************
 * backlight.c  --  LED 补光开关：PB12 推挽输出，高电平有效
 ******************************************************************************/

#include "backlight.h"

#define BL_PORT     GPIOB
#define BL_PIN      12U

static uint8_t s_on = 0;

void Backlight_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    /* PB12 在 CRH，位段 [(12-8)*4 +:3] */
    BL_PORT->CRH &= ~(0xFU << ((BL_PIN - 8U) * 4U));
    BL_PORT->CRH |=  (0x2U << ((BL_PIN - 8U) * 4U));   /* 2MHz 推挽输出 */

    s_on = 0;
    BL_PORT->BRR = (1U << BL_PIN);                     /* 上电默认熄灭 */
}

void Backlight_Set(uint8_t on)
{
    s_on = on ? 1U : 0U;
    if (s_on) BL_PORT->BSRR = (1U << BL_PIN);
    else      BL_PORT->BRR  = (1U << BL_PIN);
}

void Backlight_Toggle(void)
{
    Backlight_Set(s_on ? 0U : 1U);
}

uint8_t Backlight_Get(void)
{
    return s_on;
}
