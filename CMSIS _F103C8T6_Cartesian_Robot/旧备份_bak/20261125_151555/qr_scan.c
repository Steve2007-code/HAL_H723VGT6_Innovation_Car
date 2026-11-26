/******************************************************************************
 * qr_scan.c  --  第二路摄像头 USART3：二维码任务码接收（全双工）
 *
 * 帧：<<flag,132+312+123+231>>     包头 <<，包尾 >>，帧内不含换行
 * 成帧交给通用 frame.c 的 s_qr 实例，与 USART1 的 s_cam、板间的 s_link 互不串帧。
 ******************************************************************************/

#include "qr_scan.h"
#include "frame.h"

static FrameRx_t s_qr;

char             g_qr_code[QR_CODE_MAX];
volatile uint8_t g_qr_ready = 0;

/* 格式校验：15 字符，下标 3/7/11 必须是 '+'，其余必须是数字 */
static uint8_t qr_valid(const char *s)
{
    uint8_t i, n = 0;

    while (s[n]) n++;
    if (n != 15U) return 0;

    for (i = 0; i < 15U; i++)
    {
        if (i == 3U || i == 7U || i == 11U)
        {
            if (s[i] != '+') return 0;
        }
        else
        {
            if (s[i] < '0' || s[i] > '9') return 0;
        }
    }
    return 1;
}

static void qr_handle(const char *src)
{
    char tmp[QR_CODE_MAX];
    uint8_t n = 0, i;

    while (*src && n < (QR_CODE_MAX - 1U))
    {
        tmp[n++] = *src++;
    }
    tmp[n] = 0;

    if (!qr_valid(tmp)) return;            /* 格式不对就丢弃，保持上一帧 */

    for (i = 0; i <= n; i++)
    {
        g_qr_code[i] = tmp[i];
    }
    g_qr_ready = 1;
}

void QR_Init(uint32_t baud)
{
    Frame_Init(&s_qr);

    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

    /* PB10 = USART3_TX 复用推挽 50MHz（CRH[11:8] = 0xB） */
    GPIOB->CRH &= ~(0xFU << 8);
    GPIOB->CRH |=  (0xBU << 8);
    /* PB11 = USART3_RX 浮空输入（CRH[15:12] = 0x4） */
    GPIOB->CRH &= ~(0xFU << 12);
    GPIOB->CRH |=  (0x4U << 12);

    /* APB1 = 36MHz，BRR = 36MHz / baud（115200 -> 312） */
    USART3->BRR = (uint16_t)((SystemCoreClock / 2U) / baud);
    USART3->CR1 = (1U << 13) | (1U << 3) | (1U << 2) | (1U << 5);   /* UE | TE | RE | RXNEIE */

    NVIC_SetPriority(USART3_IRQn, 4);
    NVIC_EnableIRQ(USART3_IRQn);
}

void USART3_IRQHandler(void)
{
    if (USART3->SR & (1U << 5))             /* RXNE */
    {
        uint8_t c = (uint8_t)(USART3->DR & 0xFFU);
        Frame_FeedByte(&s_qr, c);
    }
}

void QR_Poll(void)
{
    char body[FR_BODY_SZ];

    while (Frame_Poll(&s_qr, body, FR_BODY_SZ))
    {
        if (!(body[0] == 'f' && body[1] == 'l' && body[2] == 'a' &&
              body[3] == 'g' && body[4] == ','))
        {
            continue;                       /* 非二维码帧直接丢 */
        }
        qr_handle(body + 5);
    }
}

/* ---------------- 向摄像头发送 ---------------- */
void QR_SendByte(uint8_t b)
{
    while (!(USART3->SR & (1U << 7)))       /* 等 TXE */
    {
    }
    USART3->DR = b;
}

void QR_SendStr(const char *s)
{
    while (*s)
    {
        QR_SendByte((uint8_t)*s++);
    }
}
