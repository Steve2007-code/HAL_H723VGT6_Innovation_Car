/******************************************************************************
 * vision.c  --  USART1 接收 + 帧解析状态机，全程非阻塞
 *
 * 中断只负责把字节塞进环形缓冲，主循环调用 Vision_Poll() 取帧。
 * 半包、粘包、非法字符、超长帧都能自动恢复。
 ******************************************************************************/
#include "vision.h"
#include "track.h"
#include "track_config.h"

#define RX_SZ    128         /* 必须是 2 的幂 */
#define BODY_SZ  24

static volatile uint8_t  rx[RX_SZ];
static volatile uint16_t wr;
static uint16_t          rd;

/* 时基由 track.c 提供 */
extern volatile uint32_t g_ms;
extern volatile uint32_t g_last_frame_ms;

static uint8_t has_byte(void) { return (uint16_t)(wr - rd) != 0; }
static uint8_t get_byte(void) { return rx[rd++ & (RX_SZ - 1U)]; }

void Vision_Init(uint32_t baud)
{
    /* PA9 = USART1_TX 复用推挽 50MHz，PA10 = USART1_RX 浮空输入 */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;

    GPIOA->CRH &= ~(0xFU << 4);
    GPIOA->CRH |=  (0xBU << 4);         /* PA9: MODE=11 CNF=10 */
    GPIOA->CRH &= ~(0xFU << 8);
    GPIOA->CRH |=  (0x4U << 8);         /* PA10: MODE=00 CNF=01 浮空输入 */

    USART1->BRR = (uint16_t)(SystemCoreClock / baud);
    USART1->CR1 = (1U << 13) | (1U << 2) | (1U << 5);   /* UE | RE | RXNEIE */

    NVIC_SetPriority(USART1_IRQn, 3);
    NVIC_EnableIRQ(USART1_IRQn);
}

void USART1_IRQHandler(void)
{
    if (USART1->SR & (1U << 5)) {       /* RXNE */
        uint8_t c = (uint8_t)(USART1->DR & 0xFFU);
        rx[wr++ & (RX_SZ - 1U)] = c;
    }
}

/* 解析有符号十进制整数，成功返回 1 并推进指针 */
static uint8_t parse_i(const char **pp, int32_t *v)
{
    const char *p = *pp;
    int32_t s = 1, n = 0;
    uint8_t got = 0;

    if (*p == '-') { s = -1; p++; }
    else if (*p == '+') p++;

    while (*p >= '0' && *p <= '9') {
        n = n * 10 + (*p - '0');
        p++;
        got = 1;
    }
    if (!got) return 0;

    *v = s * n;
    *pp = p;
    return 1;
}

uint8_t Vision_Poll(VisionFrame_t *f)
{
    static uint8_t st = 0;             /* 0=等< 1=等第二个< 2=收数据 3=等> */
    static char body[BODY_SZ];
    static uint8_t n;

    while (has_byte()) {
        uint8_t c = get_byte();

        switch (st) {
        case 0:
            if (c == '<') st = 1;
            break;

        case 1:
            st = (c == '<') ? 2 : 0;
            n = 0;
            break;

        case 2:
            if (c == '>') {
                st = 3;
                body[n] = 0;
            } else if (c == '<' && n == 0) {
                /* 多余包头（例如 <<<），忽略后继续等数据，增强容错 */
            } else if (n < BODY_SZ - 1U) {
                body[n++] = (char)c;
            } else {
                st = 0;                 /* 超长，丢帧 */
                n = 0;
            }
            break;

        case 3:
            st = (c == '<') ? 1 : 0;   /* 粘包时下一个包头不丢 */
            if (c != '>') break;
            {
                const char *p = body;
                int32_t vx, vy;

                if (p[0] != 'X' || p[1] != ',') break;
                p += 2;
                if (!parse_i(&p, &vx)) break;
                if (*p++ != ',' || *p++ != 'Y' || *p++ != ',') break;
                if (!parse_i(&p, &vy) || *p != 0) break;

                f->dx = (int16_t)vx;
                f->dy = (int16_t)vy;
                f->valid = (vx != LOST_SENTINEL && vy != LOST_SENTINEL);
                f->stamp_ms = g_ms;
                g_last_frame_ms = g_ms;
                return 1;
            }
        }
    }
    return 0;                           /* 没凑齐整帧也立刻返回 */
}

uint32_t Vision_AgeMs(void)
{
    uint32_t age;
    __disable_irq();
    age = g_ms - g_last_frame_ms;
    __enable_irq();
    return age;
}
