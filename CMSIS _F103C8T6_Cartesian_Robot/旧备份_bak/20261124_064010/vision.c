/******************************************************************************
 * vision.c  --  USART1 摄像头帧接收，全程非阻塞
 *
 * 成帧交给通用 frame.c 的 s_cam 实例；本文件只负责 <<X,dx,Y,dy>> 四字段解析。
 * 板间链路用另一个 FrameRx_t 实例（mcu_link.c 的 s_link），两者互不串帧。
 * 中断只负责把字节喂进实例，主循环调用 Vision_Poll() 取帧。
 * 半包、粘包、非法字符、超长帧都能自动恢复。
 ******************************************************************************/
#include "vision.h"
#include "track.h"
#include "track_config.h"
#include "frame.h"

#define BODY_SZ  FR_BODY_SZ

/* ---------------- 二维码任务码 ---------------- */
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

/* 把 "<<flag," 之后的字符串收下来 */
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

static FrameRx_t s_cam;         /* 只此一路：USART1 摄像头 */

/* 时基由 track.c 提供 */
extern volatile uint32_t g_ms;
extern volatile uint32_t g_last_frame_ms;

void Vision_Init(uint32_t baud)
{
    Frame_Init(&s_cam);

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
        Frame_FeedByte(&s_cam, c);
    }
}

/* 跳过所有非法帧，返回第一帧有效的 <<X,dx,Y,dy>>；没凑齐整帧也立刻返回 0 */
uint8_t Vision_Poll(VisionFrame_t *f)
{
    char body[BODY_SZ];

    while (Frame_Poll(&s_cam, body, BODY_SZ)) {
        const char *p = body;
        int32_t vx, vy;

        /* ---- 二维码帧：<<flag,132+312+123+231>> ---- */
        if (p[0] == 'f' && p[1] == 'l' && p[2] == 'a' && p[3] == 'g' && p[4] == ',')
        {
            qr_handle(p + 5);
            continue;                      /* 收完继续找视觉帧，不返回 */
        }

        if (p[0] != 'X' || p[1] != ',') continue;
        p += 2;
        if (!parse_i(&p, &vx)) continue;
        if (*p++ != ',' || *p++ != 'Y' || *p++ != ',') continue;
        if (!parse_i(&p, &vy) || *p != 0) continue;

        f->dx = (int16_t)vx;
        f->dy = (int16_t)vy;
        f->valid = (vx != LOST_SENTINEL && vy != LOST_SENTINEL);
        f->stamp_ms = g_ms;
        g_last_frame_ms = g_ms;
        return 1;
    }
    return 0;
}

uint32_t Vision_AgeMs(void)
{
    uint32_t age;
    __disable_irq();
    age = g_ms - g_last_frame_ms;
    __enable_irq();
    return age;
}
