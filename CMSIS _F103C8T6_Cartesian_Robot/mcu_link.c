/******************************************************************************
 * mcu_link.c  --  USART2 PA2/PA3 板间链路（机械臂侧 F103，全非阻塞）
 *
 * 物理：USART2 115200 8N1，3.3V TTL，共地，TX<->RX 交叉接 H723 的 UART7。
 * 时钟：USART2 挂 APB1(PCLK1=36MHz)，BRR 必须按 36MHz 算，不能照抄 USART1 的 72MHz。
 * 帧：  <<Y,pos_mrad,vel_mradps>>  100Hz 发（本期）
 *       <<E,1>>                   上电前 1s 每 100ms 补发
 *       <<A,pos,vel,err>>         收，只存状态
 *       <<K,code>>                收，只存状态
 * 断链：>1s 无 A/K -> link_lost，停发 Y；收到任意 A/K 即恢复。
 *       F103->H723 单向断链时 link_lost 可能不置位（仍能收到 A），
 *       此时继续发 Y 是自愈所需，安全由 H723 400ms 看门狗兜底。
 ******************************************************************************/
#include "mcu_link.h"
#include "link_config.h"
#include "frame.h"
#include "track.h"          /* g_ms */

#define TX_SZ   128         /* 必须是 2 的幂 */

/* ---------- 接收实例（s_cam 在 vision.c 内，两路互不串帧） ---------- */
static FrameRx_t s_link;

/* ---------- 发送环形缓冲 ---------- */
static volatile uint8_t  tx_buf[TX_SZ];
static volatile uint16_t tx_wr;
static volatile uint16_t tx_rd;
static volatile uint8_t  tx_busy;

/* ---------- 链路状态 ---------- */
static volatile uint32_t s_last_rx_ms;
static volatile uint32_t s_start_ms;      /* 上电时刻，保留供调试/后续任务使用 */
static volatile uint32_t s_last_e_ms;
static uint8_t           s_yaw_ack = 0;   /* 收到 K,2 置 1：停止 E,1 低频补发 */
static volatile uint8_t  s_link_lost;
static volatile uint8_t  s_bad_frame;
static volatile uint8_t  s_e_cnt;

/* ---------- yaw 目标（本期只留接口 + 默认值） ---------- */
static volatile int32_t  s_yaw_pos_mrad;
static volatile int32_t  s_yaw_vel_mradps;

/* ---------- 收到的 A / K 状态（只存不解释） ---------- */
static volatile int32_t  s_a_pos, s_a_vel, s_a_err;
static volatile uint8_t  s_k_code;

/* ---------- 内部工具：裸机整数转字符串，不用 sprintf ---------- */
static char *i2s(char *p, int32_t v)
{
    char tmp[12];
    uint8_t n = 0;
    uint32_t u;

    if (v < 0) { *p++ = '-'; u = (uint32_t)(-v); }
    else         u = (uint32_t)v;

    do {
        tmp[n++] = (char)('0' + (u % 10U));
        u /= 10U;
    } while (u);
    while (n) *p++ = tmp[--n];
    return p;
}

static void fmt_y(char *buf, int32_t pos, int32_t vel)
{
    char *p = buf;
    const char *s = "<<Y,";

    while (*s) *p++ = *s++;
    p = i2s(p, pos);
    *p++ = ',';
    p = i2s(p, vel);
    *p++ = '>';
    *p++ = '>';
    *p = 0;
}

/* 整帧原子投递；缓冲不足则整帧丢弃，绝不忙等 */
static void send_frame(const char *s)
{
    uint8_t len = 0;
    uint16_t free;
    uint8_t i;

    while (s[len]) len++;

    __disable_irq();
    free = (uint16_t)(TX_SZ - (uint16_t)(tx_wr - tx_rd));
    if (free >= len) {
        for (i = 0; i < len; i++) tx_buf[(tx_wr + i) & (TX_SZ - 1U)] = (uint8_t)s[i];
        tx_wr = (uint16_t)(tx_wr + len);
        if (!tx_busy) {
            tx_busy = 1;
            USART2->CR1 |= (1U << 7);      /* TXEIE */
        }
    }
    __enable_irq();
}

/* ---------- 初始化 ---------- */
void Link_Init(uint32_t baud)
{
    Frame_Init(&s_link);

    /* 自己置位，不依赖 Vision_Init 的初始化顺序 */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* PA2 = USART2_TX 复用推挽 50MHz；PA3 = USART2_RX 浮空输入 */
    GPIOA->CRL &= ~(0xFU << 8);
    GPIOA->CRL |=  (0xBU << 8);
    GPIOA->CRL &= ~(0xFU << 12);
    GPIOA->CRL |=  (0x4U << 12);

    /* 不碰 AFIO->MAPR，USART2 保持默认 PA2/PA3；TIM2 只做内部时基不输出 PWM，不冲突 */
    USART2->BRR = (uint16_t)((SystemCoreClock / 2U) / baud);
    USART2->CR1 = (1U << 13) | (1U << 3) | (1U << 2) | (1U << 5);  /* UE|TE|RE|RXNEIE */

    NVIC_SetPriority(USART2_IRQn, 4);
    NVIC_EnableIRQ(USART2_IRQn);

    tx_wr = 0; tx_rd = 0; tx_busy = 0;
    s_start_ms   = g_ms;
    s_last_rx_ms = g_ms;
    s_last_e_ms  = g_ms;
    s_link_lost  = 0;
    s_bad_frame  = 0;
    s_e_cnt      = 0;
    s_a_pos = 0; s_a_vel = 0; s_a_err = 0;
    s_k_code = 0;
    s_yaw_pos_mrad   = LINK_YAW_DEFAULT_MRAD;
    s_yaw_vel_mradps = LINK_YAW_VEL_MRADPS;

    /* 上电立即请求使能，随后 1s 内再按 100ms 补发 */
    s_e_cnt = 1;
    send_frame("<<E,1>>");
}

/* ---------- 1) 先收 ---------- */
void Link_PollRx(void)
{
    char body[FR_BODY_SZ];
    char cmd;
    int32_t v[FR_MAX_VALS];
    uint8_t n;

    while (Frame_Poll(&s_link, body, FR_BODY_SZ)) {
        n = parse_cmd_ints(body, &cmd, v, FR_MAX_VALS);
        if (!n) { s_bad_frame = 1; continue; }

        if (cmd == 'A' && n >= 3) {
            s_a_pos = v[0];
            s_a_vel = v[1];
            s_a_err = v[2];           /* 原始字节，只存不解释 */
            s_last_rx_ms = g_ms;
        } else if (cmd == 'K' && n >= 1) {
            s_k_code = (uint8_t)v[0];
            /* K,2 = H723 已确认使能：停止 E,1 低频补发（不是等 ACK，不阻塞） */
            if (s_k_code == LINK_K_ENABLED) s_yaw_ack = 1;
            /* K,1 = H723 刚上电: 它的使能状态已丢, 重新开始补发 E,1 */
            if (s_k_code == LINK_K_READY)   s_yaw_ack = 0;
            /* K,1 上电就绪不参与链路判定；其余 K / 全部 A 都算活性 */
            if (s_k_code != LINK_K_READY) s_last_rx_ms = g_ms;
        } else {
            s_bad_frame = 1;
        }
    }
}

/* ---------- 3)+4) 取 yaw 目标 + 组包投递 ---------- */
void Link_TxTick(void)
{
    char buf[32];
    uint32_t now = g_ms;
    int32_t pos, vel;

    /* 断链判定：收到任意 A/K 即由 Link_PollRx 刷新 s_last_rx_ms，这里自动恢复 */
    s_link_lost = ((now - s_last_rx_ms) > LINK_LOST_MS) ? 1U : 0U;

    /* 只要还没收到 <<K,2>>（H723 未确认使能），就每 100ms 补发 <<E,1>>。
       不能用「elapsed < 1s 或 link_lost」做条件：A 帧无条件 20Hz 会让 link_lost
       在 H723 上电后 <=50ms 就被清掉，而补发周期 100ms，约一半概率一帧都发不出。 */
    if (!s_yaw_ack && (now - s_last_e_ms) >= LINK_E_RETRY_MS) {
        s_last_e_ms = now;
        s_e_cnt++;
        send_frame("<<E,1>>");
    }

    if (s_link_lost) return;         /* 停发 Y；Track_Update 的 PID 不受影响 */

    pos = s_yaw_pos_mrad;
    vel = s_yaw_vel_mradps;
    /* F103 侧同样限幅 */
    if (pos >  LINK_POS_MAX_MRAD)   pos =  LINK_POS_MAX_MRAD;
    if (pos <  LINK_POS_MIN_MRAD)   pos =  LINK_POS_MIN_MRAD;
    if (vel >  LINK_VEL_MAX_MRADPS) vel =  LINK_VEL_MAX_MRADPS;
    if (vel < -LINK_VEL_MAX_MRADPS) vel = -LINK_VEL_MAX_MRADPS;

    fmt_y(buf, pos, vel);
    send_frame(buf);
}

/* ---------- 对外接口 ---------- */
void Link_SetYawTarget(int32_t pos_mrad, int32_t vel_mradps)
{
    __disable_irq();
    s_yaw_pos_mrad   = pos_mrad;
    s_yaw_vel_mradps = vel_mradps;
    __enable_irq();
}

int32_t Link_GetYawTarget(void) { return s_yaw_pos_mrad; }
int32_t Link_GetYawPos(void)    { return s_a_pos; }   /* 4340 实际角度 (来自 A 帧) */
uint8_t Link_IsLost(void)       { return s_link_lost; }
uint8_t Link_GetKCode(void)     { return s_k_code; }

/* ---------- USART2 中断：只收字节 / 只发字节 ---------- */
void USART2_IRQHandler(void)
{
    if (USART2->SR & (1U << 5)) {          /* RXNE */
        uint8_t c = (uint8_t)(USART2->DR & 0xFFU);
        Frame_FeedByte(&s_link, c);
    }
    if (USART2->SR & (1U << 7)) {          /* TXE */
        if ((uint16_t)(tx_wr - tx_rd) != 0U) {
            USART2->DR = tx_buf[tx_rd++ & (TX_SZ - 1U)];
        } else {
            USART2->CR1 &= ~(1U << 7);     /* 缓冲空，关 TXEIE */
            tx_busy = 0;
        }
    }
}
