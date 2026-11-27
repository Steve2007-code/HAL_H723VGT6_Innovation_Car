#include "Link.h"
#include "DM_BLDC_Motor.h"
#include <string.h>

/* ==================== 通用成帧层 (独立实例 s_link_h723) ==================== */
#define FR_RX_SZ    256         /* 加大接收缓冲: 主循环被 printf 占用时不易溢出丢帧 */
#define FR_BODY_SZ  32

typedef struct
{
    volatile uint8_t  rx[FR_RX_SZ];
    volatile uint16_t wr;
    uint16_t          rd;
    uint8_t           st, n;
    char              body[FR_BODY_SZ];
} FrameRx_t;

static FrameRx_t s_link_h723;

/* ==================== TX 环形缓冲 (非阻塞, 禁止忙等) ==================== */
#define TX_FIFO_SZ   256
#define TX_BURST_SZ  64
static uint8_t  s_tx_fifo[TX_FIFO_SZ];
static volatile uint16_t s_tx_wr = 0;
static volatile uint16_t s_tx_rd = 0;
static uint32_t s_tx_start_ms = 0;       /* 本次 IT 发送启动时刻, 用于状态机卡死兜底 */
static uint8_t  s_tx_buf[TX_BURST_SZ];   /* HAL_UART_Transmit_IT 期间必须保持有效 */

/* ==================== 链路状态 ==================== */
static float    s_yaw_pos = 0.0f;    /* 目标绝对角 rad */
static float    s_yaw_vel = 0.0f;    /* 限速 rad/s */
static uint8_t  s_can_pending = 0;   /* 有新 Y 待转发 0x105 */
static uint8_t  s_yaw_enabled = 0;   /* E,1 后置 1 */
static uint8_t  s_link_lost = 0;     /* 400ms 无 Y */
static uint8_t  s_first_poll = 1;
static uint32_t s_last_y_ms = 0;
static uint32_t s_last_can_ms = 0;
static uint32_t s_last_a_ms = 0;
static uint32_t s_last_k3_ms = 0;
static uint32_t s_last_k4_ms = 0;

/* F103 回传的机械臂状态(S 帧)暂存: 协议层不下探 printf, 由 Debug_Task 取走打印 */
static volatile int32_t s_arm[9];
static volatile uint8_t s_arm_new = 0;

/* M 命令回执监测: 发出后超时未收到 S 帧 -> 查询类自动补发一次, 连续 3 次才告警 */
#define LINK_M_TIMEOUT_MS   800u    /* 等回执超时(原 500ms, 放宽给 F103 更多处理时间) */
#define LINK_M_FAIL_ALARM   3u      /* 连续几次无回执才告警, 防偶发刷屏 */

static uint8_t  s_m_wait  = 0;
static uint8_t  s_m_miss  = 0;
static uint8_t  s_m_retry = 0;
static uint8_t  s_m_fail  = 0;
static uint32_t s_m_ms    = 0;
static int32_t  s_m_sub = 0, s_m_v1 = 0, s_m_v2 = 0;
static uint8_t  s_m_nv  = 1;

/* ==================== 内部函数 ==================== */
static uint8_t s_i32_str(char *d, int32_t v)
{
    char tmp[12];
    uint32_t u;
    uint8_t n = 0, t = 0, i;
    if (v < 0) { d[n++] = '-'; u = (uint32_t)(-(int64_t)v); }
    else       { u = (uint32_t)v; }
    if (u == 0) { d[n++] = '0'; return n; }
    while (u) { tmp[t++] = (char)('0' + (u % 10)); u /= 10; }
    for (i = 0; i < t; i++) d[n++] = tmp[t - 1 - i];
    return n;
}

/* 只组包并投递 TX 环形缓冲, 不等待发送完成 */
static void s_emit(const char *cmd, const int32_t *v, uint8_t n)
{
    char b[48];
    uint8_t k = 0, i;
    b[k++] = '<'; b[k++] = '<';
    b[k++] = cmd[0];
    for (i = 0; i < n; i++)
    {
        b[k++] = ',';
        k += s_i32_str(&b[k], v[i]);
    }
    b[k++] = '>'; b[k++] = '>';
    b[k] = '\0';
    /* 逐字节入队 */
    {
        const char *p = b;
        while (*p)
        {
            uint16_t nx = (uint16_t)((s_tx_wr + 1) % TX_FIFO_SZ);
            if (nx == s_tx_rd) break;      /* 缓冲满, 丢弃余下, 不阻塞 */
            s_tx_fifo[s_tx_wr] = (uint8_t)*p++;
            s_tx_wr = nx;
        }
    }
}

static void s_tx_pump(void)
{
    uint16_t n = 0;
    uint16_t rd_bak;

    if (huart7.gState != HAL_UART_STATE_READY)
    {
        /* 卡死兜底: UART7 的 IT 发送若因错误中断没等到 TC, gState 会永远停在
           BUSY_TX, 此后一帧都发不出去(表现为上位机有回显、4340/步进完全不动,
           而且没有任何报错). 超过 20ms 直接强制复位状态机. */
        if ((uint32_t)(HAL_GetTick() - s_tx_start_ms) > 20u)
        {
            __HAL_UART_DISABLE_IT(&huart7, UART_IT_TC);
            huart7.gState = HAL_UART_STATE_READY;
        }
        return;
    }

    rd_bak = s_tx_rd;
    while (s_tx_rd != s_tx_wr && n < TX_BURST_SZ)
    {
        s_tx_buf[n++] = s_tx_fifo[s_tx_rd];
        s_tx_rd = (uint16_t)((s_tx_rd + 1) % TX_FIFO_SZ);
    }
    if (n == 0) return;

    /* 关键: HAL_UART_Transmit_IT 会去抢 huart7.Lock, 而 RX 中断里的
       HAL_UART_Receive_IT 也持有同一把锁. 抢不到时返回 HAL_BUSY 且不发送,
       若此时已经推进了 s_tx_rd, 这段数据就永久丢失 -> 上位机看到"M 指令已发出"
       但下位机根本没收到. 因此失败必须把读指针退回去, 下一轮重试. */
    if (HAL_UART_Transmit_IT(&huart7, s_tx_buf, n) == HAL_OK)
    {
        s_tx_start_ms = HAL_GetTick();
    }
    else
    {
        s_tx_rd = rd_bak;
    }
}

static void s_send_k(uint8_t code)
{
    int32_t v[1];
    v[0] = (int32_t)code;
    s_emit("K", v, 1);
}

/* A 帧: 4340 实际 pos/vel + ID_or_ERR 原始字节, 单位 0.001 rad / 0.001 rad/s */
static void s_send_a(void)
{
    int32_t v[3];
    v[0] = (int32_t)(DM_Motor_4340P_Yaw_StructData.POS * 1000.0f);
    v[1] = (int32_t)(DM_Motor_4340P_Yaw_StructData.VEL * 1000.0f);
    v[2] = (int32_t)DM_Motor_4340P_Yaw_StructData.UpData[0];
    s_emit("A", v, 3);
}

/* 下行命令帧: <<M,sub[,v1[,v2]]>> 立即投递并推一次 TX 泵 */
void Link_SendM(int32_t sub, int32_t v1, int32_t v2, uint8_t nv)
{
    int32_t v[3];
    v[0] = sub;
    v[1] = v1;
    v[2] = v2;
    if (nv < 1u) nv = 1u;
    if (nv > 3u) nv = 3u;
    s_emit("M", v, nv);
    s_m_sub   = sub;             /* 记下来, 无回执时可补发 */
    s_m_v1    = v1;
    s_m_v2    = v2;
    s_m_nv    = nv;
    s_m_retry = 0;
    s_m_wait  = 1;
    s_m_ms    = HAL_GetTick();
    s_tx_pump();                 /* 查询能更快看到回包 */
}

/* 成帧状态机: 0=等< 1=等第二个< 2=收body 3=等第二个> */
static uint8_t s_frame_poll(FrameRx_t *q, char *out, uint8_t cap)
{
    while (q->rd != q->wr)
    {
        uint8_t c = q->rx[q->rd];
        q->rd = (uint16_t)((q->rd + 1) % FR_RX_SZ);

        if (q->st == 0)
        {
            if (c == '<') q->st = 1;          /* 噪声字节丢弃, 不卡死 */
            continue;
        }
        if (q->st == 1)
        {
            if (c == '<') { q->st = 2; q->n = 0; }
            else          { q->st = 0; }
            continue;
        }
        if (q->st == 2)
        {
            if (c == '>') { q->st = 3; continue; }
            if (c == '<' && q->n == 0) continue;   /* 重复包头 <<<, body 为空时忽略 */
            if (q->n >= cap - 1) { q->st = 0; q->n = 0; continue; }  /* 超长复位丢帧 */
            q->body[q->n++] = (char)c;         /* 帧内 \r\n 当普通字符 -> 解析失败 */
            continue;
        }
        /* st == 3 */
        if (c == '>')
        {
            q->body[q->n] = '\0';
            q->st = 0;
            memcpy(out, q->body, (uint16_t)(q->n + 1));
            return 1;                          /* 一次只取一帧 */
        }
        /* 残缺包尾: 整帧丢弃; 若本字节是 '<' 当作新包头起点 */
        q->st = (c == '<') ? 1 : 0;
        q->n = 0;
    }
    return 0;
}

/* 纯函数、可重入, 不使用 static/全局 */
static int s_parse_i(const char **pp, int32_t *v)
{
    const char *p = *pp;
    int32_t sign = 1, val = 0;
    int digits = 0;
    if (*p == '+' || *p == '-')
    {
        if (*p == '-') sign = -1;
        p++;
    }
    while (*p >= '0' && *p <= '9')
    {
        int32_t d = *p - '0';
        if (val > (2147483647 - d) / 10) return 0;    /* 溢出 */
        val = val * 10 + d;
        digits++;
        p++;
        if (digits > 10) return 0;
    }
    if (digits == 0) return 0;
    *v = sign * val;
    *pp = p;
    return 1;
}

/* 成功返回值字段个数(1~4), 失败返回 0; cmd 恰好 1 个字符且大小写敏感 */
static uint8_t s_parse_cmd_ints(const char *body, char *cmd, int32_t *vals, uint8_t max_vals)
{
    const char *p = body;
    uint8_t n = 0;
    if (p == NULL || *p == '\0') return 0;
    *cmd = *p++;
    if (*cmd != 'Y' && *cmd != 'E' && *cmd != 'A' &&
        *cmd != 'S' && *cmd != 'C' && *cmd != 'K') return 0;
    if (*p++ != ',') return 0;
    while (1)
    {
        int32_t v;
        if (!s_parse_i(&p, &v)) return 0;
        if (n >= max_vals) return 0;
        vals[n++] = v;
        if (*p == '\0') break;
        if (*p != ',') return 0;               /* 非法字符 / 末尾多余字符 */
        p++;
        if (*p == '\0') return 0;              /* 空字段 */
    }
    return n;
}

/* ==================== 对外接口 ==================== */
void Link_Init(void)
{
    memset(&s_link_h723, 0, sizeof(s_link_h723));
    s_tx_wr = 0; s_tx_rd = 0;
    s_tx_start_ms = 0;
    s_yaw_pos = 0.0f; s_yaw_vel = 0.0f;
    s_can_pending = 0; s_yaw_enabled = 0; s_link_lost = 0; s_first_poll = 1;
    s_last_y_ms = 0; s_last_can_ms = 0; s_last_a_ms = 0;
    s_last_k3_ms = 0; s_last_k4_ms = 0;
    s_send_k(1);            /* 上电就绪, 只发一次, F103 不依赖它 */
}

/* 取走"M 命令未回执"告警; 返回 1=确实发生过一次 */
uint8_t Link_TakeMiss(void)
{
    uint8_t m = s_m_miss;
    s_m_miss = 0;
    return m;
}

/* 取走一帧机械臂状态(取走即清标志); 返回 1=有新数据 */
uint8_t Link_GetArmStat(int32_t *out, uint8_t max)
{
    uint8_t i, n = 9;

    if (!s_arm_new) return 0;
    if (n > max) n = max;

    __disable_irq();
    for (i = 0; i < n; i++) out[i] = s_arm[i];
    s_arm_new = 0;
    __enable_irq();
    return 1;
}

void Link_FeedByte(uint8_t c)
{
    uint16_t nx = (uint16_t)((s_link_h723.wr + 1) % FR_RX_SZ);

    /* 满则不写入(丢弃这一个新字节), 绝不覆盖还没被 Link_Poll 取走的旧数据:
       覆盖会让整条字节流错位, 后续所有帧解析失败, 表现为"突然所有命令都没回执". */
    if (nx == (uint16_t)(s_link_h723.rd % FR_RX_SZ)) return;

    s_link_h723.rx[s_link_h723.wr] = c;
    s_link_h723.wr = nx;
}

void Link_Poll(void)
{
    char body[FR_BODY_SZ];
    uint32_t now = HAL_GetTick();
    char cmd;
    int32_t v[12];               /* S 帧最多 9 字段(含 dx/dy) */
    uint8_t nf;

    if (s_first_poll)
    {
        s_first_poll = 0;
        s_last_y_ms = now;      /* 避免上电 1.4s 初始化后第一次误报断链 */
        s_last_a_ms = now;
    }

    /* 1) 取帧, 可循环处理粘包 */
    while (s_frame_poll(&s_link_h723, body, FR_BODY_SZ))
    {
        nf = s_parse_cmd_ints(body, &cmd, v, 12);
        if (nf == 0)
        {
            if (now - s_last_k3_ms >= LINK_K_COOL_MS)
            {
                s_send_k(3);                 /* 非法帧, 冷却防刷屏 */
                s_last_k3_ms = now;
            }
            continue;
        }
        if (cmd == 'Y' && nf == 2)
        {
            int32_t p = v[0], vel = v[1];
            uint8_t clipped = 0;
            if (p > LINK_POS_MAX)   { p = LINK_POS_MAX;   clipped = 1; }
            if (p < LINK_POS_MIN)   { p = LINK_POS_MIN;   clipped = 1; }
            if (vel > LINK_VEL_LIM) { vel = LINK_VEL_LIM;  clipped = 1; }
            if (vel < -LINK_VEL_LIM){ vel = -LINK_VEL_LIM; clipped = 1; }
            s_yaw_pos = (float)p * 0.001f;
            s_yaw_vel = (float)vel * 0.001f;
            s_last_y_ms = now;               /* 刷新 400ms 看门狗 */
            if (s_link_lost) s_link_lost = 0;   /* 收到 Y 即恢复, 无需重发 E */
            /* 未使能时只刷看门狗, 不排队、不转发 0x105 */
            if (s_yaw_enabled) s_can_pending = 1;   /* 覆盖语义: 粘包多帧取最新 */
            if (clipped && now - s_last_k4_ms >= LINK_K_COOL_MS)
            {
                s_send_k(4);                 /* 限幅触发 */
                s_last_k4_ms = now;
            }
        }
        else if (cmd == 'E' && nf == 1)
        {
            if (v[0] == 1)
            {
                /* E,1 幂等: 已使能不重复 Enable、不重复回 K,2 */
                if (!s_yaw_enabled)
                {
                    DM_CAN_Motor_SetEnable(DM_Motor_4340_Yaw_ID, DM_Mototr_Enable);
                    s_yaw_enabled = 1;
                    s_send_k(2);
                }
            }
            else if (v[0] == 0)
            {
                if (s_yaw_enabled)
                {
                    DM_CAN_Motor_SetEnable(DM_Motor_4340_Yaw_ID, DM_Mototr_Disable);
                    s_yaw_enabled = 0;
                    s_can_pending = 0;
                }
            }
            else
            {
                if (now - s_last_k3_ms >= LINK_K_COOL_MS)
                {
                    s_send_k(3);
                    s_last_k3_ms = now;
                }
            }
        }
        else if (cmd == 'S' && nf >= 4)
        {
            /* F103 回传机械臂状态: <<S,apos,bpos,ytgt,yact,trun,yman>> */
            /* 只暂存, 打印交给 Debug_Task (协议层不依赖 printf) */
            s_arm[0] = v[0];                                  /* apos */
            s_arm[1] = v[1];                                  /* bpos */
            s_arm[2] = v[2];                                  /* ytgt */
            s_arm[3] = v[3];                                  /* yact */
            s_arm[4] = (nf >= 5u) ? v[4] : -1;
            s_arm[5] = (nf >= 6u) ? v[5] : -1;
            s_arm[6] = (nf >= 7u) ? v[6] : -1;      /* 当前流程步号 */
            s_arm[7] = (nf >= 8u) ? v[7] : -9999;   /* 视觉 dx */
            s_arm[8] = (nf >= 9u) ? v[8] : -9999;   /* 视觉 dy */
            s_arm_new = 1;
            s_m_wait  = 0;                          /* 收到回执 */
            s_m_miss  = 0;
            s_m_retry = 0;
            s_m_fail  = 0;
        }
        else
        {
            /* A/C/K 等上行帧到达 H723 属异常(自环/接错线), 计非法帧 */
            if (now - s_last_k3_ms >= LINK_K_COOL_MS)
            {
                s_send_k(3);
                s_last_k3_ms = now;
            }
        }
    }

    /* 2) 看门狗: 400ms 无 Y -> 停发 0x105, 保持当前位置, 只回一次 K,5 */
    if (!s_link_lost && (now - s_last_y_ms >= LINK_WD_MS))
    {
        s_link_lost = 1;
        s_can_pending = 0;
        s_send_k(5);                          /* 状态跳变触发, 禁止周期发 */
    }

    /* 3) CAN 代发: 主循环非阻塞, >=10ms 节流, 未使能/断链时不转发 */
    if (s_can_pending && !s_link_lost && s_yaw_enabled &&
        (now - s_last_can_ms >= LINK_CAN_MIN_MS))
    {
        DM_CAN_Motor_Position_Speed(&DM_Motor_4340P_Yaw_StructData, s_yaw_vel, s_yaw_pos);
        s_last_can_ms = now;
        s_can_pending = 0;
    }

    /* 4) A 帧 20Hz 上报 */
    if (now - s_last_a_ms >= LINK_A_PERIOD_MS)
    {
        s_last_a_ms = now;
        s_send_a();
    }

    /* 4.5) M 命令超时无回执: 查询(9)/中止(10) 幂等, 自动补发一次;
           运动类命令不补发(重发会让轴重新起步造成顿挫)。连续 3 次失败才告警。 */
    if (s_m_wait && (uint32_t)(now - s_m_ms) >= LINK_M_TIMEOUT_MS)
    {
        if (s_m_retry == 0u && (s_m_sub == 9 || s_m_sub == 10))
        {
            int32_t v2[3];
            v2[0] = s_m_sub;
            v2[1] = s_m_v1;
            v2[2] = s_m_v2;
            s_m_retry = 1u;
            s_m_ms    = now;
            s_emit("M", v2, s_m_nv);
            s_tx_pump();
        }
        else
        {
            s_m_wait = 0;
            if (s_m_fail < 255u) s_m_fail++;
            if (s_m_fail >= LINK_M_FAIL_ALARM)
            {
                s_m_miss = 1;
                s_m_fail = 0;
            }
        }
    }

    /* 5) TX 泵: 非阻塞 */
    s_tx_pump();
}
