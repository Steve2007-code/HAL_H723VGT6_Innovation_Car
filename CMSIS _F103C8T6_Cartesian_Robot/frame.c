/******************************************************************************
 * frame.c  --  通用 <<...>> 成帧层 + 整数解析，全部为纯函数/实例状态，可重入
 *
 * 状态机：0=等<  1=等第二个<  2=收body  3=等第二个>
 * 容错：重复包头 <<<、粘包、超长、残缺包尾、噪声，全部自恢复。
 * 注意：帧内不能出现 \r \n，会被当作普通字符塞进 body 导致解析失败。
 ******************************************************************************/
#include "frame.h"

#define PARSE_I32_MAX   2147483647L

void Frame_Init(FrameRx_t *q)
{
    q->wr = 0;
    q->rd = 0;
    q->st = 0;
    q->n  = 0;
    q->body[0] = 0;
}

void Frame_FeedByte(FrameRx_t *q, uint8_t c)
{
    /* 满则丢弃这一个新字节, 绝不覆盖还没被 Poll 取走的旧数据:
       覆盖会让整条字节流错位, 之后所有帧都解析失败(表现为"突然全部没反应")。 */
    if ((uint16_t)(q->wr - q->rd) >= FR_RX_SZ) return;
    q->rx[q->wr++ & (FR_RX_SZ - 1U)] = c;
}

uint8_t Frame_Poll(FrameRx_t *q, char *out, uint8_t cap)
{
    while ((uint16_t)(q->wr - q->rd) != 0U) {
        uint8_t c = q->rx[q->rd++ & (FR_RX_SZ - 1U)];

        switch (q->st) {
        case 0:
            if (c == '<') q->st = 1;
            break;

        case 1:
            q->st = (c == '<') ? 2 : 0;
            q->n  = 0;
            break;

        case 2:
            if (c == '>') {
                q->st = 3;
                q->body[q->n] = 0;
            } else if (c == '<' && q->n == 0) {
                /* 多余包头（例如 <<<），忽略后继续等数据 */
            } else if (q->n < FR_BODY_SZ - 1U) {
                q->body[q->n++] = (char)c;
            } else {
                q->st = 0;              /* 超长，丢帧 */
                q->n  = 0;
            }
            break;

        case 3:
            q->st = (c == '<') ? 1 : 0;   /* 粘包时下一个包头不丢 */
            if (c != '>') break;
            if (q->n >= cap) { q->n = 0; break; }   /* 输出缓冲不足，丢帧 */
            {
                uint8_t i;
                for (i = 0; i < q->n; i++) out[i] = q->body[i];
                out[q->n] = 0;
                q->n = 0;
                return 1;
            }

        default:
            q->st = 0;
            q->n  = 0;
            break;
        }
    }
    return 0;
}

/* 解析有符号十进制整数；溢出视为非法。成功返回 1 并推进指针 */
int parse_i(const char **pp, int32_t *v)
{
    const char *p = *pp;
    int32_t s = 1, n = 0;
    uint8_t got = 0;

    if (*p == '-') { s = -1; p++; }
    else if (*p == '+') p++;

    while (*p >= '0' && *p <= '9') {
        int32_t d = *p - '0';
        if (n > (PARSE_I32_MAX - d) / 10) return 0;   /* 溢出 */
        n = n * 10 + d;
        p++;
        got = 1;
    }
    if (!got) return 0;

    *v = s * n;
    *pp = p;
    return 1;
}

static uint8_t cmd_ok(char c)
{
    return (c == 'Y' || c == 'E' || c == 'A' ||
            c == 'S' || c == 'C' || c == 'K' ||
            c == 'M') ? 1U : 0U;      /* M = H723 下发的机械臂命令帧 */
}

/* 解析 "<CMD>,<V1>[,<V2>...]"。返回字段数(1~max_vals)，失败返回 0。
   cmd 恰好 1 个 ASCII 大写字符，大小写敏感；纯函数、可重入。 */
uint8_t parse_cmd_ints(const char *body, char *cmd, int32_t *vals, uint8_t max_vals)
{
    const char *p = body;
    uint8_t n = 0;

    if (!p || !*p) return 0;
    if (!cmd_ok(*p)) return 0;
    *cmd = *p++;
    if (*p++ != ',') return 0;

    for (;;) {
        int32_t v;
        if (!parse_i(&p, &v)) return 0;
        if (n >= max_vals) return 0;
        vals[n++] = v;
        if (*p == 0) break;
        if (*p++ != ',') return 0;
    }
    return n;
}
