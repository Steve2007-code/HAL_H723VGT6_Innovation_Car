#include <string.h>
#include "AStar.h"

const char MAP_DATA[MAP_N][MAP_N + 1] =
{
    "000000000000000000004444444400000000000000222222",
    "000000000000000000000444444000000000000000222222",
    "000000000000000000000000000000000000000000222222",
    "000000000000000000000000000000000000000000222222",
    "000000000000000000000000000000000000000000222222",
    "000000000000000000000000000000000000000000222222",
    "000000000000000000000000000000000000000000000000",
    "000000000000000000000000000000000000000000000000",
    "000000000000000000000000000000000000000000000000",
    "000000000000000000000000000000000000000000000000",
    "000000000000000000000000000000000000000000000000",
    "000000000001111111110000000011111111100000000000",
    "000000000001111111110000000011111111100000000000",
    "000000000001111111110000000011111111100000000000",
    "000000000001111111110000000011111111100000000000",
    "000000000001111111110000000011111111100000000000",
    "000000000001111111110000000011111111100000000000",
    "000000000001111111110000000011111111100000000000",
    "333000000001111111110000000011111111100000000000",
    "333000000001111111110000000011111111100000000000",
    "333000000000000000000000000000000000000000000000",
    "333000000000000000000000000000000000000000000005",
    "333000000000000000000000000000000000000000000005",
    "333000000000000000000000000000000000000000000005",
    "333000000000000000000000000000000000000000000005",
    "333000000000000000000000000000000000000000000005",
    "333000000000000000000000000000000000000000000005",
    "333000000000000000000000000000000000000000000000",
    "333000000001111111110000000011111111100000000000",
    "333000000001111111110000000011111111100000000000",
    "000000000001111111110000000011111111100000000000",
    "000000000001111111110000000011111111100000000000",
    "000000000001111111110000000011111111100000000000",
    "000000000001111111110000000011111111100000000000",
    "000000000001111111110000000011111111100000000000",
    "000000000001111111110000000011111111100000000000",
    "000000000001111111110000000011111111100000000000",
    "000000000000000000000000000000000000000000000000",
    "000000000000000000000000000000000000000000000000",
    "000000000000000000000000000000000000000000000000",
    "000000000000000000000000000000000000000000000000",
    "000000000000000000000000000000000000000000000000",
    "000000000000000000000000000000000000000000222222",
    "000000000000000000000000000000000000000000222222",
    "000000000000000000000000000000000000000000222222",
    "000000000000000000333333333333000000000000222222",
    "000000000000000000333333333333000000000000222222",
    "000000000000000000333333333333000000000000222222"
};

uint8_t g_map_clear_cells = 4;      /* 默认: 137.5+15 = 152.5mm -> 4 格 */

static uint8_t s_kind[MAP_CELLS];
static uint8_t s_dist[MAP_CELLS];
static uint8_t s_free[MAP_CELLS];

/* A* 工作区 */
static uint16_t s_g[MAP_CELLS];
static int16_t  s_par[MAP_CELLS];
static uint16_t s_hf[MAP_CELLS * 2];    /* 堆: f 值 */
static uint16_t s_hi[MAP_CELLS * 2];    /* 堆: 格索引 */
static uint16_t s_hn;
static uint16_t s_q[MAP_CELLS];         /* BFS 队列 */

#define IDX(r, c)   ((r) * MAP_N + (c))

/**
  * @brief  生成距离场 + 可规划表
  * @note   s_dist = 到最近实体障碍(1)的切比雪夫格数; 8 邻域 BFS 单位代价
  */
/* 地图边界也要留出车体: 车半宽 137.5mm = 2.75 格, 取 3 格 = 150mm.
   原来边界完全不算障碍, 车中心可以贴到 x=2325 -> 车体右边缘 2462 探出场地. */
#define MAP_BORDER_CELLS   3

void Map_Init(void)
{
    int i, r, c, head = 0, tail = 0;
    int dr, dc, nr, nc;
    uint8_t k;

    for (r = 0; r < MAP_N; r++)
    {
        for (c = 0; c < MAP_N; c++)
        {
            k = (uint8_t)(MAP_DATA[r][c] - '0');
            s_kind[IDX(r, c)] = k;
            s_dist[IDX(r, c)] = 99;
        }
    }

    /* 多源 BFS */
    for (r = 0; r < MAP_N; r++)
    {
        for (c = 0; c < MAP_N; c++)
        {
            if (s_kind[IDX(r, c)] == 1)
            {
                s_dist[IDX(r, c)] = 0;
                s_q[tail++] = (uint16_t)IDX(r, c);
            }
        }
    }
    while (head < tail)
    {
        i = s_q[head++];
        r = i / MAP_N;
        c = i % MAP_N;
        for (dr = -1; dr <= 1; dr++)
        {
            for (dc = -1; dc <= 1; dc++)
            {
                if (dr == 0 && dc == 0) continue;
                nr = r + dr;  nc = c + dc;
                if (nr < 0 || nr >= MAP_N || nc < 0 || nc >= MAP_N) continue;
                if (s_dist[IDX(nr, nc)] == 99)          /* 8 邻域单位代价 BFS: 首次访问即最短 */
                {
                    s_dist[IDX(nr, nc)] = s_dist[i] + 1;
                    if (tail < MAP_CELLS) s_q[tail++] = (uint16_t)IDX(nr, nc);
                }
            }
        }
    }

    /* 可规划表: 非 1/3/4/5, 且离实体障碍足够远 */
    for (r = 0; r < MAP_N; r++)
    {
        for (c = 0; c < MAP_N; c++)
        {
            k = s_kind[IDX(r, c)];
            s_free[IDX(r, c)] = 0;
            if (k == 1 || k == 3 || k == 4 || k == 5) continue;
            if (s_dist[IDX(r, c)] < g_map_clear_cells) continue;
            /* 离地图边界不足 3 格的不许走(车体会出界) */
            if (r < MAP_BORDER_CELLS || r > (MAP_N - 1 - MAP_BORDER_CELLS) ||
                c < MAP_BORDER_CELLS || c > (MAP_N - 1 - MAP_BORDER_CELLS)) continue;
            s_free[IDX(r, c)] = 1;
        }
    }
}

uint8_t Map_Free(int r, int c)
{
    if (r < 0 || r >= MAP_N || c < 0 || c >= MAP_N) return 0;
    return s_free[IDX(r, c)];
}

uint8_t Map_Kind(int r, int c)
{
    if (r < 0 || r >= MAP_N || c < 0 || c >= MAP_N) return 1;
    return s_kind[IDX(r, c)];
}

uint8_t Map_Dist(int r, int c)
{
    if (r < 0 || r >= MAP_N || c < 0 || c >= MAP_N) return 0;
    return s_dist[IDX(r, c)];
}

/* ---------------- 二叉堆 ---------------- */
static void s_heap_push(uint16_t f, uint16_t idx)
{
    uint16_t i;
    if (s_hn >= (MAP_CELLS * 2)) return;        /* 保护: 堆满就丢弃 */
    i = s_hn++;
    s_hf[i] = f;
    s_hi[i] = idx;
    while (i > 0)
    {
        uint16_t p = (uint16_t)((i - 1) >> 1);
        if (s_hf[p] <= s_hf[i]) break;
        /* swap */
        {
            uint16_t tf = s_hf[p], ti = s_hi[p];
            s_hf[p] = s_hf[i]; s_hi[p] = s_hi[i];
            s_hf[i] = tf;      s_hi[i] = ti;
        }
        i = p;
    }
}

static uint16_t s_heap_pop(void)
{
    uint16_t top = s_hi[0];
    uint16_t i = 0;
    s_hn--;
    s_hf[0] = s_hf[s_hn];
    s_hi[0] = s_hi[s_hn];
    for (;;)
    {
        uint16_t l = (uint16_t)(2 * i + 1);
        uint16_t r = (uint16_t)(l + 1);
        uint16_t m = i;
        if (l < s_hn && s_hf[l] < s_hf[m]) m = l;
        if (r < s_hn && s_hf[r] < s_hf[m]) m = r;
        if (m == i) break;
        {
            uint16_t tf = s_hf[m], ti = s_hi[m];
            s_hf[m] = s_hf[i]; s_hi[m] = s_hi[i];
            s_hf[i] = tf;      s_hi[i] = ti;
        }
        i = m;
    }
    return top;
}

/* octile 启发, 放大 10 倍取整: 直走 10, 斜走 14 */
static uint16_t s_h(int r, int c, int gr, int gc)
{
    int dx = (r > gr) ? (r - gr) : (gr - r);
    int dy = (c > gc) ? (c - gc) : (gc - c);
    int mn = (dx < dy) ? dx : dy;
    int mx = (dx < dy) ? dy : dx;
    return (uint16_t)(10 * mx + 4 * mn);
}

/**
  * @brief  A* 搜索
  * @param  sr,sc 起点格  gr,gc 终点格  out 输出缓冲(2*格数 字节)  max_cells 缓冲上限
  * @retval 路径格数, 失败 -1
  */
int AStar_Search(int sr, int sc, int gr, int gc, uint8_t *out, int max_cells)
{
    int i, r, c, nr, nc, dr, dc, cur, n = 0;
    uint16_t ng;
    int16_t t;

    if (!Map_Free(sr, sc) || !Map_Free(gr, gc)) return -1;

    for (i = 0; i < MAP_CELLS; i++)
    {
        s_g[i] = 0xFFFF;
        s_par[i] = -1;
    }
    s_hn = 0;

    s_g[IDX(sr, sc)] = 0;
    s_heap_push(s_h(sr, sc, gr, gc), (uint16_t)IDX(sr, sc));

    while (s_hn > 0)
    {
        cur = (int)s_heap_pop();
        if (cur == IDX(gr, gc)) break;

        r = cur / MAP_N;
        c = cur % MAP_N;

        for (dr = -1; dr <= 1; dr++)
        {
            for (dc = -1; dc <= 1; dc++)
            {
                if (dr == 0 && dc == 0) continue;
                nr = r + dr;  nc = c + dc;
                if (!Map_Free(nr, nc)) continue;
                /* 禁止斜穿角 */
                if (dr != 0 && dc != 0)
                {
                    if (!Map_Free(r + dr, c) || !Map_Free(r, c + dc)) continue;
                }
                /* 贴边惩罚: 离障碍 <=5 格的格子每步多花 4 (约 30%~40%).
                   目的是让路径倾向走通道中线, 拐点自然落在宽敞处,
                   而不是贴着「dist>=4」这条极限边界切内角. */
                {
                    uint16_t step = (uint16_t)((dr != 0 && dc != 0) ? 14 : 10);
                    if (Map_Dist(nr, nc) <= 5) step = (uint16_t)(step + 4);
                    ng = (uint16_t)(s_g[cur] + step);
                }
                if (ng < s_g[IDX(nr, nc)])
                {
                    s_g[IDX(nr, nc)] = ng;
                    s_par[IDX(nr, nc)] = (int16_t)cur;
                    s_heap_push((uint16_t)(ng + s_h(nr, nc, gr, gc)), (uint16_t)IDX(nr, nc));
                }
            }
        }
    }

    if (s_g[IDX(gr, gc)] == 0xFFFF) return -1;

    /* 回溯 */
    cur = IDX(gr, gc);
    while (cur >= 0 && n < max_cells)
    {
        out[2 * n]     = (uint8_t)(cur / MAP_N);
        out[2 * n + 1] = (uint8_t)(cur % MAP_N);
        n++;
        if (cur == IDX(sr, sc)) break;
        t = s_par[cur];
        cur = (int)t;
    }
    if (cur != IDX(sr, sc)) return -1;

    /* 反转成 起点->终点 */
    for (i = 0; i < n / 2; i++)
    {
        uint8_t tr = out[2 * i], tc = out[2 * i + 1];
        out[2 * i]     = out[2 * (n - 1 - i)];
        out[2 * i + 1] = out[2 * (n - 1 - i) + 1];
        out[2 * (n - 1 - i)]     = tr;
        out[2 * (n - 1 - i) + 1] = tc;
    }
    return n;
}
