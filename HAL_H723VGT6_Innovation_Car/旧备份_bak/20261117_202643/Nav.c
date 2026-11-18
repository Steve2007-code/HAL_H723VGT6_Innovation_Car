#include <math.h>
#include <stdio.h>
#include "Nav.h"
#include "AStar.h"
#include "Chassis.h"
#include "Attitude.h"
#include "Time.h"

Pose_t  g_pose;
float   g_odom_scale = 1.0f;
uint8_t g_nav_state      = NAV_IDLE;
uint8_t g_nav_wp_total   = 0;
uint8_t g_nav_wp_index   = 0;

/* 正在跟随的航点 (中断里只读) */
static float   s_wpx[NAV_WP_MAX];
static float   s_wpy[NAV_WP_MAX];
static uint8_t s_wpn = 0;
static uint8_t s_wpi = 0;
static float   s_yaw_t = 0.0f;          /* 当前段目标航向 */
static float   s_yaw_off = NAV_YAW_INIT; /* IMU yaw -> 地图 yaw 偏置 */
static float   s_seg_x0 = 0.0f;         /* 当前段起点 */
static float   s_seg_y0 = 0.0f;

/* 规划临时区 (主循环用) */
static uint8_t s_cells[MAP_CELLS * 2];
static int16_t s_px[MAP_CELLS];
static int16_t s_py[MAP_CELLS];
static float   s_tx[NAV_WP_MAX];
static float   s_ty[NAV_WP_MAX];
static uint8_t s_tn = 0;

/* 手动模式看门狗: v 命令无刷新超过该时长自动停 */
uint8_t  g_nav_manual_wd = 1;
static uint32_t s_manual_tick = 0;
#define NAV_MANUAL_TIMEOUT_MS   800u

void Nav_ManualKick(void) { s_manual_tick = Tick; }
void Nav_SetManualWd(uint8_t on) { g_nav_manual_wd = on ? 1 : 0; }

/* 增益 */
static float s_kp_pos = 3.50f;          /* 1/s */
static float s_kd_pos = 0.30f;          /* 速度阻尼 */
static float s_kp_yaw = 4.00f;
static float s_kd_yaw = 0.20f;

/*-----------------------------------------------------------------*/
static void s_cell_to_xy(int r, int c, float *x, float *y)
{
    *x = ((float)c + 0.5f) * MAP_CELL_MM;
    *y = ((float)(MAP_N - 1 - r) + 0.5f) * MAP_CELL_MM;
}

static void s_xy_to_cell(float x, float y, int *r, int *c)
{
    int cc = (int)(x / MAP_CELL_MM);
    int rr = (MAP_N - 1) - (int)(y / MAP_CELL_MM);
    if (cc < 0) cc = 0;  if (cc > MAP_N - 1) cc = MAP_N - 1;
    if (rr < 0) rr = 0;  if (rr > MAP_N - 1) rr = MAP_N - 1;
    *r = rr;  *c = cc;
}

/* 从 (r,c) 往外找最近的可行格 */
static uint8_t s_find_free(int *r, int *c)
{
    int rad, dr, dc;
    if (Map_Free(*r, *c)) return 1;
    for (rad = 1; rad <= 6; rad++)
    {
        for (dr = -rad; dr <= rad; dr++)
        {
            for (dc = -rad; dc <= rad; dc++)
            {
                if (Map_Free(*r + dr, *c + dc))
                {
                    *r += dr;  *c += dc;
                    return 1;
                }
            }
        }
    }
    return 0;
}

/* 两点之间能否直线通过 (按 0.25 格步长采样) */
static uint8_t s_los(float x0, float y0, float x1, float y1)
{
    float dx = x1 - x0, dy = y1 - y0;
    float len = sqrtf(dx * dx + dy * dy);
    int n = (int)(len / (MAP_CELL_MM * 0.25f)) + 1;
    int i, r, c;
    for (i = 0; i <= n; i++)
    {
        float t = (float)i / (float)n;
        s_xy_to_cell(x0 + dx * t, y0 + dy * t, &r, &c);
        if (!Map_Free(r, c)) return 0;
    }
    return 1;
}

/*-----------------------------------------------------------------*/
void Nav_Init(void)
{
    int r, c;
    Map_Init();

    /* 默认车放在下方启停区中心 (row45, col45) */
    r = 45;  c = 45;
    s_cell_to_xy(r, c, &g_pose.x, &g_pose.y);
    g_pose.yaw = NAV_YAW_INIT;          /* 初始车头朝地图上方(+y) */
    s_yaw_off  = NAV_YAW_INIT;
    s_yaw_t    = NAV_YAW_INIT;
    g_nav_state = NAV_IDLE;
    s_wpn = 0;  s_wpi = 0;
}

void Nav_ResetPose(uint8_t row, uint8_t col)
{
    s_cell_to_xy(row, col, &g_pose.x, &g_pose.y);
    Attitude_ZeroYaw();                 /* 摆车时车头须朝地图上方 */
    s_yaw_off  = NAV_YAW_INIT;
    g_pose.yaw = NAV_YAW_INIT;
    s_yaw_t    = NAV_YAW_INIT;
    __disable_irq();
    g_nav_state = NAV_IDLE;
    s_wpn = 0;  s_wpi = 0;
    __enable_irq();
    Chassis_Hold();
}

void Nav_Stop(void)
{
    g_nav_state = NAV_IDLE;
    s_wpn = 0;  s_wpi = 0;
    Chassis_Hold();
}

void Nav_SetGains(float kp, float kd, float kpy, float kdy)
{
    if (kp  > 0.0f) s_kp_pos = kp;
    if (kd >= 0.0f) s_kd_pos = kd;
    if (kpy > 0.0f) s_kp_yaw = kpy;
    if (kdy >= 0.0f) s_kd_yaw = kdy;
}

void Nav_GetGains(float *kp, float *kd, float *kpy, float *kdy)
{
    *kp = s_kp_pos;  *kd = s_kd_pos;
    *kpy = s_kp_yaw; *kdy = s_kd_yaw;
}

/*-----------------------------------------------------------------*
 *  里程计: 四轮正解 + yaw, 5ms 积分
 *-----------------------------------------------------------------*/
void Odom_Update(void)
{
    float c = cosf(g_pose.yaw);
    float s = sinf(g_pose.yaw);
    float vx = g_chassis.fb_vx * g_odom_scale;
    float vy = g_chassis.fb_vy * g_odom_scale;

    g_pose.x += (vx * c - vy * s) * 5.0f;       /* 0.005s * 1000mm/m */
    g_pose.y += (vx * s + vy * c) * 5.0f;
    g_pose.yaw = Attitude_WrapPi(g_att.yaw + s_yaw_off);
}

/*-----------------------------------------------------------------*
 *  跟随: 车头对准段方向, 拐点先原地转向再前进
 *-----------------------------------------------------------------*/

/* 段航向: 从段起点指向段终点 (世界系, rad) */
static float s_seg_yaw(float x0, float y0, float x1, float y1)
{
    float dx = x1 - x0, dy = y1 - y0;
    if (dx * dx + dy * dy < 1.0f) return s_yaw_t;   /* 段长<1mm 保持原航向 */
    return atan2f(dy, dx);
}

/* 推进到下一航点: 新段起点=刚过的航点, 更新段航向 */
static void s_wp_next(void)
{
    s_wpi++;
    g_nav_wp_index = s_wpi;
    if (s_wpi >= s_wpn)
    {
        g_nav_state = NAV_DONE;
        s_wpn = 0;
        Chassis_Hold();
        return;
    }
    s_seg_x0 = s_wpx[s_wpi - 1];
    s_seg_y0 = s_wpy[s_wpi - 1];
    s_yaw_t  = s_seg_yaw(s_seg_x0, s_seg_y0, s_wpx[s_wpi], s_wpy[s_wpi]);
}

void Nav_Update(void)
{
    float ex, ey, d, c, s, bx, by, vx, vy, wz, yaw_err, vlim, vn;

    if (g_nav_state == NAV_MANUAL)
    {
        if (g_nav_manual_wd && (uint32_t)(Tick - s_manual_tick) > NAV_MANUAL_TIMEOUT_MS)
            g_nav_state = NAV_IDLE;             /* 手动超时自动停 */
        else
            return;
    }

    /* ---- 航向环: 目标 = 当前段方向 ---- */
    yaw_err = Attitude_WrapPi(s_yaw_t - g_pose.yaw);
    wz = s_kp_yaw * yaw_err - s_kd_yaw * g_att.yaw_rate;

    if (g_nav_state != NAV_RUN || s_wpn == 0)
    {
        if (fabsf(yaw_err) < 0.035f) wz = 0.0f;     /* 静止 2 度死区, 防蠕动 */
        Chassis_SetBodyVel(0.0f, 0.0f, wz);
        return;
    }

    /* ---- 拐点: 段航向未对准先原地转向, 不平移 ---- */
    if (fabsf(yaw_err) > NAV_TURN_GATE)
    {
        Chassis_SetBodyVel(0.0f, 0.0f, wz);
        return;
    }

    /* ---- 误差 ---- */
    ex = s_wpx[s_wpi] - g_pose.x;
    ey = s_wpy[s_wpi] - g_pose.y;
    d  = sqrtf(ex * ex + ey * ey);

    /* ---- 到点/越过判据 ---- */
    if (d < ((s_wpi + 1 >= s_wpn) ? NAV_FINAL_MM : NAV_ARRIVE_MM))
    {
        s_wp_next();
        Chassis_SetBodyVel(0.0f, 0.0f, wz);
        return;
    }
    if ((s_wpi + 1 < s_wpn) && s_wpi > 0)
    {
        float ux = s_wpx[s_wpi] - s_wpx[s_wpi - 1];
        float uy = s_wpy[s_wpi] - s_wpy[s_wpi - 1];
        float ul = sqrtf(ux * ux + uy * uy);
        if (ul > 1.0f)
        {
            ux /= ul;  uy /= ul;
            if (((-ex) * ux + (-ey) * uy) > NAV_PASS_MM)
            {
                s_wp_next();
                Chassis_SetBodyVel(0.0f, 0.0f, wz);
                return;
            }
        }
    }

    /* ---- 世界 -> 车体 ---- */
    c = cosf(g_pose.yaw);
    s = sinf(g_pose.yaw);
    bx = ( ex * c + ey * s) * 0.001f;
    by = (-ex * s + ey * c) * 0.001f;

    /* ---- 位置环 + 速度阻尼 (vy 只做小的横向纠偏) ---- */
    vx = s_kp_pos * bx - s_kd_pos * g_chassis.fb_vx;
    vy = s_kp_pos * by - s_kd_pos * g_chassis.fb_vy;

    /* ---- 到点前按距离限速 (减速度 0.8 m/s^2) ---- */
    vlim = sqrtf(1.6f * d * 0.001f);
    if (vlim > CH_VXY_MAX) vlim = CH_VXY_MAX;
    vn = sqrtf(vx * vx + vy * vy);
    if (vn > vlim && vn > 0.0001f)
    {
        vx = vx * vlim / vn;
        vy = vy * vlim / vn;
    }

    Chassis_SetBodyVel(vx, vy, wz);
}

/*-----------------------------------------------------------------*/
/* 把 s_tx/s_ty 原子地交给中断 */
static void s_commit(void)
{
    uint8_t i;
    __disable_irq();
    s_wpn = s_tn;
    for (i = 0; i < s_tn; i++)
    {
        s_wpx[i] = s_tx[i];
        s_wpy[i] = s_ty[i];
    }
    s_wpi = 0;
    g_nav_wp_total = s_tn;
    g_nav_wp_index = 0;
    s_seg_x0 = g_pose.x;
    s_seg_y0 = g_pose.y;
    s_yaw_t = s_seg_yaw(s_seg_x0, s_seg_y0, s_wpx[0], s_wpy[0]);
    g_nav_state = NAV_RUN;
    __enable_irq();
}

/* 格序列 -> 毫米点 + 视线简化 */
static void s_build_from_cells(int n)
{
    int i, j;
    s_tn = 0;
    for (i = 0; i < n; i++)
    {
        float x, y;
        s_cell_to_xy(s_cells[2 * i], s_cells[2 * i + 1], &x, &y);
        s_px[i] = (int16_t)x;
        s_py[i] = (int16_t)y;
    }
    /* 起点 */
    s_tx[0] = (float)s_px[0];
    s_ty[0] = (float)s_py[0];
    s_tn = 1;
    i = 0;
    while (i < n - 1)
    {
        j = n - 1;
        while (j > i + 1 && !s_los((float)s_px[i], (float)s_py[i],
                                   (float)s_px[j], (float)s_py[j])) j--;
        i = j;
        if (s_tn < NAV_WP_MAX)
        {
            s_tx[s_tn] = (float)s_px[i];
            s_ty[s_tn] = (float)s_py[i];
            s_tn++;
        }
    }
}

/**
  * @brief  规划到指定格子
  * @retval 1 成功  0 失败
  */
uint8_t Nav_PlanToCell(uint8_t row, uint8_t col)
{
    int sr, sc, gr, gc, n;

    s_xy_to_cell(g_pose.x, g_pose.y, &sr, &sc);
    if (!s_find_free(&sr, &sc)) { g_nav_state = NAV_FAIL; return 0; }

    gr = row;  gc = col;
    if (!s_find_free(&gr, &gc)) { g_nav_state = NAV_FAIL; return 0; }

    n = AStar_Search(sr, sc, gr, gc, s_cells, MAP_CELLS);
    if (n <= 0) { g_nav_state = NAV_FAIL; return 0; }

    s_build_from_cells(n);
    if (s_tn == 0) { g_nav_state = NAV_FAIL; return 0; }
    if (s_tn == 1)                       /* 已经在该格 */
    {
        g_nav_state = NAV_DONE;
        return 1;
    }
    s_commit();
    return 1;
}

/**
  * @brief  某个功能区的停靠格 (离区域质心最近的可规划格)
  * @param  kind 2/3/4/5   idx 同类第几个 (2: 0=下方 1=上方; 3: 0=左侧 1=底部)
  */
uint8_t Nav_DockCell(uint8_t kind, uint8_t idx, uint8_t *row, uint8_t *col)
{
    int r, c, cnt = 0;
    float cx = 0.0f, cy = 0.0f;
    float bd = 1e9f;
    int br = -1, bc = -1;

    for (r = 0; r < MAP_N; r++)
    {
        for (c = 0; c < MAP_N; c++)
        {
            if (Map_Kind(r, c) != (uint8_t)kind) continue;
            /* 同类分区 */
            if (kind == 2)
            {
                if (idx == 0 && r < 24) continue;
                if (idx == 1 && r >= 24) continue;
            }
            else if (kind == 3)
            {
                if (idx == 0 && c >= 10) continue;
                if (idx == 1 && c < 10) continue;
            }
            cx += (float)r;  cy += (float)c;  cnt++;
        }
    }
    if (cnt == 0) return 0;
    cx /= (float)cnt;  cy /= (float)cnt;

    for (r = 0; r < MAP_N; r++)
    {
        for (c = 0; c < MAP_N; c++)
        {
            float dr, dc, d;
            if (!Map_Free(r, c)) continue;
            dr = (float)r - cx;  dc = (float)c - cy;
            d = dr * dr + dc * dc;
            if (d < bd) { bd = d;  br = r;  bc = c; }
        }
    }
    if (br < 0) return 0;
    *row = (uint8_t)br;
    *col = (uint8_t)bc;
    return 1;
}

/**
  * @brief  规划到精确世界坐标
  * @param  x_mm / y_mm  地图毫米坐标, x 向右 y 向上, 原点 = 左下角
  * @retval 1 成功  0 失败
  */
uint8_t Nav_PlanToMm(float x_mm, float y_mm)
{
    int r, c;
    float lim = (float)MAP_N * MAP_CELL_MM;

    if (x_mm < 0.0f)      x_mm = 0.0f;
    if (x_mm > lim)       x_mm = lim;
    if (y_mm < 0.0f)      y_mm = 0.0f;
    if (y_mm > lim)       y_mm = lim;

    s_xy_to_cell(x_mm, y_mm, &r, &c);
    if (!Nav_PlanToCell((uint8_t)r, (uint8_t)c)) return 0;

    /* 精确点替换最后一个航点 */
    __disable_irq();
    if (s_wpn > 0)
    {
        s_wpx[s_wpn - 1] = x_mm;
        s_wpy[s_wpn - 1] = y_mm;
    }
    __enable_irq();
    return 1;
}

uint8_t Nav_PlanToZone(uint8_t kind, uint8_t idx)
{
    uint8_t r = 0, c = 0;
    if (!Nav_DockCell(kind, idx, &r, &c)) return 0;
    return Nav_PlanToCell(r, c);
}

void Nav_PrintInfo(void)
{
    int r, c;
    s_xy_to_cell(g_pose.x, g_pose.y, &r, &c);
    printf("# pose x=%.1f y=%.1f mm  cell(%d,%d)  yaw=%.2f deg  state=%d  wp=%d/%d\r\n",
           g_pose.x, g_pose.y, r, c, g_pose.yaw * 57.29578f,
           g_nav_state, g_nav_wp_index, g_nav_wp_total);
    printf("# odom vx=%.3f vy=%.3f wz=%.3f  wheel fb %.2f / %.2f / %.2f / %.2f rad/s\r\n",
           g_chassis.fb_vx, g_chassis.fb_vy, g_chassis.fb_wz,
           g_chassis.fb_rad[0], g_chassis.fb_rad[1],
           g_chassis.fb_rad[2], g_chassis.fb_rad[3]);
    printf("# dir = %+.0f / %+.0f / %+.0f / %+.0f   (0=左前 1=右前 2=左后 3=右后)\r\n",
           g_chassis.dir[0], g_chassis.dir[1], g_chassis.dir[2], g_chassis.dir[3]);
}
