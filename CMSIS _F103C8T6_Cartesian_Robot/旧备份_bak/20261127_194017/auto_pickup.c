/******************************************************************************
 * auto_pickup.c  --  任务码驱动的第一批物料夹取（上电静止调试版）
 *
 * 一个物料一轮，共 3 轮，每轮 15 步：
 *   ① 高位归位：B 先到顶, 再 A→4000, Y→+90°, 夹爪张开
 *   ② 发 <<color N>> 并开补光灯, 摄像头发该颜色物料的位置
 *   ③ 启动视觉跟踪：yaw 跟 dx, A 轴跟 dy
 *   ④ 连续若干帧 |dx|、|dy| 都小于阈值 -> 认为对准
 *   ⑤ 发 <<color 0>> 停止追踪, 锁定 yaw 与 A
 *   ⑥ B 降到 0, 夹爪合上夹住, 关补光灯
 *   ⑦ 抬 B → 退 A + 转到对应料盘 → 靠近 → 降到 16000 → 松手 → 抬 B → 回 +90°
 *
 * 防撞约定与 <<M P>> 一致：所有 yaw 旋转都在 B=20000 高位；先抬 B 再退 A。
 *
 * 料盘坐标（与 <<M P>> 一致）：
 *   1 号 A=700  Y=-113.7°    2 号 A=600  Y=-90°    3 号 A=1000 Y=-67.5°
 ******************************************************************************/
#include "auto_pickup.h"
#include "stepper.h"
#include "servo.h"
#include "track.h"
#include "yaw_track.h"
#include "mcu_link.h"
#include "vision.h"
#include "qr_scan.h"
#include "backlight.h"
#include "link_config.h"

/* ---------------- 参数 ---------------- */
#define PU_A_PPS        12000
#define PU_B_PPS        24000
#define PU_ACCEL_MS     250
#define PU_YAW_VEL      6000        /* yaw 匀速段 mrad/s */
#define PU_SETTLE_MS    40
#define PU_STEP_TIMEOUT 2500u       /* 普通步超时 */
#define PU_WAIT_TIMEOUT 6000u       /* 等视觉对准的超时(要留够 A 轴移动时间) */
#define PU_YAW_TOL      35          /* yaw 到位容差 mrad (2.0 度) */

#define PU_ALIGN_PX     8           /* 对准判据: |dx|、|dy| 都 <= 8 像素 */
#define PU_ALIGN_FRAMES 5           /* 连续 5 帧满足即认为对准 */

#define PU_YAW_HOME     900         /* +90.0 度 (度x10) */
#define PU_YAW_PARK     (-1150)     /* -115.0 度: 上电待命位 */

/* ---------------- 步骤表哨兵 ---------------- */
#define PU_NA   (-10000)            /* 本步不动 A */
#define PU_NB   (-10000)            /* 本步不动 B */
#define PU_NY   (-32768)            /* 本步不动 yaw */
#define PU_NS   (-1)                /* 本步不动舵机 */

/* ---------------- 特殊动作 ---------------- */
#define PU_ACT_NONE      0u
#define PU_ACT_COLOR     1u         /* 发 <<color N>> + 开补光灯 */
#define PU_ACT_TRACK_ON  2u         /* 启动视觉跟踪(yaw/A 接管) */
#define PU_ACT_WAIT      3u         /* 等对准(连续 N 帧误差达标) */
#define PU_ACT_LOCK      4u         /* 发 <<color 0>> + 锁定 yaw 与 A */
#define PU_ACT_LIGHT_OFF 5u         /* 关补光灯 */

typedef struct
{
    int32_t  a;
    int32_t  b;
    int16_t  y10;      /* 度 x10 */
    int16_t  servo;
    uint16_t hold;
    uint8_t  act;
    uint8_t  mat;      /* 属于第几个物料(0~2), 0xFF=不属于任何物料 */
} PuStep_t;

/* 每轮 15 步, 共 3 轮, 末尾 2 步回待命位 */
static const PuStep_t S_PU[] =
{
    /* ---------- 第 1 个物料 → 1 号盘 (A700, Y-113.7) ---------- */
    { 4000, 20000,  PU_YAW_HOME,  65,  400, PU_ACT_NONE,      0 },  /*  0 高位归位 B顶/A4000/Y+90/张开 */
    { PU_NA, PU_NB, PU_NY,      PU_NS,    0, PU_ACT_COLOR,     0 },  /*  1 发色 + 开灯 */
    { PU_NA, PU_NB, PU_NY,      PU_NS,  250, PU_ACT_TRACK_ON,  0 },  /*  2 启动视觉跟踪 */
    { PU_NA, PU_NB, PU_NY,      PU_NS,    0, PU_ACT_WAIT,      0 },  /*  3 等对准 */
    { PU_NA, PU_NB, PU_NY,      PU_NS,  250, PU_ACT_LOCK,      0 },  /*  4 锁定 + 停追踪 */
    { PU_NA,     0, PU_NY,      PU_NS,    0, PU_ACT_NONE,      0 },  /*  5 B 降到抓取高度 */
    { PU_NA, PU_NB, PU_NY,          50, 350, PU_ACT_NONE,      0 },  /*  6 G0 夹住 */
    { PU_NA, PU_NB, PU_NY,      PU_NS,    0, PU_ACT_LIGHT_OFF, 0 },  /*  7 关灯 */
    { PU_NA, 20000, PU_NY,      PU_NS,    0, PU_ACT_NONE,      0 },  /*  8 抬 B */
    { 4000,  PU_NB,   -1137,    PU_NS,    0, PU_ACT_NONE,      0 },  /*  9 退 A + 转 1 号盘 */
    {  700,  PU_NB, PU_NY,      PU_NS,    0, PU_ACT_NONE,      0 },  /* 10 靠近 1 号盘 */
    { PU_NA, 16000, PU_NY,      PU_NS,    0, PU_ACT_NONE,      0 },  /* 11 下降到盘内 */
    { PU_NA, PU_NB, PU_NY,          65, 300, PU_ACT_NONE,      0 },  /* 12 G1 松手 */
    { PU_NA, 20000, PU_NY,      PU_NS,    0, PU_ACT_NONE,      0 },  /* 13 抬 B */
    { 4000,  PU_NB,  PU_YAW_HOME, PU_NS,  0, PU_ACT_NONE,      0 },  /* 14 退 A + 回 +90 */

    /* ---------- 第 2 个物料 → 2 号盘 (A600, Y-90) ---------- */
    { 4000, 20000,  PU_YAW_HOME,  65,  400, PU_ACT_NONE,      1 },
    { PU_NA, PU_NB, PU_NY,      PU_NS,    0, PU_ACT_COLOR,     1 },
    { PU_NA, PU_NB, PU_NY,      PU_NS,  250, PU_ACT_TRACK_ON,  1 },
    { PU_NA, PU_NB, PU_NY,      PU_NS,    0, PU_ACT_WAIT,      1 },
    { PU_NA, PU_NB, PU_NY,      PU_NS,  250, PU_ACT_LOCK,      1 },
    { PU_NA,     0, PU_NY,      PU_NS,    0, PU_ACT_NONE,      1 },
    { PU_NA, PU_NB, PU_NY,          50, 350, PU_ACT_NONE,      1 },
    { PU_NA, PU_NB, PU_NY,      PU_NS,    0, PU_ACT_LIGHT_OFF, 1 },
    { PU_NA, 20000, PU_NY,      PU_NS,    0, PU_ACT_NONE,      1 },
    { 4000,  PU_NB,    -900,    PU_NS,    0, PU_ACT_NONE,      1 },  /* 转 2 号盘 Y-90 */
    {  600,  PU_NB, PU_NY,      PU_NS,    0, PU_ACT_NONE,      1 },
    { PU_NA, 16000, PU_NY,      PU_NS,    0, PU_ACT_NONE,      1 },
    { PU_NA, PU_NB, PU_NY,          65, 300, PU_ACT_NONE,      1 },
    { PU_NA, 20000, PU_NY,      PU_NS,    0, PU_ACT_NONE,      1 },
    { 4000,  PU_NB,  PU_YAW_HOME, PU_NS,  0, PU_ACT_NONE,      1 },

    /* ---------- 第 3 个物料 → 3 号盘 (A1000, Y-67.5) ---------- */
    { 4000, 20000,  PU_YAW_HOME,  65,  400, PU_ACT_NONE,      2 },
    { PU_NA, PU_NB, PU_NY,      PU_NS,    0, PU_ACT_COLOR,     2 },
    { PU_NA, PU_NB, PU_NY,      PU_NS,  250, PU_ACT_TRACK_ON,  2 },
    { PU_NA, PU_NB, PU_NY,      PU_NS,    0, PU_ACT_WAIT,      2 },
    { PU_NA, PU_NB, PU_NY,      PU_NS,  250, PU_ACT_LOCK,      2 },
    { PU_NA,     0, PU_NY,      PU_NS,    0, PU_ACT_NONE,      2 },
    { PU_NA, PU_NB, PU_NY,          50, 350, PU_ACT_NONE,      2 },
    { PU_NA, PU_NB, PU_NY,      PU_NS,    0, PU_ACT_LIGHT_OFF, 2 },
    { PU_NA, 20000, PU_NY,      PU_NS,    0, PU_ACT_NONE,      2 },
    { 4000,  PU_NB,    -675,    PU_NS,    0, PU_ACT_NONE,      2 },  /* 转 3 号盘 Y-67.5 */
    { 1000,  PU_NB, PU_NY,      PU_NS,    0, PU_ACT_NONE,      2 },
    { PU_NA, 16000, PU_NY,      PU_NS,    0, PU_ACT_NONE,      2 },
    { PU_NA, PU_NB, PU_NY,          65, 300, PU_ACT_NONE,      2 },
    { PU_NA, 20000, PU_NY,      PU_NS,    0, PU_ACT_NONE,      2 },
    { 4000,  PU_NB,  PU_YAW_HOME, PU_NS,  0, PU_ACT_NONE,      2 },

    /* ---------- 回到上电待命位 (A1000 / B20000 / Y-115) ---------- */
    { PU_NA, 20000, PU_YAW_PARK, PU_NS,   0, PU_ACT_NONE,   0xFF },
    { 1000,  PU_NB, PU_NY,      PU_NS,    0, PU_ACT_NONE,   0xFF },
};

#define PU_STEP_N   ((uint16_t)(sizeof(S_PU) / sizeof(S_PU[0])))

/* ---------------- 状态 ---------------- */
static uint8_t  s_run  = 0;
static uint16_t s_step = 0;
static uint8_t  s_sent = 0;
static uint32_t s_step_ms = 0;
static int32_t  s_y_tgt = 0;

static uint8_t  s_color[PU_SLOT_NUM];       /* 第一批三个物料的颜色码(顺序) */
static uint8_t  s_slot_color[PU_SLOT_NUM];  /* 每个料盘上放的颜色 */

static uint32_t s_align_seen_ms = 0;
static uint8_t  s_align_cnt = 0;

/* 度 x10 -> mrad */
static int32_t pu_yaw_mrad(int16_t d10)
{
    return (int32_t)d10 * 1745 / 1000;
}

/* 发 "<<color N>>", N=0 表示停止追踪 */
static void pu_send_color(uint8_t n)
{
    char buf[16];
    char *p = buf;
    const char *s = "<<color ";

    while (*s) *p++ = *s++;
    *p++ = (char)('0' + (n % 10));
    *p++ = '>';
    *p++ = '>';
    *p   = 0;
    Vision_SendStr(buf);
}

/* 解析任务码第 1 组(3 位颜色码)。成功返回 1 */
static uint8_t pu_parse_task(void)
{
    uint8_t i;

    if (g_qr_code[3] != '+' || g_qr_code[7] != '+' || g_qr_code[11] != '+') return 0;

    for (i = 0; i < PU_SLOT_NUM; i++)
    {
        char c = g_qr_code[i];
        if (c < '0' || c > '9') return 0;
        s_color[i] = (uint8_t)(c - '0');
        s_slot_color[i] = s_color[i];       /* 第 i 个物料放进第 i 号盘 */
    }
    return 1;
}

/* 新帧到达时累计对准帧数 */
static void pu_align_tick(void)
{
    int16_t dx, dy;

    if (g_last_frame_ms == s_align_seen_ms) return;
    s_align_seen_ms = g_last_frame_ms;

    if (!Track_GetValid()) { s_align_cnt = 0; return; }

    dx = Track_GetDx();
    dy = Track_GetDy();
    if (dx < 0) dx = -dx;
    if (dy < 0) dy = -dy;

    if (dx <= PU_ALIGN_PX && dy <= PU_ALIGN_PX)
    {
        if (s_align_cnt < 255u) s_align_cnt++;
    }
    else
    {
        s_align_cnt = 0;
    }
}

static uint8_t pu_ready(const PuStep_t *st)
{
    /* 等对准: 只看帧计数, 不看轴(此时 A 在跟随模式, IsBusy 恒为 1) */
    if (st->act == PU_ACT_WAIT)
    {
        return (s_align_cnt >= PU_ALIGN_FRAMES) ? 1u : 0u;
    }

    if (Stepper_IsBusy(AXIS_A) || Stepper_IsBusy(AXIS_B)) return 0;

    if (st->y10 != PU_NY && !Link_IsLost())
    {
        int32_t e = Link_GetYawPos() - s_y_tgt;
        if (e < 0) e = -e;
        if (e > PU_YAW_TOL) return 0;
    }
    return 1;
}

void AutoPickup_Start(void)
{
    if (!pu_parse_task()) return;          /* 没有有效任务码就不启动 */

    Track_SetRun(0);
    Stepper_TrackOn(AXIS_A, 0);

    s_y_tgt        = Link_GetYawTarget();
    s_step         = 0;
    s_sent         = 0;
    s_step_ms      = g_ms;
    s_align_cnt    = 0;
    s_align_seen_ms = 0;
    s_run          = 1;
}

void AutoPickup_Stop(void)
{
    if (!s_run) return;
    s_run = 0;
    Stepper_Stop(AXIS_A, 300);
    Stepper_Stop(AXIS_B, 300);
    Backlight_Set(0);
}

void AutoPickup_Tick(void)
{
    const PuStep_t *st;
    uint32_t el, need, timeout;

    if (!s_run) return;

    st = &S_PU[s_step];

    /* ---- 下发本步 ---- */
    if (!s_sent)
    {
        if (st->a != PU_NA) Stepper_MoveTo(AXIS_A, st->a, PU_A_PPS, PU_ACCEL_MS);
        if (st->b != PU_NB) Stepper_MoveTo(AXIS_B, st->b, PU_B_PPS, PU_ACCEL_MS);

        if (st->y10 != PU_NY)
        {
            s_y_tgt = pu_yaw_mrad(st->y10);
            if (s_y_tgt >  LINK_POS_MAX_MRAD) s_y_tgt =  LINK_POS_MAX_MRAD;
            if (s_y_tgt <  LINK_POS_MIN_MRAD) s_y_tgt =  LINK_POS_MIN_MRAD;
            YawTrack_SetManual(s_y_tgt, PU_YAW_VEL);
        }
        if (st->servo != PU_NS) Servo_SetAngle((float)st->servo);

        switch (st->act)
        {
        case PU_ACT_COLOR:                    /* 让摄像头去找这个颜色, 并补光 */
            pu_send_color(s_color[st->mat]);
            Backlight_Set(1);
            break;

        case PU_ACT_TRACK_ON:                 /* yaw 跟 dx, A 轴跟 dy */
            YawTrack_SetAuto();
            Track_SetRun(1);
            Stepper_TrackOn(AXIS_A, 1);
            break;

        case PU_ACT_WAIT:                     /* 开始计数 */
            s_align_cnt     = 0;
            s_align_seen_ms = 0;
            break;

        case PU_ACT_LOCK:                     /* 停追踪, 锁住当前位置 */
            pu_send_color(0);
            Track_SetRun(0);
            Stepper_TrackOn(AXIS_A, 0);
            if (!Link_IsLost())
            {
                YawTrack_SetManual(Link_GetYawPos(), PU_YAW_VEL);
                s_y_tgt = Link_GetYawPos();
            }
            break;

        case PU_ACT_LIGHT_OFF:
            Backlight_Set(0);
            break;

        default:
            break;
        }

        s_sent    = 1;
        s_step_ms = g_ms;
        return;
    }

    /* ---- 等待 ---- */
    if (st->act == PU_ACT_WAIT) pu_align_tick();

    el      = g_ms - s_step_ms;
    need    = (uint32_t)PU_SETTLE_MS + (uint32_t)st->hold;
    timeout = (st->act == PU_ACT_WAIT) ? PU_WAIT_TIMEOUT : PU_STEP_TIMEOUT;

    if (el < need) return;
    if (el < timeout && !pu_ready(st)) return;

    s_step++;
    s_sent = 0;

    if (s_step >= PU_STEP_N) s_run = 0;
}

uint8_t AutoPickup_IsRunning(void) { return s_run; }

uint16_t AutoPickup_GetStep(void)
{
    return s_run ? (uint16_t)(s_step + 1u) : 0u;
}

uint8_t AutoPickup_GetSlotColor(uint8_t idx)
{
    return (idx < PU_SLOT_NUM) ? s_slot_color[idx] : 0u;
}

uint8_t AutoPickup_GetOrderColor(uint8_t idx)
{
    return (idx < PU_SLOT_NUM) ? s_color[idx] : 0u;
}
