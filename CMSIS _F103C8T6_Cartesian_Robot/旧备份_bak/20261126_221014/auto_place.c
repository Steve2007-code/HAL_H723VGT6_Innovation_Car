/******************************************************************************
 * auto_place.c  --  自动放料流程：依次抓 3 个物料放到车上 1/2/3 号料盘
 *
 * 触发 <<M P>>（板间 sub=8），中止 <<M S>>（sub=10）。
 * 主循环 10ms 任务调用 AutoPlace_Tick() 推进，每步都等所有轴到位再走下一步。
 *
 * 安全约定（来自实车要求）：
 *   1) 抓住物料后一律"先把 B 抬到 20000，再退 A 到 4000"，避免爪子横向扫到车上部件；
 *   2) 放料时 A 先到位，再降 B 到 16000，才算把物料放进盘里；
 *   3) 松手用 G1(65 度) 而不是 G2(100 度)，大开容易碰到旁边的东西。
 *
 * 三个料盘(A, yaw)：1 号 (700, -113.7) / 2 号 (600, -90) / 3 号 (1000, -67.5)
 ******************************************************************************/
#include "auto_place.h"
#include "stepper.h"
#include "servo.h"
#include "track.h"
#include "yaw_track.h"
#include "mcu_link.h"
#include "link_config.h"

/* ---------------- 速度 / 超时 ---------------- */
#define AP_A_PPS          6000      /* A 轴移动速度 步/秒 */
#define AP_B_PPS          12000     /* B 轴移动速度 步/秒 */
#define AP_ACCEL_MS       400       /* 加减速时间 */
#define AP_YAW_VEL        3000      /* yaw 匀速段速度 mrad/s (= 172 度/秒) */
#define AP_YAW_ACC        8000      /* yaw 加/减速度 mrad/s^2: 0->3000 约 0.38s */
#define AP_YAW_VMIN       200       /* 最小速度 mrad/s, 防起步爬行过久 */
#define AP_YAW_DT_MS      10        /* 本状态机的节拍 */
#define AP_SETTLE_MS      150       /* 每步下发后先等这么久, 让轴真正起动 */
#define AP_STEP_TIMEOUT   3000u     /* 单步最长等待: 正常单步 <2s, 3s 足够; 断链时也不会拖太久 */
#define AP_YAW_TOL        40        /* yaw 到位容差 mrad (约 2.3 度) */

/* ---------------- 步骤表哨兵 ---------------- */
#define AP_NA   (-10000)            /* 本步不动 A */
#define AP_NB   (-10000)            /* 本步不动 B */
#define AP_NY   (-32768)            /* 本步不动 yaw */
#define AP_NS   (-1)                /* 本步不动舵机 */

typedef struct
{
    int32_t  a;         /* A 轴目标微步 */
    int32_t  b;         /* B 轴目标微步 */
    int16_t  y10;       /* yaw 目标: 度 x 10 */
    int16_t  servo;     /* 舵机角度(度) 50 / 65 / 100 */
    uint16_t hold;      /* 本步额外停留 ms, 给舵机留动作时间 */
} ApStep_t;

static const ApStep_t S_PLACE[] =
{
    /*   A       B     yaw(度x10) 舵机  hold   说明 */
    { 1000,      0,        0,     65,  1200},  /* 0  抓取准备: B 降到抓取位, A 伸出, 小开 */
    { AP_NA,  AP_NB,   AP_NY,      50,   900},  /* 1  合爪, 夹住物料 */
    { AP_NA, 20000,    AP_NY,  AP_NS,     0},  /* 2  先把 B 抬到顶(防横向碰撞) */
    { 4000,  AP_NB,    -1137,  AP_NS,     0},  /* 3  退 A + 转到 1 号盘 */
    {  700,  AP_NB,    AP_NY,  AP_NS,     0},  /* 4  靠近 1 号盘 */
    { AP_NA, 16000,    AP_NY,  AP_NS,     0},  /* 5  下降到料盘高度 */
    { AP_NA,  AP_NB,   AP_NY,      65,   800},  /* 6  微松 G1 放料 */
    { AP_NA, 20000,    AP_NY,  AP_NS,     0},  /* 7  先抬 B */
    { 4000,  AP_NB,        0,  AP_NS,     0},  /* 8  退 A + 转回 */

    { 1000,      0,    AP_NY,     65,  1200},  /* 9  回到抓取位 + 小开 */
    { AP_NA,  AP_NB,   AP_NY,      50,   900},  /* 10 合爪 */
    { AP_NA, 20000,    AP_NY,  AP_NS,     0},  /* 11 抬 B */
    { 4000,  AP_NB,     -900,  AP_NS,     0},  /* 12 退 A + 转到 2 号盘 */
    {  600,  AP_NB,    AP_NY,  AP_NS,     0},  /* 13 靠近 2 号盘 */
    { AP_NA, 16000,    AP_NY,  AP_NS,     0},  /* 14 下降 */
    { AP_NA,  AP_NB,   AP_NY,      65,   800},  /* 15 微松 */
    { AP_NA, 20000,    AP_NY,  AP_NS,     0},  /* 16 抬 B */
    { 4000,  AP_NB,        0,  AP_NS,     0},  /* 17 回 */

    { 1000,      0,    AP_NY,     65,  1200},  /* 18 抓取位 */
    { AP_NA,  AP_NB,   AP_NY,      50,   900},  /* 19 合爪 */
    { AP_NA, 20000,    AP_NY,  AP_NS,     0},  /* 20 抬 B */
    { 4000,  AP_NB,     -675,  AP_NS,     0},  /* 21 退 A + 转到 3 号盘 */
    { 1000,  AP_NB,    AP_NY,  AP_NS,     0},  /* 22 靠近 3 号盘 */
    { AP_NA, 16000,    AP_NY,  AP_NS,     0},  /* 23 下降 */
    { AP_NA,  AP_NB,   AP_NY,      65,   800},  /* 24 微松 */
    { AP_NA, 20000,    AP_NY,  AP_NS,     0},  /* 25 抬 B */
    { 4000,  AP_NB,        0,  AP_NS,     0},  /* 26 退 A + 转回(结束位) */
};

#define PLACE_N     ((uint16_t)(sizeof(S_PLACE) / sizeof(S_PLACE[0])))

/* ==========================================================================
 * 卸料流程（放料的反向）：依次到 3 -> 2 -> 1 号盘把料夹起来，送回抓取位放下
 *
 * 每个盘的动作为：
 *   退到安全位并张开 -> 转到盘 -> 靠近 -> 下降到 16000 -> 夹住
 *   -> 抬到 20000 -> 退回安全位并转回 0 -> 降到抓取位 -> 松手 -> 抬升
 *
 * 抬起/退 A 的顺序同样是"先抬 B 再退 A"，避免爪子横向扫到车上部件。
 * ========================================================================== */
static const ApStep_t S_UNLOAD[] =
{
    /*   A       B     yaw(度x10) 舵机  hold   说明 */
    { 4000, 20000,        0,     65,  1000},  /* 0  起点: 回安全位并张开爪子 */
    { AP_NA,  AP_NB,    -675,  AP_NS,     0},  /* 1  转到 3 号盘 */
    { 1000,  AP_NB,    AP_NY,  AP_NS,     0},  /* 2  靠近 3 号盘 (A=1000) */
    { AP_NA, 16000,    AP_NY,  AP_NS,     0},  /* 3  下降到盘内 */
    { AP_NA,  AP_NB,   AP_NY,      50,   800},  /* 4  合爪夹住物料 */
    { AP_NA, 20000,    AP_NY,  AP_NS,     0},  /* 5  抬 B */
    { 4000,  AP_NB,        0,  AP_NS,     0},  /* 6  退 A + 转回 */

    { 1000,      0,    AP_NY,  AP_NS,     0},  /* 7  降到抓取位 */
    { AP_NA,  AP_NB,   AP_NY,      65,   900},  /* 8  松手放料 */
    { AP_NA, 20000,    AP_NY,  AP_NS,     0},  /* 9  抬 B */

    { 4000,  AP_NB,     -900,     65,     0},  /* 10 退 A + 转 2 号盘 + 张开 */
    {  600,  AP_NB,    AP_NY,  AP_NS,     0},  /* 11 靠近 2 号盘 (A=600) */
    { AP_NA, 16000,    AP_NY,  AP_NS,     0},  /* 12 下降 */
    { AP_NA,  AP_NB,   AP_NY,      50,   800},  /* 13 夹住 */
    { AP_NA, 20000,    AP_NY,  AP_NS,     0},  /* 14 抬 B */
    { 4000,  AP_NB,        0,  AP_NS,     0},  /* 15 退 A + 转回 */

    { 1000,      0,    AP_NY,  AP_NS,     0},  /* 16 降到抓取位 */
    { AP_NA,  AP_NB,   AP_NY,      65,   900},  /* 17 松手 */
    { AP_NA, 20000,    AP_NY,  AP_NS,     0},  /* 18 抬 B */

    { 4000,  AP_NB,    -1137,     65,     0},  /* 19 退 A + 转 1 号盘 + 张开 */
    {  700,  AP_NB,    AP_NY,  AP_NS,     0},  /* 20 靠近 1 号盘 (A=700) */
    { AP_NA, 16000,    AP_NY,  AP_NS,     0},  /* 21 下降 */
    { AP_NA,  AP_NB,   AP_NY,      50,   800},  /* 22 夹住 */
    { AP_NA, 20000,    AP_NY,  AP_NS,     0},  /* 23 抬 B */
    { 4000,  AP_NB,        0,  AP_NS,     0},  /* 24 退 A + 转回 */
    { 1000,      0,    AP_NY,  AP_NS,     0},  /* 25 降到抓取位 */
    { AP_NA,  AP_NB,   AP_NY,      65,   900},  /* 26 松手, 把料放到抓取位 */
    { AP_NA, 20000,    AP_NY,  AP_NS,     0},  /* 27 抬 B(结束位) */
};

#define UNLOAD_N    ((uint16_t)(sizeof(S_UNLOAD) / sizeof(S_UNLOAD[0])))



static const ApStep_t *s_tbl = 0;      /* 当前流程的步骤表 */
static uint16_t        s_n   = 0;      /* 当前流程的步数 */
static uint8_t  s_run     = 0;
static uint16_t s_step    = 0;
static uint8_t  s_sent    = 0;
static uint32_t s_step_ms = 0;
static int32_t  s_y_tgt   = 0;
static int32_t  s_y_vel   = 0;      /* 当前下发的 yaw 限速(梯形曲线用) */
static uint8_t  s_y_plan  = 0;      /* 1 = 本流程需要做 yaw 速度规划 */

/* 度 x10 -> mrad。1 度 = 17.4532925 mrad, 所以 d10(度x10) 要乘 1.74532925,
   即 d10 * 1745 / 1000 (用 1745/1000 近似, 误差 < 0.03 度)。
   注意分母必须是 1000: 写成 /100 会把目标放大 10 倍, 再被 yaw 限幅钳死。 */
static int32_t ap_yaw_mrad(int16_t d10)
{
    return (int32_t)d10 * 1745 / 1000;
}

/* 整数开方(牛顿/逐位法), 用于 v = sqrt(2*a*s) 的减速曲线 */
static uint32_t ap_isqrt(uint32_t x)
{
    uint32_t r = 0, b = 1u << 30;

    while (b > x) b >>= 2;
    while (b)
    {
        if (x >= r + b) { x -= r + b; r = (r >> 1) + b; }
        else            { r >>= 1; }
        b >>= 2;
    }
    return r;
}

/* 每个 10ms 更新一次 yaw 限速, 形成 T 型(梯形)速度曲线:
   起步: 每拍最多加 AP_YAW_ACC*DT  -> 不再"瞬间到最高速"
   接近: v <= sqrt(2*a*剩余角)     -> 提前减速, 到位不冲过头 */
static void ap_yaw_speed_tick(void)
{
    int32_t act, dist, v, vdec, vinc;

    if (!s_y_plan) return;
    if (Link_IsLost()) return;           /* 断链拿不到实际角, 保持上次速度 */

    act  = Link_GetYawPos();
    dist = s_y_tgt - act;
    if (dist < 0) dist = -dist;

    vdec = (int32_t)ap_isqrt((uint32_t)(2 * AP_YAW_ACC * dist));
    vinc = s_y_vel + (AP_YAW_ACC * AP_YAW_DT_MS / 1000);

    v = (vdec < vinc) ? vdec : vinc;
    if (v > AP_YAW_VEL)  v = AP_YAW_VEL;
    if (v < AP_YAW_VMIN) v = AP_YAW_VMIN;

    if (v != s_y_vel)
    {
        s_y_vel = v;
        YawTrack_SetManualVel(v);
    }
}

static uint8_t ap_ready(const ApStep_t *st)
{
    if (Stepper_IsBusy(AXIS_A) || Stepper_IsBusy(AXIS_B)) return 0;

    if (st->y10 != AP_NY && !Link_IsLost())
    {
        int32_t e = Link_GetYawPos() - s_y_tgt;
        if (e < 0) e = -e;
        if (e > AP_YAW_TOL) return 0;
    }
    return 1;
}

/* 通用启动: 两套流程共用同一套状态机 */
static void ap_start(const ApStep_t *tbl, uint16_t n)
{
    Track_SetRun(0);                  /* 停视觉 PID, 否则会把手动 A 轴位置顶掉 */
    Stepper_TrackOn(AXIS_A, 0);       /* 干净退出跟随模式 */

    s_y_tgt   = Link_GetYawTarget();  /* 起点沿用当前 yaw 目标, 避免第一步突变 */
    s_y_vel   = AP_YAW_VMIN;
    s_y_plan  = 0;                    /* 遇到第一个带 yaw 的步骤再开启 */
    s_tbl     = tbl;
    s_n       = n;
    s_step    = 0;
    s_sent    = 0;
    s_step_ms = g_ms;
    s_run     = 1;
}

void AutoPlace_Start(void)  { ap_start(S_PLACE,  PLACE_N);  }
void AutoUnload_Start(void) { ap_start(S_UNLOAD, UNLOAD_N); }

void AutoPlace_Stop(void)
{
    if (!s_run) return;
    s_run = 0;
    Stepper_Stop(AXIS_A, 300);
    Stepper_Stop(AXIS_B, 300);
}

void AutoPlace_Tick(void)
{
    const ApStep_t *st;
    uint32_t el, need;

    if (!s_run) return;

    st = &s_tbl[s_step];

    /* ---- 本步还没下发 ---- */
    if (!s_sent)
    {
        if (st->a != AP_NA) Stepper_MoveTo(AXIS_A, st->a, AP_A_PPS, AP_ACCEL_MS);
        if (st->b != AP_NB) Stepper_MoveTo(AXIS_B, st->b, AP_B_PPS, AP_ACCEL_MS);

        if (st->y10 != AP_NY)
        {
            s_y_tgt = ap_yaw_mrad(st->y10);
            /* 与 YawTrack_SetManual 内部限幅保持一致, 否则到位判定永远对不上 */
            if (s_y_tgt >  LINK_POS_MAX_MRAD) s_y_tgt =  LINK_POS_MAX_MRAD;
            if (s_y_tgt <  LINK_POS_MIN_MRAD) s_y_tgt =  LINK_POS_MIN_MRAD;

            /* 从最小速度起步, 之后由 ap_yaw_speed_tick() 按梯形曲线提速 */
            s_y_vel  = AP_YAW_VMIN;
            s_y_plan = 1;
            YawTrack_SetManual(s_y_tgt, s_y_vel);
        }
        if (st->servo != AP_NS) Servo_SetAngle((float)st->servo);

        s_sent    = 1;
        s_step_ms = g_ms;
        return;
    }

    /* ---- 等所有轴到位 ---- */
    ap_yaw_speed_tick();                 /* 每拍更新 yaw 限速(梯形加减速) */

    el   = g_ms - s_step_ms;
    need = (uint32_t)AP_SETTLE_MS + (uint32_t)st->hold;
    if (el < need) return;
    if (el < AP_STEP_TIMEOUT && !ap_ready(st)) return;

    s_step++;
    s_sent = 0;

    if (s_step >= s_n) s_run = 0;          /* 全部完成 */
}

uint8_t AutoPlace_IsRunning(void)
{
    return s_run;
}

uint16_t AutoPlace_GetStep(void)
{
    return s_run ? (uint16_t)(s_step + 1u) : 0u;
}
