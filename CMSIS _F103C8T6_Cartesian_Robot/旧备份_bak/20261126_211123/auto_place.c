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
#define AP_YAW_VEL        3000      /* yaw 限速 mrad/s (= 172 度/秒) */
#define AP_SETTLE_MS      150       /* 每步下发后先等这么久, 让轴真正起动 */
#define AP_STEP_TIMEOUT   8000u     /* 单步最长等待, 超时强制下一步(防卡死) */
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

static const ApStep_t S_STEP[] =
{
    /*   A       B     yaw(度x10) 舵机  hold   说明 */
    { 1000,      0,        0,    100,  1200},  /* 0  抓取准备: B 降到抓取位, A 伸出, 大开 */
    { AP_NA,  AP_NB,   AP_NY,      50,   900},  /* 1  合爪, 夹住物料 */
    { AP_NA, 20000,    AP_NY,  AP_NS,     0},  /* 2  先把 B 抬到顶(防横向碰撞) */
    { 4000,  AP_NB,    -1137,  AP_NS,     0},  /* 3  退 A + 转到 1 号盘 */
    {  700,  AP_NB,    AP_NY,  AP_NS,     0},  /* 4  靠近 1 号盘 */
    { AP_NA, 16000,    AP_NY,  AP_NS,     0},  /* 5  下降到料盘高度 */
    { AP_NA,  AP_NB,   AP_NY,      65,   800},  /* 6  微松 G1 放料 */
    { AP_NA, 20000,    AP_NY,  AP_NS,     0},  /* 7  先抬 B */
    { 4000,  AP_NB,        0,  AP_NS,     0},  /* 8  退 A + 转回 */

    { 1000,      0,    AP_NY,    100,  1200},  /* 9  回到抓取位 + 大开 */
    { AP_NA,  AP_NB,   AP_NY,      50,   900},  /* 10 合爪 */
    { AP_NA, 20000,    AP_NY,  AP_NS,     0},  /* 11 抬 B */
    { 4000,  AP_NB,     -900,  AP_NS,     0},  /* 12 退 A + 转到 2 号盘 */
    {  600,  AP_NB,    AP_NY,  AP_NS,     0},  /* 13 靠近 2 号盘 */
    { AP_NA, 16000,    AP_NY,  AP_NS,     0},  /* 14 下降 */
    { AP_NA,  AP_NB,   AP_NY,      65,   800},  /* 15 微松 */
    { AP_NA, 20000,    AP_NY,  AP_NS,     0},  /* 16 抬 B */
    { 4000,  AP_NB,        0,  AP_NS,     0},  /* 17 回 */

    { 1000,      0,    AP_NY,    100,  1200},  /* 18 抓取位 */
    { AP_NA,  AP_NB,   AP_NY,      50,   900},  /* 19 合爪 */
    { AP_NA, 20000,    AP_NY,  AP_NS,     0},  /* 20 抬 B */
    { 4000,  AP_NB,     -675,  AP_NS,     0},  /* 21 退 A + 转到 3 号盘 */
    { 1000,  AP_NB,    AP_NY,  AP_NS,     0},  /* 22 靠近 3 号盘 */
    { AP_NA, 16000,    AP_NY,  AP_NS,     0},  /* 23 下降 */
    { AP_NA,  AP_NB,   AP_NY,      65,   800},  /* 24 微松 */
    { AP_NA, 20000,    AP_NY,  AP_NS,     0},  /* 25 抬 B */
    { 4000,  AP_NB,        0,  AP_NS,     0},  /* 26 退 A + 转回(结束位) */
};

#define AP_STEP_N   ((uint16_t)(sizeof(S_STEP) / sizeof(S_STEP[0])))

static uint8_t  s_run     = 0;
static uint16_t s_step    = 0;
static uint8_t  s_sent    = 0;
static uint32_t s_step_ms = 0;
static int32_t  s_y_tgt   = 0;

/* 度 x10 -> mrad。1 度 = 17.4532925 mrad, 所以 d10(度x10) 要乘 1.74532925,
   即 d10 * 1745 / 1000 (用 1745/1000 近似, 误差 < 0.03 度)。
   注意分母必须是 1000: 写成 /100 会把目标放大 10 倍, 再被 yaw 限幅钳死。 */
static int32_t ap_yaw_mrad(int16_t d10)
{
    return (int32_t)d10 * 1745 / 1000;
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

void AutoPlace_Start(void)
{
    Track_SetRun(0);                  /* 停视觉 PID, 否则会把手动 A 轴位置顶掉 */
    Stepper_TrackOn(AXIS_A, 0);       /* 干净退出跟随模式 */

    s_y_tgt   = Link_GetYawTarget();  /* 起点沿用当前 yaw 目标, 避免第一步突变 */
    s_step    = 0;
    s_sent    = 0;
    s_step_ms = g_ms;
    s_run     = 1;
}

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

    st = &S_STEP[s_step];

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
            YawTrack_SetManual(s_y_tgt, AP_YAW_VEL);
        }
        if (st->servo != AP_NS) Servo_SetAngle((float)st->servo);

        s_sent    = 1;
        s_step_ms = g_ms;
        return;
    }

    /* ---- 等所有轴到位 ---- */
    el   = g_ms - s_step_ms;
    need = (uint32_t)AP_SETTLE_MS + (uint32_t)st->hold;
    if (el < need) return;
    if (el < AP_STEP_TIMEOUT && !ap_ready(st)) return;

    s_step++;
    s_sent = 0;

    if (s_step >= AP_STEP_N) s_run = 0;    /* 全部完成 */
}

uint8_t AutoPlace_IsRunning(void)
{
    return s_run;
}

uint16_t AutoPlace_GetStep(void)
{
    return s_run ? (uint16_t)(s_step + 1u) : 0u;
}
