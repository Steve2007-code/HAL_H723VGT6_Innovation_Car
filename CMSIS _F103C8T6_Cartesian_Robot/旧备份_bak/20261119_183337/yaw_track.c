/******************************************************************************
 * yaw_track.c  --  画面极角 -> yaw 目标角（定点，无浮点、无 libm）
 *
 * 控制律：每 10ms 把当前极角误差 phi 的一小部分累加到 yaw 目标角，
 *         即增量式积分控制；4340 内部已有位置环，F103 只需给目标角。
 * 丢失目标：目标角保持不动（yaw 停住），不清零、不猛转。
 * 上电目标 = LINK_YAW_DEFAULT_MRAD（0），由 H723 转发让 4340 回 0 位。
 ******************************************************************************/
#include "yaw_track.h"
#include "link_config.h"
#include "mcu_link.h"

/* 视觉帧时间戳 (定义在 track.c): 用于判断本次调用是否真的拿到了新帧 */
extern volatile uint32_t g_last_frame_ms;

/* ---------------------------------------------------------------------------
 * 定点 atan2 近似：返回 mrad，范围 -3142..3142（-pi..pi），误差 < 4 mrad(0.23deg)
 * 近似式：atan(v) ~= v * (0.7854 + 0.273 * (1 - |v|))，v in [0,1]
 * ------------------------------------------------------------------------- */
static int32_t atan2_mrad(int32_t y, int32_t x)
{
    int32_t ax, ay, v, t, a;

    if (x == 0 && y == 0) return 0;

    /* 先按第一象限求 atan(min/max)，Q15：0..25736 对应 0..pi/4 */
    ax = (x < 0) ? -x : x;
    ay = (y < 0) ? -y : y;

    if (ay > ax) v = (ax << 15) / ay;      /* ax==0 时必走这支，不会除零 */
    else         v = (ay << 15) / ax;

    t = 25736 + (((int32_t)8946 * (32768 - v)) >> 15);   /* 0.7854+0.273*(1-v)，Q15 */
    a = (v * t) >> 15;                                   /* atan(v)，Q15，0..25736 */

    if (ay > ax) a = 51472 - a;            /* pi/2 - a，Q15 的 pi/2 = 51472 */
    if (x < 0)   a = 102944 - a;           /* pi  - a，Q15 的 pi   = 102944 */
    if (y < 0)   a = -a;

    return (a * 3142) / 102944;            /* Q15 -> mrad，分辨率约 1 mrad */
}

static int32_t s_target_mrad;
static int32_t s_phi_f;
static uint32_t s_seen_ms;          /* 已处理过的视觉帧时间戳 */

void YawTrack_Init(void)
{
    s_target_mrad = LINK_YAW_DEFAULT_MRAD;
    s_phi_f       = 0;
    s_seen_ms     = 0;
}

int32_t YawTrack_GetTargetMrad(void)
{
    return s_target_mrad;
}

void YawTrack_Update(int16_t dx, int16_t dy, uint8_t valid)
{
    int32_t phi, d;

    /* 只在收到新视觉帧时累加一次。
       相机帧率通常 30~60Hz, 而本函数以 100Hz 被调用; 同一帧被重复累加 2~3 次
       会让 yaw 目标冲过头 -> 反向修正 -> 来回震荡。 */
    if (g_last_frame_ms == s_seen_ms) return;
    s_seen_ms = g_last_frame_ms;

    if (!valid) {
        s_phi_f = 0;                  /* 丢目标：目标角保持，yaw 停住 */
        return;
    }

    /* 画面 x 向右、y 向下；正上方为 0，顺时针为正 */
    phi = atan2_mrad((int32_t)dx, -(int32_t)dy);

    /* 死区内直接冻结: 清零低通状态并返回, 不再累加。
       若只把 phi 置 0 再走低通, s_phi_f 的残留会让目标继续动 -> 过冲 -> 原点附近来回摆 */
    if (phi > -YAW_ANGLE_DEAD && phi < YAW_ANGLE_DEAD) {
        s_phi_f = 0;
        return;
    }

    /* 一阶低通（Q12） */
    s_phi_f += (((phi - s_phi_f) * YAW_LPF_Q12) >> 12);

    /* 每 tick 增量 = KP * phi（Q12），再限幅，再按符号 */
    d = (s_phi_f * YAW_KP_Q12) >> 12;
    /* Q12 截断补偿: 误差非零但被截成 0 时至少走 1 mrad, 否则永远进不了死区 */
    if (d == 0 && s_phi_f != 0) d = (s_phi_f > 0) ? 1 : -1;
    if (d >  YAW_RATE_MAX_MRAD) d =  YAW_RATE_MAX_MRAD;
    if (d < -YAW_RATE_MAX_MRAD) d = -YAW_RATE_MAX_MRAD;
    if (YAW_SIGN < 0) d = -d;

    s_target_mrad += d;
    if (s_target_mrad >  LINK_POS_MAX_MRAD) s_target_mrad =  LINK_POS_MAX_MRAD;
    if (s_target_mrad <  LINK_POS_MIN_MRAD) s_target_mrad =  LINK_POS_MIN_MRAD;

    Link_SetYawTarget(s_target_mrad, YAW_VEL_MRADPS);
}
