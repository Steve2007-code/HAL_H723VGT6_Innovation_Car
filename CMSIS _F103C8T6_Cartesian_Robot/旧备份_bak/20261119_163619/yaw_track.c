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

/* ---------------------------------------------------------------------------
 * 定点 atan2 近似：返回 mrad，范围 -3142..3142（-pi..pi），误差 < 4 mrad(0.23deg)
 * 近似式：atan(v) ~= v * (0.7854 + 0.273 * (1 - |v|))，v in [0,1]
 * ------------------------------------------------------------------------- */
static int32_t atan2_mrad(int32_t y, int32_t x)
{
    int32_t ax, ay, v, t, a;
    uint8_t swap = 0, negx = 0, negy = 0;

    if (x == 0 && y == 0) return 0;

    if (x < 0) { x = -x; negx = 1; }
    if (y < 0) { y = -y; negy = 1; }
    ax = x; ay = y;

    /* v = min/max，保证 |v| <= 1（Q15） */
    if (ay > ax) { v = (ax << 15) / ay; swap = 1; }
    else         { v = (ay << 15) / ax; }

    t = 25736 + (((int32_t)8946 * (32768 - v)) >> 15);   /* 0.7854+0.273*(1-v)，Q15 */
    a = (v * t) >> 15;                                   /* atan(v)，Q15，0..25736 */

    if (swap) a = 25736 - a;          /* pi/2 - a */
    if (negx) a = 3142 - a;           /* pi - a   */
    if (negy) a = -a;
    return a;
}

static int32_t s_target_mrad;
static int32_t s_phi_f;

void YawTrack_Init(void)
{
    s_target_mrad = LINK_YAW_DEFAULT_MRAD;
    s_phi_f       = 0;
}

int32_t YawTrack_GetTargetMrad(void)
{
    return s_target_mrad;
}

void YawTrack_Update(int16_t dx, int16_t dy, uint8_t valid)
{
    int32_t phi, d;

    if (!valid) {
        s_phi_f = 0;                  /* 丢目标：目标角保持，yaw 停住 */
        return;
    }

    /* 画面 x 向右、y 向下；正上方为 0，顺时针为正 */
    phi = atan2_mrad((int32_t)dx, -(int32_t)dy);

    if (phi > -YAW_ANGLE_DEAD && phi < YAW_ANGLE_DEAD) phi = 0;

    /* 一阶低通（Q12） */
    s_phi_f += (((phi - s_phi_f) * YAW_LPF_Q12) >> 12);

    /* 每 tick 增量 = KP * phi（Q12），再限幅，再按符号 */
    d = (s_phi_f * YAW_KP_Q12) >> 12;
    if (d >  YAW_RATE_MAX_MRAD) d =  YAW_RATE_MAX_MRAD;
    if (d < -YAW_RATE_MAX_MRAD) d = -YAW_RATE_MAX_MRAD;
    if (YAW_SIGN < 0) d = -d;

    s_target_mrad += d;
    if (s_target_mrad >  LINK_POS_MAX_MRAD) s_target_mrad =  LINK_POS_MAX_MRAD;
    if (s_target_mrad < -LINK_POS_MAX_MRAD) s_target_mrad = -LINK_POS_MAX_MRAD;

    Link_SetYawTarget(s_target_mrad, YAW_VEL_MRADPS);
}
