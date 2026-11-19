/******************************************************************************
 * yaw_track.c  --  yaw 目标角闭环：追 x 像素误差（定点，无浮点、无 libm）
 *
 * 控制律：每收到一帧视觉（约 30fps）把 dx 的一小部分累加到 yaw 目标角，
 *         即增量式 PD；4340 内部已有位置环，F103 只需给目标角。
 * 职责解耦：yaw 只消 dx，A 轴消 dy，两者互不干扰。
 * 丢失目标：目标角保持不动（yaw 停住），不清零、不猛转。
 * 上电目标 = LINK_YAW_DEFAULT_MRAD（0），由 H723 转发让 4340 回 0 位。
 ******************************************************************************/
#include "yaw_track.h"
#include "link_config.h"
#include "mcu_link.h"

/* 视觉帧时间戳 (定义在 track.c): 用于判断本次调用是否真的拿到了新帧 */
extern volatile uint32_t g_last_frame_ms;

static int32_t s_target_mrad;
static int32_t s_phi_f;
static int32_t s_phi_prev;         /* 上一帧低通误差, 供微分项用 */
static uint32_t s_seen_ms;          /* 已处理过的视觉帧时间戳 */

void YawTrack_Init(void)
{
    s_target_mrad = LINK_YAW_DEFAULT_MRAD;
    s_phi_f       = 0;
    s_phi_prev    = 0;
    s_seen_ms     = 0;
}

int32_t YawTrack_GetTargetMrad(void)
{
    return s_target_mrad;
}

void YawTrack_Update(int16_t dx, int16_t dy, uint8_t valid)
{
    int32_t phi, d, dphi;

    /* 只在收到新视觉帧时累加一次。
       相机帧率通常 30~60Hz, 而本函数以 100Hz 被调用; 同一帧被重复累加 2~3 次
       会让 yaw 目标冲过头 -> 反向修正 -> 来回震荡。 */
    if (g_last_frame_ms == s_seen_ms) return;
    s_seen_ms = g_last_frame_ms;

    if (!valid) {
        s_phi_f = 0;                  /* 丢目标：目标角保持，yaw 停住 */
        return;
    }

    /* yaw 只追 x 像素误差: 让 dx -> 0。A 轴负责 dy, 两轴解耦。
       不能再用 atan2(dx,-dy): 本机相机 y 向上为正(与标准图像相反),
       目标在画面上方时 -dy 为负, phi 落在 +-180 附近并在 +-pi 处跳变 -> 剧烈抖动。 */
    phi = (int32_t)dx;
    (void)dy;                          /* dy 归 A 轴, yaw 不使用 */

    /* 死区内直接冻结: 清零低通状态并返回, 不再累加。
       若只把 phi 置 0 再走低通, s_phi_f 的残留会让目标继续动 -> 过冲 -> 原点附近来回摆 */
    if (phi > -YAW_ANGLE_DEAD && phi < YAW_ANGLE_DEAD) {
        s_phi_f    = 0;
        s_phi_prev = 0;
        return;
    }

    /* 一阶低通（Q12） */
    s_phi_f += (((phi - s_phi_f) * YAW_LPF_Q12) >> 12);

    /* 微分项: 误差变化率。误差快速减小时为负, 抵消 P 项, 起阻尼作用,
       专门压 10fps + 链路延迟造成的过冲震荡。加限幅防像素噪声放大。 */
    dphi = s_phi_f - s_phi_prev;
    s_phi_prev = s_phi_f;
    if (dphi >  800) dphi =  800;
    if (dphi < -800) dphi = -800;

    /* 增量式 PD: P 提供刚度, D 提供阻尼（Q12 定点） */
    d = ((s_phi_f * YAW_KP_Q12) + (dphi * YAW_KD_Q12)) >> 12;
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
