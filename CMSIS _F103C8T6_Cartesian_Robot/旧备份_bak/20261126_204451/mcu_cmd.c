/******************************************************************************
 * mcu_cmd.c  --  USART3 调试命令：A/B 轴位置、4340 yaw 目标角、夹爪、补光
 *
 * 入口：qr_scan.c 的 QR_Poll() 把非 "flag," 帧交给 Cmd_Exec()。
 * 全部在主循环上下文执行，无忙等（仅 <<Q>> 回印为短阻塞发送）。
 *
 * 安全设计：
 *   1) 收到 A 轴位置命令时自动暂停视觉 PID（Track_SetRun(0)）并退出跟随模式，
 *      否则 Track_Update 每 10ms 的 SetVel 会把手动位置指令顶掉。
 *   2) yaw 手动设目标角后视觉累加暂停（YawTrack_SetManual），需 <<H>> 恢复。
 *   3) 所有位置先按轴软件限位钳位，越界只走到边界，不会硬撞机械限位。
 ******************************************************************************/
#include "mcu_cmd.h"
#include "frame.h"
#include "stepper.h"
#include "track.h"
#include "yaw_track.h"
#include "mcu_link.h"
#include "qr_scan.h"
#include "servo.h"
#include "backlight.h"
#include "link_config.h"

#define CMD_MAX_VALS    3
#define CMD_PPS_DEF     3000        /* 未指定速度时的默认 pps */
#define CMD_PPS_MAX     12000       /* 与 stepper.c 的 MAX_PPS 保持一致 */
#define CMD_ACCEL_MS    300

/* 舵机两个工位（与 main.c 一致） */
#define CMD_GRIP_CLOSE  50.0f   /* 状态 0: 闭合 */
#define CMD_GRIP_OPEN   65.0f   /* 状态 1: 小开 */
#define CMD_GRIP_WIDE   100.0f  /* 状态 2: 大开 */

/* ---------- 裸机整数转字符串（不用 sprintf） ---------- */
static char *i2s(char *p, int32_t v)
{
    char tmp[12];
    uint8_t n = 0;
    uint32_t u;

    if (v < 0) { *p++ = '-'; u = (uint32_t)(-v); }
    else         u = (uint32_t)v;

    do {
        tmp[n++] = (char)('0' + (u % 10U));
        u /= 10U;
    } while (u);
    while (n) *p++ = tmp[--n];
    return p;
}

/* ---------- 解析 "CMD" 或 "CMD,v1[,v2[,v3]]" ---------- */
static uint8_t cmd_parse(const char *body, char *cmd, int32_t *v, uint8_t *nv)
{
    const char *p = body;

    if (!p || !*p) return 0;
    *cmd = *p++;
    *nv  = 0;

    if (*p == 0) return 1;                 /* 无参数命令，如 <<Q>> <<H>> */

    while (*p) {
        if (*p++ != ',') return 0;
        if (*nv >= CMD_MAX_VALS) return 0;
        if (!parse_i(&p, &v[*nv])) return 0;
        (*nv)++;
    }
    return 1;
}

static uint16_t cmd_pps(int32_t v)
{
    if (v <= 0)                   return CMD_PPS_DEF;
    if (v > (int32_t)CMD_PPS_MAX) return (uint16_t)CMD_PPS_MAX;
    return (uint16_t)v;
}

/* ---------- 回印状态：A/B 位置、yaw 目标/实际、跟随开关 ---------- */
static void cmd_report(void)
{
    char buf[96];
    char *p = buf;
    const char *s = "<<Q,";

    while (*s) *p++ = *s++;

    p = i2s(p, Stepper_GetPos(AXIS_A));      *p++ = ',';
    p = i2s(p, Stepper_GetPos(AXIS_B));      *p++ = ',';
    p = i2s(p, YawTrack_GetTargetMrad());    *p++ = ',';
    p = i2s(p, Link_GetYawPos());            *p++ = ',';
    p = i2s(p, (int32_t)Track_GetRun());     *p++ = ',';
    p = i2s(p, (int32_t)YawTrack_IsManual());

    *p++ = '>';
    *p++ = '>';
    *p   = 0;

    QR_SendStr(buf);
}

/* 子命令执行：USART3 字符命令与板间 M 帧共用这一套。
   sub: 1=A轴位置 2=B轴位置 3=yaw目标角 4=A轴跟随开关
        5=yaw回视觉 6=夹爪 7=补光 9=查询(回 S 帧)  */
void Cmd_ExecSub(int32_t sub, const int32_t *v, uint8_t n)
{
    uint16_t pps;

    if (n > 0 && v == 0) return;

    switch (sub) {

    case 1:                                     /* A 轴绝对位置（水平伸缩） */
        if (n < 1) return;
        pps = cmd_pps((n >= 2) ? v[1] : CMD_PPS_DEF);
        Track_SetRun(0);                        /* 先停 PID，否则会被 SetVel 顶掉 */
        Stepper_TrackOn(AXIS_A, 0);             /* 干净退出跟随模式 */
        Stepper_MoveTo(AXIS_A, v[0], pps, CMD_ACCEL_MS);
        break;

    case 2:                                     /* B 轴绝对位置（升降） */
        if (n < 1) return;
        pps = cmd_pps((n >= 2) ? v[1] : CMD_PPS_DEF);
        Stepper_MoveTo(AXIS_B, v[0], pps, CMD_ACCEL_MS);
        break;

    case 3:                                     /* 4340 yaw 目标角 */
        if (n < 1) return;
        YawTrack_SetManual(v[0], (n >= 2) ? v[1] : YAW_VEL_MRADPS);
        break;

    case 4:                                     /* A 轴跟随 开/关 */
        if (n < 1) return;
        if (v[0]) {
            Stepper_TrackOn(AXIS_A, 1);
            Track_SetRun(1);
        } else {
            Track_SetRun(0);
            Stepper_TrackOn(AXIS_A, 0);
        }
        break;

    case 5:                                     /* yaw 恢复视觉跟随 */
        YawTrack_SetAuto();
        break;

    case 6:                                     /* 夹爪: 0=合 1=小开 2=大开 */
        if (n < 1) return;
        if      (v[0] == 2) Servo_SetAngle(CMD_GRIP_WIDE);
        else if (v[0] == 1) Servo_SetAngle(CMD_GRIP_OPEN);
        else                Servo_SetAngle(CMD_GRIP_CLOSE);
        break;

    case 7:                                     /* 补光 */
        if (n < 1) return;
        Backlight_Set(v[0] ? 1 : 0);
        break;

    case 9:                                     /* 查询：回 S 帧给 H723 */
        break;

    default:
        break;
    }

    /* 执行类命令(1~7)与查询(9)都回一帧状态:
       H723 侧一旦打印出 # ARM ... 就证明 F103 真的收到了这条命令并走完了分支,
       从而把"上位机已入队"和"下位机已执行"区分开. */
    if (sub >= 1 && sub <= 9)
    {
        Link_SendStatus(Stepper_GetPos(AXIS_A), Stepper_GetPos(AXIS_B),
                        YawTrack_GetTargetMrad(), Link_GetYawPos(),
                        (int32_t)Track_GetRun(), (int32_t)YawTrack_IsManual());
    }
}

/* 字符命令入口（USART3 直连调试用）：<<A,pos,pps>> <<B,..>> <<Y,..>> ... */
void Cmd_Exec(const char *body)
{
    char    cmd;
    int32_t v[CMD_MAX_VALS];
    uint8_t n;
    int32_t sub;

    if (!cmd_parse(body, &cmd, v, &n)) return;

    switch (cmd) {
    case 'A': sub = 1; break;
    case 'B': sub = 2; break;
    case 'Y': sub = 3; break;
    case 'T': sub = 4; break;
    case 'H': sub = 5; break;
    case 'G': sub = 6; break;
    case 'L': sub = 7; break;
    case 'Q': cmd_report(); return;             /* USART3 本地回印 */
    default:  return;
    }
    Cmd_ExecSub(sub, v, n);
}
