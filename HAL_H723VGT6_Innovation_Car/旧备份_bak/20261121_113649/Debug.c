#include <stdio.h>
#include <string.h>
#include <math.h>
#include "Debug.h"
#include "Chassis.h"
#include "Attitude.h"
#include "Nav.h"
#include "AStar.h"
#include "Time.h"
#include "UART_Data.h"

/*==========================================================================
 * 串口协议:
 *   运动类命令必须包在包头 << 与包尾 >> 之间, 防止总线噪声拼成命令让车自走:
 *       <<命令 参数>>
 *   只读/急停命令 (? o i m t x e) 直接敲, 不用包头包尾
 * 手动速度 (v) 带 800ms 看门狗, 超时自动停
 *==========================================================================*/

static char     s_line[96];
static uint8_t  s_len = 0;
static uint8_t  s_telem = 0;
static uint32_t s_last = 0;

/*---------------- 轻量数值解析 ----------------*/
static float s_atof(char **pp)
{
    char *p = *pp;
    float v = 0.0f, f;
    int sign = 1;
    while (*p == ' ' || *p == ',') p++;
    if (*p == '-') { sign = -1; p++; }
    else if (*p == '+') { p++; }
    while (*p >= '0' && *p <= '9') { v = v * 10.0f + (float)(*p - '0'); p++; }
    if (*p == '.')
    {
        p++;
        f = 0.1f;
        while (*p >= '0' && *p <= '9') { v += (float)(*p - '0') * f; f *= 0.1f; p++; }
    }
    *pp = p;
    return v * (float)sign;
}

static int s_atoi(char **pp)
{
    char *p = *pp;
    int v = 0, sign = 1;
    while (*p == ' ' || *p == ',') p++;
    if (*p == '-') { sign = -1; p++; }
    while (*p >= '0' && *p <= '9') { v = v * 10 + (*p - '0'); p++; }
    *pp = p;
    return v * sign;
}

/*---------------- 必须带帧的命令 ----------------*/
static uint8_t s_need_frame(char cmd)
{
    switch (cmd)
    {
    case 'v': case 's': case 'd': case 'g': case 'z':
    case 'p': case 'r': case 'k': case 'h': case 'c': case 'w':
    case 'f': case 'R': case 'y':
        return 1;
    }
    return 0;
}

static void s_help(void)
{
    printf("\r\n---- 工创小车底盘调试 v2 (带校验帧) ----\r\n");
    printf(" 协议: 运动类要包起来  <<命令 参数>>\r\n");
    printf("       例: <<v 0.15 0 0>>   <<g 23 46>>   <<p 2325 1225>>\r\n");
    printf(" ---- 只读/急停 (直接敲) ----\r\n");
    printf(" ?                  帮助\r\n");
    printf(" o                  打印位姿/里程计/方向\r\n");
    printf(" i                  打印 IMU 诊断\r\n");
    printf(" m                  打印增益\r\n");
    printf(" t <0|1>            CSV 遥测开关\r\n");
    printf(" x                  急停\r\n");
    printf(" e                  退出单轮模式\r\n");
    printf(" ---- 运动类 (需帧) ----\r\n");
    printf(" v <vx> <vy> <wz>   手动速度 (800ms 看门狗)\r\n");
    printf(" s <idx> <rad>      单轮点动\r\n");
    printf(" d <idx> <1|-1>     轮方向标定\r\n");
    printf(" g <row> <col>      规划到格子\r\n");
    printf(" p <x> <y>          规划到世界坐标 mm\r\n");
    printf(" z <kind> <idx>     规划到功能区\r\n");
    printf(" r <row> <col> [yaw] 重置位姿, 可指定真实车头角(度, 0右/90上/-90下/180左)\r\n");
    printf(" k <kp> <kd>        位置环\r\n");
    printf(" h <kp> <kd>        航向环\r\n");
    printf(" c <scale>          里程计比例\r\n");
    printf(" w <0|1>            手动看门狗开关\r\n");
    printf(" f <deg>            到点后对准朝向 (999=取消)\r\n");
    printf(" R <0|1|2>          跑内置路线 1/2=开始 0=停止\r\n");
    printf(" y <0|1|2>          转向策略 0锁车头 1只大角度转 2跟随(默认)\r\n");
    printf("------------------------------------------\r\n");
}

static void s_exec(char *ln, uint8_t framed)
{
    char *p = ln;
    char cmd;

    while (*p == ' ') p++;
    cmd = *p;
    if (cmd == 0) return;
    p++;

    if (s_need_frame(cmd) && !framed)
    {
        printf("# '%c' 需要包在 << >> 里: <<%s>>\r\n", cmd, ln);
        return;
    }

    switch (cmd)
    {
    case '?':
        s_help();
        break;

    case 'v':
    {
        float vx = s_atof(&p);
        float vy = s_atof(&p);
        float wz = s_atof(&p);
        g_nav_state = NAV_MANUAL;
        Nav_ManualKick();
        Chassis_SetBodyVel(vx, vy, wz);
        printf("# set vel %.3f %.3f %.3f\r\n", vx, vy, wz);
        break;
    }

    case 'x':
        Nav_Stop();
        Chassis_SingleExit();
        printf("# stop\r\n");
        break;

    case 's':
    {
        int i = s_atoi(&p);
        float r = s_atof(&p);
        Chassis_Single((uint8_t)i, r);
        printf("# single %d = %.3f rad/s\r\n", i, r);
        break;
    }

    case 'e':
        Chassis_SingleExit();
        printf("# single exit\r\n");
        break;

    case 'd':
    {
        int i = s_atoi(&p);
        int sgn = s_atoi(&p);
        Chassis_SetDir((uint8_t)i, (float)sgn);
        printf("# dir %d = %d\r\n", i, sgn);
        break;
    }

    case 'g':
    {
        int r = s_atoi(&p);
        int c = s_atoi(&p);
        if (Nav_PlanToCell((uint8_t)r, (uint8_t)c))
            printf("# plan ok  wp=%d  state=%d\r\n", g_nav_wp_total, g_nav_state);
        else
            printf("# plan FAILED\r\n");
        break;
    }

    case 'p':
    {
        float x = s_atof(&p);
        float y = s_atof(&p);
        if (Nav_PlanToMm(x, y))
            printf("# plan to (%.0f, %.0f) ok  wp=%d  state=%d\r\n", x, y, g_nav_wp_total, g_nav_state);
        else
            printf("# plan FAILED\r\n");
        break;
    }

    case 'z':
    {
        int k = s_atoi(&p);
        int i = s_atoi(&p);
        uint8_t r = 0, c = 0;
        if (Nav_DockCell((uint8_t)k, (uint8_t)i, &r, &c))
            printf("# dock cell (%d,%d)\r\n", r, c);
        if (Nav_PlanToZone((uint8_t)k, (uint8_t)i))
            printf("# plan ok  wp=%d  state=%d\r\n", g_nav_wp_total, g_nav_state);
        else
            printf("# plan FAILED\r\n");
        break;
    }

    case 'r':
    {
        int r = s_atoi(&p);
        int c = s_atoi(&p);
        /* 可选第三个参数: 车的真实车头角(度), 不填则默认朝上(90) */
        while (*p == ' ' || *p == ',') p++;
        if (*p == '-' || (*p >= '0' && *p <= '9'))
        {
            float yd = s_atof(&p);
            Nav_ResetPoseTo((uint8_t)r, (uint8_t)c, yd);
            printf("# pose reset to (%d,%d)  yaw=%.0f deg\r\n", r, c, yd);
        }
        else
        {
            Nav_ResetPose((uint8_t)r, (uint8_t)c);
            printf("# pose reset to (%d,%d)\r\n", r, c);
        }
        break;
    }

    case 'o':
        Nav_PrintInfo();
        printf("# clear=%d cells  odom_scale=%.4f\r\n", g_map_clear_cells, g_odom_scale);
        break;

    case 'i':
        Attitude_PrintDiag();
        break;

    case 'k':
    {
        float a = s_atof(&p);
        float b = s_atof(&p);
        Nav_SetGains(a, b, 0.0f, -1.0f);
        printf("# pos kp=%.2f kd=%.2f\r\n", a, b);
        break;
    }

    case 'h':
    {
        float a = s_atof(&p);
        float b = s_atof(&p);
        Nav_SetGains(0.0f, -1.0f, a, b);
        printf("# yaw kp=%.2f kd=%.2f\r\n", a, b);
        break;
    }

    case 'm':
    {
        float a, b, c, d;
        Nav_GetGains(&a, &b, &c, &d);
        printf("# pos kp=%.2f kd=%.2f   yaw kp=%.2f kd=%.2f\r\n", a, b, c, d);
        break;
    }

    case 'c':
    {
        float sc = s_atof(&p);
        if (sc > 0.2f && sc < 5.0f) g_odom_scale = sc;
        printf("# odom scale = %.4f\r\n", g_odom_scale);
        break;
    }

    case 'w':
    {
        int on = s_atoi(&p);
        Nav_SetManualWd((uint8_t)(on != 0));
        printf("# manual watchdog %d\r\n", on != 0);
        break;
    }

    case 'f':
    {
        float d = s_atof(&p);
        Nav_SetDockYaw(d);
        printf("# dock yaw %.0f deg %s\r\n", d, (d < -360.0f || d > 360.0f) ? "off" : "on");
        break;
    }

    case 'y':
    {
        int m = s_atoi(&p);
        if (m >= 0 && m <= 2) g_yaw_mode = (uint8_t)m;
        printf("# yaw mode %d  %s\r\n", g_yaw_mode,
               g_yaw_mode == 0 ? "锁车头永不转" :
               (g_yaw_mode == 1 ? "只在大角度转(90度用横移)" : "车头跟随段方向"));
        break;
    }

    case 'R':
    {
        int on = s_atoi(&p);
        Nav_RouteStart((uint8_t)(on > 0 ? on : 0));   /* 1/2 选路线, 0 停 */
        break;
    }

    case 't':
        s_telem = (uint8_t)(s_atoi(&p) != 0);
        printf("# telem %d\r\n", s_telem);
        break;

    default:
        printf("# unknown cmd '%c', try '?'\r\n", cmd);
        break;
    }
}

static void s_handle(char *ln)
{
    /* 包头 << */
    if (ln[0] == '<' && ln[1] == '<')
    {
        char *body = ln + 2;
        char *end  = strstr(body, ">>");
        if (end == NULL)
        {
            printf("# 缺包尾 >>\r\n");
            return;
        }
        *end = 0;
        s_exec(body, 1);
        return;
    }
    /* 裸命令: 只允许只读/急停 */
    s_exec(ln, 0);
}

void Debug_Init(void)
{
    s_len = 0;
    s_telem = 0;
}

void Debug_Task(void)
{
    char ch;

    while (UART1_RX_Rd != UART1_RX_idx)
    {
        ch = (char)UART1_RX_Fifo[UART1_RX_Rd];
        UART1_RX_Rd++;
        if (UART1_RX_Rd >= 128) UART1_RX_Rd = 0;

        if (ch == '\r' || ch == '\n')
        {
            if (s_len > 0)
            {
                s_line[s_len] = 0;
                s_handle(s_line);
                s_len = 0;
            }
        }
        else if (s_len < (uint8_t)(sizeof(s_line) - 1))
        {
            s_line[s_len++] = ch;
        }
    }

    if (s_telem && (Tick - s_last) >= 100)
    {
        int r, c;
        s_last = Tick;
        r = (MAP_N - 1) - (int)(g_pose.y / MAP_CELL_MM);
        c = (int)(g_pose.x / MAP_CELL_MM);
        if (r < 0) r = 0;  if (r > MAP_N - 1) r = MAP_N - 1;
        if (c < 0) c = 0;  if (c > MAP_N - 1) c = MAP_N - 1;
        printf("$%lu,%d,%d,%.1f,%.1f,%.2f,%.3f,%.3f,%.3f,%d,%d\r\n",
               (unsigned long)Tick, r, c, g_pose.x, g_pose.y,
               g_pose.yaw * 57.29578f,
               g_chassis.fb_vx, g_chassis.fb_vy, g_chassis.fb_wz,
               g_nav_state, g_nav_wp_index);
    }
}
