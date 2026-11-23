#include <math.h>
#include <stdio.h>
#include "Nav.h"
#include "AStar.h"
#include "Chassis.h"
#include "Attitude.h"
#include "Time.h"

Pose_t  g_pose;
/* 里程计比例: 2026-11-21 二次标定 (0.75 m/s, a=1.0)
   直线 2050mm 指令: 原 0.99 时实测 2020mm (少 30mm = 1.46%)
   新值 = 0.99 * 2020/2050 = 0.9755 -> 复测正好 2050mm, 误差 <3mm
   反推有效轮径 48.5 * 0.9755/0.99 = 47.79mm (胶辊受压, 比硬轮缘小 0.7mm)
   注意: 改了速度/加速度/地面之后必须重标, <<c>> 可现场改 */
float   g_odom_scale = 0.9755f;
/* 到点前减速度 (T 形曲线的下降段): v_allow = sqrt(2*a*d)
   原来是写死的 1.6f = 2*0.8, 现改成变量, 用 <<D a>> 现场扫参 */
float   g_nav_a_dec = 1.00f;   /* 与 g_ch_acc_v 对称, 才是真正的 T 形梯形 */
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
static float   s_dock_yaw = 0.0f;       /* 到点后对准的朝向 (rad) */
static uint8_t s_dock_on  = 0;

/* 强制横移: 1 = 本段锁定车头, 只平移不转向 */
uint8_t g_force_strafe = 0;
uint8_t g_yaw_mode = 2;             /* 0=锁车头 1=只大角度转 2=跟随段方向 */
static float s_yaw_lock = 0.0f;     /* 进入本计划时锁定的车头 */
static float   s_goal_x = 0.0f, s_goal_y = 0.0f;   /* <<p>> 的精确终点 */
static uint8_t s_goal_exact = 0;
volatile uint8_t g_nav_fail_why = 0;   /* 1=IMU失效 2=转向超时 3=规划失败 4=不可规划 */
/* 内置路线状态 */
static uint8_t s_route_on = 0;
static uint8_t s_route_i  = 0;
/* 停稳再走: 到点后需速度归零并保持 NAV_ROUTE_SETTLE_MS 才规划下一点 */
static uint32_t s_route_settle_ms = 0;
#define NAV_ROUTE_SETTLE_MS   300u
#define NAV_ROUTE_SETTLE_V    0.02f   /* m/s */
#define NAV_ROUTE_SETTLE_W    0.05f   /* rad/s */

/* 规划临时区 (主循环用) */
static uint8_t s_cells[MAP_CELLS * 2];
static int16_t s_px[MAP_CELLS];
static int16_t s_py[MAP_CELLS];
static float   s_tx[NAV_WP_MAX];
static float   s_ty[NAV_WP_MAX];
static uint8_t s_tn = 0;

/* 正交化临时缓冲 */
#define NAV_WP_BUF   (NAV_WP_MAX * 2)
static float   s_ox[NAV_WP_BUF];
static float   s_oy[NAV_WP_BUF];

/* 手动模式看门狗: v 命令无刷新超过该时长自动停 */
uint8_t  g_nav_manual_wd = 1;
static uint32_t s_manual_tick = 0;
#define NAV_MANUAL_TIMEOUT_MS   800u

/* 原地转向保护: 限速(降电流峰值/防打滑) + 超时(防无限转) */
#define NAV_SHORT_MM            60.0f /* 末段 <=60mm 才锁车头横移(只为扫码那种几十毫米的微调); 130~200mm 的段必须正常转向 */
/* 转向「真正完成」的角速度阈值: 只看角度会在还转着的时候就放行,
   表现为「没转到位就开始直走」, 且边走边拧的麦轮打滑最大. */
#define NAV_TURN_SETTLE_RATE   0.08f   /* rad/s, 约 6.9 度/秒 (0.2->0.12: 起步前真正停稳) */
/* 模式3 纯追踪: 前瞻距离(mm). 越大越平滑, 但拐角会切得更靠内 */
#define NAV_CARROT_MM          150.0f
/* 原地转向限速(rad/s): 宏改成变量, 可用 <<T wz>> 现场调 */
static float s_turn_wz_max = 0.90f;
/* 最小转向角速度: 麦轮原地转的静摩擦很大, 误差小的时候纯 P 输出只有 0.1 rad/s,
   车根本转不动 -> 卡在差 1~2 度处直到超时. 加个下限把它推过去. */
#define NAV_TURN_MIN_WZ   0.25f
#define NAV_TURN_TIMEOUT_MS    7000u   /* 7s 转不到位就判失败 (原来 4s: 转向降到 0.9 后 180 度要 4.1s, 必然误判) */
static uint32_t s_turn_ms = 0;

/* 转到位后的「停稳驻留」: 命令零速度, 等实际反馈速度真的归零并保持一小段时间,
   才允许开始平移。
   避免「还在转就起步」: 转向残余 + 平移叠加时, 麦轮侧向刮地, 而这段滑移在
   里程计里完全看不见 (fb_vx/fb_vy 只反映轮子转速, 不反映车体侧滑)。
   2026-11-21 实测: 单腿直线误差 0mm, 组合整圈物理偏差 115mm, 怀疑点即此过渡。 */
#define NAV_SETTLE_WAIT_MS   300u    /* 静止保持时间默认值(实测 300 最稳), <<S ms>> 可现场改 */
#define NAV_SETTLE_MAX_EXTRA 600u    /* 最长再等多久(卡住也不无限等) */
#define NAV_SETTLE_V         0.015f  /* m/s,   平移速度归零阈值 */
#define NAV_SETTLE_WZ        0.03f   /* rad/s, 角速度归零阈值 */
uint16_t g_nav_settle_ms = NAV_SETTLE_WAIT_MS;
static uint8_t  s_turn_phase   = 0;  /* 0=需要对准  1=停稳驻留中  2=已放行可平移 */
static uint32_t s_settle_ms    = 0;  /* 最近一次检测到静止的时刻 */
static uint32_t s_settle_t0    = 0;  /* 进入停稳驻留的时刻 */

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

    /* 默认车放在下方启停区内侧 (row44, col44) = (2225,175).
       原来用 (45,45)=(2275,125), 车体 275mm 会压出场地边界 12.5mm. */
    r = 44;  c = 44;
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
    s_route_on = 0;  g_force_strafe = 0;   /* 重置位姿同时停掉路线 */
    __enable_irq();
    Chassis_Hold();
}

/* 带朝向的重置: yaw_deg 是车的【真实】车头角 (0=右 90=上 -90=下 180=左).
   摆车不正时用它把真实朝向告诉软件, 否则整条路线会整体旋转。 */
void Nav_ResetPoseTo(uint8_t row, uint8_t col, float yaw_deg)
{
    float ya = yaw_deg * 0.0174532925f;
    Nav_ResetPose(row, col);
    __disable_irq();
    s_yaw_off = ya;
    g_pose.yaw = ya;
    s_yaw_t   = ya;
    __enable_irq();
}

/* 按【毫米坐标】设位姿: 比 <<r row col>> 精确, 可让车中心停在任意点
   (例如启停区真正的中心 (2250,150) 不在任何格心上). */
void Nav_SetPoseMm(float x_mm, float y_mm, float yaw_deg)
{
    float ya = yaw_deg * 0.0174532925f;
    __disable_irq();
    g_pose.x = x_mm;
    g_pose.y = y_mm;
    Attitude_ZeroYaw();
    s_yaw_off = ya;
    g_pose.yaw = ya;
    s_yaw_t   = ya;
    s_wpn = 0;  s_wpi = 0;
    s_turn_ms = 0;
    s_route_on = 0;  g_force_strafe = 0;
    __enable_irq();
    g_nav_state = NAV_IDLE;
    Chassis_Hold();
}

/* 相对当前角度再转 deg 度(正=逆时针). 用于连续转向测试, 例如 <<A 90>> x16 = 4 圈. */
void Nav_RotateBy(float deg)
{
    float cur = g_pose.yaw * 57.29578f;
    Nav_RotateTo(cur + deg);
}

/* 原地转到指定角度(度), 不移动. 进 NAV_ALIGN 状态, 转到位后自动回 IDLE. */
void Nav_RotateTo(float deg)
{
    Nav_SetDockYaw(deg);
    s_turn_ms = 0;
    g_nav_state = NAV_ALIGN;
}
void Nav_Stop(void)
{
    g_nav_state = NAV_IDLE;
    s_wpn = 0;  s_wpi = 0;
    s_turn_ms = 0;
    s_route_on = 0;  g_force_strafe = 0;   /* 急停也停路线 */
    Chassis_Hold();
}

/**
  * @brief  设定到点后对准的朝向
  * @param  deg  地图坐标角度(度): 0=右(+x) 90=上(+y) 180/-180=左 -90=下
  *              传 999 或超出 +-360 表示取消
  */
void Nav_SetDockYaw(float deg)
{
    if (deg < -360.0f || deg > 360.0f)
    {
        s_dock_on = 0;
        return;
    }
    while (deg > 180.0f)  deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    s_dock_yaw = deg * 0.0174532925f;
    s_dock_on  = 1;
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

/*=================================================================
 *  按转向策略挑选本段目标航向
 *  mode 2: 车头跟随段方向 (原行为)
 *  mode 1: 段方向与当前车头差 >100 度才转, 否则锁车头直接平移
 *          (90 度拐弯用横移代替 -> 少一半转向, 也就少一半转向打滑)
 *  mode 0: 永不转, 全程锁定进入本次规划时的车头
 *  g_force_strafe 优先: 该段强制横移
 *=================================================================*/
static float s_yaw_pick(float seg_yaw)
{
    if (g_force_strafe) return s_yaw_lock;
    if (g_yaw_mode == 0) return s_yaw_lock;
    if (g_yaw_mode == 1)
    {
        float e = Attitude_WrapPi(seg_yaw - g_pose.yaw);
        return (fabsf(e) > 1.745f) ? seg_yaw : s_yaw_lock;   /* >100 度才转 */
    }
    return seg_yaw;
}

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
    s_turn_phase = 0;
    s_settle_ms  = 0;
    s_settle_t0  = 0;
    s_wpi++;
    g_nav_wp_index = s_wpi;
    if (s_wpi >= s_wpn)
    {
        s_wpn = 0;
        Chassis_Hold();
        if (s_dock_on)                  /* 到点后先对准指定朝向 */
        {
            g_nav_state = NAV_ALIGN;
            return;
        }
        g_nav_state = NAV_DONE;
        return;
    }
    s_seg_x0 = s_wpx[s_wpi - 1];
    s_seg_y0 = s_wpy[s_wpi - 1];

    /* 最后一段若是「短段」(<= NAV_SHORT_MM): 不强制车头对准段方向,
       锁定进入该段时的朝向, 纯平移到位.
       理由: 麦轮可任意方向平移; 到点拍照/扫码/放置往往不需要特定朝向,
       为 50mm 的横移原地转 90 度既慢又容易过冲(实测在扫码区白转一大圈).
       长段仍按段方向走: 侧向长时间横移麦轮打滑大, 里程计误差会累积.
       若确实需要到点朝向, 用 <<f 角度>> 指定, 到点后会自动对准. */
    {
        float sy = s_seg_yaw(s_seg_x0, s_seg_y0, s_wpx[s_wpi], s_wpy[s_wpi]);
        if (s_wpi + 1 >= s_wpn)
        {
            float lx = s_wpx[s_wpi] - s_seg_x0;
            float ly = s_wpy[s_wpi] - s_seg_y0;
            if (lx * lx + ly * ly <= NAV_SHORT_MM * NAV_SHORT_MM)
                sy = g_pose.yaw;         /* 末段短: 纯平移 */
        }
        s_yaw_t = s_yaw_pick(sy);
    }
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

    /* ---- IMU 失效保护: yaw 已不可信, 自动导航一律立刻停车 ----
       手动模式(v) 不受影响, 因为它只用车体速度, 不依赖 yaw. */
    if (g_imu_dead && (g_nav_state == NAV_RUN || g_nav_state == NAV_ALIGN))
    {
        Chassis_Hold();
        g_nav_fail_why = 1;
        g_nav_state = NAV_FAIL;
        s_turn_ms = 0;
        return;
    }

    /* ---- 到点后对准指定朝向 ---- */
    if (g_nav_state == NAV_ALIGN)
    {
        yaw_err = Attitude_WrapPi(s_dock_yaw - g_pose.yaw);
        wz = s_kp_yaw * yaw_err - s_kd_yaw * g_att.yaw_rate;
        /* 0.030 rad = 1.72 度, 太松: 实测总停在 1.5 度左右.
           收到 0.0087 rad = 0.5 度 */
        if (fabsf(yaw_err) < 0.0087f)
        {
            s_turn_ms = 0;
            Chassis_Hold();
            g_nav_state = NAV_DONE;
            return;
        }
        if (wz >  s_turn_wz_max) wz =  s_turn_wz_max;
        if (wz < -s_turn_wz_max) wz = -s_turn_wz_max;
        /* 死区补偿: 与运行段的转向分支保持一致。
           尾段 wz = 4.0 * err, 差 0.5 度时只有 0.035 rad/s, 麦轮静摩擦转不动,
           会一直等到 NAV_TURN_TIMEOUT_MS 判 why=2 失败 (180 度对准必踩)。 */
        if (fabsf(wz) < NAV_TURN_MIN_WZ)
            wz = (yaw_err > 0.0f) ? NAV_TURN_MIN_WZ : -NAV_TURN_MIN_WZ;
        if (s_turn_ms == 0) s_turn_ms = Tick;
        else if ((uint32_t)(Tick - s_turn_ms) > NAV_TURN_TIMEOUT_MS)
        {
            s_turn_ms = 0;
            Chassis_Hold();
            g_nav_fail_why = 2;
            g_nav_state = NAV_FAIL;
            return;
        }
        Chassis_SetBodyVel(0.0f, 0.0f, wz);
        return;
    }

    /* ---- 非运行态(IDLE/DONE/FAIL): 一律硬停, 不做航向保持 ----
       否则 s_yaw_t 保留上一段航向 + IMU 零偏漂移 -> 持续输出 wz -> 车原地自转 */
    if (g_nav_state != NAV_RUN || s_wpn == 0)
    {
        s_turn_phase = 0;
        Chassis_Hold();
        return;
    }

    /* ================= 模式 3: 纯追踪圆弧过渡 =================
       不停车、不原地转: 在路径上取「前瞻点」(车投影点前方 NAV_CARROT_MM 处),
       车朝它走, 转角大时自动降速。前瞻点越过拐点后自动切下一段,
       于是拐角被走成圆弧而不是「停下-转-再走」。
       注意: 圆弧会切到路径内侧, 窄通道里慎用。 */
    if (g_yaw_mode == 3)
    {
        float ux, uy, ul, t, tgt_x, tgt_y, lx, ly, kv, fx, fy, fd;
        uint8_t guard = 0;

        ux = s_wpx[s_wpi] - s_seg_x0;
        uy = s_wpy[s_wpi] - s_seg_y0;
        ul = sqrtf(ux * ux + uy * uy);
        if (ul < 1.0f) { ul = 1.0f; ux = 1.0f; uy = 0.0f; }
        ux /= ul;  uy /= ul;
        t = (g_pose.x - s_seg_x0) * ux + (g_pose.y - s_seg_y0) * uy;
        if (t < 0.0f) t = 0.0f;

        /* 前瞻点超出本段就推进下一段(不停车), 最多 3 次 */
        while ((t + NAV_CARROT_MM) > ul && (s_wpi + 1) < s_wpn && guard < 3)
        {
            s_wp_next();
            guard++;
            ux = s_wpx[s_wpi] - s_seg_x0;
            uy = s_wpy[s_wpi] - s_seg_y0;
            ul = sqrtf(ux * ux + uy * uy);
            if (ul < 1.0f) { ul = 1.0f; ux = 1.0f; uy = 0.0f; }
            ux /= ul;  uy /= ul;
            t = (g_pose.x - s_seg_x0) * ux + (g_pose.y - s_seg_y0) * uy;
            if (t < 0.0f) t = 0.0f;
        }

        {
            float tt = t + NAV_CARROT_MM;
            if (tt > ul) tt = ul;
            tgt_x = s_seg_x0 + ux * tt;
            tgt_y = s_seg_y0 + uy * tt;
        }
        lx = tgt_x - g_pose.x;
        ly = tgt_y - g_pose.y;

        c = cosf(g_pose.yaw);
        s = sinf(g_pose.yaw);
        bx = ( lx * c + ly * s) * 0.001f;
        by = (-lx * s + ly * c) * 0.001f;

        /* 车头目标: 一般情况下朝前瞻点, 但「末段且很短」时要锁车头纯平移
           (否则 100mm 的小横移也要先转个 90 度, 而且会斜着走)
           这与模式2的 NAV_SHORT_MM 规则保持一致。 */
        if ((s_wpi + 1) >= s_wpn && ul <= NAV_SHORT_MM)
            yaw_err = 0.0f;                       /* 保持当前车头 */
        else
            yaw_err = Attitude_WrapPi(atan2f(ly, lx) - g_pose.yaw);
        wz = s_kp_yaw * yaw_err - s_kd_yaw * g_att.yaw_rate;

        /* 转角越大越慢: 90 度时降到 40% */
        kv = 1.0f - 0.60f * fabsf(yaw_err) / 1.5708f;
        if (kv < 0.35f) kv = 0.35f;

        vx = (s_kp_pos * bx - s_kd_pos * g_chassis.fb_vx) * kv;
        vy = (s_kp_pos * by - s_kd_pos * g_chassis.fb_vy) * kv;

        fx = s_wpx[s_wpi] - g_pose.x;
        fy = s_wpy[s_wpi] - g_pose.y;
        fd = sqrtf(fx * fx + fy * fy);

        if ((s_wpi + 1) >= s_wpn && fd < NAV_FINAL_MM)
        {
            s_wp_next();                 /* 终点: 内部 Chassis_Hold + DONE */
            return;
        }
        if (t >= (ul - 5.0f) && (s_wpi + 1) < s_wpn)
        {
            s_wp_next();                 /* 中间点: 越过即推进, 不停车 */
        }

        vn = sqrtf(vx * vx + vy * vy);
        if (vn > CH_VXY_MAX)
        {
            vx = vx * CH_VXY_MAX / vn;
            vy = vy * CH_VXY_MAX / vn;
        }
        Chassis_SetBodyVel(vx, vy, wz);
        return;
    }

    /* ---- 航向环: 目标 = 当前段方向 (仅运行段工作) ---- */
    yaw_err = Attitude_WrapPi(s_yaw_t - g_pose.yaw);
    wz = s_kp_yaw * yaw_err - s_kd_yaw * g_att.yaw_rate;

    /* ---- 拐点状态机: 0=需要对准  1=停稳驻留  2=已放行(之后连续平移) ---- */
    if (s_turn_phase != 0 && fabsf(yaw_err) > NAV_TURN_GATE)
        s_turn_phase = 0;              /* 平移中航向跑偏过大 -> 重新停下对准 */

    if (s_turn_phase == 0)
    {
        if (fabsf(yaw_err) > NAV_TURN_GATE ||
            fabsf(g_att.yaw_rate) > NAV_TURN_SETTLE_RATE)
        {
            /* 限速: 四轮同时最大出力电流峰值大, 且麦轮原地转易打滑 */
            if (wz >  s_turn_wz_max) wz =  s_turn_wz_max;
            if (wz < -s_turn_wz_max) wz = -s_turn_wz_max;

            /* 死区补偿: 角度还差得多时保证最小角速度, 否则麦轮静摩擦会让它卡在最后几度 */
            if (fabsf(yaw_err) > NAV_TURN_GATE && fabsf(wz) < NAV_TURN_MIN_WZ)
                wz = (yaw_err > 0.0f) ? NAV_TURN_MIN_WZ : -NAV_TURN_MIN_WZ;

            /* 超时: 转不动就判失败, 绝不无限转 */
            if (s_turn_ms == 0) s_turn_ms = Tick;
            else if ((uint32_t)(Tick - s_turn_ms) > NAV_TURN_TIMEOUT_MS)
            {
                s_turn_ms = 0;
                s_settle_ms = 0;  s_settle_t0 = 0;
                Chassis_Hold();
                g_nav_fail_why = 2;
                g_nav_state = NAV_FAIL;
                return;
            }
            Chassis_SetBodyVel(0.0f, 0.0f, wz);
            return;
        }
        /* 角度和角速度都到位 -> 进入停稳驻留 */
        s_turn_phase = 1;
        s_settle_t0  = Tick;
        s_settle_ms  = Tick;
        s_turn_ms    = 0;
    }

    /* ---- 停稳驻留: 命令零速度, 等实际反馈速度真的归零, 并保持一小段时间 ----
       修 bug: 原来驻留结束后立刻把标志清 0, 下一周期又从头进入驻留,
       结果 120ms 里只有 5ms 在平移 -> 车几乎不动。改成 phase 2 放行后保持。 */
    if (s_turn_phase == 1)
    {
        if (fabsf(g_chassis.fb_vx) > NAV_SETTLE_V ||
            fabsf(g_chassis.fb_vy) > NAV_SETTLE_V ||
            fabsf(g_chassis.fb_wz) > NAV_SETTLE_WZ)
            s_settle_ms = Tick;                     /* 还在动: 重新计时 */
        Chassis_SetBodyVel(0.0f, 0.0f, 0.0f);
        if ((uint32_t)(Tick - s_settle_ms) < (uint32_t)g_nav_settle_ms &&
            (uint32_t)(Tick - s_settle_t0)  < ((uint32_t)g_nav_settle_ms + NAV_SETTLE_MAX_EXTRA))
            return;
        s_turn_phase = 2;                           /* 放行, 之后连续平移 */
    }

    /* ---- 误差 ---- */
    ex = s_wpx[s_wpi] - g_pose.x;
    ey = s_wpy[s_wpi] - g_pose.y;
    d  = sqrtf(ex * ex + ey * ey);

    /* ---- 到点/越过判据 ---- */
    if (d < ((s_wpi + 1 >= s_wpn) ? NAV_FINAL_MM : NAV_ARRIVE_MM))
    {
        /* 不停车通过: 只推进航点, 不把速度清零.
           下一周期会按新段重新算 vx/vy; 若新段要转向, 自然进入转向分支停车转.
           终点由 s_wp_next -> Chassis_Hold() 停下来. */
        s_wp_next();
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
                s_wp_next();          /* 越过即推进, 不停车 */
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
    vlim = sqrtf(2.0f * g_nav_a_dec * d * 0.001f);
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
    s_turn_phase = 0;
    s_settle_ms  = 0;
    s_settle_t0  = 0;
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

    /* 若首个航点就在脚下(<=40mm), 跳过它, 改用第二点的方向定航向.
       否则 s_yaw_t 会指成'到格心'的斜角, 车先原地乱转一大圈才开始走. */
    if (s_wpn >= 2)
    {
        float d0x = s_wpx[0] - g_pose.x;
        float d0y = s_wpy[0] - g_pose.y;
        if (d0x * d0x + d0y * d0y < 1600.0f)
        {
            s_wpi = 1;
            g_nav_wp_index = 1;
            s_seg_x0 = s_wpx[0];
            s_seg_y0 = s_wpy[0];
        }
    }

    s_yaw_lock = g_pose.yaw;
    if (s_wpi == 1)
        s_yaw_t = s_yaw_pick(s_seg_yaw(s_wpx[0], s_wpy[0], s_wpx[1], s_wpy[1]));
    else
        s_yaw_t = s_yaw_pick(s_seg_yaw(s_seg_x0, s_seg_y0, s_wpx[0], s_wpy[0]));
    g_nav_state = NAV_RUN;
    __enable_irq();

    /* ---- 调试: 打印航点表与各段航向 ---- */
    printf("# WP total=%d  pose=(%.0f,%.0f) yaw=%.1f deg\r\n",
           s_wpn, g_pose.x, g_pose.y, g_pose.yaw * 57.29578f);
    for (i = 0; i < s_wpn; i++)
    {
        float a = -999.0f;
        if (i > 0)
        {
            float ax = s_wpx[i] - s_wpx[i - 1];
            float ay = s_wpy[i] - s_wpy[i - 1];
            if (ax * ax + ay * ay > 1.0f) a = atan2f(ay, ax) * 57.29578f;
        }
        printf("   [%d] (%.0f,%.0f)  seg=%+.1f deg\r\n", i, s_wpx[i], s_wpy[i], a);
    }
}

/*-----------------------------------------------------------------*
 *  正交化: 把斜线段折成 L 形(两条正交腿),
 *  拐角优先延续当前车头方向, 并合并共线航点.
 *  结果: 所有航段都平行于地图轴, 拐点全是 90 度
 *-----------------------------------------------------------------*/
/* 合并共线: 连续三点方向相同则去掉中间点 */
static void s_merge_collinear(void)
{
    uint8_t i, m = 0;

    for (i = 0; i < s_tn; i++)
    {
        if (m >= 2)
        {
            float ax = s_ox[m - 1] - s_ox[m - 2];
            float ay = s_oy[m - 1] - s_oy[m - 2];
            float bx = s_tx[i]    - s_ox[m - 1];
            float by = s_ty[i]    - s_oy[m - 1];
            float la = sqrtf(ax * ax + ay * ay);
            float lb = sqrtf(bx * bx + by * by);

            if (la > 0.5f && lb > 0.5f)
            {
                float cr = (ax * by - ay * bx) / (la * lb);
                if (fabsf(cr) < 0.05f && (ax * bx + ay * by) > 0.0f)
                {
                    s_ox[m - 1] = s_tx[i];      /* 覆盖上一点, 不新增 */
                    s_oy[m - 1] = s_ty[i];
                    continue;
                }
            }
        }
        s_ox[m] = s_tx[i];
        s_oy[m] = s_ty[i];
        m++;
    }

    for (i = 0; i < m; i++)
    {
        s_tx[i] = s_ox[i];
        s_ty[i] = s_oy[i];
    }
    s_tn = m;
}

/* 去掉与前一点距离 < minmm 的中间点(首尾保留) */
static void s_drop_short(float minmm)
{
    uint8_t i, m = 1;
    float lx = s_tx[0], ly = s_ty[0];
    float lim = minmm * minmm;

    for (i = 1; i < s_tn; i++)
    {
        float dx = s_tx[i] - lx, dy = s_ty[i] - ly;
        if ((i == s_tn - 1) || (dx * dx + dy * dy) > lim)
        {
            s_tx[m] = s_tx[i];
            s_ty[m] = s_ty[i];
            lx = s_tx[i];  ly = s_ty[i];
            m++;
        }
    }
    s_tn = m;
}

static void s_emit(uint8_t *m, float x, float y)
{
    if (*m >= NAV_WP_MAX) return;               /* 容量保护 */

    /* 合并共线: 与上一点方向相同则覆盖上一点 */
    if (*m >= 2)
    {
        float ax = s_ox[*m - 1] - s_ox[*m - 2];
        float ay = s_oy[*m - 1] - s_oy[*m - 2];
        float bx = x - s_ox[*m - 1];
        float by = y - s_oy[*m - 1];
        float la = sqrtf(ax * ax + ay * ay);
        float lb = sqrtf(bx * bx + by * by);
        if (la > 0.5f && lb > 0.5f)
        {
            float cr = (ax * by - ay * bx) / (la * lb);
            if (fabsf(cr) < 0.03f && (ax * bx + ay * by) > 0.0f) (*m)--;
        }
    }
    s_ox[*m] = x;
    s_oy[*m] = y;
    (*m)++;
}

/* 小偏斜段(次轴 <= 主轴 25%, 且主轴 > 200mm): 直接保留斜线, 不折 L。
   折 L 只是为了几十毫米的横向位移就要多转一次 90 度(多一次停车+打滑),
   反而更慢更晃。这条斜线本身已通过 s_los 检查, 是通的, 所以安全。 */
static uint8_t s_keep_diag(float x0, float y0, float ax, float ay, uint8_t first)
{
    float ax_f = fabsf(ax), ay_f = fabsf(ay);
    float amin = (ax_f < ay_f) ? ax_f : ay_f;
    float amax = (ax_f < ay_f) ? ay_f : ax_f;
    int r, c;
    /* 只在「从启停区出发的第一段」放宽: 那几十毫米的横向偏差不值得
       多转一次 90 度(两次停车+打滑)。其它段一律保持原正交策略,
       不动已经标定好的路线。 */
    if (!first) return 0;
    s_xy_to_cell(x0, y0, &r, &c);
    if (Map_Kind(r, c) != 2) return 0;
    return (uint8_t)(amax > 300.0f && amin <= 80.0f);
}

static void s_ortho(void)
{
    uint8_t i, m;
    float cur;

    if (s_tn == 0) return;

    cur = g_pose.yaw;                           /* 起步沿当前车头方向 */
    s_ox[0] = s_tx[0];
    s_oy[0] = s_ty[0];
    m = 1;

    for (i = 1; i < s_tn; i++)
    {
        float x0 = s_tx[i - 1], y0 = s_ty[i - 1];
        float x1 = s_tx[i],     y1 = s_ty[i];
        float ax = x1 - x0, ay = y1 - y0;

        if (fabsf(ax) < 0.5f || fabsf(ay) < 0.5f)       /* 已是正交段 */
        {
            s_emit(&m, x1, y1);
            cur = atan2f(ay, ax);
        }
        else if (s_keep_diag(x0, y0, ax, ay, (uint8_t)(i == 1))) /* 启停区出口: 保留斜线 */
        {
            s_emit(&m, x1, y1);
            cur = atan2f(ay, ax);
        }
        else                                            /* 斜段: 折成 L */
        {
            float c1x, c1y, c2x, c2y;
            if (fabsf(sinf(cur)) > fabsf(cosf(cur)))    /* 当前竖向: 先竖后横 */
            {
                c1x = x0;  c1y = y1;
                c2x = x1;  c2y = y0;
            }
            else                                        /* 当前横向: 先横后竖 */
            {
                c1x = x1;  c1y = y0;
                c2x = x0;  c2y = y1;
            }

            if (s_los(x0, y0, c1x, c1y) && s_los(c1x, c1y, x1, y1))
            {
                s_emit(&m, c1x, c1y);
                s_emit(&m, x1, y1);
                cur = atan2f(y1 - c1y, x1 - c1x);
            }
            else if (s_los(x0, y0, c2x, c2y) && s_los(c2x, c2y, x1, y1))
            {
                s_emit(&m, c2x, c2y);
                s_emit(&m, x1, y1);
                cur = atan2f(y1 - c2y, x1 - c2x);
            }
            else                                        /* 两条 L 都不通: 退让保留斜线 */
            {
                s_emit(&m, x1, y1);
                cur = atan2f(ay, ax);
            }
        }
    }

    for (i = 0; i < m; i++)
    {
        s_tx[i] = s_ox[i];
        s_ty[i] = s_oy[i];
    }
    s_tn = m;
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
    /* 精确终点: 用 <<p>> 请求的 mm 取代最后一个格心(仍在同一格内),
       让后面的视线简化与正交化自然折 L, 不会事后多插一段倒退小段. */
    if (s_goal_exact && n > 1)
    {
        s_px[n - 1] = (int16_t)s_goal_x;
        s_py[n - 1] = (int16_t)s_goal_y;
    }
    /* 起点: 用车的真实位姿, 不用格心.
       用格心会凭空多出一段'先走到格心'的斜线, 初始航向被算成斜角 -> 原地乱转. */
    s_px[0] = (int16_t)g_pose.x;
    s_py[0] = (int16_t)g_pose.y;
    s_tx[0] = g_pose.x;
    s_ty[0] = g_pose.y;
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

    s_ortho();                          /* 斜线折 L, 全段正交化 */
    s_drop_short(25.0f);                /* 合并 <25mm 的碎腿(精确终点常见), 避免多余的 90 度转向 */
    s_merge_collinear();                /* 再合并方向相同的连续点, 少一次到点判据/减速 */
}

/**
  * @brief  规划到指定格子
  * @retval 1 成功  0 失败
  */
static uint8_t s_plan_cells(uint8_t row, uint8_t col)
{
    int sr, sc, gr, gc, n;

    s_xy_to_cell(g_pose.x, g_pose.y, &sr, &sc);
    if (!s_find_free(&sr, &sc)) { g_nav_fail_why = 4; g_nav_state = NAV_FAIL; return 0; }

    gr = row;  gc = col;
    if (!s_find_free(&gr, &gc)) { g_nav_fail_why = 4; g_nav_state = NAV_FAIL; return 0; }

    n = AStar_Search(sr, sc, gr, gc, s_cells, MAP_CELLS);
    if (n <= 0) { g_nav_fail_why = 3; g_nav_state = NAV_FAIL; return 0; }

    s_build_from_cells(n);
    if (s_tn == 0) { g_nav_fail_why = 3; g_nav_state = NAV_FAIL; return 0; }
    if (s_tn == 1)                       /* 已经在该格 */
    {
        g_nav_state = NAV_DONE;
        return 1;
    }
    s_commit();
    return 1;
}

uint8_t Nav_PlanToCell(uint8_t row, uint8_t col)
{
    s_goal_exact = 0;                 /* 按格心规划 */
    return s_plan_cells(row, col);
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
    uint8_t ok;
    float lim = (float)MAP_N * MAP_CELL_MM;

    if (x_mm < 0.0f)      x_mm = 0.0f;
    if (x_mm > lim)       x_mm = lim;
    if (y_mm < 0.0f)      y_mm = 0.0f;
    if (y_mm > lim)       y_mm = lim;

    s_xy_to_cell(x_mm, y_mm, &r, &c);
    /* 精确终点: 先把目标 mm 交给 s_build_from_cells, 让它参与正交化.
       早先的做法是规划完再把最后一个航点改成 mm 并补一个 L 拐角,
       若精确点在来向后方, 这个 L 会造出 20mm 的倒退段 ->
       车到点后先原地掉头 180 度再走, 看起来就是「乱转」. */
    /* 失败原因必须在规划之前清: 规划过程中 5ms 中断可能已经把
       NAV_FAIL + why 一起置上了, 事后再清会把原因擦掉(表现为 why=0) */
    g_nav_fail_why = 0;

    s_goal_x = x_mm;
    s_goal_y = y_mm;
    s_goal_exact = 1;
    ok = s_plan_cells((uint8_t)r, (uint8_t)c);
    s_goal_exact = 0;
    return ok;
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

/*=================================================================
 *  内置路线（一键跑图）  <<R 1>> 开始   <<R 0>> 停止
 *
 *  路线来源: 工创地图\路线规划器.html 导出后人工确认
 *    - 每点是一段独立 A* 规划的目标
 *    - strafe=1 表示该段锁定车头纯平移(横移), 不转向
 *    - 已校验: 15 段全部可规划; 转向点最小余量 56mm (不再走中间走廊里掉头)
 *=================================================================*/
#define NAV_R1_N   5
#define NAV_R2_N   8

/* ---- 路线 1: 只给功能区终点, 路径全交给 A* ----
       作业点由人标定, 路径交给 A*:
       扫码(2225,1195) -> 夹取(1195,2155) -> 初加工(1195,265) -> 暂存(305,1195) -> 回启停(2225,175)
       2026-11-21: P1 x 2175->2225, P3 x 1205->1195, 让 leg1/leg3 变成纯竖直线。
       原来这两个点和上一点的 x 差 50/10mm, 正交化会折出一条 50mm / 10mm 的碎腿,
       而碎腿前要原地转 90 度、走几十毫米、再在 5mm 死区里停 -> 纯浪费且精度差。 */
static const float   s_r1_x[NAV_R1_N] =
{
     2225,  1195,  1195,   305,  2225
};
static const float   s_r1_y[NAV_R1_N] =
{
     1195,  2155,   265,  1195,   175
};
static const uint8_t s_r1_s[NAV_R1_N] =
{
        0,     0,     0,     0,     0
};
/* 到点朝向 (度, 0=右 90=上 -90=下 180=左; 999 = 不指定, 车头跟随段方向)
   最后一点指定 90 度: 回程结束车头朝上, 与出发点一致。
   这样测同一测点的位移就等于车心位移 —— 否则 180 度旋转会把偏心测点的
   位移放大 2r (实测螺丝偏心 39mm -> 假漂移 78mm)。
   比赛上也更好: 一轮结束车头朝上, 可以直接接着跑下一轮。 */
static const int16_t s_r1_f[NAV_R1_N] =
{
      999,   999,   999,   999,    90
};

/* ---- 路线 2: 2026-11-22 用户标定
       扫码 -> [夹取 -> 初加工 -> 暂存] x2 -> 回启停
       起点: <<q 2250 136 90>>    末点 (2250,136) 严格闭环
       工具逐段核过: 14458mm / 18 航点 / 12 次转向 / 可规划
       备注: y=136 处车横过来(朝左右)时右侧面会越过场地线 9mm,
             若场地外沿没有护栏/桌面余量足够, 不影响行驶。 ---- */
static const float   s_r2_x[NAV_R2_N] =
{
     2105,  1195,  1175,   315,  1205,  1185,   315,  2250
};
static const float   s_r2_y[NAV_R2_N] =
{
     1185,  2065,   355,  1155,  2065,   355,  1165,   136
};
static const int16_t s_r2_f[NAV_R2_N] =
{
      999,   999,   999,   999,   999,   999,   999,   999
};

/* ---- 编译期长度检查 ----
   数组初始化项少于 NAV_R?_N 时 C 会用 0 补齐，那个点是 (0,0) 会跑到地图角落，
   而且编译不报错。这里用一个负数组长度的 typedef 把这种情况变成编译错误。 */
typedef char s_r1_len_chk[(sizeof(s_r1_x)/sizeof(s_r1_x[0])) == NAV_R1_N ? 1 : -1];
typedef char s_r1_len_chk2[(sizeof(s_r1_y)/sizeof(s_r1_y[0])) == NAV_R1_N ? 1 : -1];
typedef char s_r2_len_chk[(sizeof(s_r2_x)/sizeof(s_r2_x[0])) == NAV_R2_N ? 1 : -1];
typedef char s_r2_len_chk2[(sizeof(s_r2_y)/sizeof(s_r2_y[0])) == NAV_R2_N ? 1 : -1];

/* ---- 当前绑定的路线 ---- */
static const float   *rt_x;
static const float   *rt_y;
static const uint8_t *rt_s;
static const int16_t *rt_f;
static uint8_t rt_n  = 0;
static uint8_t rt_id = 0;

static void s_route_bind(uint8_t id)
{
    if (id == 1)
    {
        rt_x = s_r1_x;  rt_y = s_r1_y;  rt_s = s_r1_s;  rt_f = s_r1_f;
        rt_n = NAV_R1_N;
    }
    else
    {
        rt_x = s_r2_x;  rt_y = s_r2_y;  rt_s = 0;        rt_f = s_r2_f;
        rt_n = NAV_R2_N;
    }
    rt_id = id;
}

/* 按表设置该点的到点朝向; 没有表或 999 就取消 */
static void s_route_dock_yaw(uint8_t i)
{
    if (rt_f && rt_f[i] != 999) Nav_SetDockYaw((float)rt_f[i]);
    else                        Nav_SetDockYaw(999.0f);
}

void Nav_RouteStart(uint8_t which)
{
    if (which == 0)
    {
        s_route_on = 0;
        s_route_settle_ms = 0;
        g_force_strafe = 0;
        Nav_SetDockYaw(999.0f);
        Nav_Stop();
        printf("# route stopped at %d\r\n", s_route_i + 1);
        return;
    }
    if (which != 1 && which != 2)
    {
        printf("# route: 1 或 2\r\n");
        return;
    }

    /* IMU 失效时直接拒绝: yaw 不可信, 强行跑只会乱转 */
    if (g_imu_dead)
    {
        s_route_on = 0;
        g_force_strafe = 0;
        printf("# route refused: IMU DEAD (三轴恒零/无回帧), 请断电重启\r\n");
        return;
    }

    s_route_bind(which);
    s_route_on = 1;
    s_route_i  = 0;
    s_route_settle_ms = 0;
    g_force_strafe = rt_s ? rt_s[0] : 0;
    s_route_dock_yaw(0);

    if (Nav_PlanToMm(rt_x[0], rt_y[0]))
        printf("# route%d start: 1/%d%s\r\n", rt_id, rt_n,
               (rt_s && rt_s[0]) ? " (strafe)" : "");
    else
    {
        s_route_on = 0;
        g_force_strafe = 0;
        printf("# route%d FAIL at 1\r\n", rt_id);
    }
}

void Nav_RoutePoll(void)
{
    if (!s_route_on) return;

    if (g_nav_state == NAV_FAIL)
    {
        s_route_on = 0;
        s_route_settle_ms = 0;
        g_force_strafe = 0;
        Nav_SetDockYaw(999.0f);
        printf("# route%d FAIL at %d  why=%d  imu_dead=%d  state=%d\r\n",
               rt_id, s_route_i + 1, g_nav_fail_why, g_imu_dead, g_nav_state);
        return;
    }
    if (g_nav_state != NAV_DONE)
    {
        s_route_settle_ms = 0;
        return;
    }

    /* ---- 停稳再走 ---- */
    if (fabsf(g_chassis.fb_vx) > NAV_ROUTE_SETTLE_V ||
        fabsf(g_chassis.fb_vy) > NAV_ROUTE_SETTLE_V ||
        fabsf(g_chassis.fb_wz) > NAV_ROUTE_SETTLE_W)
    {
        s_route_settle_ms = 0;
        return;
    }
    if (s_route_settle_ms == 0) { s_route_settle_ms = Tick; return; }
    if ((uint32_t)(Tick - s_route_settle_ms) < NAV_ROUTE_SETTLE_MS) return;
    s_route_settle_ms = 0;

    s_route_i++;
    if (s_route_i >= rt_n)
    {
        s_route_on = 0;
        s_route_settle_ms = 0;
        g_force_strafe = 0;
        Nav_SetDockYaw(999.0f);
        printf("# route%d DONE (%d pts)  final odom=(%.0f,%.0f) yaw=%.1f\r\n",
               rt_id, rt_n, g_pose.x, g_pose.y, g_pose.yaw * 57.29578f);
        return;
    }

    g_force_strafe = rt_s ? rt_s[s_route_i] : 0;
    s_route_dock_yaw(s_route_i);
    /* 点位到达时的里程计位姿: 和卷尺实测逐点对照, 才能分清是"里程计漂"还是"没走到" */
    printf("# wp%d reached  odom=(%.0f,%.0f) yaw=%.1f  ->  next target (%.0f,%.0f)\r\n",
           s_route_i, g_pose.x, g_pose.y, g_pose.yaw * 57.29578f,
           rt_x[s_route_i], rt_y[s_route_i]);
    if (Nav_PlanToMm(rt_x[s_route_i], rt_y[s_route_i]))
        printf("# route%d %d/%d%s%s\r\n", rt_id, s_route_i + 1, rt_n,
               (rt_s && rt_s[s_route_i]) ? " (strafe)" : "",
               (rt_f && rt_f[s_route_i] != 999) ? " (dock)" : "");
    else
    {
        s_route_on = 0;
        s_route_settle_ms = 0;
        g_force_strafe = 0;
        printf("# route%d FAIL at %d\r\n", rt_id, s_route_i + 1);
    }
}

/* 供遥测使用: 当前段的目标航向 (rad) */
float Nav_GetYawTarget(void) { return s_yaw_t; }

void  Nav_SetTurnWz(float wz)
{
    if (wz >= 0.3f && wz <= 2.5f) s_turn_wz_max = wz;
}
float Nav_GetTurnWz(void) { return s_turn_wz_max; }

void  Nav_SetADec(float a)
{
    if (a > 0.10f && a < 8.0f) g_nav_a_dec = a;
}
float Nav_GetADec(void) { return g_nav_a_dec; }

void Nav_SetSettleMs(uint16_t ms)
{
    if (ms <= 800u) g_nav_settle_ms = ms;   /* 超过 800ms 不接受, 防止把路线拖死 */
}
uint16_t Nav_GetSettleMs(void) { return g_nav_settle_ms; }

uint8_t Nav_RouteBusy(void)  { return s_route_on; }
uint8_t Nav_RouteIndex(void) { return s_route_i; }
