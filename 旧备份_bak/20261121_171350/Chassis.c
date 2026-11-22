#include <math.h>
#include "Chassis.h"
#include "DM_BLDC_Motor.h"

Chassis_t g_chassis;
/* 加减速度限值: 默认 = CH_ACC_V; <<D a>> 现场改, 不发命令则行为与改前完全一致 */
float g_ch_acc_v = CH_ACC_V;

/* 轮序 0/1/2/3 与电机数据块的对应关系 */
static DM_Motor_Data *const s_mot[4] =
{
    &DM_Motor_4310P_Next_StructData,    /* 0x01 左前 A */
    &DM_Motor_4310P_StructData,         /* 0x02 右前 B */
    &DM_Motor_4310_StructData,          /* 0x03 左后 B */
    &DM_Motor_4310_Next_StructData,     /* 0x04 右后 A */
};

static float s_lim(float x, float lim)
{
    if (x >  lim) return  lim;
    if (x < -lim) return -lim;
    return x;
}

static float s_ramp(float cur, float tgt, float step)
{
    float d = tgt - cur;
    if (d >  step) d =  step;
    if (d < -step) d = -step;
    return cur + d;
}

/**
  * @brief  底盘初始化
  */
void Chassis_Init(void)
{
    uint8_t i;
    /* 实测: 右前/右后两轮安装镜像, 默认反向 (2026-11-16 实车标定) */
    static const float s_dir0[4] = { 1.0f, -1.0f, 1.0f, -1.0f };
    for (i = 0; i < 4; i++)
    {
        g_chassis.dir[i]    = s_dir0[i];
        g_chassis.w_rad[i]  = 0.0f;
        g_chassis.fb_rad[i] = 0.0f;
    }
    g_chassis.vx  = 0.0f;  g_chassis.vy  = 0.0f;  g_chassis.wz  = 0.0f;
    g_chassis.cvx = 0.0f;  g_chassis.cvy = 0.0f;  g_chassis.cwz = 0.0f;
    g_chassis.fb_vx = 0.0f; g_chassis.fb_vy = 0.0f; g_chassis.fb_wz = 0.0f;
    g_chassis.single_mode = 0;
    g_chassis.single_idx  = 0;
    g_chassis.single_rad  = 0.0f;
}

/**
  * @brief  设定目标车体速度 (m/s, m/s, rad/s)
  * @note   直接覆盖目标值, 斜坡在下发时做
  */
void Chassis_SetBodyVel(float vx, float vy, float wz)
{
    g_chassis.vx = s_lim(vx, CH_VXY_MAX);
    g_chassis.vy = s_lim(vy, CH_VXY_MAX);
    g_chassis.wz = s_lim(wz, CH_WZ_MAX);
}

/**
  * @brief  目标速度清零
  */
void Chassis_Hold(void)
{
    g_chassis.vx = 0.0f;
    g_chassis.vy = 0.0f;
    g_chassis.wz = 0.0f;
}

/**
  * @brief  方向标定
  * @param  idx 0=左前 1=右前 2=左后 3=右后, d=+1/-1
  */
void Chassis_SetDir(uint8_t idx, float d)
{
    if (idx < 4) g_chassis.dir[idx] = (d >= 0.0f) ? 1.0f : -1.0f;
}

/**
  * @brief  单轮标定: 只让一个轮子按指定 rad/s 转, 其余三轮为 0
  */
void Chassis_Single(uint8_t idx, float rad)
{
    if (idx > 3) return;
    g_chassis.single_mode = 1;
    g_chassis.single_idx  = idx;
    g_chassis.single_rad  = s_lim(rad, CH_RAD_USE);
}

void Chassis_SetAccV(float a)
{
    if (a > 0.10f && a < 8.0f) g_ch_acc_v = a;
}
float Chassis_GetAccV(void) { return g_ch_acc_v; }

void Chassis_SingleExit(void)
{
    g_chassis.single_mode = 0;
    g_chassis.single_rad  = 0.0f;
}

/**
  * @brief  反馈正解: 四轮角速度 -> 车体速度
  */
void Chassis_GetBodyVel(float *vx, float *vy, float *wz)
{
    float a = g_chassis.fb_rad[0];
    float b = g_chassis.fb_rad[1];
    float c = g_chassis.fb_rad[2];
    float d = g_chassis.fb_rad[3];

    *vx =  CH_WHEEL_R * 0.25f * (  a + b + c + d );
    *vy =  CH_WHEEL_R * 0.25f * ( -a + b + c - d );
    *wz =  CH_WHEEL_R * 0.25f / CH_K * ( -a + b - c + d );
}

/**
  * @brief  底盘周期任务 (5 ms)
  * @note   斜坡 -> 逆解 -> 等比限幅 -> 四帧下发; 同时做反馈正解
  */
void Chassis_Update(void)
{
    uint8_t i;
    float w[4];
    float mx = 0.0f, sc;
    const float dtv = g_ch_acc_v * 0.005f;
    const float dtw = CH_ACC_W * 0.005f;

    /* ---- 反馈正解 ---- */
    for (i = 0; i < 4; i++)
        g_chassis.fb_rad[i] = s_mot[i]->VEL * g_chassis.dir[i];
    Chassis_GetBodyVel(&g_chassis.fb_vx, &g_chassis.fb_vy, &g_chassis.fb_wz);

    /* ---- 斜坡 ---- */
    g_chassis.cvx = s_ramp(g_chassis.cvx, g_chassis.vx, dtv);
    g_chassis.cvy = s_ramp(g_chassis.cvy, g_chassis.vy, dtv);
    g_chassis.cwz = s_ramp(g_chassis.cwz, g_chassis.wz, dtw);

    /* ---- 单轮标定模式 ---- */
    if (g_chassis.single_mode)
    {
        for (i = 0; i < 4; i++)
        {
            g_chassis.w_rad[i] = 0.0f;
            DM_CAN_Motor_Velocity_Rad(s_mot[i], 0.0f);
        }
        g_chassis.w_rad[g_chassis.single_idx] = g_chassis.single_rad * g_chassis.dir[g_chassis.single_idx];
        DM_CAN_Motor_Velocity_Rad(s_mot[g_chassis.single_idx], g_chassis.w_rad[g_chassis.single_idx]);
        return;
    }

    /* ---- 逆解 ----
       v_FL = +vx - vy - k*wz
       v_FR = +vx + vy + k*wz
       v_RL = +vx + vy - k*wz
       v_RR = +vx - vy + k*wz                            */
    w[CH_FL] = ( g_chassis.cvx - g_chassis.cvy - CH_K * g_chassis.cwz ) / CH_WHEEL_R;
    w[CH_FR] = ( g_chassis.cvx + g_chassis.cvy + CH_K * g_chassis.cwz ) / CH_WHEEL_R;
    w[CH_RL] = ( g_chassis.cvx + g_chassis.cvy - CH_K * g_chassis.cwz ) / CH_WHEEL_R;
    w[CH_RR] = ( g_chassis.cvx - g_chassis.cvy + CH_K * g_chassis.cwz ) / CH_WHEEL_R;

    /* ---- 等比限幅 (保持方向) ---- */
    for (i = 0; i < 4; i++)
    {
        if (fabsf(w[i]) > mx) mx = fabsf(w[i]);
    }
    if (mx > CH_RAD_USE)
    {
        sc = CH_RAD_USE / mx;
        for (i = 0; i < 4; i++) w[i] *= sc;
    }

    /* ---- 下发 ---- */
    for (i = 0; i < 4; i++)
    {
        g_chassis.w_rad[i] = w[i] * g_chassis.dir[i];
        DM_CAN_Motor_Velocity_Rad(s_mot[i], g_chassis.w_rad[i]);
    }
}
