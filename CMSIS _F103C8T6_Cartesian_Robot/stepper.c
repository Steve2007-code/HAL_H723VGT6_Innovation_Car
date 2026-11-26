/******************************************************************************
 * stepper.c  --  双路步进电机驱动：速度斜坡 + 位置控制
 *
 * 接线：
 *   通道A: ST1=PA7  DIR1=PA6  EN1=PA5
 *   通道B: ST2=PB0  DIR2=PB1  EN2=PA4
 *   （EN2 由 PB10 改到 PA4，让出 PB10/PB11 给 USART3 接第二路摄像头）
 *   必须共地：驱动板 GND <-> STM32 GND
 *   J2 上的 5V 是驱动板输出，不要和蓝板的 5V 并接
 *
 * 【控制方式】
 *   TIM2 定时中断 50kHz（20us 一拍）作为时基：
 *     - 脉冲由"速度累加器"产生，所以速度可以连续变化（斜坡不丢步）
 *     - 每 1ms 调整一次速度，就是"匀加速/匀减速"
 *   CPU 只负责下命令，加减速和发脉冲都在中断里自己跑，不阻塞主循环。
 *
 * 【实测结论】
 *   EN 高电平使能；STEP 空闲低、脉冲高。
 *   细分 1/8 -> 1600 步/圈（拨码 SW1=ON, SW2=OFF, SW3=OFF, SW4~SW6=ON）
 ******************************************************************************/

#include "stepper.h"
#include "track_config.h"       /* 取 TRACK_ACC_PPS_PER_MS，跟随减速度 */

/* ------------------------- 时基 ------------------------- */
#define TICK_HZ            50000UL      /* TIM2 中断频率 50kHz */
#define TIMER_CNT_HZ       1000000UL    /* 计数器频率 1MHz */

/* ------------------------- 速度单位 ------------------------- */
/* 速度内部用 0.1 步/秒 为单位，避免整除误差 */
#define SPEED_UNIT         10
#define MIN_PPS            10u          /* 最低启动速度 10 步/秒 */
#define MAX_PPS            12000u       /* 最高 12000 步/秒 (32 细分约 112RPM, 远低于电机能力) */
#define MIN_SPEED_U        ((int32_t)MIN_PPS * SPEED_UNIT)

/* 脉冲累加器阈值：每 1/50kHz 加一次 speed，累加到该值就发一步 */
#define PULSE_TH           ((int32_t)(TICK_HZ * SPEED_UNIT))

/* ------------------------- 运行模式 ------------------------- */
#define MODE_IDLE          0u
#define MODE_JOG           1u           /* 连续跑，直到 Stop */
#define MODE_POS           2u           /* 走到目标位置后自停 */
#define MODE_TRACK         3u           /* 连续速度跟随（串口 PID） */

#define MAX_SPEED_U        ((int32_t)MAX_PPS * SPEED_UNIT)
#define TRACK_ACC_U        ((int32_t)TRACK_ACC_PPS_PER_MS * SPEED_UNIT)  /* 每 1ms 速度增量 */

/* ------------------------- 引脚定义 ------------------------- */
typedef struct {
    GPIO_TypeDef *st_port;  uint8_t st_pin;
    GPIO_TypeDef *dir_port; uint8_t dir_pin;
    GPIO_TypeDef *en_port;  uint8_t en_pin;
} StepperPins;

static const StepperPins pins[2] = {
    { GPIOA, 7,  GPIOA, 6,  GPIOA, 5  },   /* 通道A: ST=PA7 DIR=PA6 EN=PA5 */
    { GPIOB, 0,  GPIOB, 1,  GPIOA, 4  }    /* 通道B: ST=PB0 DIR=PB1 EN=PA4 (原 PB10 让给 USART3) */
};

/* ------------------------- 每轴状态 ------------------------- */
typedef struct {
    volatile int32_t pos;        /* 当前位置（微步） */
    volatile int32_t target;     /* 目标位置（位置模式用） */
    volatile int32_t speed;      /* 当前速度，单位 0.1 步/秒 */
    volatile int32_t cmd;        /* 目标（匀速）速度 */
    volatile int32_t vel_cmd;    /* 跟随模式：有符号目标速度，0.1 步/秒 */
    volatile int32_t ramp_num;   /* 每 1ms 的速度增量 = ramp_num / ramp_den */
    volatile int32_t ramp_den;   /* 斜坡分母（毫秒） */
    volatile int32_t ramp_rem;   /* 斜坡累加余数 */
    volatile int32_t acc;        /* 脉冲累加器 */
    volatile uint16_t ms_cnt;    /* 1ms 分频计数 */
    volatile uint8_t idx;
    volatile uint8_t dir;        /* 1 = pos 递增 */
    volatile uint8_t mode;
    volatile uint8_t decel;      /* 1 = 正在减速 */
    volatile uint8_t pulse;      /* 1 = 本拍有脉冲在拉高 */
} Axis_t;

static Axis_t ax[2];

/* ------------------------- 软件限位 ------------------------- */
/* 命令层先钳目标；点动(JOG)由 axis_ramp 提前减速；脉冲输出前再兜底 */
static const int32_t POS_MIN[2] = { AXIS_A_POS_MIN, AXIS_B_POS_MIN };
static const int32_t POS_MAX[2] = { AXIS_A_POS_MAX, AXIS_B_POS_MAX };

static int32_t pos_clamp(uint8_t idx, int32_t p)
{
    if (p < POS_MIN[idx]) return POS_MIN[idx];
    if (p > POS_MAX[idx]) return POS_MAX[idx];
    return p;
}

/* ------------------------- 底层操作 ------------------------- */
static void pin_out_init(GPIO_TypeDef *port, uint8_t pin)
{
    if (pin < 8) {
        port->CRL &= ~(0xFU << (pin * 4));
        port->CRL |=  (0x3U << (pin * 4));      /* 50MHz 推挽输出 */
    } else {
        port->CRH &= ~(0xFU << ((pin - 8) * 4));
        port->CRH |=  (0x3U << ((pin - 8) * 4));
    }
}

static void pin_write(GPIO_TypeDef *port, uint8_t pin, uint8_t level)
{
    if (level) port->BSRR = (uint32_t)1 << pin;
    else       port->BRR  = (uint32_t)1 << pin;
}

static void st_write(uint8_t a, uint8_t level)
{
    pin_write(pins[a].st_port, pins[a].st_pin, level);
}

static void dir_write(uint8_t a, uint8_t dir)
{
    pin_write(pins[a].dir_port, pins[a].dir_pin, dir ? 1 : 0);
}

/* ------------------------- 内部：一段运动结束 ------------------------- */
static void axis_finish(Axis_t *a)
{
    a->speed    = 0;
    a->acc      = 0;
    a->ramp_rem = 0;
    a->decel    = 0;
    a->vel_cmd  = 0;
    a->mode     = MODE_IDLE;
}

/* ------------------------- 内部：还要走多少步（位置模式） ------------------------- */
static int32_t axis_remain(Axis_t *a)
{
    return a->dir ? (a->target - a->pos) : (a->pos - a->target);
}

/* ------------------------- 内部：按当前速度算减速距离（步） ------------------------- */
/* 匀减速距离 = v^2 / (2a)。v、a 都是 0.1 步/秒 单位，
   换算后 decel_steps = v^2 / (20 * a)                              */
static int32_t axis_decel_steps(Axis_t *a)
{
    int32_t    a_units;
    long long  num;

    if (a->ramp_num <= 0 || a->ramp_den <= 0) return 0;

    a_units = (int32_t)((long long)a->ramp_num * 1000LL / a->ramp_den);
    if (a_units <= 0) a_units = 1;

    num = (long long)a->speed * (long long)a->speed;
    num = num / (20LL * (long long)a_units);
    if (num > 0x7FFFFFFFLL) num = 0x7FFFFFFFLL;
    return (int32_t)num;
}

/* ------------------------- 内部：每 1ms 调一次速度（核心斜坡） ------------------------- */
static void axis_ramp(Axis_t *a)
{
    int32_t floor_speed;
    int32_t rem;

    if (a->mode == MODE_POS) {
        rem = axis_remain(a);
        if (rem <= 0) {                 /* 已经到位 */
            axis_finish(a);
            return;
        }
        if (!a->decel && rem <= axis_decel_steps(a)) {
            a->decel    = 1;            /* 该开始减速了，保证正好停在目标上 */
            a->ramp_rem = 0;
        }
    } else if (a->mode == MODE_JOG && !a->decel) {
        int32_t lim_rem;

        /* 点动没有目标：按剩余步数提前进入减速，绝不硬停。
           沿用加速阶段的 ramp_num/ramp_den（对称斜坡），
           axis_decel_steps() 算出的距离就是实际减速距离，正好停在边界附近。 */
        lim_rem = a->dir ? (POS_MAX[a->idx] - a->pos)
                         : (a->pos - POS_MIN[a->idx]);
        if (lim_rem <= axis_decel_steps(a)) {
            a->decel    = 1;
            a->ramp_rem = 0;
        }
    } else if (a->mode == MODE_TRACK) {
        int32_t want_dir;
        int32_t mag;
        int32_t lim_rem;

        /* 跟随模式完全自己管斜坡，不落下面的公共加减速段 */

        /* 限位减速收尾：只把速度降到 0，保持 MODE_TRACK 待命，不退出跟随 */
        if (a->decel) {
            if (a->speed <= TRACK_ACC_U) {
                a->speed   = 0;
                a->decel   = 0;
                a->vel_cmd = 0;
            } else {
                a->speed -= TRACK_ACC_U;
            }
            return;
        }

        /* 限位预判：剩余步数不够减速距离就提前减速，并清指令防止顶着边界反复启停 */
        lim_rem = a->dir ? (POS_MAX[a->idx] - a->pos)
                         : (a->pos - POS_MIN[a->idx]);
        if (a->speed > 0 && lim_rem <= axis_decel_steps(a)) {
            a->decel   = 1;
            a->vel_cmd = 0;
            if (a->speed <= TRACK_ACC_U) {
                a->speed = 0;
                a->decel = 0;
            } else {
                a->speed -= TRACK_ACC_U;
            }
            return;
        }

        /* 换向：指令方向与当前方向不一致时，先减到 0 再翻 DIR，过零软换向 */
        want_dir = (a->vel_cmd >= 0) ? 1 : 0;
        if (a->vel_cmd != 0 && want_dir != (int32_t)a->dir) {
            if (a->speed <= TRACK_ACC_U) {
                a->speed = 0;
                a->dir   = (uint8_t)want_dir;
                dir_write(a->idx, a->dir);      /* 速度为 0 时翻方向，安全 */
            } else {
                a->speed -= TRACK_ACC_U;
            }
            return;
        }

        /* 正常跟随：速度朝 |vel_cmd| 靠拢，加速度固定 TRACK_ACC_U */
        mag = a->dir ? a->vel_cmd : -a->vel_cmd;
        if (mag < 0) mag = 0;
        if (mag > MAX_SPEED_U) mag = MAX_SPEED_U;

        if (a->speed < mag) {
            a->speed += TRACK_ACC_U;
            if (a->speed > mag) a->speed = mag;
        } else if (a->speed > mag) {
            a->speed -= TRACK_ACC_U;
            if (a->speed < mag) a->speed = mag;
        }
        return;
    }

    /* 位置模式减速时保留一个最低速度，用来把最后几个脉冲走完；
       点动模式则一直减到 0 才停 */
    floor_speed = (a->mode == MODE_POS) ? MIN_SPEED_U : 0;

    if (a->decel) {
        a->ramp_rem += a->ramp_num;
        while (a->ramp_rem >= a->ramp_den) {
            a->ramp_rem -= a->ramp_den;
            if (a->speed <= floor_speed) {
                a->speed    = floor_speed;
                a->ramp_rem = 0;
                break;
            }
            a->speed -= 1;
            if (a->speed < floor_speed) a->speed = floor_speed;
        }
        if (a->mode == MODE_JOG && a->speed <= 0) {
            axis_finish(a);             /* 匀减速完成 */
        }
    } else {
        a->ramp_rem += a->ramp_num;
        while (a->ramp_rem >= a->ramp_den) {
            a->ramp_rem -= a->ramp_den;
            if (a->speed >= a->cmd) {
                a->speed    = a->cmd;
                a->ramp_rem = 0;
                break;
            }
            a->speed += 1;
        }
        if (a->speed > a->cmd) a->speed = a->cmd;
    }
}

/* ------------------------- 内部：一拍（20us） ------------------------- */
static void axis_tick(Axis_t *a)
{
    /* 上一拍的脉冲结束，拉回空闲（脉冲宽度 = 20us） */
    if (a->pulse) {
        st_write(a->idx, 0);
        a->pulse = 0;
    }

    if (a->mode == MODE_IDLE) return;

    /* 1ms 到，调一次速度 */
    if (++a->ms_cnt >= (uint16_t)(TICK_HZ / 1000UL)) {
        a->ms_cnt = 0;
        axis_ramp(a);
        if (a->mode == MODE_IDLE) return;
    }

    if (a->speed <= 0) return;

    /* 速度累加器：累加到阈值就发一步，速度变慢变快都不会丢步 */
    a->acc += a->speed;
    if (a->acc >= PULSE_TH) {
        a->acc -= PULSE_TH;

        /* 最后一道防越界保险：正常由 axis_ramp 提前减速完毕，走不到这里。
           跟随模式下只停住、保持 MODE_TRACK，避免被踢出跟随导致后续指令失效 */
        if ((a->dir  && a->pos >= POS_MAX[a->idx]) ||
            (!a->dir && a->pos <= POS_MIN[a->idx])) {
            if (a->mode == MODE_TRACK) {
                a->speed   = 0;
                a->acc     = 0;
                a->decel   = 0;
                a->vel_cmd = 0;
            } else {
                axis_finish(a);
            }
            return;
        }

        st_write(a->idx, 1);
        a->pulse = 1;
        a->pos += (a->dir ? 1 : -1);
    }
}

/* ------------------------- 初始化 ------------------------- */
void Stepper_Init(void)
{
    uint8_t i;

    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    for (i = 0; i < 2; i++) {
        pin_out_init(pins[i].st_port,  pins[i].st_pin);
        pin_out_init(pins[i].dir_port, pins[i].dir_pin);
        pin_out_init(pins[i].en_port,  pins[i].en_pin);

        st_write(i, 0);                       /* STEP 空闲低 */
        dir_write(i, 0);
        pin_write(pins[i].en_port, pins[i].en_pin, 0);   /* 上电先不使能 */

        ax[i].pos      = 0;
        ax[i].target   = 0;
        ax[i].speed    = 0;
        ax[i].cmd      = 0;
        ax[i].vel_cmd  = 0;
        ax[i].ramp_num = 0;
        ax[i].ramp_den = 1;
        ax[i].ramp_rem = 0;
        ax[i].acc      = 0;
        ax[i].ms_cnt   = 0;
        ax[i].idx      = i;
        ax[i].dir      = 0;
        ax[i].mode     = MODE_IDLE;
        ax[i].decel    = 0;
        ax[i].pulse    = 0;
    }

    /* TIM2：计数 1MHz，中断 50kHz */
    TIM2->PSC = (uint16_t)((SystemCoreClock / TIMER_CNT_HZ) - 1U);
    TIM2->ARR = (uint16_t)((TIMER_CNT_HZ / TICK_HZ) - 1U);
    TIM2->EGR = TIM_EGR_UG;                 /* 立即装载 PSC/ARR */
    TIM2->CNT = 0;
    TIM2->SR  = 0;
    TIM2->DIER = TIM_DIER_UIE;
    TIM2->CR1  = TIM_CR1_CEN | TIM_CR1_URS;

    NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_EnableIRQ(TIM2_IRQn);
}

/* ------------------------- 使能 ------------------------- */
void Stepper_Enable(uint8_t axis, uint8_t en)
{
    if (axis > AXIS_B) return;
    pin_write(pins[axis].en_port, pins[axis].en_pin, en ? 1 : 0);
}

/* ------------------------- 内部：启动一段运动 ------------------------- */
static void axis_start(uint8_t idx, int32_t target, uint16_t speed_pps,
                       uint16_t accel_ms, uint8_t mode, uint8_t dir)
{
    Axis_t *a = &ax[idx];
    int32_t cmd;

    if (speed_pps < MIN_PPS)  speed_pps = MIN_PPS;
    if (speed_pps > MAX_PPS)  speed_pps = MAX_PPS;
    if (accel_ms  == 0)       accel_ms  = 1;

    cmd = (int32_t)speed_pps * SPEED_UNIT;

    __disable_irq();

    axis_finish(a);                 /* 先彻底停下，换向时不会错步 */

    a->target   = pos_clamp(idx, target);   /* 兜底：任何入口都写不进越界目标 */
    a->cmd      = cmd;
    a->ramp_num = cmd;              /* 加速到 cmd 用 accel_ms 毫秒 */
    a->ramp_den = (int32_t)accel_ms;
    a->ramp_rem = 0;
    a->acc      = 0;
    a->decel    = 0;
    a->dir      = dir;

    dir_write(idx, dir);            /* DIR 先建立，再发脉冲 */
    {
        volatile uint32_t n;        /* 约 5us 方向建立时间 */
        for (n = 0; n < 60u; n++) { }
    }

    a->speed  = MIN_SPEED_U;        /* 从最低速度起步，避免丢步 */
    if (a->speed > cmd) a->speed = cmd;
    a->mode   = mode;               /* 最后再置模式，中断从这里开始接管 */

    __enable_irq();
}

/* ------------------------- 1. 连续运行 + 匀减速停止 ------------------------- */
void Stepper_Jog(uint8_t axis, int8_t dir, uint16_t speed_pps, uint16_t accel_ms)
{
    if (axis > AXIS_B) return;
    axis_start(axis, ax[axis].pos, speed_pps, accel_ms, MODE_JOG, dir ? 1u : 0u);
}

void Stepper_Stop(uint8_t axis, uint16_t decel_ms)
{
    Axis_t *a;

    if (axis > AXIS_B) return;
    a = &ax[axis];

    __disable_irq();
    if (a->mode == MODE_IDLE) {
        __enable_irq();
        return;
    }
    if (decel_ms == 0 || a->speed <= MIN_SPEED_U) {
        axis_finish(a);
        __enable_irq();
        return;
    }
    a->ramp_num = a->speed;         /* 当前速度 / decel_ms = 每毫秒减多少 */
    a->ramp_den = (int32_t)decel_ms;
    a->ramp_rem = 0;
    a->mode     = MODE_JOG;         /* 减速到 0 就停，不再管原目标 */
    a->decel    = 1;
    __enable_irq();
}

void Stepper_StopNow(uint8_t axis)
{
    if (axis > AXIS_B) return;
    __disable_irq();
    axis_finish(&ax[axis]);
    __enable_irq();
}

/* ------------------------- 3. 连续速度跟随 ------------------------- */
void Stepper_SetVel(uint8_t axis, int32_t vel_pps)
{
    Axis_t *a;
    int32_t cmd;

    if (axis > AXIS_B) return;
    if (vel_pps >  (int32_t)MAX_PPS) vel_pps =  (int32_t)MAX_PPS;
    if (vel_pps < -(int32_t)MAX_PPS) vel_pps = -(int32_t)MAX_PPS;

    a = &ax[axis];
    __disable_irq();
    if (a->mode == MODE_TRACK) {
        cmd = (int32_t)vel_pps * SPEED_UNIT;
        /* 已顶到边界时禁止继续朝边界外给速度 */
        if (a->pos <= POS_MIN[axis] && cmd < 0) cmd = 0;
        if (a->pos >= POS_MAX[axis] && cmd > 0) cmd = 0;
        a->vel_cmd = cmd;           /* 只改目标，不碰 speed/acc/pulse */
    }
    __enable_irq();
}

void Stepper_TrackOn(uint8_t axis, uint8_t on)
{
    Axis_t *a;

    if (axis > AXIS_B) return;
    a = &ax[axis];

    __disable_irq();
    if (on) {
        axis_finish(a);              /* 从任意模式干净切入 */
        a->ramp_num = TRACK_ACC_U;   /* 供 axis_decel_steps() 用跟随减速度 */
        a->ramp_den = 1;
        a->vel_cmd  = 0;
        a->dir      = 1;
        dir_write(axis, 1);
        a->mode     = MODE_TRACK;
    } else {
        axis_finish(a);              /* 退出跟随即停机 */
    }
    __enable_irq();
}

/* ------------------------- 2. 位置控制 ------------------------- */
void Stepper_MoveTo(uint8_t axis, int32_t target_steps, uint16_t speed_pps, uint16_t accel_ms)
{
    uint8_t dir;

    if (axis > AXIS_B) return;

    target_steps = pos_clamp(axis, target_steps);   /* 先钳目标，再算方向，dir 才不会错 */
    if (target_steps == ax[axis].pos) return;

    dir = (target_steps > ax[axis].pos) ? 1u : 0u;
    axis_start(axis, target_steps, speed_pps, accel_ms, MODE_POS, dir);
}

void Stepper_MoveSteps(uint8_t axis, int32_t steps, uint16_t speed_pps, uint16_t accel_ms)
{
    if (axis > AXIS_B || steps == 0) return;
    Stepper_MoveTo(axis, ax[axis].pos + steps, speed_pps, accel_ms);
}

/* ------------------------- 状态查询 ------------------------- */
uint8_t Stepper_IsBusy(uint8_t axis)
{
    if (axis > AXIS_B) return 0;
    /* 跟随模式常驻，用速度而非 mode 判断，停机时返回 0 */
    if (ax[axis].mode == MODE_TRACK)
        return (ax[axis].speed > 0 || ax[axis].vel_cmd != 0) ? 1u : 0u;
    return (ax[axis].mode != MODE_IDLE) ? 1u : 0u;
}

int32_t Stepper_GetPos(uint8_t axis)
{
    if (axis > AXIS_B) return 0;
    return ax[axis].pos;
}

void Stepper_SetPos(uint8_t axis, int32_t steps)
{
    if (axis > AXIS_B) return;
    __disable_irq();
    ax[axis].pos = pos_clamp(axis, steps);
    __enable_irq();
}

uint16_t Stepper_GetSpeed(uint8_t axis)
{
    if (axis > AXIS_B) return 0;
    return (uint16_t)(ax[axis].speed / SPEED_UNIT);
}

int32_t Stepper_GetPosMin(uint8_t axis)
{
    if (axis > AXIS_B) return 0;
    return POS_MIN[axis];
}

int32_t Stepper_GetPosMax(uint8_t axis)
{
    if (axis > AXIS_B) return 0;
    return POS_MAX[axis];
}

/* ------------------------- TIM2 中断（50kHz） ------------------------- */
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR = (uint16_t)~TIM_SR_UIF;
        axis_tick(&ax[0]);
        axis_tick(&ax[1]);
    }
}
