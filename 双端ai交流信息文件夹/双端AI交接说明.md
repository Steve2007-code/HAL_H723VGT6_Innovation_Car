# 双端 AI 交接说明 —— F103C8T6 直角坐标机械臂「视觉跟随 + 双机通信」工程

> **这份文档是写给另一端 AI（或接手的合作方）的交接材料**，目的是让对方在不依赖本会话上下文的情况下，
> 能准确理解「已经做了什么、验证到什么程度、接下来准备做什么、有哪些坑不能踩」。
>
> - 整理日期：2026-11-18
> - 工程归属：2027Innovation_Car / 直角坐标机械臂（Cartesian Robot）
> - 文档本身为 **UTF-8**；**工程源码是 GBK 无 BOM + CRLF**，两者不要混淆（详见第 8 节）

---

## 0. 一句话现状

STM32F103C8T6 上，**A 轴（水平轴）的摄像头视觉 PID 跟随系统已经完整落盘并通过编译与协议仿真验证**；
**B 轴（上下轴）只使能、不参与 PID**。下一步准备新增 **USART2 与另一块单片机双向通信（收命令 + 回状态）**，
该部分已完成方案确认与资源盘点，**代码尚未落盘**，等待用户明确授权后实现。

---

## 1. 工程基本信息

| 项目 | 内容 |
|---|---|
| MCU | STM32F103C8T6，LQFP48 |
| 开发方式 | **CMSIS + 直接寄存器裸机**（不是 HAL、不是 StdPeriph、没有 `.ioc`） |
| 判据 | 工程无 `stm32f1xx_hal*`、无 `stm32f10x_gpio.c/tim.c`；代码直接写 `RCC->APB2ENR`、`GPIOA->CRH`、`TIM2->PSC`、`USART1->BRR` |
| uvprojx 组件 | 仅 `CMSIS:CORE` + `Device:Startup` |
| IDE / 编译器 | Keil µVision / ARMCC V5.06 update 7；批编译 `UV4.exe -j0 -b` |
| 系统时钟 | `SYSCLK_FREQ_72MHz = 72000000`；`PCLK2 = 72MHz`（USART1）、`PCLK1 = 36MHz`（USART2/3） |
| 源码编码 | **GBK 无 BOM + CRLF**（关键约束，见第 8 节） |
| 当前工程路径 | `C:\Users\13687\Desktop\MCU\Project\STM32H723\2027Innovation_Car\CMSIS _F103C8T6_Cartesian_Robot` |
| ⚠ 路径陷阱 | 目录名 `CMSIS ` 后面**有一个空格**，`_F103C8T6` 前是下划线；复制路径时极易漏掉 |
| 备份目录 | 工程内 `旧备份_bak\20261118_194909\`、`旧备份_bak\20261118_204939\` |
| 早期工程 | `C:\Users\13687\Desktop\D36A`（限位功能最初在此实现并验证，后复制到当前工程） |

> 历史：工程文件夹曾叫 `HLA_F103C8T6_Cartesian_Robot`，后被用户改名为现在的 `CMSIS _F103C8T6_Cartesian_Robot`。
> 若查资料看到旧名，指同一个工程。`uvprojx` 内引用均为 `.\xxx.c` 相对路径，改名不影响编译。

---

## 2. 当前文件清单与职责

| 文件 | 大小(B) | 状态 | 职责 |
|---|---:|---|---|
| `D36A_main.uvprojx` | 18145 | 已改 | Keil 工程，Source Group 1 加了 `vision.c/pid.c/track.c`，New Group 加了 4 个 `.h` |
| `main.c` | 2070 | **重写** | 全非阻塞主循环：初始化 → `while(1)` 里 10ms 调度 + LED 指示 |
| `stepper.c` | 18584 | 已改 | 步进驱动核心：软件限位、Jog 限位减速、`MODE_TRACK` 连续速度跟随 |
| `stepper.h` | 2328 | 已改 | 对外 API + A/B 限位宏 + `Stepper_SetVel/TrackOn` 声明 |
| `vision.c` | 3898 | **新增** | USART1 RXNE 中断 + 环形缓冲 + `<<X,dx,Y,dy>>` 状态机解析 |
| `vision.h` | 948 | 新增 | `VisionFrame_t`、`Vision_Init/Poll/AgeMs` |
| `pid.c` / `pid.h` | 1198 / 806 | 新增 | 速度式 PID：死区、积分抗饱和、输出限幅 |
| `track.c` / `track.h` | 2452 / 749 | 新增 | TIM3 10ms 时基、dx 低通、PID 调度、丢失目标处理 |
| `track_config.h` | 1574 | 新增 | **所有可标定参数集中处**（含 2 个待现场标定值） |
| `main.c.bak*` / `stepper.c.bak` | — | 历史 | 用户早期版本备份，非当前编译输入 |
| `RTE/Device/STM32F103C8/system_stm32f10x.c` | — | 只读 | 确认时钟宏与分频 |

---

## 3. 已完成的工作（按阶段）

### 3.1 阶段 A：`Stepper_MoveTo()` 非阻塞机制诊断（只诊断，未改）

- 用户疑惑「函数里没写到位后去下一个位置」。结论：**`Stepper_MoveTo()` 是「下一段命令」函数**，
  只算方向、设 `target`、置 `MODE_POS` 立即返回；**到位判定在 TIM2 中断里的 `axis_ramp()` / `axis_finish()`**。
- 用户最初第二条指令目标仍是 `17000`，被 `if (target_steps == ax[axis].pos) return;` 直接吞掉；改成 `8000` 后正常。
- `wait_axis()`（即 `while (Stepper_IsBusy(axis)) {}`）才是**同步屏障**；`delay_ms()` 只是到位后停留，
  **去掉后仍可往复**，带负载时建议保留 20~100ms 抑制机械抖动。
- 另发现 `speed_pps=6000` 会被 `MAX_PPS=4000` 静默钳位，用户已改 4000。
- 「多条位置自动排队」曾提出方案 B（驱动层环形点位队列），**用户未授权，未实现**；最终落盘的是后面的视觉速度跟随，不是点位队列。

### 3.2 阶段 B：A/B 软件限位 + Jog 撞限位减速（已落盘 + 仿真验证）

- 限位宏：`AXIS_A = 0~5000`、`AXIS_B = 0~20000`（单位：微步）。
- **多层钳制**：`Stepper_MoveTo()` → `axis_start()` → `Stepper_SetPos()` → `axis_tick()` 发脉冲前边界兜底。

```c
static int32_t pos_clamp(uint8_t idx, int32_t p)
{
    if (p < POS_MIN[idx]) return POS_MIN[idx];
    if (p > POS_MAX[idx]) return POS_MAX[idx];
    return p;
}
```

- 用户明确要求「**Jog 撞限位不能硬停，也要减速停止**」，因此在 `axis_ramp()` 增加 `MODE_JOG` 预判，
  复用原有加速斜坡提前减速：

```c
} else if (a->mode == MODE_JOG && !a->decel) {
    int32_t lim_rem;
    lim_rem = a->dir ? (POS_MAX[a->idx] - a->pos)
                     : (a->pos - POS_MIN[a->idx]);
    if (lim_rem <= axis_decel_steps(a)) {
        a->decel    = 1;
        a->ramp_rem = 0;
    }
}
```

- 数值仿真覆盖「B 正向到 20000 / A 正向到 5000 / B 反向到 0」，均未触发硬停兜底。

### 3.3 阶段 C：摄像头视觉 PID 跟随系统（重点，已完整落盘）

#### 数据流

```
摄像头串口 <<X,dx,Y,dy>>
   -> USART1 RXNE 中断只收字节进环形缓冲（不解析）
   -> 主循环 Vision_Poll() 非阻塞状态机解析出帧
   -> TIM3 每 10ms 置 g_tick_10ms 标志
   -> Track_Update(): 超时/丢失判断 + dx 低通 + PID -> 有符号 pps
   -> Stepper_SetVel(AXIS_A, TRACK_SIGN * v)
   -> TIM2 50kHz 中断: 速度斜坡(平滑换向/限位预减速) + 速度累加器发脉冲
```

#### 帧协议（用户给定）

| 项 | 内容 |
|---|---|
| 包头 / 包尾 | `<<` / `>>` |
| 格式 | `<<X,x像素差值,Y,y像素差值>>` |
| 识别到目标 | `<<X,-24,Y,35>>` |
| 丢失目标 | `<<X,9999,Y,9999>>`（`LOST_SENTINEL = 9999`） |
| 硬件 | USART1 PA9/PA10，**115200 8N1** |

#### 4 个架构决策（用户通过选择题明确确认）

| 决策点 | 选定值 |
|---|---|
| 串口硬件 | `USART1 PA9/PA10 @ 115200` |
| PID 输出 | **速度式 pps**（不是位置增量） |
| PID 周期 | **固定 10ms 定时**（TIM3 标志位，非阻塞） |
| 丢失目标策略 | **平滑减速停住保持**（不是急停、不是保持上一速度） |

#### 关键设计理由（务必保留，改动前先读懂）

- **为什么不能直接用 `MoveTo` / `Jog` 跟随像素流**：这两者都走 `axis_start()`，
  其中 `axis_finish()` 会清 `speed/acc`；每 10ms 调一次会导致「硬停 → 重启」反复，产生抖动与丢步。
  因此专门新增 `MODE_TRACK` + `Stepper_SetVel()`，**只改 `vel_cmd`，不碰 `speed/acc/pulse`**。
- `MODE_TRACK` 三个关键行为：
  1. 固定加速度 `TRACK_ACC_U` 朝 `|vel_cmd|` 靠拢；
  2. **速度指令反号时先减到 0 再翻 DIR**，实现过零软换向；
  3. 到软件限位前按 `axis_decel_steps()` 提前减速并清 `vel_cmd`，避免顶边界反复启停。

```c
} else if (a->mode == MODE_TRACK) {
    int32_t want_dir, mag, lim_rem;
    if (a->decel) { /* 减到 0 后清 vel_cmd 并退出 decel */ ... }
    lim_rem = a->dir ? (POS_MAX[a->idx] - a->pos)
                     : (a->pos - POS_MIN[a->idx]);
    if (a->speed > 0 && lim_rem <= axis_decel_steps(a)) {
        a->decel = 1; a->vel_cmd = 0; ... return;
    }
    want_dir = (a->vel_cmd >= 0) ? 1 : 0;
    if (a->vel_cmd != 0 && want_dir != (int32_t)a->dir) {
        if (a->speed <= TRACK_ACC_U) { a->speed = 0; a->dir = (uint8_t)want_dir; dir_write(a->idx, a->dir); }
        else                        { a->speed -= TRACK_ACC_U; }
        return;
    }
    mag = a->dir ? a->vel_cmd : -a->vel_cmd;
    if (mag < 0) mag = 0;
    if (mag > MAX_SPEED_U) mag = MAX_SPEED_U;
    if      (a->speed < mag) { a->speed += TRACK_ACC_U; if (a->speed > mag) a->speed = mag; }
    else if (a->speed > mag) { a->speed -= TRACK_ACC_U; if (a->speed < mag) a->speed = mag; }
    return;
}
```

- `Stepper_SetVel()` 在临界区写 `vel_cmd`，并在边界处把「继续往外」的命令直接钳成 0：

```c
void Stepper_SetVel(uint8_t axis, int32_t vel_pps)
{
    ...
    __disable_irq();
    if (a->mode == MODE_TRACK) {
        cmd = (int32_t)vel_pps * SPEED_UNIT;
        if (a->pos <= POS_MIN[axis] && cmd < 0) cmd = 0;
        if (a->pos >= POS_MAX[axis] && cmd > 0) cmd = 0;
        a->vel_cmd = cmd;
    }
    __enable_irq();
}
```

- 丢失目标处理（`track.c`）：`PID_Reset` + 清低通状态 + `Stepper_SetVel(TRACK_AXIS, 0)`，
  由 `MODE_TRACK` 的固定减速度自然平滑停下。

```c
if (!s_valid || Vision_AgeMs() > TRACK_TIMEOUT_MS) {
    PID_Reset(&s_pid);
    s_dx_f = 0.0f;
    g_track_valid = 0;
    Stepper_SetVel(TRACK_AXIS, 0);
    return;
}
s_dx_f += TRACK_LPF_ALPHA * ((float)s_dx - s_dx_f);
v = PID_Calc(&s_pid, (int16_t)s_dx_f, (float)TRACK_PERIOD_MS / 1000.0f);
Stepper_SetVel(TRACK_AXIS, (int32_t)TRACK_SIGN * v);
```

- 中断优先级规划（**新增 USART2 时必须遵守，避免打断 50kHz 脉冲**）：
  `TIM2 = 1`（最高，脉冲）> `TIM3 = 2`（10ms PID）> `USART1 = 3`（摄像头字节）> 规划 `USART2 = 4`。

#### 参数总表（`track_config.h`）

| 宏 | 值 | 说明 |
|---|---|---|
| `TRACK_UART_BAUD` | 115200 | USART1 波特率 |
| `TRACK_TIMEOUT_MS` | 250 | 超时无有效帧判丢失 |
| `TRACK_PERIOD_MS` | 10 | PID 周期 |
| **`TRACK_SIGN`** | **`(-1)`** | **待现场标定 1**：目标放画面右侧观察 A 轴方向，跑反了改 `+1` |
| **`TRACK_KP`** | **`3.0f`** | **待现场标定 2**：建议先设 1.0，按 `Kp ≈ v/100` 估算 |
| `TRACK_KI` / `TRACK_KD` | `0.0f` / `0.0f` | 先纯 P 调通 |
| `TRACK_DEAD_PX` | 5 | 死区，抑制小幅抖动 |
| `TRACK_OUT_MAX_PPS` | 1200 | 输出限幅，压小数控风险 |
| `TRACK_INTEG_MAX` | 200.0f | 积分抗饱和 |
| `TRACK_AXIS` | `AXIS_A` | 只跟随 A 轴 |
| `TRACK_ACC_PPS_PER_MS` | 8 | 速度斜坡每 ms 变化量 |
| `TRACK_LPF_ALPHA` | 0.5f | dx 一阶低通（1.0 = 不滤波） |
| `LOST_SENTINEL` | 9999 | 丢失目标哨兵值 |

#### 主循环（全非阻塞，`main.c`）

```c
Vision_Init(TRACK_UART_BAUD);
Track_Timer_Init();
Track_Init();
Stepper_Enable(AXIS_A, 1);
Stepper_Enable(AXIS_B, 1);          /* B 轴只使能，不参与 PID */
delay_ms(300);
Stepper_TrackOn(TRACK_AXIS, 1);     /* A 轴进入连续速度跟随模式 */

while (1) {
    if (g_tick_10ms) { g_tick_10ms = 0; Track_Update(); }
    if (g_track_valid) led_on(); else led_off();
    /* 后续双机串口命令解析 / 状态上报挂在这里，禁止忙等 */
}
```

---

## 4. 编译与验证结果

| 验证项 | 结果 |
|---|---|
| Keil 批编译 | **0 Error(s), 0 Warning(s)** |
| 镜像大小 | `Code=6236  RO=332  RW=48  ZI=1904` |
| 帧解析器离线仿真 | 10 个场景全部 `ALL_PASS=True`：正常帧、丢失帧、粘包、半包、噪声、残缺包尾、超长、重复包头 `<<<`、连续 100 帧、空帧恢复 |
| 限位数值仿真 | B→20000 / A→5000 / B→0 均未硬停 |
| USART1 波特率 | `BRR = 72000000/115200 = 625`，误差 **0%** |

> 本次交接前重新核对：11 个源/头文件字节数与上述编译验证时**完全一致**，说明此后未改动，编译结论仍有效。
> 用户已用串口助手实际发过几帧模拟数据（未反馈异常）。

---

## 5. 串口资源盘点（LQFP48）

| 串口 | 默认脚 | 状态 |
|---|---|---|
| USART1 | PA9/PA10 | **摄像头占用**（视觉帧输入），APB2 / PCLK2=72MHz |
| **USART2** | **PA2/PA3** | **空闲，推荐用于新增板间通信**，APB1 / PCLK1=36MHz |
| USART3 | PB10/PB11 | PB10 已被 **B 轴 EN** 占用；部分重映射 PC10/PC11、全重映射 PD8/PD9 在 C8T6 48 脚封装**未引出**，基本不可用 |

---

## 6. 准备做的工作：USART2 双机通信（**方案已确认，代码未落盘**）

> 触发背景：用户「现在这个 32 还有空闲串口吗，我要开一个新串口和其他单片机通信」。

### 6.1 已确认决策（选择题结果）

| 决策点 | 选定值 |
|---|---|
| 通信方向 | **双向：收命令 + 回状态** |
| 速率帧格式 | **115200 8N1** |
| 电平 | **3.3V TTL，直接对接**（注意共地；不是 RS232/RS485） |

### 6.2 计划改动

| 文件 | 动作 |
|---|---|
| `mcu_link.h` / `mcu_link.c` | **新增**：USART2 初始化、RXNE 接收环形缓冲、TXE 中断发送环形缓冲、`Link_Update()` 非阻塞命令解析与状态上报 |
| `track.c` / `track.h` | 小改：增加 `g_track_enabled` 接管钩子，处理「手动速度命令」与「视觉 PID」对 A 轴的**互斥** |
| `main.c` | 小改：`while(1)` 里挂 `Link_Update()`（禁止忙等） |
| `D36A_main.uvprojx` | 小改：把 `mcu_link.c` 加入 Source Group、`mcu_link.h` 加入头文件组 |

### 6.3 协议骨架（规划，未定稿）

- 状态帧：`<<S,<posA>,<posB>,<velA>,<valid>>`
- **已知缺口**：当前 `Stepper_GetSpeed()` 只返回**幅值、不带方向**；若状态帧 `velA` 需要带符号，
  需补一个 `Stepper_GetVelSigned()` 接口（尚未实现）。

### 6.4 ⚠ 关键坑点（改动时务必逐条核对）

1. **USART2 波特率分频绝不能照抄 `vision.c`**。
   `vision.c` 用 `USART1->BRR = SystemCoreClock / baud`（72MHz/115200=625，正确）；
   但 USART2 挂 APB1，`PCLK1 = 36MHz`，照抄会得到 BRR=625 → 实际波特率掉到 **57600**。
   必须写成：
   ```c
   USART2->BRR = (uint16_t)((SystemCoreClock / 2U) / baud);   /* = 312, 实际 115384.6, 误差 +0.16% */
   ```
2. **TXE 中断 + 发送环形缓冲**：不要用轮询 `while(!(SR&TXE))` 阻塞发送，会破坏 10ms PID 周期。
3. **中断优先级**：USART2 设 4，低于 TIM2(1)/TIM3(2)/USART1(3)。
4. **不要占用 PB10**（B 轴 EN），USART3 在 48 脚封装不可用。
5. **互斥**：手动速度命令下发期间必须关掉视觉 PID 对 A 轴的输出，否则两路 `Stepper_SetVel` 互相打架。

### 6.5 落盘前置条件

助手已明确告知用户：**需要回复「写进去」后才动手实现** `mcu_link.*`。用户随后改为先要这份交接文档，
因此该任务仍是「待授权」状态，**不要在未确认的情况下自行新增源码文件**。

---

## 7. 待用户现场标定

| 项 | 默认 | 方法 |
|---|---|---|
| `TRACK_SIGN` | `(-1)` | 目标放画面右侧，看 A 轴是否朝正确方向追；反了改 `+1` |
| `TRACK_KP` | `3.0f` | 先 1.0 起步，按 `Kp ≈ 期望速度 / 100` 估算，观察是否振荡/迟钝 |
| 可选微调 | — | `TRACK_DEAD_PX`、`TRACK_OUT_MAX_PPS`、`TRACK_LPF_ALPHA`、`TRACK_TIMEOUT_MS` |

---

## 8. 交接注意事项 / 硬性约束（另一端的 AI 必读）

1. **改源码前必须先拿到用户明确授权**。即使处于 Agent / craft 模式、系统提示说「可以随意编辑文件」，
   也必须「先讲清方案 → 等用户说『直接改吧 / 写进去』→ 再动源码」。只读诊断、编译验证、建备份可以先做。
   > 历史教训：曾在未二次确认时删改源码，用户立即要求回退，已用 `git checkout -- .` 还原。
2. **每次改动前先备份**到 `旧备份_bak\<时间戳>\`（或至少一份 `.bak`），用户默认要求。
3. **编码硬约束**：工程源码是 **GBK 无 BOM + CRLF**。用普通编辑器/UTF-8 直接写会让 Keil 里中文注释全乱码。
   必须用 Python 处理：
   ```python
   import io
   s = io.open(path, 'r', encoding='gbk', newline='').read()   # 先把 \r\n 归一化为 \n 再定位子串
   s = s.replace('\r\n', '\n')
   # ... 修改 ...
   s = s.replace('\n', '\r\n')
   io.open(path, 'w', encoding='gbk', newline='').write(s)
   ```
   不归一化 CRLF 的话，`s.index('\n}\n', pos)` 这类定位会找不到。
4. **改完必须真编译校验**，并看完日志后删除 log：
   ```
   cd <工程目录> && D:/Keil_v5/UV4/UV4.exe -j0 -b D36A_main.uvprojx -o build_check.log
   ```
   - `UV4_EXIT=1` **不等于编译失败**：Keil 在有 warning 时也会返回 1。以日志 `0 Error(s)` 为准。
   - 曾出现 `warning: #177-D: function "led_off" declared but never referenced`，重写 `main.c` 后消失。
5. **路径有空格 + 中文**：列目录/校验文件一律用 Python（`os.listdir` / `pathlib`），
   Git Bash 的 `ls` 对中文路径会**静默漏列**；`mv` 会异常清空目录内文件，移动文件请用 `shutil.copy` / `os.replace`。
6. **不要删除 `.workbuddy` 文件夹**（项目相关数据，非缓存）。
7. 本交接文档是 UTF-8，**不要把它当源码塞进 Keil 工程**。

---

## 9. 待办清单

- [x] A/B 软件限位（0~5000 / 0~20000）
- [x] Jog 撞限位减速停止
- [x] 视觉帧解析（USART1 + 状态机 + 容错）
- [x] 速度式 PID（死区/抗饱和/限幅）
- [x] `MODE_TRACK` 连续速度跟随（软换向、限位预减速）
- [x] 丢失目标平滑停住保持
- [x] 编译 0 Error / 0 Warning；协议仿真 ALL_PASS
- [ ] **USART2 双机双向通信 `mcu_link.h/c`（方案已定，代码未落盘，待用户「写进去」授权）**
- [ ] `g_track_enabled` 手动/视觉互斥钩子（随上一项一起做）
- [ ] 现场标定 `TRACK_SIGN` / `TRACK_KP`
- [ ] （可选）`Stepper_GetVelSigned()`，让状态帧速度带方向
- [ ] （可选，未授权）驱动层点位队列，让多条 `MoveTo` 自动排队
- [ ] （可选）单独的协议 `.md` 说明书给视觉/上位机/另一块单片机方

---

## 10. 另一端 AI 快速上手指引

1. 先按第 1 节确认工程路径与「CMSIS 直接寄存器裸机」定位，别去找 `.ioc` 或 HAL。
2. 通读 `track_config.h` → `vision.h/c` → `pid.h/c` → `track.h/c` → `stepper.h` 的 API 段 → `main.c`，
   照第 3.3 节的数据流把链路串起来。
3. 若要继续第 6 节：**先向用户确认授权**，再按 6.4 的 5 条坑点逐条实现，改前备份、改后编译。
4. 任何源码写入遵守第 8 节的 GBK+CRLF 编码流程。
5. 现场联调优先动 `track_config.h` 两个标定值，**不要一上来改 `stepper.c` 的 `MODE_TRACK` 斜坡逻辑**。
