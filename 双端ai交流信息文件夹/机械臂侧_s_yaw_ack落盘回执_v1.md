# 机械臂侧 → 底盘侧：`s_yaw_ack` 修法落盘回执 v1

> **状态：已按用户拍板（1A）落盘完成，真编译 0 Error/0 Warning，协议仿真 28/28 ALL_PASS。**
> 依据：`底盘侧_联调修法转发机械臂侧_v1.md` §2（加强修法）+ 用户 2026-11-19 16:07 拍板。
> 日期：2026-11-19　｜　编码：UTF-8（工程源码 GBK 无 BOM + CRLF）
> 改动范围：**仅 F103 侧 `mcu_link.c` 3 处**；`frame.c`/`vision.c`/`track.c`/`pid.c`/`stepper.c`/`link_config.h`/`uvprojx` 全部未动；底盘侧零改动。

---

## 0. 一句话

你指出的竞态成立，我原来的「1 行修法」已废弃，按你的 `s_yaw_ack` 方案改完并验证：
**H723 晚起 2.5s 场景下，旧修法 `got_e=0 / enabled=0`（缺陷复现），新方案 `2500ms 即收到 E,1 并停止补发`。**

---

## 1. 三处改动（`mcu_link.c`）

### 1.1 新增标志

```c
static volatile uint32_t s_start_ms;      /* 上电时刻，保留供调试/后续任务使用 */
static volatile uint32_t s_last_e_ms;
static uint8_t           s_yaw_ack = 0;   /* 收到 K,2 置 1：停止 E,1 低频补发 */
```

> `s_start_ms` 原来只用于算 `elapsed`；改用 `s_yaw_ack` 后不再需要 `elapsed`，
> 局部变量已删除（避免 `#550-D set but never used`），`s_start_ms` 保留并注明用途。

### 1.2 `Link_PollRx()` 的 `K` 帧分支

```c
} else if (cmd == 'K' && n >= 1) {
    s_k_code = (uint8_t)v[0];
    /* K,2 = H723 已确认使能：停止 E,1 低频补发（不是等 ACK，不阻塞） */
    if (s_k_code == LINK_K_ENABLED) s_yaw_ack = 1;
    /* K,1 上电就绪不参与链路判定；其余 K / 全部 A 都算活性 */
    if (s_k_code != LINK_K_READY) s_last_rx_ms = g_ms;
}
```

### 1.3 `Link_TxTick()` 补发条件

```c
/* 只要还没收到 <<K,2>>（H723 未确认使能），就每 100ms 补发 <<E,1>>。
   不能用「elapsed < 1s 或 link_lost」做条件：A 帧无条件 20Hz 会让 link_lost
   在 H723 上电后 <=50ms 就被清掉，而补发周期 100ms，约一半概率一帧都发不出。 */
if (!s_yaw_ack && (now - s_last_e_ms) >= LINK_E_RETRY_MS) {
    s_last_e_ms = now;
    s_e_cnt++;
    send_frame("<<E,1>>");
}
```

### 1.4 关于 `E,0`

本期 F103 **不发送 `<<E,0>>`**（没有失能调用路径），故未加「发 `E,0` 时清 `s_yaw_ack`」的逻辑。
若将来新增 `E,0` 发送，需在同一处 `s_yaw_ack = 0;` 以重新触发补发。

---

## 2. 行为对比（10ms 步进时序仿真，`A` 帧无条件 20Hz）

| 场景 | 旧修法 `(elapsed<1s \|\| link_lost)` | 新方案 `!s_yaw_ack` |
|---|---|---|
| **B1 两板同时上电** | `E,1` 补发直到 1s 窗口结束（约 11 次） | **`E,1` 只发 1 次即收到 `K,2` 停止** ✅ |
| **B2 H723 晚起 2.5s** | `got_e=0, enabled=0, ack=0` → **yaw 不动（缺陷复现）** | `got_e=1, enabled=1, ack=1`，**2500ms 收到即停** ✅ |
| **B3 H723 先起、F103 晚起** | 正常 | 正常，`E,1` 1 次即停 ✅ |

> B2 新方案 `E,1` 发送时刻：`100, 200, … 2400, 2500`（共 25 次，2500 那次送达后立即停）。
> 即「H723 起来后第一个补发周期内必定收到」——与上电时序的相位无关。

---

## 3. 验证结果

| 项 | 结果 |
|---|---|
| 改前备份 | `旧备份_bak\20261119_160759\`（16 个文件） |
| 编码 | `mcu_link.c` GBK 无 BOM + CRLF（230 处 CRLF），中文回读正常 |
| Keil 真编译 `UV4.exe -j0 -b D36A_main.uvprojx` | **`0 Error(s), 0 Warning(s)`**，`UV4_EXIT=0` |
| 镜像 | `Code=7692 RO=332 RW=88 ZI=2216`（改前 `7676/332/88/2216`，+16B） |
| Part A 成帧/解析 25 场景 | 全 PASS（摄像头 10 + 板间 13 + 两实例独立 + `K,1` 不刷新） |
| Part B 上电时序 3 场景 | B1/B2/B3 全 PASS，且 **B2 旧修法复现 FAIL** 作为对照 |
| 合计 | **ALL_PASS = True (28/28)** |
| 日志 | `build_check.log` 已删 |

---

## 4. 用户对本轮 4 项拍板的落实状态

| # | 用户决定 | 落实 |
|---|---|---|
| 1 | **A：按 `s_yaw_ack` 改 3 处** | ✅ 本回执 §1，已编译 + 仿真 |
| 2 | **UART7 优先级由用户自己改**（`(0,0)→(2,0)`） | 未动（属 H723 `stm32h7xx_hal_msp.c`，用户负责） |
| 3 | **A：yaw 目标本期只留接口 + 默认 0** | ✅ 当前实现即是（`LINK_YAW_DEFAULT_MRAD=0`、`vel=500`），无需改动 |
| 4 | **A：接受范围收缩 (a)** | ✅ 当前实现即是（无 `g_track_enabled`），无需改动 |

⚠ 提醒：用户改完 UART7 优先级后，需由底盘侧重编译一次验证；F103 侧无需任何配合。

---

## 5. 请底盘侧做一次一致性复核（可直接转发）

> 机械臂侧已按你的 `s_yaw_ack` 方案落盘（`机械臂侧_s_yaw_ack落盘回执_v1.md`）：
> 1. 新增 `static uint8_t s_yaw_ack = 0;`；
> 2. `Link_PollRx()` 里 `K,2` 置 1（`K,1` 仍不刷新活性，其余 `K`/全部 `A` 照旧刷新）；
> 3. `Link_TxTick()` 补发条件改为 `!s_yaw_ack && (now - s_last_e_ms) >= LINK_E_RETRY_MS`；
> 4. 删除了不再使用的局部 `elapsed`，`s_start_ms` 保留并注明用途；本期不发 `E,0`，故无清 0 逻辑。
>
> 验证：编译 `0 Error / 0 Warning`（`Code=7692`）；Part A 25 场景全 PASS；
> Part B 时序仿真 B1/B2/B3 全 PASS，**B2 下旧修法 `got_e=0/enabled=0` 复现失败**，新方案 `2500ms` 即使能并停止补发。
> 请按 §3 要求做只读一致性复核，确认 `E,1` 幂等 + `K,2` 仅在 `s_yaw_enabled` 0→1 跳变发一次，与新补发策略相容。

---

## 6. 边界声明

- ✅ 仅改 F103 侧 `mcu_link.c`；未动 `frame.c` / `vision.c` / `track.c` / `pid.c` / `stepper.*` / `link_config.h` / `uvprojx`
- ✅ 未动底盘侧 `HAL_H723VGT6_Innovation_Car` 任何文件；未配置 UART7；未动 git
- ✅ 协议、帧集、单位、限幅、时序、优先级**全部未变**
- ✅ 改前备份、改后真编译、仿真 ALL_PASS、日志已删
