#!/usr/bin/env bash
# ============================================================================
#  start.sh —— 一键启动（Orange Pi AIpro / 通用 aarch64 Linux 板子）
#
#  和 run_task1.sh 的区别：
#    run_task1.sh 是「参数模板」，用它之前你得自己敲一长串前置：
#        conda activate base
#        source /usr/local/Ascend/ascend-toolkit/set_env.sh
#        BACKEND=om LINK=print SHOW=1 bash run_task1.sh
#    本脚本把这些前置全部包掉，**只是薄封装，不改 run_task1.sh 一行**：
#      1. 自动准备 conda base + CANN 环境（找不到就跳过，按 CPU 路线跑）
#      2. 自动选后端：best.om + pyACL 可用 -> Ascend NPU；否则 onnx(best.onnx)
#      3. 自动选链路：存在 /dev/ttyUSB* -> serial 真串口；否则 print 只打印
#      4. 自动选显示：**固定** DISPLAY=:0 + SHOW=1（香橙派本地 HDMI）
#      5. run_task1.sh 的所有环境变量照样能用，同名覆盖即可
#
#  【最简用法】把本文件丢进板子上那个目录，然后：
#      bash start.sh
#
#  【常用覆盖】
#      SHOW=0 bash start.sh                          # 比赛：不开窗，控制延时最低
#      PI_DISPLAY=localhost:10.0 bash start.sh      # 临时把画面传回 Windows
#      LINK=serial COM=/dev/ttyUSB0 bash start.sh   # 强制真串口
#      BACKEND=onnx bash start.sh                   # 强制 CPU（无 NPU 的板子）
#      CAM=0 TASK=532 bash start.sh                 # 手动指定相机号 / 任务码
#      EXP_FIXED=200 bash start.sh                  # 换个固定曝光值（默认 157）
#      ISP_GUARD_HZ=10 bash start.sh               # 提高曝光守护频率（默认 5）
#      ISP_GUARD_HZ=0 bash start.sh                # 关守护，用于对比测试
#      MODEL=best_320.onnx BACKEND=onnx bash start.sh
#
#  【本地 HDMI】默认即本地 HDMI，不再受 SSH 里的 DISPLAY 影响。
#    没接 HDMI 显示器又想后台跑时，给 SHOW=0。
#
#  编码：UTF-8 + LF。别用 Windows 记事本另存，会变 CRLF 导致
#  `\r: command not found`。
# ============================================================================

set -u
cd "$(dirname "$0")" || exit 1

# ---- 1. 环境准备 ----------------------------------------------------------
# 只在「看起来要用 NPU」时才折腾 conda。显式 BACKEND=onnx 时完全不碰 conda，
# 否则会把系统 python3 里装好的 onnxruntime 弄丢（CPU 路线在 conda deactivate
# 之后，base 里是没有 onnxruntime 的）。
want_npu=0
[ -z "${BACKEND:-}" ] && [ -f best.om ] && want_npu=1
[ "${BACKEND:-}" = "om" ] && want_npu=1

if [ "$want_npu" = "1" ] && ! python3 -c 'import acl' >/dev/null 2>&1; then
    if command -v conda >/dev/null 2>&1; then
        conda_base="$(conda info --base 2>/dev/null)"
        conda_sh="$conda_base/etc/profile.d/conda.sh"
        if [ -f "$conda_sh" ]; then
            . "$conda_sh"
            conda activate base >/dev/null 2>&1 || true
        fi
    fi
    # CANN 运行时（LD_LIBRARY_PATH / PYTHONPATH 等）。source 的输出吞掉，
    # 但环境变量会留在当前 shell 里。
    for envf in /usr/local/Ascend/ascend-toolkit/set_env.sh \
                /usr/local/Ascend/ascend-toolkit/latest/set_env.sh; do
        if [ -f "$envf" ]; then
            . "$envf" >/dev/null 2>&1 || true
            break
        fi
    done
fi

# ---- 2. 后端自动选择 ------------------------------------------------------
if [ -z "${BACKEND:-}" ]; then
    if [ -f best.om ] && python3 -c 'import acl' >/dev/null 2>&1; then
        BACKEND=om
    else
        BACKEND=onnx
    fi
fi
export BACKEND

# 模型体检：别让 Detector 抛一堆看不懂的错
case "$BACKEND" in
    om)   need=best.om ;;
    onnx) need=best.onnx ;;
    *)    need='' ;;
esac
if [ -n "$need" ] && [ ! -f "$need" ]; then
    echo "[错误] BACKEND=$BACKEND 需要 $need，但当前目录没有。"
    echo "       先把模型文件一起传进来；或改 BACKEND（onnx / cvdnn / none）。"
    echo "       注意 best.om 是在板上用 bash atc_convert.sh 生成的，PC 包里没有。"
    exit 1
fi

# ---- 3. 链路自动选择：插了 CH340 就真发串口，没插就只打印 ----------------
if [ -z "${LINK:-}" ]; then
    usb="$(ls /dev/ttyUSB* 2>/dev/null | head -n 1)"
    if [ -n "$usb" ]; then
        LINK=serial
        if [ -z "${COM:-}" ]; then
            COM="$usb"
            export COM
        fi
    else
        LINK=print
    fi
fi
export LINK

# ---- 4. 显示：一律香橙派本地 HDMI（2026-11-24 用户明确要求）---------------
# 旧逻辑只在 DISPLAY 为空时补 :0，MobaXterm SSH 里带进来的 localhost:10.0
# 会被原样继承，窗口又跑回 Windows 去了。现在无条件钉死 :0。
# 想临时 X11 转发： PI_DISPLAY=localhost:10.0 bash start.sh
# 比赛要无窗口后台： SHOW=0 bash start.sh
export DISPLAY="${PI_DISPLAY:-:0}"
if [ -z "${SHOW:-}" ]; then
    SHOW=1
fi
export SHOW

# ---- 5. 启动横幅：一眼确认当前配置 ----------------------------------------
echo '=============================================================='
echo '  一键启动 · 任务一抓取'
echo "  BACKEND = $BACKEND     MODEL = ${MODEL:-<run_task1.sh 自动选>}"
echo "  LINK    = $LINK        COM   = ${COM:-<默认 /dev/ttyUSB0>}"
echo "  DISPLAY = ${DISPLAY:-<无>}      SHOW  = $SHOW"
echo '  覆盖示例： SHOW=0 bash start.sh   /   LINK=serial bash start.sh'
if [ "$LINK" = 'print' ]; then
    echo '  [提示] 没发现 /dev/ttyUSB*，报文只打印不发送；接上 CH340 后自动切 serial'
fi
if [ "$BACKEND" = 'onnx' ] && [ -f best.om ]; then
    echo '  [提示] best.om 在但 pyACL 不可用，已回退 CPU 路线（约 5 FPS）'
fi
echo '=============================================================='

exec bash run_task1.sh
