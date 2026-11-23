#!/usr/bin/env bash
# ============================================================================
#  start_desktop.sh —— 桌面双击图标的入口（由 Task1_Start.desktop 调用）
#
#  为什么要多这一层：
#    双击图标启动的进程**不读 .bashrc**，SSH 里那个 (base) 环境不会自动带上，
#    conda / CANN 的 LD_LIBRARY_PATH、PYTHONPATH 全都没有。
#    直接跑 start.sh 会 `import acl` 失败 -> 悄悄回退 CPU 路线（约 5 FPS）。
#    本脚本主动把 conda base + CANN 环境补上，再交给 start.sh。
#
#  2026-11-24 板上验证过的四个要点（别改回去）：
#    1) **无条件** source conda.sh + activate base + source CANN set_env.sh。
#       早期版本只在「import acl 失败」时才补，桌面那种干净环境里
#       配合 set -u 会直接崩，日志都看不到。所以这里连 set -u 都不加。
#    2) 日志 tee 到 /tmp/icon_start.log —— Terminal=false 时没有终端窗口，
#       只能靠这个文件查启动过程。
#    3) DISPLAY 一律 :0（本地 HDMI），**不**继承 SSH 里的 localhost:10.0。
#    4) **单例锁 flock**（2026-11-24 第九轮补）：Terminal=false 看不到终端，
#       用户容易重复双击，或 SSH 里的旧实例没退。多个 live_detect_pi.py
#       并存会各自守护线程写 v4l2 控件、各自重开相机、甚至跑着不同版本
#       代码，互相干扰 → 画面反复自动变亮/振荡。拿不到锁就安静退出。
#       注意本锁只覆盖「桌面图标」路径；终端直接 bash start.sh 不经过
#       这里，手动排查时要先 pkill -f live_detect_pi.py。
#
#  放哪儿：和 start.sh 放同一个目录（/home/HwHiAiUser/Steve_folder/物料识别/）。
#  .desktop 的 Exec 已经指向这里的绝对路径；改目录要同步改 Exec 那一行。
#
#  编码：UTF-8 + LF。别用 Windows 记事本另存，会变 CRLF 导致
#  `\r: command not found`。
# ============================================================================

cd "$(dirname "$0")" || exit 1
[ -w /tmp ] && exec > >(tee -a /tmp/icon_start.log) 2>&1

# ---- 单例锁：全局只允许一个视觉实例 --------------------------------------
# 2026-11-24 第九轮：板上反复「自动变亮」，而 exposure_auto / exposure_absolute
# / gain 读回全部稳定，怀疑是 Terminal=false 下重复双击 + SSH 旧实例残留，
# 导致多个 live_detect_pi.py 并存、互相写 v4l2 控件 / 重开相机。
# flock 锁加在 fd 9 上，随进程（含其 exec 出去的 python）生命周期自动释放。
if command -v flock >/dev/null 2>&1; then
    LOCK_FILE=/tmp/task1_vision.lock
    exec 9>"$LOCK_FILE"
    if ! flock -n 9; then
        echo "[桌面启动] ==== 忽略重复启动 $(date +%T) ===="
        echo "  已有实例持有 $LOCK_FILE（Terminal=false 看不到终端，多半是重复双击）"
        echo "  想重启： pkill -f live_detect_pi.py  然后再双击图标"
        exit 0
    fi
    printf 'pid=%s  start=%s\n' "$$" "$(date +%T)" >&9
    echo "[桌面启动] 单例锁 OK  PID=$$  lock=$LOCK_FILE"
else
    echo "[桌面启动] 警告：找不到 flock，本次无单例保护"
    echo "           建议： sudo apt install -y util-linux"
fi

# 加锁之前遗留的旧实例不持有锁，flock 拦不住它 —— 只在日志里提示，不自动杀。
if pgrep -f 'live_detect_pi.py' >/dev/null 2>&1; then
    echo "[桌面启动] 警告：检测到未持锁的 live_detect_pi.py 残留进程！"
    echo "           多为加本锁之前启动的旧实例，建议 pkill -f live_detect_pi.py 后重启"
fi

echo "[桌面启动] ==== 新一次启动 $(date +%T) ===="

# ---- 显示：固定香橙派本地 HDMI --------------------------------------------
export DISPLAY="${PI_DISPLAY:-:0}"
if [ -z "${XAUTHORITY:-}" ] && [ -f "$HOME/.Xauthority" ]; then
    export XAUTHORITY="$HOME/.Xauthority"
fi

# ---- 无条件补 conda base（找 conda.sh，不依赖 conda 命令在 PATH 里）--------
for c in "$HOME/miniconda3/etc/profile.d/conda.sh" \
         "$HOME/anaconda3/etc/profile.d/conda.sh" \
         "$HOME/miniforge3/etc/profile.d/conda.sh" \
         /opt/conda/etc/profile.d/conda.sh \
         /usr/local/miniconda3/etc/profile.d/conda.sh \
         /usr/local/anaconda3/etc/profile.d/conda.sh; do
    if [ -f "$c" ]; then
        . "$c"
        conda activate base >/dev/null 2>&1 || true
        printf '[桌面启动] conda base: %s\n' "$c"
        break
    fi
done

# ---- 无条件补 CANN 运行时环境变量 ----------------------------------------
for e in /usr/local/Ascend/ascend-toolkit/set_env.sh \
         /usr/local/Ascend/ascend-toolkit/latest/set_env.sh; do
    if [ -f "$e" ]; then
        . "$e" >/dev/null 2>&1 || true
        printf '[桌面启动] CANN env: %s\n' "$e"
        break
    fi
done

echo "[桌面启动] conda base + CANN 已加载, DISPLAY=$DISPLAY"
if python3 -c 'import acl, serial' >/dev/null 2>&1; then
    echo '[桌面启动] 依赖 OK（pyACL + pyserial）'
else
    echo '[桌面启动] 依赖缺失（acl 或 serial）；start.sh 会自动回退 CPU 路线'
    echo '          补装： conda activate base && python3 -m pip install pyserial==3.5'
fi
echo '[桌面启动] 交给 start.sh ...'
echo

bash start.sh
code=$?

echo
printf '[已退出] exit=%s  窗口 60 秒后自动关闭，按回车立即关闭。\n' "$code"
read -t 60 -r _ || true
exit "$code"
