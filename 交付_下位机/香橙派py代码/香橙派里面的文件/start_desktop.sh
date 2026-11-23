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
#  2026-11-24 板上验证过的三个要点（别改回去）：
#    1) **无条件** source conda.sh + activate base + source CANN set_env.sh。
#       早期版本只在「import acl 失败」时才补，桌面那种干净环境里
#       配合 set -u 会直接崩，日志都看不到。所以这里连 set -u 都不加。
#    2) 日志 tee 到 /tmp/icon_start.log —— Terminal=false 时没有终端窗口，
#       只能靠这个文件查启动过程。
#    3) DISPLAY 一律 :0（本地 HDMI），**不**继承 SSH 里的 localhost:10.0。
#
#  放哪儿：和 start.sh 放同一个目录（/home/HwHiAiUser/Steve_folder/物料识别/）。
#  .desktop 的 Exec 已经指向这里的绝对路径；改目录要同步改 Exec 那一行。
#
#  编码：UTF-8 + LF。别用 Windows 记事本另存，会变 CRLF 导致
#  `\r: command not found`。
# ============================================================================

cd "$(dirname "$0")" || exit 1
[ -w /tmp ] && exec > >(tee -a /tmp/icon_start.log) 2>&1
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
