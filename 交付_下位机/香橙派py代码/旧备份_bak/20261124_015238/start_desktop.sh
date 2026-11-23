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
#  放哪儿：和 start.sh 放同一个目录（/home/HwHiAiUser/Steve_folder/物料识别/）。
#  .desktop 的 Exec 已经指向这里的绝对路径；改目录要同步改 Exec 那一行。
#
#  编码：UTF-8 + LF。
# ============================================================================

set -u
cd "$(dirname "$0")" || exit 1

# ---- 补 conda base（找 conda.sh，不依赖 conda 命令在 PATH 里）---------------
if ! python3 -c 'import acl' >/dev/null 2>&1; then
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
    # 兜底：有些镜像把 conda init 写进 .bashrc，试着 source 一下（无效也无害）
    if ! python3 -c 'import acl' >/dev/null 2>&1 && [ -f "$HOME/.bashrc" ]; then
        . "$HOME/.bashrc" >/dev/null 2>&1 || true
    fi
    # 补 CANN 运行时环境变量
    for e in /usr/local/Ascend/ascend-toolkit/set_env.sh \
             /usr/local/Ascend/ascend-toolkit/latest/set_env.sh; do
        if [ -f "$e" ]; then
            . "$e" >/dev/null 2>&1 || true
            printf '[桌面启动] CANN env: %s\n' "$e"
            break
        fi
    done
fi

if python3 -c 'import acl' >/dev/null 2>&1; then
    echo '[桌面启动] pyACL 就绪，将走 Ascend NPU'
else
    echo '[桌面启动] pyACL 不可用，start.sh 会自动回退 CPU 路线（约 5 FPS）'
fi
echo '[桌面启动] 交给 start.sh ...'
echo

bash start.sh
code=$?

echo
printf '[已退出] exit=%s  窗口 60 秒后自动关闭，按回车立即关闭。\n' "$code"
read -t 60 -r _ || true
exit "$code"
