#!/usr/bin/env bash
# ============================================================================
#  run_task1.sh —— 香橙派「任务一：抓取」一键启动
#  对应 PC 端的 启动_任务一_抓取.bat，参数逐条对齐
#
#  【放哪儿】把本目录所有文件一起拖进 /home/HwHiAiUser/Steve_folder/
#            （用 MobaXterm 左边 SFTP 面板直接拖，或 rz / scp 均可）
#
#  【第一次跑：装依赖】只需一次
#    sudo apt update
#    sudo apt install -y python3-opencv python3-numpy python3-serial python3-pil v4l-utils
#    pip3 install onnxruntime          # 装不上就跳过，用 BACKEND=cvdnn
#    sudo usermod -aG video,dialout $USER   # 摄像头 + 串口权限，之后重新登录 SSH
#
#  【先自检】
#    python3 check_pi.py
#
#  【跑起来（比赛用：无窗口后台，不依赖 DISPLAY）】
#    bash run_task1.sh
#
#  【香橙派 HDMI 显示器上看画面】
#    export DISPLAY=:0
#    SHOW=1 bash run_task1.sh
#    # 若报 X 鉴权错： export XAUTHORITY=/home/HwHiAiUser/.Xauthority 再跑
#
#  【MobaXterm 把画面传回 Windows】
#    用带 X11 转发的 SSH 重新连上（echo $DISPLAY 应是 localhost:10.0 之类）
#    SHOW=1 bash run_task1.sh
#
#  【NPU 加速（推荐，实测 33 FPS）】—— 注意和 CPU 路线的环境正好相反
#    conda activate base                                  # CPU 路线是 deactivate！
#    source /usr/local/Ascend/ascend-toolkit/set_env.sh
#    bash atc_convert.sh                                 # 一次性，生成 best.om
#    python3 om_backend.py --model best.om                # 冒烟，确认 NPU 能跑
#    BACKEND=om LINK=print SHOW=1 bash run_task1.sh
#    # 正式接上 CH340 后：
#    COM=/dev/ttyUSB0 BACKEND=om bash run_task1.sh
#
#  【摄像头号：自动探测，不用手填】
#    脚本会从 `v4l2-ctl --list-devices` 里挑 (usb-...) 段落下编号最小的
#    /dev/videoN，避免踩板载 ISP 虚拟节点（详见下面 detect_usb_cam 注释）。
#    想强制指定： CAM=0 bash run_task1.sh
#
#  【临时改参数，不用改脚本】环境变量覆盖即可：
#    CAM=1 TASK=532 bash run_task1.sh
#    COM=/dev/ttyS0 BAUD=115200 bash run_task1.sh
#    FOURCC=YUY2 FPS=15 RES=320x240 bash run_task1.sh   # 摄像头不支持 MJPG 时
#    LINK=print bash run_task1.sh          # 只打印报文，不真发，先确认该发什么
#    BACKEND=cvdnn bash run_task1.sh       # 没装 onnxruntime 时用 OpenCV DNN
#    BACKEND=om bash run_task1.sh          # Ascend NPU
#    BACKEND=none LINK=print bash run_task1.sh   # 只验摄像头和链路，不推理
#    MODEL=best_320.onnx bash run_task1.sh # CPU 保底小模型（自动 IMGSZ=320）
#
#  【调曝光 / 亮度】（V4L2 下 OpenCV 的 set 多半无效，用 v4l2-ctl）
#    先 v4l2-ctl --list-devices 看当前 USB 摄像头是 /dev/videoN，再：
#    v4l2-ctl -d /dev/video0 --list-ctrls
#    v4l2-ctl -d /dev/video0 --set-ctrl=exposure_absolute=200
#    v4l2-ctl -d /dev/video0 --set-ctrl=brightness=0
# ============================================================================

set -u
cd "$(dirname "$0")" || exit 1

# ---- 摄像头号自动探测 -----------------------------------------------------
# 坑（2026-11-22 板上实测）：AIpro 重启后 /dev/video* 会重排。
#   vin-media_mainpath (platform:media:isp-vir0) -> /dev/video2、/dev/video3
#     是板载 ISP 虚拟节点，没有传感器输入，STREAMON 直接
#     "Operation not permitted"，怎么调格式 / 曝光 / 分辨率都读不到帧。
#   USB 2.0 Camera: HD USB Camera (usb-xhci-hcd.3.auto-1) -> /dev/video0、1
#     才是真摄像头，支持 MJPG。
# 之前写死 CAM=2，重启后就踩了这个坑。现在优先用环境变量 CAM，
# 否则从 v4l2-ctl --list-devices 里挑 (usb-...) 段落下编号最小的 videoN；
# 探测不到（没装 v4l-utils / 输出格式不同）就回退 0，并明确打印来源。
detect_usb_cam() {
    command -v v4l2-ctl >/dev/null 2>&1 || return 0
    v4l2-ctl --list-devices 2>/dev/null | awk '
        /\(usb-/ { inusb=1; next }              # USB 设备段落头
        /^[^[:space:]]/ { inusb=0 }             # 顶格新段落 → 离开 USB 段
        inusb && /\/dev\/video[0-9]+/ {
            d = $0
            gsub(/.*\/dev\/video/, "", d)
            gsub(/[^0-9].*/, "", d)
            if (d != "" && (best == "" || d+0 < best+0)) best = d
        }
        END { if (best != "") print best }
    '
}

if [ -n "${CAM:-}" ]; then
    CAM_FROM='手动指定'
else
    CAM="$(detect_usb_cam)"
    if [ -n "$CAM" ]; then
        CAM_FROM='自动探测(UVC)'
    else
        CAM=0
        CAM_FROM='回退默认(未探测到)'
    fi
fi

# ---- 与 启动_任务一_抓取.bat 一一对应，可用同名环境变量覆盖
COM="${COM:-/dev/ttyUSB0}"          # bat: COM_PORT=COM12
BAUD="${BAUD:-115200}"              # bat: BAUD=115200
RES="${RES:-640x480}"               # bat: RES=640x480（USB2 带宽下别上 720p）
FOURCC="${FOURCC:-MJPG}"            # 摄像头格式；不支持 MJPG 时改 YUY2/UYVY
FPS="${FPS:-30}"                    # 采集帧率（不是推理帧率）
TASK="${TASK:-4}"                   # bat: TASK=4（绿）。1红 2黄 3蓝 4绿 5黑 6浅蓝
LINK="${LINK:-serial}"              # bat: LINK=serial
TOL="${TOL:-2000}"                  # bat: 自由跑模式，容差极大 → 永远 locked
MOTION_TOL="${MOTION_TOL:-100000}"  # bat: 停稳检查关闭
LOCK_FRAMES="${LOCK_FRAMES:-1}"     # bat: 不要 3 帧确认，看到就发
BACKEND="${BACKEND:-onnx}"
SHOW="${SHOW:-0}"

# 模型默认文件按后端选：om 用 .om，pt 用 .pt，其余用 .onnx
case "$BACKEND" in
    pt)  MODEL="${MODEL:-best.pt}" ;;
    om)  MODEL="${MODEL:-best.om}" ;;
    *)   MODEL="${MODEL:-best.onnx}" ;;
esac

# 模型输入尺寸必须与 ONNX 导出 / ATC 转换时一致，否则形状不匹配。
# 按模型名自动推断，也可显式 IMGSZ=320 覆盖。
case "$MODEL" in
    *320*)  IMGSZ="${IMGSZ:-320}" ;;
    *256*)  IMGSZ="${IMGSZ:-256}" ;;
    *)      IMGSZ="${IMGSZ:-640}" ;;
esac

# ---- NPU 路线前置检查：早失败，别等到 import 才报一堆看不懂的错
if [ "$BACKEND" = "om" ]; then
    if ! python3 -c 'import acl' 2>/dev/null; then
        echo '[错误] BACKEND=om 需要 pyACL，当前 python3 里 import acl 失败。'
        echo '       先执行：'
        echo '         conda activate base'
        echo '         source /usr/local/Ascend/ascend-toolkit/set_env.sh'
        echo '       再跑本脚本。（CPU 路线反而是要 conda deactivate）'
        exit 1
    fi
    if [ ! -f "$MODEL" ]; then
        echo "[错误] 找不到 $MODEL —— 先跑一次： bash atc_convert.sh"
        exit 1
    fi
fi

SHOW_ARG=(--no-show)
if [ "$SHOW" = "1" ]; then
    SHOW_ARG=(--show)
    if [ -z "${DISPLAY:-}" ]; then
        echo '[提示] SHOW=1 但 DISPLAY 为空：想本地 HDMI 看画面先 export DISPLAY=:0'
        echo '       （想传回 Windows 就用带 X11 转发的 SSH 重新连）'
    fi
fi

echo '----------------------------------------------'
echo "  CAM    = $CAM  [$CAM_FROM]   ($RES $FOURCC ${FPS}fps)"
echo "  COM    = $COM  @ $BAUD"
echo "  TASK   = $TASK     LINK = $LINK"
echo "  BACKEND= $BACKEND  MODEL= $MODEL  IMGSZ= $IMGSZ"
if [ "$BACKEND" = "om" ]; then
    echo '  >> Ascend 310B4 NPU 推理（pyACL / best.om）'
fi
echo "  TOL=$TOL  MOTION_TOL=$MOTION_TOL  LOCK_FRAMES=$LOCK_FRAMES"
echo "  SHOW   = $SHOW"
echo '----------------------------------------------'

exec python3 live_detect_pi.py \
    --backend "$BACKEND" --model "$MODEL" --imgsz "$IMGSZ" \
    --cam "$CAM" --res "$RES" --fourcc "$FOURCC" --fps "$FPS" \
    --task "$TASK" \
    --link "$LINK" --com "$COM" --baud "$BAUD" \
    --tol "$TOL" --motion-tol "$MOTION_TOL" --lock-frames "$LOCK_FRAMES" \
    "${SHOW_ARG[@]}"
