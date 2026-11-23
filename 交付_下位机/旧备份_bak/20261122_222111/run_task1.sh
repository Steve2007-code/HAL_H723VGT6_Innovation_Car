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
#  【临时改参数，不用改脚本】环境变量覆盖即可：
#    CAM=3 TASK=532 bash run_task1.sh
#    COM=/dev/ttyS0 BAUD=115200 bash run_task1.sh
#    LINK=print bash run_task1.sh          # 只打印报文，不真发，先确认该发什么
#    BACKEND=cvdnn bash run_task1.sh       # 没装 onnxruntime 时用 OpenCV DNN
#    BACKEND=none LINK=print bash run_task1.sh   # 只验摄像头和链路，不推理
#
#  【调曝光 / 亮度】（V4L2 下 OpenCV 的 set 多半无效，用 v4l2-ctl）
#    v4l2-ctl -d /dev/video2 --list-ctrls
#    v4l2-ctl -d /dev/video2 --set-ctrl=exposure_absolute=200
#    v4l2-ctl -d /dev/video2 --set-ctrl=brightness=0
# ============================================================================

set -u
cd "$(dirname "$0")" || exit 1

# ---- 与 启动_任务一_抓取.bat 一一对应，可用同名环境变量覆盖
CAM="${CAM:-2}"                     # bat: CAM=1（PC 上是 1 号；香橙派实测多为 2）
COM="${COM:-/dev/ttyUSB0}"          # bat: COM_PORT=COM12
BAUD="${BAUD:-115200}"              # bat: BAUD=115200
RES="${RES:-640x480}"               # bat: RES=640x480（USB2 带宽下别上 720p）
TASK="${TASK:-4}"                   # bat: TASK=4（绿）。1红 2黄 3蓝 4绿 5黑 6浅蓝
LINK="${LINK:-serial}"              # bat: LINK=serial
TOL="${TOL:-2000}"                  # bat: 自由跑模式，容差极大 → 永远 locked
MOTION_TOL="${MOTION_TOL:-100000}"  # bat: 停稳检查关闭
LOCK_FRAMES="${LOCK_FRAMES:-1}"     # bat: 不要 3 帧确认，看到就发
BACKEND="${BACKEND:-onnx}"
SHOW="${SHOW:-0}"

if [ "$BACKEND" = "pt" ]; then
    MODEL="${MODEL:-best.pt}"
else
    MODEL="${MODEL:-best.onnx}"
fi

# 模型输入尺寸必须与 ONNX 导出时一致，否则 ORT 会报形状不匹配。
# 按模型名自动推断，也可显式 IMGSZ=320 覆盖。
case "$MODEL" in
    *320*)  IMGSZ="${IMGSZ:-320}" ;;
    *256*)  IMGSZ="${IMGSZ:-256}" ;;
    *)      IMGSZ="${IMGSZ:-640}" ;;
esac

SHOW_ARG=(--no-show)
if [ "$SHOW" = "1" ]; then
    SHOW_ARG=(--show)
    if [ -z "${DISPLAY:-}" ]; then
        echo '[提示] SHOW=1 但 DISPLAY 为空：想本地 HDMI 看画面先 export DISPLAY=:0'
        echo '       （想传回 Windows 就用带 X11 转发的 SSH 重新连）'
    fi
fi

echo '----------------------------------------------'
echo "  CAM    = $CAM     ($RES MJPG)"
echo "  COM    = $COM  @ $BAUD"
echo "  TASK   = $TASK     LINK = $LINK"
echo "  BACKEND= $BACKEND  MODEL= $MODEL  IMGSZ= $IMGSZ"
echo "  TOL=$TOL  MOTION_TOL=$MOTION_TOL  LOCK_FRAMES=$LOCK_FRAMES"
echo "  SHOW   = $SHOW"
echo '----------------------------------------------'

exec python3 live_detect_pi.py \
    --backend "$BACKEND" --model "$MODEL" --imgsz "$IMGSZ" \
    --cam "$CAM" --res "$RES" --task "$TASK" \
    --link "$LINK" --com "$COM" --baud "$BAUD" \
    --tol "$TOL" --motion-tol "$MOTION_TOL" --lock-frames "$LOCK_FRAMES" \
    "${SHOW_ARG[@]}"
