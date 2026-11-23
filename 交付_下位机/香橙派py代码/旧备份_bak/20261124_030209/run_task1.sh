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
#  【跑起来：默认就在香橙派本地 HDMI 显示器上开窗】
#    bash run_task1.sh          # 脚本自动 export DISPLAY=:0 + SHOW=1
#    # 若报 X 鉴权错： export XAUTHORITY=/home/HwHiAiUser/.Xauthority 再跑
#  【比赛要无窗口后台（延时更低）】
#    SHOW=0 bash run_task1.sh
#  【临时把画面传回 Windows，才需要显式覆盖】
#    PI_DISPLAY=localhost:10.0 bash run_task1.sh
#  【AE 变黑 / 过曝振荡】：启动前用 v4l2-ctl 关死自动曝光并定死曝光值，
#    python 主循环里还有守护线程 5 Hz 持续压回固件漂移。详见下面 ISP 复位段。
#    默认固定曝光 120（2026-11-24 板上实测正常；157 固件 default 会
#    过曝到 mean~226）。启动前还会整套复位 contrast / saturation /
#    backlight_compensation / 白平衡自动，避免旧毒值把画面推亮。
#    改固定曝光： EXP_FIXED=140 bash run_task1.sh
#    放宽范围： EXP_MIN=60 EXP_MAX=1000 bash run_task1.sh
#    提高守护频率： ISP_GUARD_HZ=10；关掉对比： ISP_GUARD_HZ=0
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
#  【调明暗】程序运行后在画面窗口按小写单键即可，程序会用 apply_isp()
#    把**整套**控件一次性下发：e/r 明暗(粗)、b/v 明暗(细)、m/n gamma、
#    g/h 增益、w 锁白平衡。固定曝光只在启动前用 EXP_FIXED 改。
#    ★ 千万别在程序跑着时手工单写 exposure_absolute（例如
#      v4l2-ctl --set-ctrl=exposure_absolute=200）：这颗 UVC 收到
#      「半配置」会重跑内部 3A 补偿，正好造成画面忽明忽暗来回跳
#      （2026-11-24 第三轮实测根因）。
#    查控件范围（程序没在跑时）： v4l2-ctl -d /dev/video0 --list-ctrls
#    换固定曝光： EXP_FIXED=200 bash run_task1.sh
# ============================================================================

set -u
cd "$(dirname "$0")" || exit 1

# ---- 显示一律走香橙派本地 HDMI（2026-11-24 用户明确要求）------------------
# 之前只打印提示不强制，结果从 MobaXterm SSH 里继承的 localhost:10.0
# 会把窗口画回 Windows。现在无条件钉死 :0；真要 X11 转发才显式给
# PI_DISPLAY=localhost:10.0（一般不用）。
export DISPLAY="${PI_DISPLAY:-:0}"

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
# 用户明确要求「以后都本地 HDMI 显示」，默认开窗；
# 比赛要极致低延时的无窗口后台跑，显式给 SHOW=0 bash run_task1.sh
SHOW="${SHOW:-1}"

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
    if [ ! -S /tmp/.X11-unix/X0 ] && [ "$DISPLAY" = ":0" ]; then
        echo '[提示] DISPLAY=:0 但没检测到本地 X server；没接 HDMI 显示器时'
        echo '       用 SHOW=0 bash run_task1.sh 走无窗口后台，避免 imshow 报错'
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
echo "  SHOW   = $SHOW    DISPLAY = $DISPLAY"
echo '----------------------------------------------'

# ---- 启动前复位 ISP 控件：把 AE 关死 --------------------------------------
# 2026-11-24：AE 开着时逐帧均值 144→0.9 单调收敛到全黑；关死后固件又只在
# 持续出流时偷偷漂曝光（画面正常↔过曝来回跳），所以 python 里还有守护
# 线程周期性压回。这里先在 python 启动前复位一次，冲掉固件里的旧毒值。
# OpenCV 的 cap.set() 对这颗 UVC 无效 → 必须用 v4l2-ctl 直接写驱动。
# 顺序有意义：exposure_auto=1(手动) 排最前，exposure_absolute 才生效。
# 跳过本段（交给 python 自己锁）： CAM_RESET=0 bash run_task1.sh
if [ "${CAM_RESET:-1}" = "1" ] && command -v v4l2-ctl >/dev/null 2>&1; then
    case "$CAM" in
        /dev/*) DEV="$CAM" ;;
        *)      DEV="/dev/video${CAM}" ;;
    esac
    # ★ 与 live_detect_pi.py 完全一致：启动固定曝光只认 EXP_FIXED，
    #   默认 120（2026-11-24 板上实测正常；157 固件 default 会过曝
    #   mean~226），**不再读 camera_settings.json**。范围由 EXP_MIN /
    #   EXP_MAX 控制，默认 100~400（硬件真实 1~5000，100 是为远离
    #   80 那个历史停流坏值）。
    EXP_MIN="${EXP_MIN:-100}"
    EXP_MAX="${EXP_MAX:-400}"
    EXP_FIXED="${EXP_FIXED:-120}"
    EXP_FIXED="$(awk -v v="$EXP_FIXED" -v lo="$EXP_MIN" -v hi="$EXP_MAX" \
        'BEGIN{v+=0; if(v<lo)v=lo; if(v>hi)v=hi; printf "%d",v}')"
    export EXP_MIN EXP_MAX EXP_FIXED
    # 板上 --list-ctrls 实测的**真实控件名 / 出厂默认值**。旧版
    # white_balance_automatic 是错名，整条命令失败 → 背光补偿从未被关；
    # contrast=64 / saturation=128 旧毒值也一直没人压 → 稳定过曝。
    ISP_EXTRA="${ISP_EXTRA:-white_balance_temperature_auto=0,backlight_compensation=0,contrast=32,saturation=60}"
    export ISP_EXTRA
    if [ -e "$DEV" ]; then
        # ★ 整套一条命令写完（core + 附加），绝不单独写附加项：
        #   单独写属于「半配置」，会让固件重跑 3A 补偿（第三轮定论）。
        #   整条失败（控件名不认识）才退回只写核心。
        CTRL_CORE="exposure_auto=1,exposure_auto_priority=0,gain=0,gamma=100,brightness=0,exposure_absolute=$EXP_FIXED"
        if [ -n "$ISP_EXTRA" ]; then
            v4l2-ctl -d "$DEV" --set-ctrl="$CTRL_CORE,$ISP_EXTRA" >/dev/null 2>&1 || \
                v4l2-ctl -d "$DEV" --set-ctrl="$CTRL_CORE" >/dev/null 2>&1 || true
        else
            v4l2-ctl -d "$DEV" --set-ctrl="$CTRL_CORE" >/dev/null 2>&1 || true
        fi
        echo "  ISP 复位 $DEV：AE=手动  曝光=$EXP_FIXED  core+附加整套下发"
    else
        echo "  [提示] 找不到 $DEV，跳过 ISP 复位（python 里会再试一次）"
    fi
fi

exec python3 live_detect_pi.py \
    --backend "$BACKEND" --model "$MODEL" --imgsz "$IMGSZ" \
    --cam "$CAM" --res "$RES" --fourcc "$FOURCC" --fps "$FPS" \
    --task "$TASK" \
    --link "$LINK" --com "$COM" --baud "$BAUD" \
    --tol "$TOL" --motion-tol "$MOTION_TOL" --lock-frames "$LOCK_FRAMES" \
    "${SHOW_ARG[@]}"
