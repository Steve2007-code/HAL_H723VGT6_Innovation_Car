# -*- coding: utf-8 -*-
"""
live_detect_pi.py —— 香橙派版「任务一：抓取」实时视觉（Linux / aarch64）

由 PC 版 任务一_抓取/live_detect.py 移植，业务逻辑（判色、顶心、任务码、
对准判定、<<X,dx,Y,dy>> 报文、20Hz 节流）**一字未改**，只换了平台相关的部分：

  | 项目           | PC 版 (Windows)                 | 香橙派版                          |
  |----------------|----------------------------------|-----------------------------------|
  | 推理           | ultralytics YOLO(best.pt)+torch | best.onnx + onnxruntime / cv2.dnn|
  | 摄像头后端     | CAP_DSHOW                       | CAP_V4L2                          |
  | 采集格式       | YUY2                            | MJPG（USB2 带宽下 640x480 才 30fps）|
  | 串口           | COM12                           | /dev/ttyUSB0                      |
  | 中文字体       | C:\\Windows\\Fonts\\msyh.ttc    | Noto CJK / 文泉驿；找不到就不贴牌  |
  | 显示           | 必须 imshow                     | --show 才开窗；默认无窗口可后台跑  |

启动（复刻 启动_任务一_抓取.bat 的参数）：
    bash run_task1.sh
或手工：
    python3 live_detect_pi.py --backend onnx --model best.onnx \
        --cam 2 --res 640x480 --task 4 --link serial \
        --com /dev/ttyUSB0 --baud 115200 \
        --tol 2000 --motion-tol 100000 --lock-frames 1

显示一律走香橙派本地 HDMI（2026-11-24 起固定，不再走 X11 转发）：
    bash start.sh           # 自动 export DISPLAY=:0 + SHOW=1
    SHOW=1 bash run_task1.sh
想临时传回 Windows 才显式给： PI_DISPLAY=localhost:10.0 bash start.sh

【AE 变黑的坑，2026-11-24 定论】这颗 UVC(05a3:9230) 的自动曝光在
open() 之后会自己收敛：逐帧均值 144 → 114 → 85 → ... → 0.9，约 20 帧
(0.67 s) 后锁死近全黑。原因是 exposure_auto=3(Aperture Priority)，
且 OpenCV 的 cap.set() 既关不掉 AE、写进去的值还会持久留在固件里。
本程序在 VideoCapture() **之前**用 v4l2-ctl 把
    exposure_auto=1(手动) / exposure_absolute=120 / priority=0 /
    gain=0 / gamma=100 / brightness=0
写死，之后不再碰任何曝光属性。（120 是 2026-11-24 板上实测正常值；
157 是固件 default，在补光台面下会过曝，见下面第四轮。）

【过曝振荡，2026-11-24 第二轮】关死 AE 后画面仍「正常→过曝→跳回」。
诊断显示 open 前后读控件完全一致、日志无重开相机 —— 说明固件只在
**持续出流**时才跑内部 3A 线程偷偷漂曝光，静止读控件看不出来。
解法：后台守护线程默认 5 Hz 把核心控件重新压回去（每次只 1 个
v4l2-ctl 进程），按键调参同步更新目标值，重开相机/退出时暂停。
    EXP_FIXED=140 bash start.sh   # 换固定曝光（默认 120，范围 100~400）
    EXP_MIN=60 EXP_FIXED=100 bash start.sh   # 想更暗先放宽下限
    ISP_GUARD_HZ=10               # 提高守护频率（默认 5，0=关闭）
    ISP_EXTRA='contrast=32'       # 可选；默认空 = 不写附加控件
    EXP_LOCK=0                    # 关掉整套锁定，仅用于对比测试
注意：启动固定曝光只认 EXP_FIXED，不再读 camera_settings.json；
附加控件默认一个都不写（见第六轮），否则会叫醒 AE。

【按 e 后振荡，2026-11-24 第三轮定论】守护线程上线后仍跳，但变成
「按几下 e 之后才开始跳」。根因在按键本身：旧 _ctrl() 只单独写
exposure_absolute（**半配置**），这颗 UVC 收到后重新跑内部 3A 补偿
→ 过曝 → 守护压回 → 又正常，循环振荡；同时守护线程与按键并发各起
一个 v4l2-ctl 抢设备，失败那条回退 cap.set() 又把已关死的 AE 叫醒。
三条一起改，缺一不可：
  1. 运行时**绝不碰曝光时间**；e/r 改调 brightness（ISP 后端数字量，
     立即生效、不触发重收敛）；固定曝光只在启动时由 EXP_FIXED 决定。
  2. 任何写入都走 apply_isp() **整套**下发（含 AE=手动 + priority=0），
     不再有「只写一个控件」的半配置路径。
  3. isp_mutex 互斥，守护线程与按键不再并发抢设备；彻底删掉
     cap.set(CAP_PROP_*) 回退分支。

【过曝，2026-11-24 第四轮：按真实控件范围校正】不跳之后画面仍稳定
过曝（mean 226），把 brightness/gamma/gain 按到代码下限也没用。
板上 `v4l2-ctl -d /dev/video0 --list-ctrls` 给出真实范围：
    brightness -64~64 / gamma 72~500 / gain 0~100 /
    exposure_absolute 1~5000 / contrast 0~64 / saturation 0~128
三个真凶：
  ① contrast=64(满)、saturation=128(满)、backlight_compensation=1
     三个旧毒值**不在整套下发里**，一直稳定挂着把画面往亮推；
  ② 控件名写错：这颗叫 white_balance_temperature_auto，旧版写
     white_balance_automatic → 整条 --set-ctrl 因未知控件名失败 →
     isp_lock 兜底只写核心 → backlight_compensation=0 从未生效；
  ③ 钳位全按 PC 版猜的：brightness 卡在 -32(硬件能到 -64)、
     gamma 按到 50 写不进(硬件最小 72)、gain 上限给了 255(实际 100)。
解法：ISP_EXTRA_DEFAULT 换成真实控件名并复位 contrast/saturation/
backlight/白平衡自动，且整条下发；钳位按真实 min/max 校正；默认
固定曝光 157 → **120**，EXP_MIN 120 → 100。isp_lock 兜底增加
「逐项带整套 core 重试」，避免一个错名把全部附加项拖垮。

【曝光又自己变，2026-11-24 第五轮：附加控件不能高频写】第四轮把
ISP_EXTRA 控件名修对、毒值也清了，画面却「又开始自动变化曝光」。
根因：第三轮之所以稳，是因为旧版控件名写错 → 整条 --set-ctrl 失败
→ 兜底只写 6 个 core；第四轮名字对了以后，守护线程 5 Hz **每次都
真的写** white_balance_temperature_auto / backlight_compensation /
contrast / saturation。这颗 UVC 的 AWB / 背光补偿与 AE 共用内部 3A
线程，持续出流时反复写它们会把 3A 叫醒 → 曝光重新自己收敛/振荡。
解法：下发分级（apply_isp 的 full 参数）——
  full=True  整套(core + extra)：只在启动首次 / 重开相机 / 用户按 w
             时写一次，此时还没出流或刚重开，安全；
  full=False 只压 core：守护线程、按键调参这两条高频路径。
附加控件是**一次性复位**的静态值，不需要每 0.2 s 压回；真正会
被固件偷偷漂的只有曝光链路，压 core 就够了。

【AE 又被打开，2026-11-24 第六轮：附加控件一个都别写】第五轮分级
后用户实测 `ISP_GUARD_HZ=0 EXP_FIXED=120 bash start.sh` **仍自动增加
曝光** —— 关守护后程序只在启动时整套写一次，之后不写任何控件，
说明不是高频问题，而是**那一次整套写就把 AE 打开了**。
决定性线索：--list-ctrls 里 `exposure_auto default=3`(自动)；而
两处整套命令都是 `exposure_auto=1` 排最前、附加控件排最后。写
backlight_compensation / white_balance_temperature_auto 会触发这颗
UVC 内部 3A 状态机重置，把 exposure_auto 恢复成默认 3 → AE 接管。
起点 120 偏暗 → AE 逐渐增加曝光（用户看到「自动增加」）；这与
第一轮 157 起点偏亮导致 144→0.9 单调变黑是同一个 AE、方向相反。
**解法：ISP_EXTRA_DEFAULT 改为空，附加控件默认一个都不写**，
启动/守护都只写 6 个 core → 回到第三轮 + 120 的已验证稳定态。
过曝主因是 157，120 已解决；contrast/saturation 旧毒值不是主因。
分级下发结构保留（显式给 ISP_EXTRA 时仍能保护高频路径）。

报文协议（与 PC 版、下位机完全一致）：
    <<X,dx,Y,dy>>          有目标，右/下为正，整数
    <<X,9999,Y,9999>>      无目标 / 未对准
    115200 8N1，无换行，收到 >> 即断帧
"""

import argparse
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from sixcolor import COLORS, Prototypes, classify, label_of_lab, roi_lab
from link import Link
from taskcode import NAMES as TASK_NAMES, Tracker, parse as parse_task
from top_center import material_mask, top_center_edge, top_center_from_mask
from yolo_backend import Detector


def read_image(path):
    return cv2.imdecode(np.fromfile(str(path), np.uint8), cv2.IMREAD_COLOR)


# ---------------------------------------------------------------- V4L2 控件
def dev_path(cam):
    """把 --cam 转成 /dev/videoN 绝对路径（v4l2-ctl 只认路径）。"""
    s = str(cam)
    if s.startswith('/dev/'):
        return s
    if s.isdigit():
        return '/dev/video' + s
    return '/dev/video0'


def v4l2_set_str(dev, ctrl):
    """直接把整串喂给 v4l2-ctl --set-ctrl=，返回是否成功。

    合并成一条命令（逗号分隔多个控件）是为了守护线程高频下发时
    每次只起 1 个进程，而不是 N 个。
    """
    import shutil
    import subprocess
    if shutil.which('v4l2-ctl') is None:
        return False
    try:
        r = subprocess.run(['v4l2-ctl', '-d', dev, '--set-ctrl=' + ctrl],
                           stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL,
                           timeout=2)
        return r.returncode == 0
    except Exception:
        return False


def v4l2_set(dev, pairs):
    """用 v4l2-ctl 写 UVC 控件。

    2026-11-24 板上实测：OpenCV 的 cap.set(CAP_PROP_AUTO_EXPOSURE /
    EXPOSURE / GAMMA ...) 对这颗 UVC(05a3:9230) 大半无效，而这些值
    又会**持久写进摄像头固件**跨进程保留。必须绕开 OpenCV 直接写驱动。
    """
    return v4l2_set_str(dev, ','.join('%s=%s' % (k, v) for k, v in pairs))


# 核心 ISP 控件：这 5 个已由板上 v4l2-ctl --list-ctrls 确认存在。
# 顺序有意义：exposure_auto=1(手动) 必须排最前，exposure_absolute 才生效。
ISP_CORE = (
    ('exposure_auto', '1'),
    ('exposure_auto_priority', '0'),
    ('gain', '0'),
    ('gamma', '100'),
    ('brightness', '0'),
)
# 2026-11-24 第六轮定论：**默认不写任何附加 ISP 控件**。
# 板上 --list-ctrls 里 exposure_auto default=3(自动)。第四/五轮把
# white_balance_temperature_auto / backlight_compensation / contrast /
# saturation 加进整套下发后，哪怕只写一次（ISP_GUARD_HZ=0 实测），
# 这颗 UVC 的内部 3A 状态机也会把 exposure_auto 重置回默认 3 →
# AE 接管 → 从 EXP_FIXED=120(偏暗) 起逐渐增加曝光、画面自己变亮。
# 第三轮之所以稳，正是因为附加控件名写错、整条失败、只写 6 个 core。
# 过曝主因是 157 曝光太长，120 已解决；contrast/saturation 旧毒值
# 不是主因，不复位也不影响（第三轮 + 120 已验证画面正常）。
# → 默认空 = 启动/守护都只写 core，AE 锁得住。需要临时复位某些
#   附加控件时可显式给 ISP_EXTRA=...（分级下发仍保护高频路径）。
ISP_EXTRA_DEFAULT = ''


def isp_lock(dev, target, extra=ISP_EXTRA_DEFAULT):
    """把 AE 关死、曝光定死；target 是「当前期望的整套控件值」。

    2026-11-24 板上诊断：open 前后读控件完全一致、日志无重开相机，
    但画面仍「正常→过曝→跳回」——说明固件只在**持续出流**时
    才跑内部 3A 线程偷偷漂曝光，静止读控件看不出来。
    因此启动时写一次不够，主循环里还会有守护线程周期性重下发。

    任何一次下发都必须是「整套 core + 附加项」，绝不能只写单个控件
    （第三轮定论：半配置会让固件重跑 3A 补偿 → 画面来回跳）。
    """
    core = [(k, target[k]) for k, _ in ISP_CORE]
    core.append(('exposure_absolute', target['exposure_absolute']))
    ctrl = ','.join('%s=%s' % (k, v) for k, v in core)
    if not extra:
        return v4l2_set_str(dev, ctrl)
    # 正常路径：整套一次写完，只起 1 个进程
    if v4l2_set_str(dev, ctrl + ',' + extra):
        return True
    # 兜底：某个附加控件名在这颗固件上不存在 → 整条失败。
    # 先保证核心锁住，再逐项「core + 单个附加项」重试（仍是整套，
    # 不是半配置），让其余存在的控件照样生效，不被一个错名全拖垮。
    ok = v4l2_set_str(dev, ctrl)
    for one in extra.split(','):
        one = one.strip()
        if one and v4l2_set_str(dev, ctrl + ',' + one):
            ok = True
    return ok


# ---------------------------------------------------------------- 汉字牌子
def _font(size):
    """Linux 下找一款能画汉字的字体；找不到就返回 None（不贴牌，不影响判色）。"""
    from PIL import ImageFont
    cands = [
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',
        '/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc',
        '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc',
        '/usr/share/fonts/truetype/wqy/wqy-microhei.ttc',
        '/usr/share/fonts/truetype/arphic/uming.ttc',
        '/usr/local/share/fonts/NotoSansCJK-Regular.ttc',
    ]
    for p in cands:
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, size)
            except Exception:
                pass
    return None


def make_patches(size=22):
    """把六个汉字预渲成小牌子。cv2.putText 画不了汉字，得靠 PIL。

    找不到 CJK 字体时返回 {} —— draw() 会跳过贴牌，判色与顶心逻辑照常，
    功能不受影响，只是框上不写颜色名。
    """
    from PIL import Image, ImageDraw
    font = _font(size)
    if font is None:
        return {}
    out = {}
    for cid, name in COLORS.items():
        probe = Image.new('RGB', (8, 8))
        l, t, r, b = ImageDraw.Draw(probe).textbbox((0, 0), name, font=font)
        w, h = (r - l) + 8, (b - t) + 6
        im = Image.new('RGB', (max(w, 8), max(h, 8)), (0, 0, 0))
        ImageDraw.Draw(im).text((4 - l, 3 - t), name, font=font, fill=(0, 255, 0))
        out[cid] = cv2.cvtColor(np.array(im), cv2.COLOR_RGB2BGR)
    return out


_PATCHES = None


def draw(img, boxes, protos, votes=None, annotate=True):
    """判色 + 估顶心（必做，votes 靠它）；annotate=True 时才往图上画框。

    后台无窗口跑时 annotate=False，省掉全部绘图开销，但业务结果完全一致。
    """
    global _PATCHES
    if annotate and _PATCHES is None:
        _PATCHES = make_patches()

    for (x0, y0, x1, y1, conf) in boxes:
        # 在"干净"的画面上取样与求顶心——画过框线、贴过牌子之后再取便污了
        vec = roi_lab(img, (x0, y0, x1 - x0, y1 - y0), inset=0.30)
        cid = None
        if protos is not None and protos.ids():
            r = classify(vec, protos)
            if r['ok']:
                cid = r['id']
                votes[r['id']] = votes.get(r['id'], 0) + 1
        else:
            cid = label_of_lab(vec)

        tc = estimate_top(img, (x0, y0, x1, y1))
        if tc is not None:
            votes.setdefault('_tops', []).append((cid, tc, (x0, y0, x1, y1), conf))

        if not annotate:
            continue

        cv2.rectangle(img, (x0, y0), (x1, y1), (0, 255, 0), 2)
        tx = x0 + 3
        if cid is not None and cid in _PATCHES:
            patch = _PATCHES[cid]
            ph, pw = patch.shape[:2]
            h_av = min(ph, img.shape[0])
            w_av = min(pw, img.shape[1])
            px = min(x0, max(0, img.shape[1] - w_av))
            py = y0 - ph if y0 - ph >= 0 else y0 + 2
            py = min(max(0, py), max(0, img.shape[0] - h_av))
            img[py:py + h_av, px:px + w_av] = patch[:h_av, :w_av]
            tx = px + w_av + 4
        cv2.putText(img, '%.2f' % conf, (tx, max(14, y0 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        if tc is not None:
            cv2.drawMarker(img, (int(tc[0]), int(tc[1])), (0, 0, 255),
                           cv2.MARKER_CROSS, 16, 2)
    return img


def filter_boxes(boxes, max_obj=3, loose=1.9, tight=0.45):
    """剔误检：尺寸离群、至多三只、位置重合。与 PC 版完全一致。"""
    if not boxes:
        return boxes
    out = [list(b) for b in boxes]
    if len(out) >= 3:
        areas = sorted((b[2] - b[0]) * (b[3] - b[1]) for b in out)
        med = areas[len(areas) // 2]
        out = [b for b in out
               if tight * med <= (b[2] - b[0]) * (b[3] - b[1]) <= loose * med]
    kept = []
    for b in sorted(out, key=lambda t: -t[4]):
        dup = False
        for k in kept:
            dx = (b[0] + b[2] - k[0] - k[2]) / 2.0
            dy = (b[1] + b[3] - k[1] - k[3]) / 2.0
            lim = 0.5 * min(b[2] - b[0], b[3] - b[1], k[2] - k[0], k[3] - k[1])
            if (dx * dx + dy * dy) ** 0.5 < lim:
                dup = True
                break
        if not dup:
            kept.append(b)
    if max_obj and len(kept) > max_obj:
        kept = sorted(kept, key=lambda t: -t[4])[:max_obj]
    return kept


def _top_fallback(box, res):
    """不跑边缘法的快速兜底：亮度法结果可用就用，否则 0.34 框高。"""
    bh = box[3] - box[1]
    if res is not None:
        ry = (res[1] - box[1]) / max(1, bh)
        if 0.20 <= ry <= 0.50:
            return res[0], res[1]
    return (box[0] + box[2]) / 2.0, box[1] + 0.34 * bh


def estimate_top(img, box):
    """估顶面中心。默认走快速路径（亮度法 + 0.34 兜底），跳过 300 次 RANSAC。

    2026-11-24 装机后实测控制回路 ~0.2 s 延时，PID 一直飘。边缘法
    top_center_edge() 每框要跑 300 次 fitEllipse + 数组距离运算，是隐藏大头
    （之前分段测速 boxes=[] 属于空转，没量到）。亮度分层法 + 0.34 框高
    兜底精度足够对准；确需最高精度时 TOP_FAST=0 恢复旧行为。
    """
    bh = box[3] - box[1]
    res = None
    got = material_mask(img, box)
    if got is not None:
        res = top_center_from_mask(img, got[0], got[1])

    if res is not None and res[2] != 'prior':
        tx, ty = res[0], res[1]
        ry = (ty - box[1]) / max(1, bh)
        if 0.20 <= ry <= 0.50:
            return tx, ty

    if os.environ.get('TOP_FAST', '1') != '0':
        return _top_fallback(box, res)

    e = top_center_edge(img, box)
    if e is not None and e[2] >= 200:
        return e[0], e[1]
    return _top_fallback(box, res)


def main():
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description='香橙派 · 任务一抓取实时视觉')

    # ---- 推理
    ap.add_argument('--backend', default='onnx',
                    choices=['onnx', 'cvdnn', 'pt', 'om', 'none'],
                    help='onnx=onnxruntime(默认) / cvdnn=OpenCV DNN / pt=ultralytics / '
                         'om=Ascend NPU(.om + pyACL，最快) / none=不推理')
    ap.add_argument('--model', default=str(here / 'best.onnx'))
    ap.add_argument('--weights', default=None,
                    help='兼容 PC 版写法；--backend pt 时等价于 --model')
    ap.add_argument('--conf', type=float, default=0.25)
    ap.add_argument('--iou', type=float, default=0.45)
    ap.add_argument('--imgsz', type=int, default=640,
                    help='模型输入尺寸，必须与 ONNX 导出时一致：'
                         'best.onnx=640 / best_320.onnx=320 / best_256.onnx=256')
    ap.add_argument('--threads', type=int, default=4,
                    help='onnxruntime 线程数，香橙派 4 核给 4 就够，别开满')

    # ---- 相机
    ap.add_argument('--cam', default='2',
                    help='摄像头号 2，或直接给设备路径 /dev/video2')
    ap.add_argument('--res', default='640x480')
    ap.add_argument('--fourcc', default='MJPG',
                    help='MJPG(推荐，USB2 下 640x480 才能 30fps) / YUYV')
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--image', default=None, help='用一张静态图试，不接相机')

    # ---- 业务参数（与 启动_任务一_抓取.bat 对齐）
    ap.add_argument('--proto', default=None, help='六色原型 proto.json（可选）')
    ap.add_argument('--tol', type=float, default=20.0)
    ap.add_argument('--target-up', type=float, default=100.0)
    ap.add_argument('--lock-frames', type=int, default=3)
    ap.add_argument('--motion-tol', type=float, default=8.0)
    ap.add_argument('--still-frames', type=int, default=5)
    ap.add_argument('--task', default=None, help='任务码，如 4 / 532 / 156+123')
    ap.add_argument('--advance-gap', type=int, default=12)
    ap.add_argument('--aim-dx', type=float, default=None)
    ap.add_argument('--aim-dy', type=float, default=None)
    ap.add_argument('--aim-file', default=str(here / 'aim_material.json'))
    ap.add_argument('--wire', default='offs', choices=['offs', 'grab'])
    ap.add_argument('--msg', default='GRAB {id}\n')

    # ---- 链路
    ap.add_argument('--link', default='print', choices=['print', 'udp', 'serial'])
    ap.add_argument('--udp-host', default='127.0.0.1')
    ap.add_argument('--udp-port', type=int, default=9000)
    ap.add_argument('--com', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)

    # ---- 显示 / 输出
    ap.add_argument('--show', action='store_true', help='强制开窗口（需要 DISPLAY）')
    ap.add_argument('--no-show', action='store_true', help='强制无窗口（比赛后台跑）')
    ap.add_argument('--period', type=float, default=1.0,
                    help='无窗口时每隔几秒在终端打一行状态')
    ap.add_argument('--out', default='live_shot.jpg')
    args = ap.parse_args()

    # ---------------- 显示模式判定
    if args.no_show:
        show = False
    elif args.show:
        show = True
    else:
        show = bool(os.environ.get('DISPLAY'))
    if show and not os.environ.get('DISPLAY') and sys.platform.startswith('linux'):
        print('[警告] 没有 DISPLAY，自动关窗。想本地看画面先 export DISPLAY=:0')
        show = False
    print('显示模式：%s' % ('开窗口 imshow' if show else '无窗口（后台）'))

    # ---------------- 推理器
    model = args.weights or args.model
    if args.backend == 'pt' and (args.weights is None):
        model = str(here / 'best.pt')
    det = Detector(args.backend, model, args.imgsz,
                   conf=args.conf, iou=args.iou, threads=args.threads)

    protos = Prototypes.load(args.proto) if args.proto else None
    if protos:
        print('已载入原型：', ', '.join(protos.name_of(c) for c in protos.ids()))

    # ---------------- 静态图模式
    if args.image:
        img = read_image(args.image)
        if img is None:
            raise SystemExit('读不到 %s' % args.image)
        t = time.time()
        boxes = det(img)
        dt = (time.time() - t) * 1000
        votes = {}
        vis = draw(img.copy(), boxes, protos, votes, annotate=True)
        print('%s: %d 个框，%.0f ms' % (Path(args.image).name, len(boxes), dt))
        for b in boxes:
            print('   (%d,%d)-(%d,%d)  conf=%.2f' % (b[0], b[1], b[2], b[3], b[4]))
        cv2.imencode('.jpg', vis)[1].tofile(str(here / '_live_preview.jpg'))
        print('预览已存 _live_preview.jpg')
        return

    # ---------------- 曝光 / ISP 参数（必须在打开相机之前定好）
    # 2026-11-24 结论：AE 会在 open() 后约 20 帧把画面收敛到全黑；
    # 关死 AE 后固件又只在**持续出流**时偷偷漂曝光（静止读控件看不出来）。
    # 所以两手都要：① open 前用 v4l2-ctl 锁一次；② 主循环里守护线程
    # 周期性重下发，把固件漂移按回去。
    exp_lock = os.environ.get('EXP_LOCK', '1') != '0'
    # 板上 --list-ctrls 实测 exposure_absolute 真实范围 1~5000。
    # 157(固件 default) 在补光台面下偏亮 → mean 226 过曝；
    # 120 是 2026-11-24 板上实测正常值，故默认改 120。
    # 下限留到 100 给微调空间，同时远离 80 那个历史停流坏值。
    exp_min = float(os.environ.get('EXP_MIN', '100'))
    exp_max = float(os.environ.get('EXP_MAX', '400'))
    # ★ 不再从 camera_settings.json 读 exposure 当启动值：板上日志实测
    #   json 里残留的旧毒值被钳到下限，画面随之「正常→过曝→跳回」。
    #   启动固定值只认 EXP_FIXED，默认 120（实测正常）。
    #   remember() 仍会写 json 供参考，想沿用某次调好的值请显式给
    #   EXP_FIXED=140 bash start.sh（也避免旧毒值回流）。
    if 'EXP_FIXED' in os.environ:
        try:
            exp_fixed = float(os.environ['EXP_FIXED'])
        except ValueError:
            exp_fixed = 120.0
    else:
        exp_fixed = 120.0
    exp_fixed = max(exp_min, min(exp_max, exp_fixed))
    # 钳位按板上 --list-ctrls 的真实 min/max 校正，别再按 PC 版猜：
    #   brightness -64~64（旧 -32~32，白浪费一半下调空间）
    #   gamma     72~500（旧 50~300，50 根本写不进固件）
    #   gain        0~100（旧 0~255）
    bri_min, bri_max = -64.0, 64.0
    gam_min, gam_max = 72.0, 500.0
    gai_min, gai_max = 0.0, 100.0
    tx_gap = float(os.environ.get('TX_GAP', '0'))
    tuned = False          # 只有真的调过参才在退出时写盘
    cfg = here / 'camera_settings.json'

    # 守护线程参数：出流时固件 3A 会漂，默认每 0.2 s 压回去一次。
    # 每次只起 1 个 v4l2-ctl 进程，开销可忽略；ISP_GUARD_HZ=0 可关。
    guard_hz = float(os.environ.get('ISP_GUARD_HZ', '5'))
    isp_extra = os.environ.get('ISP_EXTRA', ISP_EXTRA_DEFAULT)
    # 「当前期望的整套控件值」，按键调参时同步改这里，守护会用新值，
    # 不会把用户刚调好的曝光又拉回启动值。
    isp_target = {k: v for k, v in ISP_CORE}
    isp_target['exposure_absolute'] = '%d' % int(exp_fixed)
    guard_pause = [False]      # 重开相机期间暂停，避免和 close/open 抢设备
    guard_stop = None

    cam = args.cam
    if isinstance(cam, str) and cam.isdigit():
        cam = int(cam)
    dev = dev_path(cam)

    # ---- 所有 ISP 写入统一走这里：互斥 + 分级下发 -------------------------
    # 第三轮的坑：按键只单独写 exposure_absolute（半配置）会让固件重跑
    # 3A 补偿；守护线程和按键并发抢设备，失败那条回退 cap.set() 又把 AE
    # 叫醒。→ 统一加一把锁，任何写入都必须是整套 core。
    #
    # 第五轮（2026-11-24）的新坑：附加 ISP 控件（白平衡自动 / 背光补偿 /
    # contrast / saturation）**不能在持续出流时被高频重写** —— 这颗 UVC
    # 的 AWB / 背光补偿与 AE 共用内部 3A 线程，5 Hz 反复写会把 3A 叫醒，
    # 曝光又开始自己变。它们只需在「未出流」时整套写一次清掉毒值即可。
    # 因此分级：
    #   full=True  整套(core + extra)：启动首次 / 重开相机 / 用户按 w
    #   full=False 只压 core：守护线程、按键调参（高频路径）
    # 第六轮：默认 ISP_EXTRA='' → 两种模式实际都只写 core，AE 锁得住；
    #   分级结构保留，供显式给 ISP_EXTRA 时仍能保护高频路径。
    import threading
    isp_mutex = threading.Lock()

    def apply_isp(full=True):
        """full=True 整套下发；full=False 只压核心，避开 3A 触发源。"""
        with isp_mutex:
            return isp_lock(dev, isp_target, isp_extra if full else '')

    # ★ 第一手：在 cv2.VideoCapture() 之前锁一次
    if exp_lock:
        ok_lock = apply_isp()
        print('[ISP] 锁定 %s：exposure_auto=1(手动)  曝光=%s  '
              'gain/gamma/brightness 归零  [%s]' %
              (dev, isp_target['exposure_absolute'],
               'OK' if ok_lock else 'v4l2-ctl 失败'))
        if not ok_lock:
            print('[ISP] 警告：v4l2-ctl 不可用或写失败，AE 可能仍会把画面拉黑。')
            print('      先装： sudo apt install -y v4l-utils')
    else:
        print('[ISP] EXP_LOCK=0，跳过锁定（仅用于对比测试，画面大概率变黑）')

    # ---------------- 打开相机（V4L2 + MJPG）
    v4l2 = getattr(cv2, 'CAP_V4L2', 200)
    try:
        cap = cv2.VideoCapture(cam, v4l2)
    except Exception:
        cap = cv2.VideoCapture(cam)
    if not cap.isOpened():
        cap = cv2.VideoCapture(cam)
    if not cap.isOpened():
        raise SystemExit(
            '[错误] 相机打不开：%s\n'
            '       查设备： ls -l /dev/video*   或  v4l2-ctl --list-devices\n'
            '       换号：   --cam 0 / 1 / 3   或  --cam /dev/video0\n'
            '       查权限： sudo usermod -aG video $USER 然后重新登录' % cam)

    w, h = [int(v) for v in args.res.lower().split('x')]
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    # 缓冲数直接决定「读到的是几帧前的旧画面」：4 个缓冲在 30fps 下
    # 约 100 ms 固有滞后。但 1 个缓冲对 MJPG 变长帧不稳，会读到半帧/
    # 坏帧（表现为 mean 在 1/37/79/80 乱跳、fps 掉到个位数）。
    # 默认 2 = 低延时与稳定性的折中；真要极致低延时用 BUFSIZE=1，
    # 出现坏帧则用 BUFSIZE=4。控制延时的大头已由 TX_GAP=0 +
    # TOP_FAST=1 解决，不必靠把缓冲压到 1。
    buf_n = int(os.environ.get('BUFSIZE', '2'))
    bufsize = getattr(cv2, 'CAP_PROP_BUFFERSIZE', None)
    if bufsize is not None:
        try:
            cap.set(bufsize, buf_n)
        except Exception:
            pass
    rw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    rh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    got_fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc_s = ''.join(chr((got_fourcc >> (8 * i)) & 0xFF) for i in range(4))
    print('相机 %s  实际 %dx%d  fourcc=%s  目标 %s' %
          (cam, rw, rh, fourcc_s, args.fourcc))

    # ---------------- ★ 第二手：ISP 守护线程 -------------------------------
    # 2026-11-24 诊断：open 前后读控件完全一致（①③）、日志无重开相机，
    # 但画面仍「正常→过曝→跳回正常」。说明固件只在**持续出流**时才跑
    # 内部 3A / 自动增益线程偷偷漂曝光，静止读控件根本看不出来。
    # 解法：后台线程周期性把核心控件重新压回去。每次只起 1 个
    # v4l2-ctl 进程、默认 5 Hz，开销可忽略，也不占主循环时间。
    if exp_lock and guard_hz > 0:
        guard_stop = threading.Event()

        def _guard_loop():
            period = 1.0 / guard_hz
            while not guard_stop.wait(period):
                if guard_pause[0]:
                    continue
                try:
                    apply_isp(full=False)   # 高频路径：只压 core，不碰 3A
                except Exception:
                    pass

        threading.Thread(target=_guard_loop, name='isp-guard',
                         daemon=True).start()
        print('[ISP] 守护线程已启动：%g Hz（ISP_GUARD_HZ=0 关闭）' % guard_hz)

    # ---------------- 控件显示值：一律以 isp_target 为准 ------------------
    # 守护线程持续把固件压回 isp_target，所以状态栏 / 按键起点 / 落盘
    # 都用同一份值，避免「显示的」和「固件里的」不一致。
    # 不再 cap.get() 读回，也不再从 camera_settings.json 读 —— json 里的
    # 旧毒值被钳到 120 正是这轮「过曝振荡」的诱因。
    exp = float(isp_target['exposure_absolute'])
    gam = float(isp_target['gamma'])
    bri = float(isp_target['brightness'])
    gai = float(isp_target['gain'])

    def remember():
        """每调一次便落盘，万一强退也不丢（与 PC 版一致）。"""
        import json
        try:
            cfg.write_text(json.dumps({
                'cam': str(cam), 'res': '%dx%d' % (rw, rh),
                'exposure': round(float(exp), 1), 'gamma': round(float(gam), 1),
                'brightness': round(float(bri), 1), 'gain': round(float(gai), 1)},
                ensure_ascii=False, indent=2), encoding='utf-8')
        except Exception as e:
            print('  参数存不下来：', e)

    print('曝光固定=%d  范围[%d,%d]  GAM=%.0f  BRI=%.0f  GAIN=%.0f  [%s]' %
          (int(exp), int(exp_min), int(exp_max), gam, bri, gai,
           'AE 关死' if exp_lock else 'AE 未锁'))
    print('低延时: BUFSIZE=%d  TX_GAP=%.3fs  TOP_FAST=%s  SHOW=%s  '
          'ISP守护=%gHz' %
          (buf_n, tx_gap, os.environ.get('TOP_FAST', '1'),
           'on' if show else 'off', guard_hz if exp_lock else 0))
    if show:
        print('q 退出   s 存帧   t 打印顶心   j/l/i/k 挪标定点')
        print('e/r 明暗(粗 ±10)   b/v 明暗(细 ±5)   m/n gamma   g/h 增益   w 重发核心')
        print('（全小写单键，不用 Shift；每组左键减小、右键增大）')
        print('固定曝光=%d，运行时不碰曝光时间 —— 单写它会让固件重跑 3A 补偿、画面来回跳' %
              int(exp))
        print('AE 已关 + 只写核心曝光控件；默认不碰白平衡/背光/对比度(会叫醒 AE)')
        print('真实范围 BRI[-64,64] GAM[72,500] GAIN[0,100]；亮度/增益别拉满，会放大底噪')
        print('曝光只在启动时由 EXP_FIXED 决定（默认 120）；换值 EXP_FIXED=140 bash start.sh')
        print('关守护对比：ISP_GUARD_HZ=0；关整个锁定：EXP_LOCK=0')

    # ---------------- 链路
    def _load_aim(p):
        import json
        try:
            d = json.loads(Path(p).read_text(encoding='utf-8'))
            return float(d.get('dx', 0)), float(d.get('dy', 0))
        except Exception:
            return 0.0, 0.0

    def _save_aim(p, dx, dy):
        import json
        Path(p).write_text(json.dumps({'dx': dx, 'dy': dy},
                                      ensure_ascii=False, indent=2),
                           encoding='utf-8')

    aim_x, aim_y = _load_aim(args.aim_file)
    if args.aim_dx is not None:
        aim_x = args.aim_dx
    if args.aim_dy is not None:
        aim_y = args.aim_dy
    print('标定点偏移：dx=%+.0f  dy=%+.0f 像素（对准点 = 顶心 + 此偏移）' % (aim_x, aim_y))

    lk = Link(args.link, host=args.udp_host, port=args.udp_port,
              com=args.com, baud=args.baud)
    if args.wire == 'offs':
        print('信号通道：%s   协议 <<X,dx,Y,dy>>（无目标发 <<X,9999,Y,9999>>）'
              % lk.describe())
    else:
        print('信号通道：%s   模板 %r' % (lk.describe(), args.msg))
    print('容差 %.0fpx   连 %d 帧才发' % (args.tol, args.lock_frames))
    print('目标点 = 画面中心上移 %.0fpx，偏差与对准均以它为准' % args.target_up)

    tracker = None
    if args.task:
        groups = parse_task(args.task)
        tracker = Tracker(groups, advance_when_gone=args.advance_gap)
        seq = tracker.seq
        print('任务码：' + ' → '.join('%s(%d)' % (TASK_NAMES[d], d) for d in seq)
              + '　共 %d 只，当前第 1 只' % len(seq))
        if len(groups) > 1:
            print('  （给了 %d 批，本程序只跑第 1 批；换批请重开并改 --task）'
                  % len(groups))

    fps, prev = 0.0, time.time()
    locked_n = 0
    last_send = 0.0
    last_report = 0.0
    tx_hist = []
    tx_n, tx_t0, tx_hz = 0, time.time(), 0.0
    dup_t = 0.0
    last_line = ''
    k = 0

    def _tx(msg):
        nonlocal tx_n, tx_t0, tx_hz
        lk.send(msg)
        tx_hist.append(msg)
        if len(tx_hist) > 4:
            del tx_hist[0]
        tx_n += 1
        el = time.time() - tx_t0
        if el >= 1.0:
            tx_hz = tx_n / el
            tx_n, tx_t0 = 0, time.time()

    miss = 0
    reopens = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                # UVC 在 MJPG 下会偶发停流（cap_v4l select() timeout）。
                # 一次 read 失败就退太脆弱：先小睡重试，连续失败就重开相机。
                miss += 1
                if miss <= 10:
                    time.sleep(0.1)
                    continue
                if reopens < 2:
                    reopens += 1
                    miss = 0
                    print('[采集] 连续读不到帧，重开相机（第 %d 次）' % reopens)
                    guard_pause[0] = True     # 暂停守护，别和 close/open 抢设备
                    try:
                        cap.release()
                    except Exception:
                        pass
                    # 重开前先重新锁一次 ISP：设备被 close 后固件可能恢复
                    # 默认 AE，不锁的话新画面又会收敛变黑。用的是当前
                    # isp_target（用户按键调过的值也保留）。
                    if exp_lock:
                        apply_isp()
                    cap = cv2.VideoCapture(cam, v4l2)
                    if not cap.isOpened():
                        cap = cv2.VideoCapture(cam)
                    cap.set(cv2.CAP_PROP_FOURCC,
                            cv2.VideoWriter_fourcc(*args.fourcc))
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
                    cap.set(cv2.CAP_PROP_FPS, args.fps)
                    if bufsize is not None:
                        try:
                            cap.set(bufsize, buf_n)
                        except Exception:
                            pass
                    if exp_lock:
                        apply_isp()                            # open 后再补一次
                    guard_pause[0] = False
                    time.sleep(0.3)
                    continue
                print('读不到帧，相机重开仍失败，退出。')
                break
            miss = 0
            m = float(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).mean())
            boxes = det(frame)
            boxes = filter_boxes(boxes, max_obj=3)
            votes = {}
            vis = draw(frame, boxes, protos, votes, annotate=show)

            now = time.time()
            dt = now - prev
            prev = now
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt) if fps else 1.0 / dt

            # ---------------- 顶心与画面中心的重合判定 → 发信号
            cx0, cy0 = vis.shape[1] / 2.0, vis.shape[0] / 2.0
            tx0, ty0 = cx0, cy0 - args.target_up
            best = None
            tops = votes.get('_tops', [])
            if tracker is not None:
                target = tracker.target()
                if target is None:
                    tops = []
                else:
                    # 已去掉"抓完自动跳到下一只"：永远只盯任务码第一位颜色
                    tops = [t for t in tops if t[0] == target]

            for cid, tc, bb, cf in tops:
                ax, ay = tc[0] + aim_x, tc[1] + aim_y
                d = ((ax - tx0) ** 2 + (ay - ty0) ** 2) ** 0.5
                if best is None or d < best[0]:
                    best = (d, cid, tc, (ax, ay))

            # 几何与颜色互校：盘上三只既成等边，颜色必是三只不同
            if len(tops) == 3 and len({t[0] for t in tops}) < 3:
                ids = [t[0] for t in tops]
                if now - dup_t > 2.0:
                    dup_t = now
                    print('[警告] 三只料判出重复颜色 %s——必有一只判错，暂停发信号'
                          % [COLORS.get(i, '?') for i in ids])
                best = None

            locked = best is not None and best[0] <= args.tol
            locked_n = locked_n + 1 if locked else 0

            sending = False
            if args.wire == 'offs':
                ok_now = locked and locked_n >= args.lock_frames
                if ok_now:
                    # 符号：目标点 - 标定点，即下位机需要移动的量
                    ix = int(round(tx0 - best[3][0]))
                    iy = int(round(ty0 - best[3][1]))
                    if now - last_send > tx_gap:          # 默认 0：每帧发
                        last_line = '<<X,%d,Y,%d>>' % (ix, iy)
                        _tx(last_line)
                        last_send = now
                    sending = True
                    if locked_n == args.lock_frames:
                        print('\n[重合] 颜色 %s 偏差 %s，开始发'
                              % (COLORS.get(best[1], '?'), last_line))
                else:
                    if now - last_send > tx_gap:          # 无目标也每帧发
                        last_line = '<<X,9999,Y,9999>>'
                        _tx(last_line)
                        last_send = now
            elif locked and locked_n >= args.lock_frames:
                if now - last_send > tx_gap:
                    _tx(args.msg.format(id=best[1],
                                        dx=tx0 - best[2][0], dy=ty0 - best[2][1]))
                    last_send = now
                sending = True

            # ---------------- 可视化（仅开窗口时）
            if show:
                warn = '  << 过曝' if m > 235 else ('  << 太暗' if m < 15 else '')
                cv2.putText(vis,
                            'fps %.1f obj %d mean %.0f%s TX %.1fHz | exp %.0f gam %.0f bri %.0f gain %.0f' %
                            (fps, len(boxes), m, warn, tx_hz, exp, gam, bri, gai),
                            (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                cv2.drawMarker(vis, (int(cx0), int(cy0)), (0, 255, 0),
                               cv2.MARKER_CROSS, 30, 2)
                cv2.drawMarker(vis, (int(tx0), int(ty0)), (0, 0, 255),
                               cv2.MARKER_TILTED_CROSS, 34, 2)
                cv2.line(vis, (int(cx0), int(cy0)), (int(tx0), int(ty0)),
                         (0, 0, 255), 1)
                cv2.putText(vis, 'TARGET -%.0fpx' % args.target_up,
                            (int(tx0) + 14, int(ty0) - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                if tracker is not None:
                    tg = tracker.target()
                    ttxt = 'task done' if tg is None else \
                           'goal %s(%d)  %s' % (TASK_NAMES.get(tg, '?'), tg,
                                                tracker.progress())
                    cv2.putText(vis, ttxt, (10, 122), cv2.FONT_HERSHEY_SIMPLEX,
                                0.62, (255, 255, 255) if tg is None else (0, 200, 255), 2)
                if best is not None:
                    d = best[0]
                    ax, ay = best[3]
                    col = (0, 255, 0) if locked else (0, 165, 255)
                    cv2.drawMarker(vis, (int(ax), int(ay)), col,
                                   cv2.MARKER_TILTED_CROSS, 26, 2)
                    cv2.circle(vis, (int(ax), int(ay)), 8, col, 2)
                    if locked:
                        cv2.circle(vis, (int(ax), int(ay)), int(args.tol),
                                   (0, 255, 0), 2)
                    txt = ('LOCK %s d=%.0fpx' % (COLORS.get(best[1], '?'), d)
                           if locked else 'track d=%.0fpx' % d)
                    if sending:
                        txt += '  >>> SEND'
                    cv2.putText(vis, txt + '  aim(%+.0f,%+.0f)' % (aim_x, aim_y),
                                (10, 98), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                                (0, 255, 0) if sending else
                                ((0, 255, 255) if locked else (200, 200, 200)), 2)
                base = vis.shape[0] - 12
                box_top = max(0, base - 108)
                cv2.rectangle(vis, (0, box_top),
                              (272, min(vis.shape[0] - 1, base + 8)), (0, 0, 0), -1)
                cv2.putText(vis, 'TX  %.1f Hz' % tx_hz, (10, base - 88),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                for i, _m in enumerate(reversed(tx_hist)):
                    _c = (0, 255, 255) if '9999' in _m else (0, 255, 0)
                    cv2.putText(vis, _m, (10, base - i * 22),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.62, _c, 2)
                cv2.imshow('live_detect_pi  (q quit)', vis)
                k = cv2.waitKey(1) & 0xFF
                if k in (ord('q'), 27):
                    break
                # ---- 视觉调参：全部独立小写单键
                # ★ 2026-11-24 第三轮定论：**运行时绝不碰曝光时间**。
                #   单独写 exposure_absolute 属于「半配置」，这颗 UVC 收到后
                #   会重新跑内部 3A 补偿 → 过曝 → 守护压回 → 循环振荡，
                #   表现就是「按几下 e 之后画面才开始跳」。固定曝光只在
                #   启动时由 EXP_FIXED 决定；现场明暗改用 brightness
                #   （ISP 后端数字量，立即生效、不触发重收敛）。
                # ★ _ctrl 只改 isp_target，再由 apply_isp() **整套**下发；
                #   删掉了 cap.set() 回退 —— 它对这颗 UVC 有害无益，
                #   会把已经关死的 AE 重新叫醒（守护与按键并发时尤其容易）。
                def _ctrl(name, val):
                    isp_target[name] = '%d' % int(val)
                    apply_isp(full=False)   # 高频路径：只压 core，不碰 3A

                tune_keys = (ord('e'), ord('r'), ord('m'), ord('n'),
                             ord('b'), ord('v'), ord('g'), ord('h'))
                if k == ord('e'):
                    bri = max(bri_min, bri - 10.0)          # 明暗(粗)
                    _ctrl('brightness', bri)
                elif k == ord('r'):
                    bri = min(bri_max, bri + 10.0)          # 明暗(粗)
                    _ctrl('brightness', bri)
                elif k == ord('m'):
                    gam = max(gam_min, gam - 20.0)
                    _ctrl('gamma', gam)
                elif k == ord('n'):
                    gam = min(gam_max, gam + 20.0)
                    _ctrl('gamma', gam)
                elif k == ord('b'):
                    bri = max(bri_min, bri - 5.0)           # 明暗(细)
                    _ctrl('brightness', bri)
                elif k == ord('v'):
                    bri = min(bri_max, bri + 5.0)           # 明暗(细)
                    _ctrl('brightness', bri)
                elif k == ord('g'):
                    gai = max(gai_min, gai - 10.0)
                    _ctrl('gain', gai)
                elif k == ord('h'):
                    gai = min(gai_max, gai + 10.0)
                    _ctrl('gain', gai)
                elif k == ord('w'):
                    apply_isp()          # 默认只重发 core，不碰 3A 附加控件
                    print('  核心曝光控件已重新下发（白平衡/背光保持固件原值）')
                if k in tune_keys:
                    tuned = True
                    print('  曝光(固定)=%.0f  GAM=%.0f  BRI=%.0f  GAIN=%.0f' %
                          (exp, gam, bri, gai))
                    remember()
                if k == ord('j'):
                    aim_x -= 2
                elif k == ord('l'):
                    aim_x += 2
                elif k == ord('i'):
                    aim_y -= 2
                elif k == ord('k'):
                    aim_y += 2
                if k in (ord('j'), ord('l'), ord('i'), ord('k')):
                    print('  标定点 dx=%+.0f dy=%+.0f' % (aim_x, aim_y))
                if k == ord('t'):
                    for cid, tc, bb, cf in votes.get('_tops', []):
                        print('  %s 顶心=(%.0f,%.0f)  dx=%+.0f dy=%+.0f px' %
                              (COLORS.get(cid, '?'), tc[0], tc[1],
                               tc[0] - vis.shape[1] / 2.0,
                               tc[1] - vis.shape[0] / 2.0))
                if k == ord('s'):
                    cv2.imwrite(args.out, vis)
                    lines = []
                    H, W = vis.shape[:2]
                    for (x0, y0, x1, y1, c) in boxes:
                        lines.append('0 %.6f %.6f %.6f %.6f' %
                                     ((x0 + x1) / 2 / W, (y0 + y1) / 2 / H,
                                      (x1 - x0) / W, (y1 - y0) / H))
                    Path(args.out).with_suffix('.txt').write_text(
                        '\n'.join(lines) + '\n', encoding='utf-8')
                    _save_aim(args.aim_file, aim_x, aim_y)
                    print('  已存 %s 与标签；标定点 → %s' % (args.out, args.aim_file))
            # 定时在终端打一行状态：开窗口时也打，方便直接读 FPS
            # （窗口状态行画在画面上，MobaXterm 里看不清，终端这行是权威值）
            if now - last_report >= args.period:
                last_report = now
                who = COLORS.get(best[1], '-') if best else '-'
                dist = '%.0f' % best[0] if best else '-'
                print('fps %5.1f  obj %d  mean %3.0f  goal %-3s  d %-5s  '
                      'TX %4.1fHz  %s' %
                      (fps, len(boxes), m, who, dist, tx_hz, last_line))
    except KeyboardInterrupt:
        print('\n收到 Ctrl+C，收工。')
    finally:
        # 先停守护线程，别在 cap.release() 之后还去写设备
        if guard_stop is not None:
            guard_stop.set()
        guard_pause[0] = True
        cap.release()
        lk.close()
        try:
            det.close()          # om 后端要释放 ACL 资源；其它后端是 no-op
        except Exception:
            pass
        if show:
            cv2.destroyAllWindows()
        try:
            if tuned:
                remember()
                print('相机参数已存 %s' % cfg.name)
            else:
                print('本次未调参，不改写 %s（避免把固件毒值写回）' % cfg.name)
        except Exception:
            pass
        print('共发出 %d 帧报文，失败 %d 次。' % (lk.sent, lk.errors))


if __name__ == '__main__':
    main()
