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

想在香橙派 HDMI 显示器上看画面：
    export DISPLAY=:0
    SHOW=1 bash run_task1.sh
想通过 MobaXterm 把画面传回 Windows：
    用带 X11 转发的 SSH 重新连上（DISPLAY 是 localhost:10.0 之类），SHOW=1 即可

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


def estimate_top(img, box):
    """估一只料的顶面中心（亮度法 → 边缘法 → 0.34 框高兜底）。与 PC 版一致。"""
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

    e = top_center_edge(img, box)
    if e is not None and e[2] >= 200:
        return e[0], e[1]

    if res is not None:
        ry = (res[1] - box[1]) / max(1, bh)
        if 0.20 <= ry <= 0.50:
            return res[0], res[1]
    return (box[0] + box[2]) / 2.0, box[1] + 0.34 * bh


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

    # ---------------- 打开相机（V4L2 + MJPG）
    cam = args.cam
    if isinstance(cam, str) and cam.isdigit():
        cam = int(cam)
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
            '       换号：   --cam 0 / 1 / 3   或  --cam /dev/video2\n'
            '       查权限： sudo usermod -aG video $USER 然后重新登录' % cam)

    w, h = [int(v) for v in args.res.lower().split('x')]
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    # UVC 只给 1 个 buffer 时，MJPG 变长帧 + 主机读取稍慢就会让驱动停流，
    # 表现就是 cap_v4l.cpp 的 select() timeout、然后"读不到帧"。给 4 个缓冲更稳。
    bufsize = getattr(cv2, 'CAP_PROP_BUFFERSIZE', None)
    if bufsize is not None:
        try:
            cap.set(bufsize, 4)
        except Exception:
            pass
    rw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    rh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    got_fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc_s = ''.join(chr((got_fourcc >> (8 * i)) & 0xFF) for i in range(4))
    print('相机 %s  实际 %dx%d  fourcc=%s  目标 %s' %
          (cam, rw, rh, fourcc_s, args.fourcc))

    # ---------------- V4L2 曝光控件（接键鼠后可用按键现场调）
    # 坑（2026-11-22 板上实测）：这颗 UVC 摄像头(05a3:9230) 在手动曝光被设到
    # 某些值（实测 80）后会直接停止出流 → cap_v4l select() timeout。更麻烦的是
    # 该值会**持久写进摄像头固件**，下次 open() 用 cap.get() 仍能读回来，
    # 于是「删 camera_settings.json 也没用」：旧代码启动会把它重新 set 回驱动，
    # 退出时 remember() 又写回 json，形成死循环，之后每次启动都秒挂。
    #
    # 因此默认 EXP_AUTO=1：启动时**只让 AE 接管、不下发任何手动值**，
    # 由此冲掉固件里的坏值；只有用户真的按了 e/r 这类曝光键才切手动并设值。
    # gamma / 亮度 / 增益是独立控件，AE 开着也能调，不必切手动（更安全）。
    # 确认某组值安全、想每次自动应用时，用 EXP_AUTO=0。
    # EXP_AUTO_VAL 写进 V4L2_CID_EXPOSURE_AUTO：3 = aperture priority
    # （UVC 上等价于 AE 开，兼容性最好）；个别摄像头要 0，用环境变量覆盖。
    exp_auto = os.environ.get('EXP_AUTO', '1') != '0'
    exp_auto_val = float(os.environ.get('EXP_AUTO_VAL', '3'))
    exp_manual_val = float(os.environ.get('EXP_MANUAL_VAL', '1'))
    # 2026-11-22 板上实测：这颗 UVC(05a3:9230) 的 exposure_absolute=80 会
    # 直接停流，而旧版 e 键是 exp *= 0.8 —— 从默认 100 按一下正好落到 80，
    # 这就是「按 e 之前 27 帧、按一下就 select() timeout」的真正原因。
    # 因此按键时把曝光钳在 [EXP_MIN, EXP_MAX]，并改成固定 ±20 步进（可预测）。
    exp_min = float(os.environ.get('EXP_MIN', '100'))
    exp_max = float(os.environ.get('EXP_MAX', '5000'))
    tuned = False          # 只有真的调过参才在退出时写盘，避免把毒值写回 json
    cfg = here / 'camera_settings.json'

    def _get(prop, default):
        v = cap.get(prop)
        return float(v) if v is not None and v > 0 else float(default)

    exp = _get(cv2.CAP_PROP_EXPOSURE, 100.0)
    gam = _get(cv2.CAP_PROP_GAMMA, 100.0)
    bri = _get(cv2.CAP_PROP_BRIGHTNESS, 0.0)
    gai = _get(cv2.CAP_PROP_GAIN, 0.0)

    # 读入保存值仅作为按键起点，不再自动下发到驱动
    try:
        import json
        d = json.loads(cfg.read_text(encoding='utf-8'))
        exp = float(d.get('exposure', exp))
        gam = float(d.get('gamma', gam))
        bri = float(d.get('brightness', bri))
        gai = float(d.get('gain', gai))
        # PC 版存的是 Windows 的 EV 值（如 -8），别把它喂给 V4L2
        if exp <= 0 or exp > 5000:
            exp = 100.0
        if gam <= 0:
            gam = 100.0
    except Exception:
        pass

    if exp_auto:
        try:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, exp_auto_val)   # AE 接管，冲掉坏值
        except Exception:
            pass
    else:
        try:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, exp_manual_val)
            for prop, val in ((cv2.CAP_PROP_EXPOSURE, exp),
                              (cv2.CAP_PROP_GAMMA, gam),
                              (cv2.CAP_PROP_BRIGHTNESS, bri),
                              (cv2.CAP_PROP_GAIN, gai)):
                cap.set(prop, val)
        except Exception:
            pass

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

    print('初值 EXP=%.0f  GAM=%.0f  BRI=%.0f  GAIN=%.0f  [%s]' %
          (exp, gam, bri, gai,
           '启动不接管曝光(AE)' if exp_auto else '启动应用保存的手动值'))
    if show:
        print('q 退出   s 存帧   t 打印顶心   j/l/i/k 挪标定点')
        print('e/r 曝光   m/n gamma   b/v 亮度   g/h 增益   w 锁白平衡')
        print('（全小写单键，不用 Shift；每组左键减小、右键增大）')
        print('压过曝优先用 b/v 亮度、m/n gamma —— 不碰曝光时间，UVC 不会停流')
        print('曝光已钳在 %.0f~%.0f 避开固件毒值；仅按 e/r 才切手动曝光' %
              (exp_min, exp_max))
        print('  救活： v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto=3')
        print('        或重插 USB 摄像头后重启本程序（默认 EXP_AUTO=1 会自愈）')

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
                    try:
                        cap.release()
                    except Exception:
                        pass
                    cap = cv2.VideoCapture(cam, v4l2)
                    if not cap.isOpened():
                        cap = cv2.VideoCapture(cam)
                    cap.set(cv2.CAP_PROP_FOURCC,
                            cv2.VideoWriter_fourcc(*args.fourcc))
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
                    cap.set(cv2.CAP_PROP_FPS, args.fps)
                    try:
                        # 重开时让 AE 接管，冲掉可能把摄像头搞停流的手动曝光值
                        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, exp_auto_val)
                    except Exception:
                        pass
                    if bufsize is not None:
                        try:
                            cap.set(bufsize, 4)
                        except Exception:
                            pass
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
                    if now - last_send > 0.05:            # 约 20Hz
                        last_line = '<<X,%d,Y,%d>>' % (ix, iy)
                        _tx(last_line)
                        last_send = now
                    sending = True
                    if locked_n == args.lock_frames:
                        print('\n[重合] 颜色 %s 偏差 %s，开始发'
                              % (COLORS.get(best[1], '?'), last_line))
                else:
                    if now - last_send > 0.10:            # 无目标约 10Hz
                        last_line = '<<X,9999,Y,9999>>'
                        _tx(last_line)
                        last_send = now
            elif locked and locked_n >= args.lock_frames:
                if now - last_send > 0.05:
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
                # ---- 视觉调参：全部改成独立小写单键，不再依赖 Shift+大写
                #   （这块板子 X11 + cv2.waitKey 对 Shift+字母返回不可靠）
                #   每组：左键减小 / 右键增大
                #   e/r 曝光   m/n gamma   b/v 亮度   g/h 增益   w 锁白平衡
                tune_keys = (ord('e'), ord('r'), ord('m'), ord('n'),
                             ord('b'), ord('v'), ord('g'), ord('h'))
                # 只有曝光键才切手动曝光；其余控件 AE 开着也能设，不碰 sensor
                if k in (ord('e'), ord('r')):
                    try:
                        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, exp_manual_val)
                    except Exception:
                        pass
                if k == ord('e'):                   # 曝光变暗
                    exp = max(exp_min, exp - 20.0)
                    cap.set(cv2.CAP_PROP_EXPOSURE, exp)
                elif k == ord('r'):                 # 曝光变亮
                    exp = min(exp_max, exp + 20.0)
                    cap.set(cv2.CAP_PROP_EXPOSURE, exp)
                elif k == ord('m'):
                    gam = max(50.0, gam - 20.0)
                    cap.set(cv2.CAP_PROP_GAMMA, gam)
                elif k == ord('n'):
                    gam = min(300.0, gam + 20.0)
                    cap.set(cv2.CAP_PROP_GAMMA, gam)
                elif k == ord('b'):
                    bri = max(-64.0, bri - 5.0)
                    cap.set(cv2.CAP_PROP_BRIGHTNESS, bri)
                elif k == ord('v'):
                    bri = min(64.0, bri + 5.0)
                    cap.set(cv2.CAP_PROP_BRIGHTNESS, bri)
                elif k == ord('g'):
                    gai = max(0.0, gai - 10.0)
                    cap.set(cv2.CAP_PROP_GAIN, gai)
                elif k == ord('h'):
                    gai = min(255.0, gai + 10.0)
                    cap.set(cv2.CAP_PROP_GAIN, gai)
                elif k == ord('w'):
                    cap.set(cv2.CAP_PROP_AUTO_WB, 0)
                    print('  白平衡已锁定')
                if k in tune_keys:
                    tuned = True
                    print('  EXP=%.0f  GAM=%.0f  BRI=%.0f  GAIN=%.0f' %
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
