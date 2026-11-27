# -*- coding: utf-8 -*-
"""
live_detect.py —— 实时联调：相机 → YOLO 检框 → （可选）框内判色 → 屏幕上标出

用法：
    # 静态图先试（不必接相机）
    python live_detect.py --image ..\\sample_frame.png --weights ..\\best.pt

    # 接相机
    python live_detect.py --weights ..\\best.pt --cam 0

    # 有六色原型时，顺手把颜色也判出来
    python live_detect.py --weights ..\\best.pt --cam 0 --proto proto.json

屏幕上的记号：
    绿框 = 物料；框上写 置信度 /（有原型时）颜色名 + 后验概率
    左上角 = fps、本帧框数
快捷键：
    q 退出   s 存帧（存成 live_shot.jpg，附同名 .txt 标签，可直接拿去做数据）
"""

import argparse
import time
from pathlib import Path

import cv2
import numpy as np

from sixcolor import COLORS, Prototypes, classify, label_of_lab, roi_lab
from link import Link
from taskcode import NAMES as TASK_NAMES, Tracker, parse as parse_task
from top_center import material_mask, top_center_edge, top_center_from_mask


def read_image(path):
    return cv2.imdecode(np.fromfile(str(path), np.uint8), cv2.IMREAD_COLOR)


def _font(size):
    import os
    from PIL import ImageFont
    for name in ("msyh.ttc", "simhei.ttf", "simsun.ttc"):
        p = os.path.join(os.environ.get("WINDIR", r"C:\Windows"), "Fonts", name)
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, size)
            except Exception:
                pass
    return ImageFont.load_default()


def make_patches(size=22):
    """把六个汉字预渲成小牌子。

    cv2.putText 用 Hershey 字体，画不了汉字——先前"红 0.93"只剩"0.93"，
    便是这个缘故。故用 PIL 先渲好，再贴上去。
    """
    from PIL import Image, ImageDraw
    font = _font(size)
    out = {}
    for cid, name in COLORS.items():
        probe = Image.new("RGB", (8, 8))
        l, t, r, b = ImageDraw.Draw(probe).textbbox((0, 0), name, font=font)
        w, h = (r - l) + 8, (b - t) + 6
        im = Image.new("RGB", (max(w, 8), max(h, 8)), (0, 0, 0))
        ImageDraw.Draw(im).text((4 - l, 3 - t), name, font=font, fill=(0, 255, 0))
        out[cid] = cv2.cvtColor(np.array(im), cv2.COLOR_RGB2BGR)
    return out


_PATCHES = None


def draw(img, boxes, protos, votes=None):
    """boxes: [(x0,y0,x1,y1,conf)]。无原型时按 Lab 规则定名，有原型时走 ΔE。"""
    global _PATCHES
    if _PATCHES is None:
        _PATCHES = make_patches()

    for (x0, y0, x1, y1, conf) in boxes:
        # 先在"干净"的画面上取样与求顶心——画过框线、贴过牌子之后再取，便污了
        vec = roi_lab(img, (x0, y0, x1 - x0, y1 - y0), inset=0.30)
        cid = None
        if protos is not None and protos.ids():
            r = classify(vec, protos)
            if r["ok"]:
                cid = r["id"]
                votes[r["id"]] = votes.get(r["id"], 0) + 1
        else:
            cid = label_of_lab(vec)

        tc = estimate_top(img, (x0, y0, x1, y1))
        if tc is not None:
            votes.setdefault("_tops", []).append((cid, tc, (x0, y0, x1, y1), conf))

        cv2.rectangle(img, (x0, y0), (x1, y1), (0, 255, 0), 2)
        tx = x0 + 3
        if cid is not None and cid in _PATCHES:
            patch = _PATCHES[cid]
            ph, pw = patch.shape[:2]
            # 靠边时把牌子往回收，免得被画面裁掉、形状对不上；画面比牌子还小时按画面裁
            h_av = min(ph, img.shape[0])
            w_av = min(pw, img.shape[1])
            px = min(x0, max(0, img.shape[1] - w_av))
            py = y0 - ph if y0 - ph >= 0 else y0 + 2
            py = min(max(0, py), max(0, img.shape[0] - h_av))
            img[py:py + h_av, px:px + w_av] = patch[:h_av, :w_av]
            tx = px + w_av + 4
        cv2.putText(img, f"{conf:.2f}", (tx, max(14, y0 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        if tc is not None:
            cv2.drawMarker(img, (int(tc[0]), int(tc[1])), (0, 0, 255),
                           cv2.MARKER_CROSS, 16, 2)
    return img


def filter_boxes(boxes, max_obj=3, loose=1.9, tight=0.45):
    """剔误检，三招：

    ① 尺寸离群：框面积与中位相比，大出 1.9 倍或小到 0.45 倍以下的，剔掉。
       实拍里那只"浅蓝"被框了两次，多出的那个圈着阴影、面积大一倍，便是这样逮住的。
    ② 至多 max_obj 只：赛场上盘内最多三只，多出的按置信度从低往高砍。
    ③ 位置重合：两框中心相距不足较短边一半的，只留置信度高的那个。

    这些都在判色之前做，免得误检把颜色也带乱。
    """
    if not boxes:
        return boxes
    out = [list(b) for b in boxes]
    if len(out) >= 3:
        areas = sorted((b[2] - b[0]) * (b[3] - b[1]) for b in out)
        med = areas[len(areas) // 2]
        out = [b for b in out
               if tight * med <= (b[2] - b[0]) * (b[3] - b[1]) <= loose * med]
    # 位置重合
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
    """估一只料的顶面中心。

    先用亮度法；若它只能退回"猜"（prior），再动用边缘法（专治浅蓝那种低对比）；
    两者都不成，才用 0.34 框高的形状先验收底。
    """
    bh = box[3] - box[1]
    res = None
    got = material_mask(img, box)
    if got is not None:
        res = top_center_from_mask(img, got[0], got[1])

    if res is not None and res[2] != "prior":
        tx, ty = res[0], res[1]
        ry = (ty - box[1]) / max(1, bh)
        if 0.20 <= ry <= 0.50:
            return tx, ty

    e = top_center_edge(img, box)          # 亮度不可靠时的替补
    if e is not None and e[2] >= 200:
        return e[0], e[1]

    if res is not None:
        ry = (res[1] - box[1]) / max(1, bh)
        if 0.20 <= ry <= 0.50:
            return res[0], res[1]
    return (box[0] + box[2]) / 2.0, box[1] + 0.34 * bh


def main():
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description="实时检测联调")
    ap.add_argument("--weights", default=str(here.parent / "best.pt"))
    ap.add_argument("--cam", type=int, default=None)
    ap.add_argument("--image", default=None, help="用一张静态图试，不接相机")
    ap.add_argument("--proto", default=None, help="六色原型 proto.json（可选）")
    ap.add_argument("--res", default=None, help="分辨率，如 1280x720；不给则用上次存下的")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--out", default="live_shot.jpg")
    ap.add_argument("--tol", type=float, default=20.0, help="顶心与画面中心相差多少像素内算重合")
    ap.add_argument("--lock-frames", type=int, default=3, help="连续几帧重合才发信号")
    ap.add_argument("--motion-tol", type=float, default=8.0,
                    help="帧间位移小于此像素算静止；分辨率减半时应同步减半")
    ap.add_argument("--still-frames", type=int, default=5, help="连续几帧静止才判停稳")
    ap.add_argument("--link", default="print", choices=["print", "udp", "serial"])
    ap.add_argument("--udp-host", default="127.0.0.1")
    ap.add_argument("--udp-port", type=int, default=9000)
    ap.add_argument("--com", default="COM3")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--wire", default="offs", choices=["offs", "grab"],
                    help="offs=<<X,dx,Y,dy>> 偏移协议（默认）；grab=旧的自定义模板")
    ap.add_argument("--msg", default="GRAB {id}\n", help="wire=grab 时的内容模板，可用 {id} {dx} {dy}")
    ap.add_argument("--aim-dx", type=float, default=None, help="标定点相对顶心的 x 偏移（像素）")
    ap.add_argument("--aim-dy", type=float, default=None, help="标定点相对顶心的 y 偏移（像素，向下为正）")
    ap.add_argument("--aim-file", default=str(here / "aim_material.json"), help="偏移存盘处")
    ap.add_argument("--task", default=None, help="任务码，如 123 或 156+123；按序只盯当前该抓的那一色")
    ap.add_argument("--advance-gap", type=int, default=12, help="目标连续几帧看不见才算抓走")
    args = ap.parse_args()

    from ultralytics import YOLO
    model = YOLO(args.weights)
    protos = Prototypes.load(args.proto) if args.proto else None
    if protos:
        print("已载入原型：", ", ".join(protos.name_of(c) for c in protos.ids()))

    def detect(img):
        r = model.predict(img, imgsz=args.imgsz, conf=args.conf, verbose=False)[0]
        out = []
        for b in r.boxes:
            x0, y0, x1, y1 = [int(v) for v in b.xyxy[0].tolist()]
            out.append((x0, y0, x1, y1, float(b.conf[0])))
        return out

    # ---------------- 静态图模式
    if args.image:
        img = read_image(args.image)
        if img is None:
            raise SystemExit(f"读不到 {args.image}")
        t = time.time()
        boxes = detect(img)
        dt = (time.time() - t) * 1000
        votes = {}
        vis = draw(img.copy(), boxes, protos, votes)
        print(f"{Path(args.image).name}: {len(boxes)} 个框，{dt:.0f} ms")
        for b in boxes:
            print(f"   ({b[0]},{b[1]})-({b[2]},{b[3]})  conf={b[4]:.2f}")
        cv2.imencode(".jpg", vis)[1].tofile(str(here.parent / "_live_preview.jpg"))
        print("预览已存 _live_preview.jpg")
        return

    # ---------------- 相机模式
    if args.cam is None:
        ap.print_help()
        return

    # 上次的相机参数（曝光/亮度/增益/分辨率），存在工创\camera_settings.json
    cfg_path = here.parent / "camera_settings.json"

    def load_cfg():
        try:
            import json
            return json.loads(cfg_path.read_text(encoding="utf-8"))
        except Exception:
            return {}

    def save_cfg(d):
        try:
            import json
            cfg_path.write_text(json.dumps(d, ensure_ascii=False, indent=2), encoding="utf-8")
        except Exception as e:
            print("  参数存不下来：", e)

    cfg = load_cfg()
    res = args.res or cfg.get("res") or "1280x720"

    cap = cv2.VideoCapture(args.cam, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(args.cam)
    if not cap.isOpened():
        raise SystemExit("相机打不开")
    w, h = [int(v) for v in res.lower().split("x")]
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUY2"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_AUTO_WB, 0)

    print("q 退出   s 存帧   e 变暗 / E 变亮（调曝光值）   b/B 亮度   g/G 增益   w 锁白平衡")
    print("（绿框=物料。画面一片白就按 e，按几次即暗下来；太暗按 E）")

    if cfg:
        print(f"读到上次参数：{cfg}")
        for key, prop in (("exposure", cv2.CAP_PROP_EXPOSURE),
                          ("brightness", cv2.CAP_PROP_BRIGHTNESS),
                          ("gain", cv2.CAP_PROP_GAIN)):
            if key in cfg:
                cap.set(prop, float(cfg[key]))

    exp = cap.get(cv2.CAP_PROP_EXPOSURE)
    bri = cap.get(cv2.CAP_PROP_BRIGHTNESS)
    gai = cap.get(cv2.CAP_PROP_GAIN)
    print(f"初值：EXPOSURE={exp:.1f}  BRIGHTNESS={bri:.0f}  GAIN={gai:.0f}  分辨率 {w}x{h}")

    def remember():
        save_cfg({"cam": args.cam, "res": f"{w}x{h}",
                  "exposure": round(float(exp), 2),
                  "brightness": round(float(bri), 1),
                  "gain": round(float(gai), 1)})

    fps, prev = 0.0, time.time()
    prev_centers = None
    still = 0
    announced = False
    locked_n = 0
    last_send = 0.0

    def _load_aim(p):
        import json
        try:
            d = json.loads(Path(p).read_text(encoding="utf-8"))
            return float(d.get("dx", 0)), float(d.get("dy", 0))
        except Exception:
            return 0.0, 0.0

    def _save_aim(p, dx, dy):
        import json
        Path(p).write_text(json.dumps({"dx": dx, "dy": dy}, ensure_ascii=False, indent=2),
                           encoding="utf-8")

    aim_x, aim_y = _load_aim(args.aim_file)
    if args.aim_dx is not None:
        aim_x = args.aim_dx
    if args.aim_dy is not None:
        aim_y = args.aim_dy
    print(f"标定点偏移：dx={aim_x:+.0f}  dy={aim_y:+.0f} 像素（对准点 = 顶心 + 此偏移）")
    dup_t = 0.0
    lk = Link(args.link, host=args.udp_host, port=args.udp_port,
              com=args.com, baud=args.baud)
    if args.wire == "offs":
        print(f"信号通道：{lk.describe()}   协议 <<X,dx,Y,dy>>（无目标发 <<X,9999,Y,9999>>）")
    else:
        print(f"信号通道：{lk.describe()}   模板 {args.msg!r}")
    print(f"容差 {args.tol:.0f}px   连 {args.lock_frames} 帧才发")

    tracker = None
    if args.task:
        groups = parse_task(args.task)
        tracker = Tracker(groups, advance_when_gone=args.advance_gap)
        seq = tracker.seq
        print("任务码：" + " → ".join(f"{TASK_NAMES[d]}({d})" for d in seq)
              + f"　共 {len(seq)} 只，当前第 1 只")
        if len(groups) > 1:
            print(f"  （给了 {len(groups)} 批，本程序只跑第 1 批；换批请重开并改 --task）")
    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("读不到帧，退出。")
            break
        m = float(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).mean())
        boxes = detect(frame)
        boxes = filter_boxes(boxes, max_obj=3)
        centers = [((b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0) for b in boxes]
        votes = {}
        vis = draw(frame, boxes, protos, votes)

        # ---------------- 转盘停稳判定：看检出中心的帧间位移
        # 转起来时（6~10 秒一圈）每帧挪二三十像素，停下不到一像素，极好分。
        motion = None
        if prev_centers is not None and centers and len(centers) == len(prev_centers):
            disps = sorted(min(((c[0]-p[0])**2 + (c[1]-p[1])**2) ** 0.5 for p in prev_centers)
                           for c in centers)
            motion = disps[len(disps) // 2]        # 中位数，免受个别坏匹配牵连
            # 门槛 8 像素：框本身有 ±5 像素抖动，取 3 会永远判"在动"
            if motion < args.motion_tol:
                still += 1
            else:
                still = 0
                announced = False
        else:
            still = 0
            announced = False
        prev_centers = centers

        ready = still >= args.still_frames
        if ready and not announced:
            announced = True
            print(f"\n[停稳] 检出 {len(boxes)} 只，位移 {motion:.1f} 像素")
            for (x0, y0, x1, y1, c) in boxes:
                print(f"   ({x0},{y0})-({x1},{y1})  conf={c:.2f}")

        now = time.time()
        dt = now - prev
        prev = now
        if dt > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / dt) if fps else 1.0 / dt

        # ---------------- 顶心与画面中心的重合判定 → 发信号
        cx0, cy0 = vis.shape[1] / 2.0, vis.shape[0] / 2.0
        best = None
        tops = votes.get("_tops", [])
        if tracker is not None:
            target = tracker.target()
            if target is None:
                # 任务码跑完了：不再跟踪，也不再发
                tops = []
            else:
                seen = [cid for cid, _tc, _bb, _cf in tops if cid is not None]
                if tracker.observe(seen):
                    nt = tracker.target()
                    if nt is None:
                        print("\n[任务完成] 三只都已抓过")
                    else:
                        print(f"\n[轮到下一只] 目标 {TASK_NAMES.get(nt, '?')}({nt})　"
                              f"进度 {tracker.progress()}")
                if tracker.gap == 30 and tracker.target() is not None:
                    print(f"[提示] 目标 {TASK_NAMES.get(tracker.target(), '?')} 已 30 帧未见"
                          f"——可能被遮挡、不在盘上，或等转盘转到面前")
                tops = [t for t in tops if t[0] == tracker.target()]

        for cid, tc, bb, cf in tops:
            # 标定点 = 顶心 + 偏移；对准它，爪子才落在料上（相机不在爪子正中）
            ax, ay = tc[0] + aim_x, tc[1] + aim_y
            d = ((ax - cx0) ** 2 + (ay - cy0) ** 2) ** 0.5
            if best is None or d < best[0]:
                best = (d, cid, tc, (ax, ay))

        # 几何与颜色互校：盘上三只既成等边，颜色必是三只不同；若两只判成同色，
        # 说明有一只判错（或 YOLO 把一只拆成两个框），此时一个信号都不发。
        dup_warn = False
        if len(tops) == 3 and len({t[0] for t in tops}) < 3:
            ids = [t[0] for t in tops]
            dup_warn = True
            if now - dup_t > 2.0:
                dup_t = now
                print(f"[警告] 三只料判出重复颜色 {[COLORS.get(i, '?') for i in ids]}——"
                      f"必有一只判错，暂停发信号")
            best = None

        locked = best is not None and best[0] <= args.tol
        if locked:
            locked_n += 1
        else:
            locked_n = 0

        sending = False
        if args.wire == "offs":
            # 协议：<<X,±dx,Y,±dy>>；无目标 <<X,9999,Y,9999>>
            # 差值一律取"标定点 - 画面中心"，四舍五入成整数，不带小数点
            ok_now = locked and locked_n >= args.lock_frames and ready
            if ok_now:
                ix = int(round(best[3][0] - cx0))
                iy = int(round(best[3][1] - cy0))
                if now - last_send > 0.05:             # 约 20Hz
                    lk.send(f"<<X,{ix},Y,{iy}>>")
                    last_send = now
                sending = True
                if locked_n == args.lock_frames:
                    print(f"\n[重合] 颜色 {COLORS.get(best[1], '?')} 偏差 "
                          f"<<X,{ix},Y,{iy}>>，开始发")
            else:
                if now - last_send > 0.10:             # 无目标约 10Hz
                    lk.send("<<X,9999,Y,9999>>")
                    last_send = now
        elif locked and locked_n >= args.lock_frames and ready:
            if now - last_send > 0.05:                 # 约 20Hz 上限，别把串口灌满
                lk.send(args.msg.format(id=best[1],
                                        dx=best[2][0] - cx0, dy=best[2][1] - cy0))
                last_send = now
            sending = True
            if locked_n == args.lock_frames:
                print(f"\n[重合] 颜色 {COLORS.get(best[1], '?')} 距中心 {best[0]:.0f}px，开始发信号")

        warn = "  << 过曝" if m > 235 else ("  << 太暗" if m < 15 else "")
        cv2.putText(vis, f"fps {fps:.1f}  obj {len(boxes)}  exp {exp:.0f} bri {bri:.0f} gain {gai:.0f} mean {m:.0f}{warn}",
                    (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        # 注：OpenCV 是 BGR，(0,0,255)=红，(0,255,255)=黄，(0,255,0)=绿
        mo = "-" if motion is None else f"{motion:.0f}"
        state = "STOPPED" if ready else ("MOVING" if still == 0 else f"settling {still}/{args.still_frames}")
        cv2.putText(vis, f"motion {mo}px  {state}",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0) if ready else (0, 255, 255), 2)

        # 相机中心：绿十字
        cv2.drawMarker(vis, (int(cx0), int(cy0)), (0, 255, 0), cv2.MARKER_CROSS, 30, 2)
        if tracker is not None:
            tg = tracker.target()
            if tg is None:
                ttxt = "task done"
            else:
                ttxt = "goal %s(%d)  %s" % (TASK_NAMES.get(tg, "?"), tg, tracker.progress())
            cv2.putText(vis, ttxt, (10, 122), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                        (255, 255, 255) if tg is None else (0, 200, 255), 2)
        if best is not None:
            d = best[0]
            ax, ay = best[3]
            col = (0, 255, 0) if locked else (0, 165, 255)
            cv2.drawMarker(vis, (int(ax), int(ay)), col, cv2.MARKER_TILTED_CROSS, 26, 2)
            cv2.circle(vis, (int(ax), int(ay)), 8, col, 2)
            if locked:
                cv2.circle(vis, (int(ax), int(ay)), int(args.tol), (0, 255, 0), 2)
            txt = (f"LOCK {COLORS.get(best[1], '?')} d={d:.0f}px" if locked
                   else f"track d={d:.0f}px")
            if sending:
                txt += "  >>> SEND"
            cv2.putText(vis, txt + f"  aim({aim_x:+.0f},{aim_y:+.0f})", (10, 98),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                        (0, 255, 0) if sending else ((0, 255, 255) if locked else (200, 200, 200)), 2)
        cv2.imshow("live_detect  (q quit)", vis)
        k = cv2.waitKey(1) & 0xFF
        if k in (ord("q"), 27):
            break
        elif k == ord("e"):
            exp -= 1.0
            cap.set(cv2.CAP_PROP_EXPOSURE, exp)
        elif k == ord("E"):
            exp += 1.0
            cap.set(cv2.CAP_PROP_EXPOSURE, exp)
        elif k == ord("b"):
            bri = max(0.0, bri - 5.0)
            cap.set(cv2.CAP_PROP_BRIGHTNESS, bri)
        elif k == ord("B"):
            bri = min(255.0, bri + 5.0)
            cap.set(cv2.CAP_PROP_BRIGHTNESS, bri)
        elif k == ord("g"):
            gai = max(0.0, gai - 10.0)
            cap.set(cv2.CAP_PROP_GAIN, gai)
        elif k == ord("G"):
            gai = min(255.0, gai + 10.0)
            cap.set(cv2.CAP_PROP_GAIN, gai)
        elif k == ord("w"):
            cap.set(cv2.CAP_PROP_AUTO_WB, 0)
            print("  白平衡已锁定")
        elif k == ord("j"):
            aim_x -= 2
            print(f"  标定点 dx={aim_x:+.0f} dy={aim_y:+.0f}")
        elif k == ord("l"):
            aim_x += 2
            print(f"  标定点 dx={aim_x:+.0f} dy={aim_y:+.0f}")
        elif k == ord("i"):
            aim_y -= 2
            print(f"  标定点 dx={aim_x:+.0f} dy={aim_y:+.0f}")
        elif k == ord("k"):
            aim_y += 2
            print(f"  标定点 dx={aim_x:+.0f} dy={aim_y:+.0f}")
        if k == ord("t"):
            tops = votes.get("_tops", [])
            if not tops:
                print("  本帧没有可用的顶心")
            for cid, tc, bb, cf in tops:
                dx = tc[0] - vis.shape[1] / 2.0
                dy = tc[1] - vis.shape[0] / 2.0
                print(f"  {COLORS.get(cid, '?'):<3} 顶心=({tc[0]:.0f},{tc[1]:.0f})"
                      f"  dx={dx:+.0f} dy={dy:+.0f} px")
        if k in (ord("e"), ord("E"), ord("b"), ord("B"), ord("g"), ord("G")):
            remember()                      # 每调一次便落盘，万一强退也不丢
        if k == ord("s"):
            cv2.imwrite(args.out, vis)
            lines = []
            H, W = vis.shape[:2]
            for (x0, y0, x1, y1, c) in boxes:
                lines.append(f"0 {(x0+x1)/2/W:.6f} {(y0+y1)/2/H:.6f} {(x1-x0)/W:.6f} {(y1-y0)/H:.6f}")
            Path(args.out).with_suffix(".txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
            _save_aim(args.aim_file, aim_x, aim_y)
            print(f"  已存 {args.out} 与标签；标定点 dx={aim_x:+.0f} dy={aim_y:+.0f} → {args.aim_file}")

    remember()
    print(f"参数已存 {cfg_path.name}：EXPOSURE={exp:.1f} BRIGHTNESS={bri:.0f} GAIN={gai:.0f}（下次启动自动套用）")
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
