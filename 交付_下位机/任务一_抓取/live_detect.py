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
    """把"编号 + 颜色名"预渲成小牌子，如 "4 绿"。

    cv2.putText 用 Hershey 字体，画不了汉字——先前"红 0.93"只剩"0.93"，
    便是这个缘故。故用 PIL 先渲好，再贴上去。
    前面带上 1~6 的编号：编号是判色的结果本身，屏上、console、报文里统一用这个数。
    """
    from PIL import Image, ImageDraw
    font = _font(size)
    out = {}
    for cid, name in COLORS.items():
        label = "%d %s" % (cid, name)
        probe = Image.new("RGB", (8, 8))
        l, t, r, b = ImageDraw.Draw(probe).textbbox((0, 0), label, font=font)
        w, h = (r - l) + 8, (b - t) + 6
        im = Image.new("RGB", (max(w, 8), max(h, 8)), (0, 0, 0))
        ImageDraw.Draw(im).text((4 - l, 3 - t), label, font=font, fill=(0, 255, 0))
        out[cid] = cv2.cvtColor(np.array(im), cv2.COLOR_RGB2BGR)
    return out


_PATCHES = None


def parse_cmd(payload):
    """解析下位机发来的一条命令。

    返回：
        ("set", groups)    设定目标；groups 为空表示清空目标（继续发 9999）
        ("pause", None)    停止追踪：一条报文都不再发，画面照常预览
        ("resume", None)   解除停止，按当前目标继续发
        None               不认识，忽略（下次照旧用原目标）

        color 3        → ("set", [[3]])         只抓 3 号色
        task 156+123   → ("set", parse_task)    换整串任务码
        color 0        → ("set", [])            清空目标，但仍发 9999
        stop / pause   → ("pause", None)        完全静默
        resume / go    → ("resume", None)

    注意 stop 与 color 0 不是一回事：
        color 0  还在发 <<X,9999,Y,9999>>，下位机知道"没目标"；
        stop     一条都不发，下位机连"没目标"都收不到，会停在原地等。
    分隔符宽容：逗号、分号、空格、Tab 都当分隔；大小写不敏感。
    """
    parts = payload.strip().replace(",", " ").replace(";", " ").split()
    if not parts:
        return None
    head = parts[0].lower()
    arg = " ".join(parts[1:]).strip()

    if head in ("stop", "pause", "halt"):
        return ("pause", None)
    if head in ("resume", "go", "run"):
        return ("resume", None)
    if head == "color":
        if not arg.isdigit():
            return None
        n = int(arg)
        if n == 0:
            return ("set", [])
        return ("set", [[n]]) if 1 <= n <= 6 else None
    if head == "task":
        code = arg.replace(" ", "")
        if not code or set(code) <= {"0", "+"}:
            return ("set", [])
        try:
            return ("set", parse_task(code))
        except Exception:
            return None
    return None


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
    ap.add_argument("--tol", type=float, default=20.0, help="顶心与目标点相差多少像素内算重合")
    ap.add_argument("--target-up", type=float, default=100.0,
                    help="目标点相对画面中心上移多少像素（屏上红叉；偏差与对准都以它为基准）")
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
    ap.add_argument("--no-cmd", action="store_true",
                    help="不接收串口命令（默认接收下位机发来的 <<color N>> / <<task ...>>）")
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

    print("q 退出   s 存帧   e 变暗 / E 变亮（调曝光值）   m 变暗 / M 变亮（调 gamma）")
    print("b/B 亮度   g/G 增益   w 锁白平衡")
    print("（绿框=物料。曝光一动就过曝/发黑时，把曝光压在 -8 保 30fps，改用 m/M 调 gamma）")

    if cfg:
        print(f"读到上次参数：{cfg}")
        for key, prop in (("exposure", cv2.CAP_PROP_EXPOSURE),
                          ("brightness", cv2.CAP_PROP_BRIGHTNESS),
                          ("gamma", cv2.CAP_PROP_GAMMA),
                          ("gain", cv2.CAP_PROP_GAIN)):
            if key in cfg:
                cap.set(prop, float(cfg[key]))

    exp = cap.get(cv2.CAP_PROP_EXPOSURE)
    bri = cap.get(cv2.CAP_PROP_BRIGHTNESS)
    gai = cap.get(cv2.CAP_PROP_GAIN)
    gam = cap.get(cv2.CAP_PROP_GAMMA)
    if gam is None or gam < 0:          # 驱动没暴露 gamma 时给个安全默认
        gam = 100.0
    print(f"初值：EXPOSURE={exp:.1f}  BRIGHTNESS={bri:.0f}  GAMMA={gam:.0f}  "
          f"GAIN={gai:.0f}  分辨率 {w}x{h}")

    def remember():
        save_cfg({"cam": args.cam, "res": f"{w}x{h}",
                  "exposure": round(float(exp), 2),
                  "brightness": round(float(bri), 1),
                  "gamma": round(float(gam), 1),
                  "gain": round(float(gai), 1)})

    fps, prev = 0.0, time.time()
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
    if args.no_cmd or args.link != "serial":
        print("串口命令：未启用（要收命令需 --link serial 且不加 --no-cmd）")
    else:
        print("串口命令：<<color 3>> 换单色   <<task 156+123>> 换整串   <<color 0>> 清空目标")
        print("          <<stop>> 停止追踪（一条都不发）   <<resume>> 恢复")
    print(f"目标点 = 画面中心上移 {args.target_up:.0f}px（屏上红叉），偏差与对准均以它为准")

    # ---- 屏上显示真正发出去的原始报文（最近 4 帧 + 实际发送速率）
    tx_hist = []
    tx_n, tx_t0, tx_hz = 0, time.time(), 0.0
    paused = False          # <<stop>> / <<pause>> 置 True：一条报文都不发，只留预览

    def _tx(msg):
        nonlocal tx_n, tx_t0, tx_hz
        # 拦截点放在这里：两条 wire 分支、以及无目标时的 9999，全都要经过 _tx，
        # 所以只在这一处挡就够，不必去每个分支各加一遍判断。
        if paused:
            return
        lk.send(msg)
        tx_hist.append(msg)
        if len(tx_hist) > 4:
            del tx_hist[0]
        tx_n += 1
        el = time.time() - tx_t0
        if el >= 1.0:
            tx_hz = tx_n / el
            tx_n, tx_t0 = 0, time.time()

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

        now = time.time()
        dt = now - prev
        prev = now
        if dt > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / dt) if fps else 1.0 / dt

        # ---------------- 串口命令：下位机随时可换目标
        # 放在判定之前，这样本帧就能按新目标过滤，不用等下一帧
        for _cmd in (() if args.no_cmd else lk.poll()):
            _r = parse_cmd(_cmd)
            if _r is None:
                print(f"  [命令] 看不懂，忽略：<<{_cmd}>>")
                continue
            _act, _g = _r
            if _act == "pause":
                paused = True
                print(f"\n[命令] <<{_cmd}>>  已停止追踪：一条报文都不再发"
                      f"（<<resume>> 恢复，<<color N>> 也会顺带恢复）")
                continue
            if _act == "resume":
                paused = False
                print(f"\n[命令] <<{_cmd}>>  已恢复发送")
                continue
            # 设定目标
            tracker = Tracker(_g, advance_when_gone=args.advance_gap)
            locked_n = 0                       # 换目标必须清计数，否则会误判"已连续锁定"
            if paused:
                paused = False                 # 给了新目标就当作"顺便恢复"
                print("\n[命令] 顺带解除停止追踪")
            if not _g:
                print(f"\n[命令] <<{_cmd}>>  目标已清空，持续发 9999")
            else:
                _d = " / ".join(" ".join(f"{TASK_NAMES[c]}({c})" for c in g) for g in _g)
                print(f"\n[命令] <<{_cmd}>>  新目标：{_d}")
                if len(_g) > 1:
                    print(f"  （给了 {len(_g)} 批，本程序只跑第 1 批）")

        # ---------------- 顶心与画面中心的重合判定 → 发信号
        cx0, cy0 = vis.shape[1] / 2.0, vis.shape[0] / 2.0
        # 目标点（红叉）= 画面中心上移 --target-up 像素。爪子不在光轴上时对准它
        tx0, ty0 = cx0, cy0 - args.target_up
        best = None
        tops = votes.get("_tops", [])
        if tracker is not None:
            target = tracker.target()
            if target is None:
                tops = []
            else:
                # 已去掉"抓完自动跳到下一只"：不再调用 observe，永远只盯任务码第一位颜色
                tops = [t for t in tops if t[0] == target]

        for cid, tc, bb, cf in tops:
            # 标定点 = 顶心 + 偏移；对准它，爪子才落在料上（相机不在爪子正中）
            ax, ay = tc[0] + aim_x, tc[1] + aim_y
            d = ((ax - tx0) ** 2 + (ay - ty0) ** 2) ** 0.5
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
            # 差值一律取"目标点(红叉) - 标定点"，四舍五入成整数，不带小数点
            ok_now = locked and locked_n >= args.lock_frames
            if ok_now:
                # 符号已反转：改为"目标点 - 标定点"，即下位机需要移动的量
                # dx>0 → 目标在红叉左侧，要往左走 dx 像素才对准
                # dy>0 → 目标在红叉上方，要往上走 dy 像素才对准
                ix = int(round(tx0 - best[3][0]))
                iy = int(round(ty0 - best[3][1]))
                if now - last_send > 0.05:             # 约 20Hz
                    _tx(f"<<X,{ix},Y,{iy}>>")
                    last_send = now
                sending = True
                if locked_n == args.lock_frames:
                    print(f"\n[重合] 颜色 {COLORS.get(best[1], '?')} 偏差 "
                          f"<<X,{ix},Y,{iy}>>，开始发")
            else:
                if now - last_send > 0.10:             # 无目标约 10Hz
                    _tx("<<X,9999,Y,9999>>")
                    last_send = now
        elif locked and locked_n >= args.lock_frames:
            if now - last_send > 0.05:                 # 约 20Hz 上限，别把串口灌满
                _tx(args.msg.format(id=best[1],
                                    dx=tx0 - best[2][0], dy=ty0 - best[2][1]))
                last_send = now
            sending = True
            if locked_n == args.lock_frames:
                if not paused:
                    print(f"\n[重合] 颜色 {COLORS.get(best[1], '?')} 距中心 {best[0]:.0f}px，开始发信号")

        if paused:
            sending = False        # 暂停期间一条都没发，别让屏上还显示 >>> SEND

        warn = "  << OVER" if m > 235 else ("  << DARK" if m < 15 else "")
        cv2.putText(vis, f"fps {fps:.1f}  obj {len(boxes)}  exp {exp:.0f} gam {gam:.0f} bri {bri:.0f} gain {gai:.0f} mean {m:.0f}{warn}",
                    (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        # 注：OpenCV 是 BGR，(0,0,255)=红，(0,255,255)=黄，(0,255,0)=绿

        # 物料编号图例。编号即判色结果：1红 2黄 3蓝 4绿 5黑 6浅蓝。
        # 屏上只能用英文——Hershey 字体没有汉字字形，塞中文进去会变成问号。
        cv2.putText(vis, "MAT  1=red 2=yellow 3=blue 4=green 5=black 6=cyan",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
        if paused:      # 停止追踪要一眼能看出来，否则会以为程序卡了
            cv2.rectangle(vis, (6, 58), (216, 86), (0, 0, 0), -1)
            cv2.putText(vis, "PAUSED - no TX", (12, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.66, (0, 0, 255), 2)

        # 相机中心：绿十字（仅作参考）
        cv2.drawMarker(vis, (int(cx0), int(cy0)), (0, 255, 0), cv2.MARKER_CROSS, 30, 2)
        # 目标点：红十字（中心上移 target_up 像素）——偏差与对准都以它为准
        cv2.drawMarker(vis, (int(tx0), int(ty0)), (0, 0, 255),
                       cv2.MARKER_TILTED_CROSS, 34, 2)
        cv2.line(vis, (int(cx0), int(cy0)), (int(tx0), int(ty0)), (0, 0, 255), 1)
        cv2.putText(vis, f"TARGET -{args.target_up:.0f}px",
                    (int(tx0) + 14, int(ty0) - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        if tracker is not None:
            tg = tracker.target()
            if tg is None:
                ttxt = "task done"
            else:
                ttxt = "goal #%d  %s" % (tg, tracker.progress())
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
            _cid = "?" if best[1] is None else str(best[1])
            txt = (f"LOCK #{_cid} d={d:.0f}px" if locked
                   else f"track #{_cid} d={d:.0f}px")
            if sending:
                txt += "  >>> SEND"
            cv2.putText(vis, txt + f"  aim({aim_x:+.0f},{aim_y:+.0f})", (10, 98),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                        (0, 255, 0) if sending else ((0, 255, 255) if locked else (200, 200, 200)), 2)
        # ---- 左下角：真正发出去的原始报文（最新一帧在最下面）
        base = vis.shape[0] - 12
        box_top = max(0, base - 108)
        cv2.rectangle(vis, (0, box_top), (272, min(vis.shape[0] - 1, base + 8)),
                      (0, 0, 0), -1)
        cv2.putText(vis, "TX  PAUSED" if paused else f"TX  {tx_hz:.1f} Hz",
                    (10, base - 88), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 0, 255) if paused else (255, 255, 255), 2)
        for i, _m in enumerate(reversed(tx_hist)):
            _c = (0, 255, 255) if "9999" in _m else (0, 255, 0)
            cv2.putText(vis, _m, (10, base - i * 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.62, _c, 2)
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
        elif k == ord("m"):
            gam = max(72.0, gam - 20.0)
            cap.set(cv2.CAP_PROP_GAMMA, gam)
            print(f"  GAMMA={gam:.0f}")
        elif k == ord("M"):
            gam = min(500.0, gam + 20.0)
            cap.set(cv2.CAP_PROP_GAMMA, gam)
            print(f"  GAMMA={gam:.0f}")
        elif k == ord("b"):
            bri = max(-64.0, bri - 5.0)
            cap.set(cv2.CAP_PROP_BRIGHTNESS, bri)
        elif k == ord("B"):
            bri = min(64.0, bri + 5.0)
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
        if k in (ord("e"), ord("E"), ord("b"), ord("B"), ord("g"), ord("G"),
                 ord("m"), ord("M")):
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
