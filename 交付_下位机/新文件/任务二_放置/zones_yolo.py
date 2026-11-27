# -*- coding: utf-8 -*-
"""
zones_yolo.py —— YOLO 找盘 / 读号（可选），传统法精求中心

三种分工，用 --digit-from 选：

    model     只用 YOLO 的类别（原方案；异域时会认错，如把 2 读成 1）
    template  只认模板（字归一化后比对，抗尺度与字体；跨场地按 b 重建）
    hybrid    两者都算：不一致时以模板为准，并在屏上标出分歧（默认）

为何有这一层：实测同一张"2"号盘，YOLO 在其训练域内 0.96 认对；
换到暗光、异机位、打印纸上，框仍然准，类别却给了"1（0.81）"，
而模板法读对。故框归 YOLO、字归模板，各用所长。

用法：
    python zones_yolo.py --weights dial_v3.pt --image 现场.jpg
    python zones_yolo.py --weights dial_v3.pt --cam 0
    python zones_yolo.py --weights dial_v3.pt --cam 0 --digit-from model

窗口里的按键：
    q / Esc   退出（退出时存标定偏移与相机参数）
    s         存一帧图 + 存标定偏移
    j / l     标定点左移 / 右移 2 像素（相机不在爪子正上方时用它校准）
    i / k     标定点上移 / 下移 2 像素
    e / E     曝光变暗 / 变亮   b / B  亮度   g / G  增益   m / M  gamma   w  锁白平衡
    （相机参数每次改动都落盘到 camera_settings_zones.json，下次启动自动套用）

屏幕左下角是"串口输出"窗口：标题显示通道与已发条数，下面是最近 4 条报文，
最新一条亮绿。--link print 时不会真发，只上屏，用来核对报文格式。
"""

import argparse
import json
from collections import deque
from pathlib import Path

import cv2
import numpy as np

from link import Link
from refine_center import refine_center
from zones import classify_digit, digit_patch


def detect_dials(model, img, conf=0.40, imgsz=640):
    """返回 [(x0,y0,x1,y1,conf,cls,cls_name)]"""
    # agnostic_nms：一个盘只该有一个框；跨类重复也一并压掉
    r = model.predict(img, imgsz=imgsz, conf=conf, agnostic_nms=True, verbose=False)[0]
    names = model.names
    out = []
    for b in r.boxes:
        x0, y0, x1, y1 = [int(v) for v in b.xyxy[0].tolist()]
        c = int(b.cls[0])
        out.append((x0, y0, x1, y1, float(b.conf[0]), c, str(names.get(c, c))))
    return out


def load_templates(path):
    p = Path(path)
    if not p.exists():
        return {}
    data = np.load(p)
    return {k: data[k] for k in data.files}


def read_digit(img, box, tpl, mode, model_cls):
    """按 --digit-from 决定号码来源。返回 (号码, 相似度, 与模型是否一致)。"""
    if mode == "model" or not tpl:
        return model_cls, 1.0, True
    cx, cy, r = (box[0] + box[2]) // 2, (box[1] + box[3]) // 2, max(6, min(box[2] - box[0], box[3] - box[1]) // 2)
    name, score = classify_digit(img, (cx, cy, r), tpl)
    if mode == "template":
        return name, score, (name == model_cls)
    # hybrid：模板有把握才采信，否则听模型的
    if name and score >= 0.35:
        return name, score, (name == model_cls)
    return model_cls, score, True


def annotate(img, dials, tpl, mode, aim=(0.0, 0.0), tol=15.0):
    """画框、盘心、以及"标定点"。

    标定点 = 盘心 + aim（像素偏移）。相机不在爪子正中，故对不准盘心；
    把标定点挪到"相机在该处时爪子正好在盘心"的位置，机器对准它即可。
    """
    vis = img.copy()
    pts = []
    H, W = img.shape[:2]
    icx, icy = W / 2.0, H / 2.0
    cv2.drawMarker(vis, (int(icx), int(icy)), (0, 255, 0), cv2.MARKER_CROSS, 30, 2)   # 画面中心
    best = None
    for (x0, y0, x1, y1, cf, c, mname) in dials:
        rc = refine_center(img, (x0, y0, x1, y1))
        if rc:
            cx, cy, how = rc
        else:
            cx, cy, how = (x0 + x1) / 2.0, (y0 + y1) / 2.0, "box"
        if not (0 <= cx < W and 0 <= cy < H):
            continue                       # 半个盘，放不了心，弃
        num, score, agree = read_digit(img, (x0, y0, x1, y1), tpl, mode, mname)
        ax, ay = cx + aim[0], cy + aim[1]
        d = ((ax - icx) ** 2 + (ay - icy) ** 2) ** 0.5
        pts.append((str(num), cx, cy, how, cf, agree, str(mname), ax, ay, d))

        cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 255, 0), 2)
        cv2.drawMarker(vis, (int(cx), int(cy)), (0, 0, 255), cv2.MARKER_CROSS, 22, 2)
        # 标定点：橙色十字加小圈；对准了就变绿
        col = (0, 255, 0) if d <= tol else (0, 165, 255)
        cv2.drawMarker(vis, (int(ax), int(ay)), col, cv2.MARKER_TILTED_CROSS, 26, 2)
        cv2.circle(vis, (int(ax), int(ay)), 8, col, 2)
        txt = f"{num} {cf:.2f}" if agree else f"{num}!=模型{mname}"
        cv2.putText(vis, txt, (x0 + 4, max(16, y0 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 255) if agree else (0, 0, 255), 2)
        if best is None or d < best[9]:
            best = pts[-1]
    return vis, pts, best


def _load_aim(path):
    import json
    try:
        d = json.loads(Path(path).read_text(encoding="utf-8"))
        return float(d.get("dx", 0)), float(d.get("dy", 0))
    except Exception:
        return 0.0, 0.0


def _save_aim(path, dx, dy):
    import json
    Path(path).write_text(json.dumps({"dx": dx, "dy": dy}, ensure_ascii=False, indent=2),
                          encoding="utf-8")


def _num(v, default):
    """cap.get() 拿到的值转 float；None / nan / 抛异常一律退回 default。"""
    try:
        v = float(v)
    except (TypeError, ValueError):
        return float(default)
    return float(default) if v != v else v          # nan 自己不等于自己


def _bump(cap, prop, cur, delta, name):
    """改一个相机参数，再以"驱动实际接受的值"回读为准。

    不能硬编码 0~255 去夹：实测这台相机曝光是 -13~-1，亮度读出来是 -64 而且写不进去。
    照老代码 max(0.0, bri-5) 那套，从 -64 按 b 会跳到 0，画面反而变亮——方向都是反的。
    所以改成写进去再回读：驱动不认就当场说明，别让人白按二十下。
    """
    want = cur + delta
    try:
        cap.set(prop, want)
        got = _num(cap.get(prop), want)
    except Exception as e:
        print(f"  {name} 写不进去：{e}")
        return cur
    if abs(got - cur) < 1e-6:
        print(f"  {name}={got:g} 没变化 —— 这个相机没开放这一项，实际只有 EXPOSURE(E/e) 有效")
    else:
        print(f"  {name} {cur:g} -> {got:g}")
    return got


def _panel(vis, lines, x, y, fs, colors=None, alpha=0.55, border=(0, 200, 200)):
    """画一块半透明黑底 + 若干行文字，返回底板高度。

    cv2.putText 用的是 Hershey 笔画字体，没有汉字字形，所以这里只认 ASCII，
    塞中文进去会变成一串问号。底板是为了压住亮背景——实测白墙上一行黄字根本看不见。
    """
    lines = list(lines)
    if not lines:
        return 0
    lh = max(16, int(fs * 38))
    pad = max(8, int(fs * 16))
    bw = 0
    for s in lines:
        (tw, _th), _ = cv2.getTextSize(s, cv2.FONT_HERSHEY_SIMPLEX, fs, 1)
        bw = max(bw, tw)
    bw += pad * 2 + 4          # +4：Hershey 的最后一个字符advance 偏小，不留会贴边
    bh = lh * len(lines) + pad * 2
    H, W = vis.shape[:2]
    x0 = max(4, min(x, W - bw - 4))
    y0 = max(4, min(y, H - bh - 4))
    if x0 + bw > W or y0 + bh > H:
        return 0

    roi = vis[y0:y0 + bh, x0:x0 + bw]
    dark = np.zeros_like(roi)
    cv2.addWeighted(dark, alpha, roi, 1.0 - alpha, 0, roi)      # 半黑底，不全遮
    if border is not None:
        cv2.rectangle(vis, (x0, y0), (x0 + bw - 1, y0 + bh - 1), border, 1)
    if colors is None:
        colors = [(255, 255, 255)] * len(lines)
    for i, s in enumerate(lines):
        cv2.putText(vis, s, (x0 + pad, y0 + pad + lh * (i + 1) - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, fs, colors[i % len(colors)], 1)
    return bh


def _draw_tx_box(vis, lines, title, alpha=0.55):
    """左下角的"串口输出"小终端：标题 + 最近几条报文，最新一条亮绿。"""
    lines = list(lines)
    body = [title] + lines
    cols = [(0, 220, 220)]
    for i in range(len(lines)):
        cols.append((0, 255, 0) if i == len(lines) - 1 else (185, 185, 185))
    H, W = vis.shape[:2]
    fs = max(0.45, W / 2300.0)
    lh = max(16, int(fs * 38))
    pad = max(8, int(fs * 16))
    _panel(vis, body, 6, H - (lh * len(body) + pad * 2) - 6, fs, cols, alpha)


def main():
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", required=True)
    ap.add_argument("--image", default=None)
    ap.add_argument("--cam", type=int, default=None)
    ap.add_argument("--res", default="1280x720")
    ap.add_argument("--conf", type=float, default=0.40, help="置信度门槛")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--tpl", default=str(here / "zones_templates.npz"))
    ap.add_argument("--digit-from", default="hybrid", choices=["model", "template", "hybrid"])
    ap.add_argument("--order", default="1,2,3", help="按 b 重建模板时，从左到右依次是几号")
    ap.add_argument("--clahe", action="store_true", help="暗场景先做 CLAHE 拉对比再推理")
    ap.add_argument("--wb-gray", action="store_true", help="灰世界白平衡，治偏色（锁了自动白平衡时尤其需要）")
    ap.add_argument("--aim-dx", type=float, default=None, help="标定点相对盘心的 x 偏移（像素）")
    ap.add_argument("--aim-dy", type=float, default=None, help="标定点相对盘心的 y 偏移（像素，向下为正）")
    ap.add_argument("--aim-file", default=str(here / "aim_offset.json"), help="偏移存盘处")
    ap.add_argument("--tol", type=float, default=15.0, help="标定点与画面中心相差多少像素算对准")
    ap.add_argument("--link", default="print", choices=["print", "udp", "serial"])
    ap.add_argument("--com", default="COM3")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--udp-host", default="127.0.0.1")
    ap.add_argument("--udp-port", type=int, default=9000)
    ap.add_argument("--out", default=str(here.parent / "_zones_yolo.jpg"))
    args = ap.parse_args()

    aim_x, aim_y = _load_aim(args.aim_file)
    if args.aim_dx is not None:
        aim_x = args.aim_dx
    if args.aim_dy is not None:
        aim_y = args.aim_dy
    print(f"标定点偏移：dx={aim_x:+.1f}  dy={aim_y:+.1f} 像素（轴心 = 盘心 + 此偏移）")

    _clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    def enhance(im):
        """可选预处理（顺序要紧：先白平衡，后拉对比）。

        --wb-gray：灰世界白平衡。"相机锁了 AWB"时，若场地光是冷色/暖色，
        整幅会偏色，盘的黑环被"洗淡"，YOLO 直接零检出——实测这张偏蓝图，
        不做白平衡 0 只、做了 1 只且判对。
        --clahe：只拉 Lab 的 L 通道，颜色留住。
        """
        if args.wb_gray:
            f = im.astype(np.float32)
            b, g, r = (float(f[:, :, i].mean()) for i in range(3))
            k = (b + g + r) / 3.0
            f[:, :, 0] *= k / max(b, 1e-3)
            f[:, :, 1] *= k / max(g, 1e-3)
            f[:, :, 2] *= k / max(r, 1e-3)
            im = np.clip(f, 0, 255).astype(np.uint8)
        if args.clahe:
            lab = cv2.cvtColor(im, cv2.COLOR_BGR2LAB)
            l, a, b2 = cv2.split(lab)
            lab = cv2.merge([_clahe.apply(l), a, b2])
            im = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
        return im

    from ultralytics import YOLO
    model = YOLO(args.weights)
    tpl = load_templates(args.tpl) if args.digit_from != "model" else {}
    if args.digit_from != "model":
        print(f"模板：{Path(args.tpl).name}（{len(tpl)} 个）" if tpl else "无模板，退用模型类别")

    if args.image:
        img = cv2.imdecode(np.fromfile(args.image, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            raise SystemExit("读不到图")
        img = enhance(img)
        dials = detect_dials(model, img, args.conf, args.imgsz)
        vis, pts, best = annotate(img, dials, tpl, args.digit_from, (aim_x, aim_y), args.tol)
        print(f"检出 {len(dials)} 个盘（有效 {len(pts)}）")
        for (num, cx, cy, how, cf, agree, mname, ax, ay, d) in pts:
            print(f"  {num} 号  盘心 ({cx:.1f},{cy:.1f})  标定点 ({ax:.1f},{ay:.1f})  "
                  f"距画面中心 {d:.1f}px  法={how}  conf={cf:.2f}"
                  + ("" if agree else f"  （模型读作 {mname}）"))
        cv2.imencode(".jpg", vis)[1].tofile(args.out)
        print(f"预览：{args.out}")
        return

    if args.cam is None:
        ap.print_help()
        return

    cap = cv2.VideoCapture(args.cam, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(args.cam)
    if not cap.isOpened():
        raise SystemExit("相机打不开")
    w, h = [int(v) for v in args.res.lower().split("x")]
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_AUTO_WB, 0)

    # ---- 相机参数存盘：曝光 / 亮度 / 增益 / gamma，下次启动自动套用 ----
    cfg_path = here / "camera_settings_zones.json"

    def _load_cfg():
        try:
            return json.loads(cfg_path.read_text(encoding="utf-8"))
        except Exception:
            return {}

    def _save_cfg(d):
        try:
            cfg_path.write_text(json.dumps(d, ensure_ascii=False, indent=2),
                                encoding="utf-8")
        except Exception as e:
            print("  参数存不下来：", e)

    cfg = _load_cfg()
    for _k, _p in (("exposure", cv2.CAP_PROP_EXPOSURE),
                   ("brightness", cv2.CAP_PROP_BRIGHTNESS),
                   ("gain", cv2.CAP_PROP_GAIN),
                   ("gamma", cv2.CAP_PROP_GAMMA)):
        if _k in cfg:
            try:
                cap.set(_p, float(cfg[_k]))
            except Exception:
                pass

    exp = _num(cap.get(cv2.CAP_PROP_EXPOSURE), -6.0)
    bri = _num(cap.get(cv2.CAP_PROP_BRIGHTNESS), 128.0)
    gai = _num(cap.get(cv2.CAP_PROP_GAIN), 0.0)
    gam = _num(cap.get(cv2.CAP_PROP_GAMMA), 100.0)
    if gam <= 0:
        gam = 100.0                    # 驱动没暴露 gamma 时给个安全默认

    def remember():
        _save_cfg({"cam": args.cam, "res": f"{w}x{h}",
                   "exposure": round(float(exp), 2),
                   "brightness": round(float(bri), 1),
                   "gain": round(float(gai), 1),
                   "gamma": round(float(gam), 1)})

    print(f"初值：EXPOSURE={exp:+.1f}  GAMMA={gam:.0f}  BRIGHTNESS={bri:.0f}  "
          f"GAIN={gai:.0f}   分辨率 {w}x{h}")
    print(f"参数存盘：{cfg_path.name}（每调一次自动落盘，下次启动自动套用）")

    lk = Link(args.link, com=args.com, baud=args.baud, host=args.udp_host, port=args.udp_port)
    print(f"信号通道：{lk.describe()}   协议 <<N,号码,X,dx,Y,dy>>（无目标 <<N,0,X,9999,Y,9999>>）")
    last_send = 0.0
    send_cnt = 0
    pkt_log = deque(maxlen=4)                                   # 屏上只留最近 4 条
    tx_tag = f"TX {args.link}" + ("  (not sent)" if args.link == "print" else "")
    import time as _t
    print("q 退出   s 存帧并保存偏移   j/l 左移/右移标定点   i/k 上移/下移标定点")
    print("e 变暗 / E 变亮（曝光）   b/B 亮度   g/G 增益   m/M gamma   w 锁白平衡")
    print("红十=盘心，橙十/绿十=标定点（绿=已对准），绿十=画面中心，左下角=串口输出")

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            break
        frame = enhance(frame)
        dials = detect_dials(model, frame, args.conf, args.imgsz)
        vis, pts, best = annotate(frame, dials, tpl, args.digit_from, (aim_x, aim_y), args.tol)
        mn = float(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).mean())
        now = _t.time()
        pkt = None
        if best is not None:
            num, cx, cy, how, cf, agree, mname, ax, ay, d = best
            icx, icy = vis.shape[1] / 2.0, vis.shape[0] / 2.0
            ix, iy = int(round(ax - icx)), int(round(ay - icy))
            if d <= args.tol:
                if now - last_send > 0.05:            # 对准了：约 20Hz
                    pkt = f"<<N,{num},X,{ix},Y,{iy}>>"
                    last_send = now
            elif now - last_send > 0.10:              # 有盘但没对准：10Hz 发无效帧
                pkt = "<<N,0,X,9999,Y,9999>>"
                last_send = now
        elif now - last_send > 0.10:                  # 一个盘也没检出
            pkt = "<<N,0,X,9999,Y,9999>>"
            last_send = now
        if pkt is not None:                           # 一条报文，既真发也上屏
            lk.send(pkt)
            pkt_log.append(pkt)
            send_cnt += 1
        # ---- 顶部 HUD：固定三行，黑底垫着，压在亮背景上也看得清 ----
        warn = "  DARK:add light" if mn < 80 else ("  OVER" if mn > 235 else "")
        hud = [f"dials {len(pts)}  mean {mn:.0f}{warn}  aim({aim_x:+.0f},{aim_y:+.0f})"]
        hud_col = [(0, 80, 255)]
        if best is not None:
            num, cx, cy, how, cf, agree, mname, ax, ay, d = best
            hud.append(f"AIM #{num}  " + ("ALIGNED" if d <= args.tol else f"d={d:.0f}px"))
            hud_col.append((0, 255, 0) if d <= args.tol else (0, 200, 255))
        else:
            hud.append("AIM --  no dial")
            hud_col.append((160, 160, 160))
        hud.append(f"exp {exp:+.1f}  bri {bri:.0f}  gain {gai:.0f}  gam {gam:.0f}")
        hud_col.append((255, 255, 0))
        _panel(vis, hud, 6, 6, 0.6, hud_col, alpha=0.45)

        _draw_tx_box(vis, pkt_log, f"{tx_tag}  n={send_cnt}")
        cv2.imshow("zones_yolo  (q quit)", vis)
        k = cv2.waitKey(1) & 0xFF
        if k in (ord("q"), 27):
            break
        elif k == ord("j"):
            aim_x -= 2
        elif k == ord("l"):
            aim_x += 2
        elif k == ord("i"):
            aim_y -= 2
        elif k == ord("k"):
            aim_y += 2
        elif k == ord("e"):
            exp = _bump(cap, cv2.CAP_PROP_EXPOSURE, exp, -1.0, "EXPOSURE")      # 变暗
        elif k == ord("E"):
            exp = _bump(cap, cv2.CAP_PROP_EXPOSURE, exp, +1.0, "EXPOSURE")      # 变亮
        elif k == ord("b"):
            bri = _bump(cap, cv2.CAP_PROP_BRIGHTNESS, bri, -5.0, "BRIGHTNESS")
        elif k == ord("B"):
            bri = _bump(cap, cv2.CAP_PROP_BRIGHTNESS, bri, +5.0, "BRIGHTNESS")
        elif k == ord("g"):
            gai = _bump(cap, cv2.CAP_PROP_GAIN, gai, -10.0, "GAIN")
        elif k == ord("G"):
            gai = _bump(cap, cv2.CAP_PROP_GAIN, gai, +10.0, "GAIN")
        elif k == ord("m"):
            gam = _bump(cap, cv2.CAP_PROP_GAMMA, gam, -20.0, "GAMMA")
        elif k == ord("M"):
            gam = _bump(cap, cv2.CAP_PROP_GAMMA, gam, +20.0, "GAMMA")
        elif k == ord("w"):
            cap.set(cv2.CAP_PROP_AUTO_WB, 0)
            print("  白平衡已锁定")
        if k in (ord("e"), ord("E"), ord("b"), ord("B"),
                 ord("g"), ord("G"), ord("m"), ord("M")):
            remember()                      # 每调一次便落盘，万一强退也不丢
        if k == ord("s"):
            cv2.imwrite(args.out, vis)
            _save_aim(args.aim_file, aim_x, aim_y)
            for (num, cx, cy, how, cf, agree, mname, ax, ay, d) in pts:
                print(f"  {num} 号  盘心 ({cx:.1f},{cy:.1f})  标定点 ({ax:.1f},{ay:.1f})  "
                      f"距中心 {d:.1f}px  {how}")
            print(f"  已存 {args.out}；偏移 dx={aim_x:+.0f} dy={aim_y:+.0f} → {args.aim_file}")

    _save_aim(args.aim_file, aim_x, aim_y)
    remember()                             # 退出时再存一次，保住最后的相机参数
    lk.close()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
