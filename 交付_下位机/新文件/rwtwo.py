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
"""

import argparse
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


def read_digit(img, box, tpl, mode, model_cls, model_conf=1.0):
    """按 --digit-from 决定号码来源。返回 (号码, 相似度, 与模型是否一致)。

    两家不合时，光看"模板相似度 > 0.35"不够——实测有模板把"2"读成"3"、
    而模型 0.55 读对的情形。故 hybrid 下多比一层**谁的把握大**。
    """
    if mode == "model" or not tpl:
        return model_cls, 1.0, True
    cx, cy, r = (box[0] + box[2]) // 2, (box[1] + box[3]) // 2, max(6, min(box[2] - box[0], box[3] - box[1]) // 2)
    name, score = classify_digit(img, (cx, cy, r), tpl)
    if not name:
        return model_cls, 0.0, True
    if name == model_cls:
        return name, score, True
    if mode == "template":
        return name, score, False
    if score >= 0.55 and model_conf < 0.60:
        return name, score, False
    return model_cls, score, False


def annotate(img, dials, tpl, mode, aim=(0.0, 0.0), tol=15.0):
    """画框、盘心，并判断"盘心是否落在画面中心"。

    参数 aim 仅为兼容旧调用保留，恒为零——偏移已弃用。
    """
    vis = img.copy()
    pts = []
    H, W = img.shape[:2]
    icx, icy = W / 2.0, H / 2.0
    cv2.drawMarker(vis, (int(icx), int(icy)), (0, 255, 0), cv2.MARKER_CROSS, 30, 2)   # 画面中心
    # 固定靶：盘心该停之处 = 画面中心 − 偏移。把红十拖到这里即算对准，比追橙十直观
    best = None
    for (x0, y0, x1, y1, cf, c, mname) in dials:
        rc = refine_center(img, (x0, y0, x1, y1))
        if rc:
            cx, cy, how = rc
        else:
            cx, cy, how = (x0 + x1) / 2.0, (y0 + y1) / 2.0, "box"
        if not (0 <= cx < W and 0 <= cy < H):
            continue                       # 半个盘，放不了心，弃
        num, score, agree = read_digit(img, (x0, y0, x1, y1), tpl, mode, mname, cf)
        ax, ay = cx, cy          # 对准点即盘心
        d = ((ax - icx) ** 2 + (ay - icy) ** 2) ** 0.5
        pts.append((str(num), cx, cy, how, cf, agree, str(mname), ax, ay, d))

        cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 255, 0), 2)
        cv2.drawMarker(vis, (int(cx), int(cy)), (0, 0, 255), cv2.MARKER_CROSS, 22, 2)
        col = (0, 255, 0) if d <= tol else (0, 165, 255)
        cv2.drawMarker(vis, (int(ax), int(ay)), col, cv2.MARKER_TILTED_CROSS, 26, 2)
        cv2.circle(vis, (int(ax), int(ay)), 8, col, 2)
        if agree:
            txt = f"{num} {cf:.2f}"
        elif str(num) == str(mname):
            txt = f"{num} {cf:.2f} [模板异议]"
        else:
            txt = f"{num}!=模型{mname}"
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


def main():
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default=str(here / "dial_v5.pt"))
    ap.add_argument("--image", default=None)
    ap.add_argument("--cam", type=int, default=None)
    ap.add_argument("--res", default="1280x720")
    ap.add_argument("--conf", type=float, default=0.40, help="置信度门槛")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--tpl", default=str(here / "zones_templates.npz"))
    ap.add_argument("--digit-from", default="model", choices=["model", "template", "hybrid"],
                    help="默认 model：只信模型。template/hybrid 需现场重建模板才可用")
    ap.add_argument("--order", default="1,2,3", help="按 b 重建模板时，从左到右依次是几号")
    ap.add_argument("--clahe", action="store_true", help="暗场景先做 CLAHE 拉对比再推理")
    ap.add_argument("--wb-gray", action=argparse.BooleanOptionalAction, default=False,
                    help="灰世界白平衡，默认关。只在换光/换相机后画面偏色时才开——"
                         "模型若已在偏色帧上训过，再洗一遍反而不准")
    ap.add_argument("--aim-dx", type=float, default=None, help="标定点相对盘心的 x 偏移（像素）")
    ap.add_argument("--aim-dy", type=float, default=None, help="标定点相对盘心的 y 偏移（像素，向下为正）")
    ap.add_argument("--aim-file", default=str(here / "aim_offset.json"), help="偏移存盘处")
    ap.add_argument("--tol", type=float, default=15.0, help="标定点与画面中心相差多少像素算对准")
    ap.add_argument("--soft-exposure", type=float, default=1.0,
                    help="软件压暗倍数，如 0.6；像素已饱和时无效")
    ap.add_argument("--exposure", type=float, default=None, help="启动时设定曝光（-13~-1 常见）")
    ap.add_argument("--brightness", type=float, default=None, help="启动时设定亮度")
    ap.add_argument("--gain", type=float, default=None, help="启动时设定增益")
    ap.add_argument("--link", default="print", choices=["print", "udp", "serial"])
    ap.add_argument("--com", default="COM3")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--udp-host", default="127.0.0.1")
    ap.add_argument("--udp-port", type=int, default=9000)
    ap.add_argument("--out", default=str(here / "_zones_yolo.jpg"))
    args = ap.parse_args()

    print("对准方式：盘心对齐画面中心（不再使用标定点偏移）")

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
        if args.soft_exposure != 1.0:
            # 软件压暗：只对"未顶到 255"的像素有效；已饱和的信息救不回来
            im = np.clip(im.astype(np.float32) * args.soft_exposure, 0, 255).astype(np.uint8)
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
        vis, pts, best = annotate(img, dials, tpl, args.digit_from, (0.0, 0.0), args.tol)
        print(f"检出 {len(dials)} 个盘（有效 {len(pts)}）")
        for (num, cx, cy, how, cf, agree, mname, ax, ay, d) in pts:
            print(f"  {num} 号  盘心 ({cx:.1f},{cy:.1f})  "
                  f"距画面中心 {d:.1f}px  法={how}  conf={cf:.2f}"
                  + ("" if agree else (f"  （模板异议，已采信模型 {mname}）"
                                       if str(num) == str(mname)
                                       else f"  （模板异议，模型读作 {mname}）")))
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
    # 曝光 / 亮度 / 增益：命令行给了就用命令行的，否则用上次存的（与任务一共用一份）
    import json as _json
    _cfg_path = here.parent / "camera_settings.json"
    try:
        _cfg = _json.loads(_cfg_path.read_text(encoding="utf-8"))
    except Exception:
        _cfg = {}
    _want = (
        (cv2.CAP_PROP_EXPOSURE, args.exposure if args.exposure is not None else _cfg.get("exposure")),
        (cv2.CAP_PROP_BRIGHTNESS, args.brightness if args.brightness is not None else _cfg.get("brightness")),
        (cv2.CAP_PROP_GAIN, args.gain if args.gain is not None else _cfg.get("gain")),
    )
    for _prop, _val in _want:
        if _val is not None:
            cap.set(_prop, float(_val))
    exp = cap.get(cv2.CAP_PROP_EXPOSURE)
    bri = cap.get(cv2.CAP_PROP_BRIGHTNESS)
    gai = cap.get(cv2.CAP_PROP_GAIN)

    def _save_cfg():
        try:
            _cfg_path.write_text(_json.dumps(
                {"cam": args.cam, "res": args.res,
                 "exposure": round(float(exp), 2),
                 "brightness": round(float(bri), 1),
                 "gain": round(float(gai), 1)}, ensure_ascii=False, indent=2),
                encoding="utf-8")
        except Exception:
            pass

    print(f"相机：exposure {exp:.0f}  亮度 {bri:.0f}  增益 {gai:.0f}")
    print("过曝时按 b 降亮度、B 升；g/G 改增益；e/E 改曝光；s 存偏移，q 退出")
    lk = Link(args.link, com=args.com, baud=args.baud, host=args.udp_host, port=args.udp_port)
    print(f"信号通道：{lk.describe()}   协议 <<N,号码,X,dx,Y,dy>>（无目标 <<N,0,X,9999,Y,9999>>）")
    last_send = 0.0
    import time as _t
    print("q 退出   s 存帧并保存偏移   j/l 左移/右移标定点   i/k 上移/下移")
    print("红十=盘心，绿十=画面中心；两者重合即对准")

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            break
        frame = enhance(frame)
        dials = detect_dials(model, frame, args.conf, args.imgsz)
        vis, pts, best = annotate(frame, dials, tpl, args.digit_from, (0.0, 0.0), args.tol)
        _g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        mn = float(_g.mean())
        sat = float((_g >= 250).mean() * 100.0)
        if mn < 80:
            warn = "  太暗:补光"
        elif sat > 15:
            warn = f"  过曝(sat{sat:.0f}%):减光或 --soft-exposure"
        elif mn > 235:
            warn = "  偏亮"
        else:
            warn = ""
        cv2.putText(vis, f"dials {len(pts)}  mean {mn:.0f}{warn}  exp {exp:.0f} bri {bri:.0f} gain {gai:.0f}",
                    (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
        cv2.putText(vis, "b/B 亮度  g/G 增益  e/E 曝光  s 存  q 退",
                    (10, vis.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
        now = _t.time()
        if best is not None:
            num, cx, cy, how, cf, agree, mname, ax, ay, d = best
            icx, icy = vis.shape[1] / 2.0, vis.shape[0] / 2.0
            ix, iy = int(round(ax - icx)), int(round(ay - icy))
            if d <= args.tol:
                if now - last_send > 0.05:            # 对准了：约 20Hz
                    lk.send(f"<<N,{num},X,{ix},Y,{iy}>>")
                    last_send = now
            elif now - last_send > 0.10:              # 有盘但没对准：10Hz 发无效帧
                lk.send("<<N,0,X,9999,Y,9999>>")
                last_send = now
        elif now - last_send > 0.10:                  # 一个盘也没检出
            lk.send("<<N,0,X,9999,Y,9999>>")
            last_send = now
        if best is not None:
            num, cx, cy, how, cf, agree, mname, ax, ay, d = best
            ok_txt = "ALIGNED" if d <= args.tol else f"d={d:.0f}px"
            cv2.putText(vis, f"AIM {num}号  {ok_txt}",
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0) if d <= args.tol else (0, 165, 255), 2)
        cv2.imshow("zones_yolo  (q quit)", vis)
        k = cv2.waitKey(1) & 0xFF
        if k in (ord("q"), 27):
            break
        elif k == ord("b"):
            bri = max(0.0, bri - 5.0)
            cap.set(cv2.CAP_PROP_BRIGHTNESS, bri)
            print(f"  亮度 {bri:.0f}")
        elif k == ord("B"):
            bri = min(255.0, bri + 5.0)
            cap.set(cv2.CAP_PROP_BRIGHTNESS, bri)
            print(f"  亮度 {bri:.0f}")
        elif k == ord("g"):
            gai = max(0.0, gai - 10.0)
            cap.set(cv2.CAP_PROP_GAIN, gai)
            print(f"  增益 {gai:.0f}")
        elif k == ord("G"):
            gai = min(255.0, gai + 10.0)
            cap.set(cv2.CAP_PROP_GAIN, gai)
            print(f"  增益 {gai:.0f}")
        elif k == ord("e"):
            exp -= 1.0
            cap.set(cv2.CAP_PROP_EXPOSURE, exp)
            print(f"  曝光 {exp:.0f}")
        elif k == ord("E"):
            exp += 1.0
            cap.set(cv2.CAP_PROP_EXPOSURE, exp)
            print(f"  曝光 {exp:.0f}")
        if k == ord("s"):
            cv2.imwrite(args.out, vis)
            for (num, cx, cy, how, cf, agree, mname, ax, ay, d) in pts:
                print(f"  {num} 号  盘心 ({cx:.1f},{cy:.1f})  "
                      f"距中心 {d:.1f}px  {how}")
            _save_cfg()
            print(f"  已存 {args.out}；相机参数已存")

    _save_cfg()
    lk.close()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
