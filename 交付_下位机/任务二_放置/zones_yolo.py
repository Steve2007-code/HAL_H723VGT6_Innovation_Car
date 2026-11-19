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
    lk = Link(args.link, com=args.com, baud=args.baud, host=args.udp_host, port=args.udp_port)
    print(f"信号通道：{lk.describe()}   协议 <<N,号码,X,dx,Y,dy>>（无目标 <<N,0,X,9999,Y,9999>>）")
    last_send = 0.0
    import time as _t
    print("q 退出   s 存帧并保存偏移   j/l 左移/右移标定点   i/k 上移/下移")
    print("红十=盘心，橙十/绿十=标定点（绿=已对准），绿十=画面中心")

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            break
        frame = enhance(frame)
        dials = detect_dials(model, frame, args.conf, args.imgsz)
        vis, pts, best = annotate(frame, dials, tpl, args.digit_from, (aim_x, aim_y), args.tol)
        mn = float(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).mean())
        warn = "  太暗:补光" if mn < 80 else ("  过曝" if mn > 235 else "")
        cv2.putText(vis, f"dials {len(pts)}  mean {mn:.0f}{warn}  aim({aim_x:+.0f},{aim_y:+.0f})",
                    (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
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
        elif k == ord("j"):
            aim_x -= 2
        elif k == ord("l"):
            aim_x += 2
        elif k == ord("i"):
            aim_y -= 2
        elif k == ord("k"):
            aim_y += 2
        if k == ord("s"):
            cv2.imwrite(args.out, vis)
            _save_aim(args.aim_file, aim_x, aim_y)
            for (num, cx, cy, how, cf, agree, mname, ax, ay, d) in pts:
                print(f"  {num} 号  盘心 ({cx:.1f},{cy:.1f})  标定点 ({ax:.1f},{ay:.1f})  "
                      f"距中心 {d:.1f}px  {how}")
            print(f"  已存 {args.out}；偏移 dx={aim_x:+.0f} dy={aim_y:+.0f} → {args.aim_file}")

    _save_aim(args.aim_file, aim_x, aim_y)
    lk.close()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
