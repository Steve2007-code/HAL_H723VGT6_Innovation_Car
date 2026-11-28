# -*- coding: utf-8 -*-
"""
zones.py —— 找号码盘、认数字、给中心

场景：暂存区/粗加工区印着若干圆形"号码盘"（1–6 环，中心一个数字）。
放置任务要的，就是"第 N 号盘的中心在哪"。

两件事：
    ① 找盘：盘边是同心圆环，用 HoughCircles 找大圆；找不到再用轮廓外接圆兜底。
    ② 认数：裁中心小块，二值化后与模板比对（归一化互相关），取最像的那个数字。

模板怎么来（只需做一次）：
    python zones.py --build --image 现场盘.jpg --order 1,2,3
    它把从左到右的盘认作 1、2、3，抠出数字存成 zones_templates.npz

认盘：
    python zones.py --image 另一张.jpg

注意：模板与现场是同一张印刷盘、同一机位距离时最准；换距离要重建模板。
"""

import argparse
from pathlib import Path

import cv2
import numpy as np


def find_dials(img):
    """找号码盘，返回 [(cx, cy, r)]，按 x 从左到右排序。"""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    g = cv2.GaussianBlur(gray, (9, 9), 2)
    circles = cv2.HoughCircles(g, cv2.HOUGH_GRADIENT, dp=1.2, minDist=max(60, img.shape[1] // 8),
                               param1=120, param2=55,
                               minRadius=int(img.shape[1] * 0.05),
                               maxRadius=int(img.shape[1] * 0.22))
    out = []
    if circles is not None:
        for x, y, r in np.round(circles[0]).astype(int):
            out.append((int(x), int(y), int(r)))
    if not out:
        # 半径先验放空一次：相机远近不同，盘大小会差很多
        circles = cv2.HoughCircles(g, cv2.HOUGH_GRADIENT, dp=1.2,
                                   minDist=max(40, img.shape[1] // 10),
                                   param1=120, param2=45,
                                   minRadius=int(img.shape[1] * 0.02),
                                   maxRadius=int(img.shape[1] * 0.45))
        if circles is not None:
            for x, y, r in np.round(circles[0]).astype(int):
                out.append((int(x), int(y), int(r)))
    if not out:
        # 兜底：暗块轮廓的外接圆
        m = (gray < 120).astype(np.uint8) * 255
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k, iterations=3)
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            a = cv2.contourArea(c)
            if a < 0.002 * img.shape[0] * img.shape[1]:
                continue
            (x, y), r = cv2.minEnclosingCircle(c)
            out.append((int(x), int(y), int(r)))
    out.sort(key=lambda t: t[0])
    # 去重（同心的只留半径最大的）
    ded = []
    for c in out:
        if all(abs(c[0] - d[0]) > c[2] * 0.6 or abs(c[1] - d[1]) > c[2] * 0.6 for d in ded):
            ded.append(c)
    return ded


def digit_patch(img, dial, frac=0.45, size=64):
    """裁盘心那块，二值化、居中、归一到 size×size。"""
    cx, cy, r = dial
    d = int(r * frac)
    x0, y0 = max(0, cx - d), max(0, cy - d)
    x1, y1 = min(img.shape[1], cx + d), min(img.shape[0], cy + d)
    roi = img[y0:y1, x0:x1]
    if roi.size == 0:
        return None
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, bw = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    ys, xs = np.nonzero(bw)
    if len(xs) < 20:
        return None
    bx0, bx1, by0, by1 = xs.min(), xs.max(), ys.min(), ys.max()
    glyph = bw[by0:by1 + 1, bx0:bx1 + 1]
    h, w = glyph.shape
    s = (size - 8) / max(h, w)
    glyph = cv2.resize(glyph, (max(1, int(w * s)), max(1, int(h * s))), interpolation=cv2.INTER_NEAREST)
    canvas = np.zeros((size, size), np.uint8)
    y0c = (size - glyph.shape[0]) // 2
    x0c = (size - glyph.shape[1]) // 2
    canvas[y0c:y0c + glyph.shape[0], x0c:x0c + glyph.shape[1]] = glyph
    return canvas


def build_templates(img, order, out_path):
    dials = find_dials(img)
    print(f"找到 {len(dials)} 个盘：{dials}")
    if len(dials) != len(order):
        print("[警告] 盘数与 --order 给的数目不符，仍按从左到右对应")
    tpl = {}
    for dial, name in zip(dials, order):
        p = digit_patch(img, dial)
        if p is None:
            print(f"  {name}: 抠不出数字")
            continue
        tpl[str(name)] = p
        print(f"  盘 {dial} → 模板 {name}")
    np.savez_compressed(out_path, **tpl)
    print(f"已存模板：{out_path}（{len(tpl)} 个）")


def classify_digit(img, dial, tpl):
    best, score = None, -1.0
    # 半径估得未必准，故多试几种裁剪比例，取最像的那次
    for frac in (0.25, 0.32, 0.40, 0.48, 0.56):
        patch = digit_patch(img, dial, frac=frac)
        if patch is None:
            continue
        for name, t in tpl.items():
            s = float(cv2.matchTemplate(patch, t, cv2.TM_CCOEFF_NORMED).max())
            if s > score:
                best, score = name, s
    return best, score


def glyph_center(img, dial, frac=0.40):
    """数字的质心（图像坐标）。印刷时数字在盘心，故它比 Hough 圆心更贴"区域中心"。"""
    cx, cy, r = dial
    d = int(r * frac)
    x0, y0 = max(0, cx - d), max(0, cy - d)
    roi = img[y0:min(img.shape[0], cy + d), x0:min(img.shape[1], cx + d)]
    if roi.size == 0:
        return None
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, bw = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    ys, xs = np.nonzero(bw)
    if len(xs) < 20:
        return None
    return float(x0 + xs.mean()), float(y0 + ys.mean())


def run_camera(args):
    """实时看盘：认号、标中心。缺模板时可当场重建。"""
    cap = cv2.VideoCapture(args.cam, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(args.cam)
    if not cap.isOpened():
        raise SystemExit("相机打不开")
    w, h = [int(v) for v in args.res.lower().split("x")]
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_AUTO_WB, 0)

    tpl = {}
    tpl_path = Path(args.tpl)
    if tpl_path.exists():
        data = np.load(tpl_path)
        tpl = {k: data[k] for k in data.files}
        print(f"模板：{tpl_path.name}（{len(tpl)} 个）")
    else:
        print(f"没有模板 {tpl_path.name}——先按 b 用当前画面重建")

    print("q 退出   b 用当前画面重建模板（按 --order 从左到右）   s 存帧")
    print(f"重建时顺序：{args.order}")

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("读不到帧，退出。")
            break
        vis = frame.copy()
        dials = find_dials(frame)
        for dial in dials:
            cv2.circle(vis, (dial[0], dial[1]), dial[2], (0, 255, 0), 2)
            gc = glyph_center(frame, dial)
            if gc:
                cv2.drawMarker(vis, (int(gc[0]), int(gc[1])), (0, 0, 255), cv2.MARKER_CROSS, 18, 2)
            if tpl:
                name, score = classify_digit(frame, dial, tpl)
                okk = score > 0.5
                cv2.putText(vis, f"{name if okk else '?'} {score:.2f}",
                            (dial[0] - 40, dial[1] - dial[2] - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0) if okk else (0, 0, 255), 2)
        cv2.putText(vis, f"dials {len(dials)}   b=rebuild  s=save  q=quit",
                    (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.imshow("zones  (q quit)", vis)
        k = cv2.waitKey(1) & 0xFF
        if k in (ord("q"), 27):
            break
        if k == ord("b"):
            order = [s.strip() for s in args.order.split(",")]
            if len(order) != len(dials):
                print(f"  画面里 {len(dials)} 个盘，--order 给了 {len(order)} 个——先对齐数目")
            else:
                tpl = {}
                for dial, name in zip(dials, order):
                    patch = digit_patch(frame, dial)
                    if patch is not None:
                        tpl[str(name)] = patch
                np.savez_compressed(tpl_path, **tpl)
                print(f"  已用当前画面重建模板：{tpl_path.name}（{len(tpl)} 个）")
        if k == ord("s"):
            cv2.imwrite(args.out, vis)
            print(f"  已存 {args.out}（{len(dials)} 个盘）")

    cap.release()
    cv2.destroyAllWindows()


def main():
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description="找号码盘并认数字")
    ap.add_argument("--image", default=None)
    ap.add_argument("--cam", type=int, default=None, help="相机编号；给了就实时看")
    ap.add_argument("--res", default="1280x720")
    ap.add_argument("--build", action="store_true", help="建模板")
    ap.add_argument("--order", default="1,2,3", help="建模板时，从左到右依次是几号")
    ap.add_argument("--tpl", default=str(here / "zones_templates.npz"))
    ap.add_argument("--out", default=str(here.parent / "_zones.jpg"))
    args = ap.parse_args()

    if args.cam is not None:
        return run_camera(args)

    if not args.image:
        ap.print_help()
        return

    img = cv2.imdecode(np.fromfile(args.image, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise SystemExit("读不到图")

    if args.build:
        build_templates(img, [s.strip() for s in args.order.split(",")], args.tpl)
        return

    tpl_path = Path(args.tpl)
    if not tpl_path.exists():
        raise SystemExit(f"没有模板 {tpl_path}，先跑 --build")
    data = np.load(tpl_path)
    tpl = {k: data[k] for k in data.files}

    vis = img.copy()
    for dial in find_dials(img):
        name, score = classify_digit(img, dial, tpl)
        ok = score > 0.5
        print(f"盘 ({dial[0]},{dial[1]}) r={dial[2]}  → 数字 {name if ok else '?'}  相似度 {score:.2f}")
        cv2.circle(vis, (dial[0], dial[1]), dial[2], (0, 255, 0), 2)
        gc = glyph_center(img, dial)
        if gc:
            cv2.drawMarker(vis, (int(gc[0]), int(gc[1])), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
            print(f"      数字质心 ({gc[0]:.0f},{gc[1]:.0f})  ← 以此作区域中心")
        else:
            cv2.drawMarker(vis, (dial[0], dial[1]), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(vis, f"{name if ok else '?'} {score:.2f}", (dial[0] - 40, dial[1] - dial[2] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0) if ok else (0, 0, 255), 2)
    cv2.imencode(".jpg", vis)[1].tofile(args.out)
    print(f"预览：{args.out}")


if __name__ == "__main__":
    main()
