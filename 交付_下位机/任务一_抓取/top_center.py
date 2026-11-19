# -*- coding: utf-8 -*-
"""
top_center.py —— 物料顶部中心点，与画面中心的偏差

为什么要单独算"顶部中心"：
    YOLO 的框是**整只料的二维外接框**，框中心不等于顶面中心。
    相机斜俯视时，框把"顶面 + 侧面"一并框进去，框心落在侧面中部，
    比顶面中心偏低。抓取要对准的是**顶面圆的圆心**。

怎么估：
    顶面在解剖上是"料的最上缘那条椭圆"，其圆心 = 椭圆中心。
    取掩膜上部 45% 的边界点拟合椭圆，其中心即顶面中心；
    拟合失败（点太少/太散）则退回"最宽那一行的中点"。

用法：
    python top_center.py --image ..\\1000.jpg --weights ..\\best.pt
    python top_center.py --cam 0 --weights ..\\best.pt --k 3.2

    --k  每毫米多少像素（在**料顶面所在高度**标定，不是台面）
         给了它，才会把像素差换算成毫米。

输出：屏幕打印每只料的 顶面中心、与画面中心的 dx/dy（像素/毫米），并画图存盘。
"""

import argparse
from pathlib import Path

import cv2
import numpy as np

from sixcolor import COLORS, label_of_lab, roi_lab, to_lab


def read_image(path):
    return cv2.imdecode(np.fromfile(str(path), np.uint8), cv2.IMREAD_COLOR)


def material_mask(img, box):
    """框内按"有彩或暗"取物料掩膜，再把框边一圈剥掉。

    框是紧的，故框内多数像素就是料本身；剥边是为了去掉与背景混色的那一圈。
    """
    x0, y0, x1, y1 = box
    pad = 2
    x0, y0 = max(0, x0 + pad), max(0, y0 + pad)
    x1, y1 = min(img.shape[1], x1 - pad), min(img.shape[0], y1 - pad)
    if x1 - x0 < 8 or y1 - y0 < 8:
        return None
    roi = img[y0:y1, x0:x1]
    lab = to_lab(roi)
    C = np.hypot(lab[:, :, 1], lab[:, :, 2])
    V = roi.max(axis=2)
    m = ((C > 25) | (V < 120)).astype(np.uint8) * 255
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k, iterations=2)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k, iterations=1)
    return m, (x0, y0)


def top_center_from_mask(img, mask, offset, prior_ratio=0.34):
    """返回 (顶面中心x, 顶面中心y, 方法, 掩膜质心)。坐标已还原到整幅图。"""
    ox, oy = offset
    ys, xs = np.nonzero(mask)
    if len(xs) < 50:
        return None
    ys = ys.astype(np.float32)
    xs = xs.astype(np.float32)
    cy, cx = float(ys.mean()), float(xs.mean())
    y_min, y_max = float(ys.min()), float(ys.max())
    h = y_max - y_min

    # ---------------- 主路：按亮度分层摘出顶面
    lab = to_lab(img[oy:oy + mask.shape[0], ox:ox + mask.shape[1]])
    L = lab[:, :, 0]
    band = mask > 0
    band[:int(y_min + 0.12 * h)] = False        # 最上缘那一线是轮廓，混着背景
    band[int(y_min + 0.62 * h):] = False        # 下半是侧壁，不参与
    vals = L[band]
    if vals.size >= 80:
        thr = float(np.percentile(vals, 62))    # 取较亮的一档
        sel = ((L >= thr) & band).astype(np.uint8) * 255
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        sel = cv2.morphologyEx(sel, cv2.MORPH_CLOSE, k, iterations=2)
        sel = cv2.morphologyEx(sel, cv2.MORPH_OPEN, k, iterations=1)
        n, lb, st, _ = cv2.connectedComponentsWithStats(sel, 8)
        if n > 1:
            i = 1 + int(np.argmax(st[1:, 4]))
            pys, pxs = np.nonzero(lb == i)
            if len(pxs) >= 30:
                pts = np.column_stack([pxs, pys]).astype(np.float32)
                try:
                    (ex, ey), (MA, ma), ang = cv2.fitEllipse(pts)
                    box_w = float(mask.shape[1])
                    ratio = min(MA, ma) / max(MA, ma)
                    # 验一验这椭圆像不像顶面：长轴接近料宽、短轴不为零、中心在上半
                    if 0.45 * box_w <= max(MA, ma) <= 1.15 * box_w and 0.12 <= ratio <= 1.0:
                        return ex + ox, ey + oy, "topFace", (cx + ox, cy + oy)
                except cv2.error:
                    pass

    # ---------------- 退路：上部 45% 边界拟椭圆（旧法）
    band2 = ys < (y_min + 0.45 * h)
    pts = np.column_stack([xs[band2], ys[band2]]).astype(np.float32)
    if len(pts) >= 20:
        try:
            (ex, ey), (MA, ma), ang = cv2.fitEllipse(pts)
            ey += abs(ma * np.cos(np.radians(ang))) * 0.5
            return ex + ox, ey + oy, "fitEllipse", (cx + ox, cy + oy)
        except cv2.error:
            pass
    # 再退：最宽那一行的中点
    widths = mask.sum(axis=1)
    r = int(np.argmax(widths))
    cols = np.nonzero(mask[r])[0]
    return float(cols.mean()) + ox, float(r) + oy, "widestRow", (cx + ox, cy + oy)


def _ellipse_dist(pts, cx, cy, MA, ma, ang):
    """点到椭圆的近似像素距离，供 RANSAC 数内点。"""
    a, b = max(MA, ma) / 2.0, min(MA, ma) / 2.0
    th = np.radians(ang)
    dx, dy = pts[:, 0] - cx, pts[:, 1] - cy
    xr = dx * np.cos(th) + dy * np.sin(th)
    yr = -dx * np.sin(th) + dy * np.cos(th)
    r = np.sqrt((xr / max(a, 1e-3)) ** 2 + (yr / max(b, 1e-3)) ** 2)
    return np.abs(r - 1.0) * min(a, b)


def top_center_edge(img, box, iters=300, seed=0):
    """边缘法：Canny + RANSAC 拟合顶面椭圆。

    专治"浅蓝压在白盘上"这类亮度分不开的情形——亮度分不出，边却分得出。
    返回 (x, y, 内点数, 边点总数)；失败返回 None。内点数可作可信度用。
    """
    x0, y0, x1, y1 = [int(v) for v in box]
    w, h = x1 - x0, y1 - y0
    if w < 20 or h < 20:
        return None
    pad = 6
    rx0, ry0 = max(0, x0 - pad), max(0, y0 - pad)
    rx1, ry1 = min(img.shape[1], x1 + pad), min(img.shape[0], y1 + pad)
    roi = img[ry0:ry1, rx0:rx1]
    if roi.size == 0:
        return None

    g = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    g = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(g)
    e = cv2.Canny(g, 40, 120)
    ys, xs = np.nonzero(e)
    if len(xs) < 30:
        return None
    keep = ys < 0.65 * roi.shape[0]          # 只留上部：侧壁与盘沿的边不要
    pts = np.column_stack([xs[keep], ys[keep]]).astype(np.float32)
    if len(pts) < 20:
        return None

    rng = np.random.default_rng(seed)
    best = None
    for _ in range(iters):
        idx = rng.choice(len(pts), 5, replace=False)
        try:
            (cx, cy), (MA, ma), ang = cv2.fitEllipse(pts[idx])
        except cv2.error:
            continue
        if not (0.40 * w <= max(MA, ma) <= 1.25 * w):
            continue
        ratio = min(MA, ma) / max(MA, ma)
        if not (0.15 <= ratio <= 1.0):
            continue
        if not (0.05 * h <= cy <= 0.55 * h):
            continue
        inl = int((_ellipse_dist(pts, cx, cy, MA, ma, ang) < 3.0).sum())
        if best is None or inl > best[0]:
            best = (inl, cx + rx0, cy + ry0)
    if best is None:
        return None
    return best[1], best[2], best[0], len(pts)


def draw(img, items, center):
    vis = img.copy()
    H, W = vis.shape[:2]
    cv2.drawMarker(vis, (int(center[0]), int(center[1])), (255, 255, 255),
                   cv2.MARKER_CROSS, 28, 2)
    for it in items:
        x0, y0, x1, y1 = it["box"]
        cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 255, 0), 2)
        tc = it["top"]
        if tc:
            cv2.drawMarker(vis, (int(tc[0]), int(tc[1])), (0, 0, 255),
                           cv2.MARKER_CROSS, 18, 2)
            cv2.line(vis, (int(center[0]), int(center[1])), (int(tc[0]), int(tc[1])),
                     (0, 255, 255), 1)
    return vis


def main():
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description="物料顶部中心与画面中心的偏差")
    ap.add_argument("--weights", default=str(here.parent / "best.pt"))
    ap.add_argument("--image")
    ap.add_argument("--cam", type=int)
    ap.add_argument("--res", default="1280x720")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--k", type=float, default=0.0, help="每毫米像素数（顶面高度处标定）")
    ap.add_argument("--out", default="top_center.jpg")
    args = ap.parse_args()

    from ultralytics import YOLO
    model = YOLO(args.weights)

    def process(frame):
        r = model.predict(frame, imgsz=args.imgsz, conf=args.conf, verbose=False)[0]
        H, W = frame.shape[:2]
        cx0, cy0 = W / 2.0, H / 2.0
        items = []
        for b in r.boxes:
            box = [int(v) for v in b.xyxy[0].tolist()]
            conf = float(b.conf[0])
            got = material_mask(frame, box)
            tc = None
            method = "-"
            if got is not None:
                mask, off = got
                res = top_center_from_mask(frame, mask, off)
                if res:
                    tc, method = (res[0], res[1]), res[2]
                # 形状先验校验：顶心纵位须在框顶往下 20%~50%，否则按 0.34 回正
                bh = box[3] - box[1]
                if tc is not None:
                    ry = (tc[1] - box[1]) / max(1, bh)
                    if not (0.20 <= ry <= 0.50):
                        tc = ((box[0] + box[2]) / 2.0, box[1] + 0.34 * bh)
                        method = "prior"
            cid = label_of_lab(roi_lab(frame, (box[0], box[1], box[2] - box[0], box[3] - box[1]),
                                       inset=0.30))
            items.append({"box": box, "conf": conf, "top": tc, "method": method,
                          "color": COLORS.get(cid, "?")})
        return items, (cx0, cy0)

    if args.image:
        img = read_image(args.image)
        if img is None:
            raise SystemExit("读不到图")
        items, center = process(img)
        for it in items:
            if not it["top"]:
                continue
            dx, dy = it["top"][0] - center[0], it["top"][1] - center[1]
            H_ = it["box"][3]-it["box"][1]; W_ = it["box"][2]-it["box"][0]
            ry = (it["top"][1]-it["box"][1])/max(1,H_)
            line = f"{it['color']:>3} 顶心=({it['top'][0]:.0f},{it['top'][1]:.0f}) dx={dx:+.0f} dy={dy:+.0f} px  框={W_}x{H_}  顶心纵位={ry:.2f}  法={it['method']}"
            if args.k > 0:
                line += f"   ({dx/args.k:+.1f}, {dy/args.k:+.1f}) mm"
            print(line)
        vis = draw(img, items, center)
        cv2.imencode(".jpg", vis)[1].tofile(str(here.parent / "_top_center.jpg"))
        print(f"\n图幅 {img.shape[1]}x{img.shape[0]}，画面中心=({center[0]:.0f},{center[1]:.0f})")
        print("预览已存 _top_center.jpg")
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
    print("q 退出。白十字=画面中心，红十字=顶面中心，连线=二者的差。")
    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            break
        items, center = process(frame)
        vis = draw(frame, items, center)
        cv2.putText(vis, f"center=({center[0]:.0f},{center[1]:.0f})", (10, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.imshow("top_center (q quit)", vis)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
