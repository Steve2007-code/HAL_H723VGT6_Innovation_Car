# -*- coding: utf-8 -*-
"""
ring_center.py —— 用"最外一圈环"拟合圆心（比 Hough 稳）

为何弃 Hough：号码盘是**虚线同心环**，Hough 靠边缘投票，虚线段一断，票数便
往一侧堆，计出的圆心能偏二三十像素。实拍验过：1 号盘偏左 27px、2 号盘偏上 14px。

本法：
    ① 框内取暗像素（环与数字都是暗的），开运算去散点；
    ② 以框心为初值，取各暗点到框心的半径，第 99 分位记作 R；
    ③ 只留 0.75R～1.05R 那一圈（外环）的点，做**Kasa 最小二乘圆拟合**；
    ④ 以新圆心重算半径，再拟合一遍。

返回 (cx, cy, 点数) 或 None。
"""

import cv2
import numpy as np


def _kasa_fit(pts):
    """代数法圆拟合：x²+y² = 2ax + 2by + c。返回 (a, b, r) 或 None。"""
    if len(pts) < 10:
        return None
    x = pts[:, 0].astype(np.float64)
    y = pts[:, 1].astype(np.float64)
    A = np.column_stack([2 * x, 2 * y, np.ones_like(x)])
    b = x * x + y * y
    try:
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    except np.linalg.LinAlgError:
        return None
    a, bb, c = sol
    r2 = c + a * a + bb * bb
    if r2 <= 0:
        return None
    return float(a), float(bb), float(np.sqrt(r2))


def ring_center(img, box, iters=2, dark_thr=None):
    x0, y0, x1, y1 = [int(v) for v in box[:4]]
    H, W = img.shape[:2]
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(W, x1), min(H, y1)
    if x1 - x0 < 20 or y1 - y0 < 20:
        return None
    roi = cv2.cvtColor(img[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
    if dark_thr is None:
        # 自适应阈值：整幅光不均时，固定 120 会把较亮那一侧的环丢掉，
        # 拟合便朝有环的一侧偏（实测可偏 20–40 像素）。Otsu 随画面走。
        otsu, _ = cv2.threshold(roi, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        dark_thr = float(np.clip(otsu, 60, 200))
    m = (roi < dark_thr).astype(np.uint8) * 255
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k, iterations=1)
    ys, xs = np.nonzero(m)
    if len(xs) < 50:
        return None
    pts = np.column_stack([xs + x0, ys + y0]).astype(np.float64)   # 还原到整幅坐标

    cx = (x0 + x1) / 2.0
    cy = (y0 + y1) / 2.0
    r = None
    for _ in range(max(1, iters)):
        d = np.hypot(pts[:, 0] - cx, pts[:, 1] - cy)
        R = np.percentile(d, 99)
        if R < 8:
            return None
        band = (d > 0.75 * R) & (d < 1.05 * R)
        sel = pts[band]
        fit = _kasa_fit(sel)
        if fit is None:
            return None
        cx, cy, r = fit
    if r is None or r < 8:
        return None
    return cx, cy, int(band.sum())
