# -*- coding: utf-8 -*-
"""
refine_center.py —— YOLO 给了框之后，用传统法把中心抠精

YOLO 的框中心有 ±3~8 像素抖动；号码盘的同心环却是极好的几何基准。
故：框只用来"圈定范围"，中心由环来定。

三招，依次尝试：
    ① 外环拟合圆（ring_center）——最稳。虚线环不碍事，只要外圈那些暗点在。
       Hough 曾被弃用：虚线段使其投票偏移，实拍里 1 号盘偏左 27 像素。
    ② Hough ——外环点太少时的替补。
    ③ 椭圆/外接圆 —— 暗环闭成环带后取外轮廓，斜视下可用。

返回 (cx, cy, 用法)；失败返回 None。
"""

import cv2
import numpy as np

from ring_center import ring_center


def _expand(box, frac=0.12):
    x0, y0, x1, y1 = [int(v) for v in box[:4]]
    dx = int((x1 - x0) * frac)
    dy = int((y1 - y0) * frac)
    return x0 - dx, y0 - dy, x1 + dx, y1 + dy


def refine_center(img, box):
    """在 YOLO 框内精求盘心。返回 (cx, cy, 用法) 或 None。"""
    rc = ring_center(img, box)
    if rc is not None:
        return float(rc[0]), float(rc[1]), "ring"

    H, W = img.shape[:2]
    ex0, ey0, ex1, ey1 = _expand(box)
    rx0, ry0 = max(0, ex0), max(0, ey0)
    rx1, ry1 = min(W, ex1), min(H, ey1)
    roi = img[ry0:ry1, rx0:rx1]
    if roi.size == 0:
        return None
    h, w = roi.shape[:2]
    bx = (box[0] + box[2]) / 2.0 - rx0
    by = (box[1] + box[3]) / 2.0 - ry0

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    g = cv2.GaussianBlur(gray, (7, 7), 1.5)
    r_guess = min(w, h) / 2.0
    circles = cv2.HoughCircles(g, cv2.HOUGH_GRADIENT, dp=1.2,
                               minDist=max(20, int(r_guess * 0.5)),
                               param1=120, param2=45,
                               minRadius=int(r_guess * 0.45),
                               maxRadius=int(r_guess * 1.05))
    if circles is not None:
        best, bd = None, 1e9
        for x, y, r in circles[0]:
            d = ((x - bx) ** 2 + (y - by) ** 2) ** 0.5
            if d < bd:
                bd, best = d, (x, y)
        if best is not None:
            return float(best[0] + rx0), float(best[1] + ry0), "hough"

    # 椭圆：取暗环，闭成环带，外轮廓拟合
    m = (gray < 120).astype(np.uint8) * 255
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k, iterations=3)
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts:
        if cv2.contourArea(c) < 0.15 * w * h:
            continue
        if cv2.pointPolygonTest(c, (float(bx), float(by)), False) < 0:
            continue
        if len(c) >= 5:
            try:
                (cx, cy), _ax, _ang = cv2.fitEllipse(c)
                return float(cx + rx0), float(cy + ry0), "ellipse"
            except cv2.error:
                pass
        (cx, cy), _r = cv2.minEnclosingCircle(c)
        return float(cx + rx0), float(cy + ry0), "circle"
    return None
