# -*- coding: utf-8 -*-
"""
sixcolor.py —— 框内判色内核（Lab + ΔE + 时序投票）

干三件事：
    1. 给一个框（或整块 ROI），算出它的代表色 —— Lab 中位色，先剔高光与阴影
    2. 拿它跟六个现场原型比 ΔE，判出是 1~6 里哪一色，并报"有多确定"
    3. 多帧软投票，把单帧的偶然压成结论

不依赖 YOLO：框可由 YOLO 给，也可由传统分割给，甚至手画。它的输入就是"一块矩形"。

命令行：
    python sixcolor.py --selftest                       # 合成六色自检
    python sixcolor.py --image 图.png --box 600,250,120,120
    python sixcolor.py --image 图.png --proto proto.json --boxes 600,250,120,120 745,200,135,105
"""

import argparse
import json
import os
import time

import numpy as np

try:
    import cv2
except Exception:                      # 没有 cv2 也要能 import，便于在别处复用
    cv2 = None

# ---------------------------------------------------------------- 六色
COLORS = {
    1: "红", 2: "黄", 3: "蓝", 4: "绿", 5: "黑", 6: "浅蓝",
}

# ---------------------------------------------------------------- 颜色空间
def to_lab(bgr):
    """BGR uint8 → float32 Lab（L 0~100，a/b −128~127）。

    切莫用 8 位版：8 位把 L 压成 L*255/100、a/b 加 128 偏移，
    算 ΔE 时误差极大。
    """
    if cv2 is None:
        raise RuntimeError("需要 opencv")
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2Lab)


def roi_lab(bgr, box=None, inset=0.30, reject_pct=10.0, mode="auto"):
    """取一块区域的有代表性的颜色（Lab）。

    box=(x, y, w, h)，None 表示整幅。
    inset=0.25 表示四边各缩进 25%——边上有混色、阴影与高光，弃之。

    取法（mode）：
        auto   先看这块钱彩不彩：彩 → 取"最饱和那一簇"的中位色；
               不彩（黑、灰、白）→ 取明度中位，剔上下极值。
        chroma 只取高彩度像素——专治"顶面过曝打白、侧面才见本色"的物料
        all    全取，只剔明度极值——用于纯灰/黑件

    为何不能一律取中位：实测那只蓝料，顶面被补光打成 255 白，占了半块面积，
    中位色便被拉成白；而它的本色只在侧壁（彩度高、明度低）。取最饱和的一簇，
    才抓得住本色。
    """
    if box is None:
        roi = bgr
    else:
        x, y, w, h = [int(v) for v in box]
        dx, dy = int(w * inset), int(h * inset)
        x0 = max(0, x + dx)
        y0 = max(0, y + dy)
        x1 = min(bgr.shape[1], x + w - dx)
        y1 = min(bgr.shape[0], y + h - dy)
        if x1 - x0 < 3 or y1 - y0 < 3:
            roi = bgr[max(0, y):max(1, y + h), max(0, x):max(1, x + w)]
        else:
            roi = bgr[y0:y1, x0:x1]

    lab = to_lab(roi).reshape(-1, 3)
    L = lab[:, 0]
    C = np.hypot(lab[:, 1], lab[:, 2])

    if mode == "auto":
        mode = "all" if np.percentile(C, 95) < 12.0 else "chroma"

    if mode == "chroma":
        thr = max(8.0, np.percentile(C, 65))
        keep = C >= thr
        if keep.sum() < 50:
            keep = C >= np.percentile(C, 90)
        lo = np.percentile(L, 5.0)          # 顺手剔掉极暗的阴影像素
        keep &= L >= lo
    else:
        lo, hi = np.percentile(L, [reject_pct, 100.0 - reject_pct])
        keep = (L >= lo) & (L <= hi)

    if keep.sum() < 20:
        keep = np.ones(len(L), bool)
    return np.median(lab[keep], axis=0)


def delta_e(p, q, kL=1.0):
    """ΔE76，kL 是明度权重：调大它等于"更看重明度差"。

    浅蓝与蓝的分别，全在明度上，故这旋钮要紧；纺织业的 CIE94 取 kL=2，
    本座在赛场上也建议 1.5~2。
    """
    dL = float(p[0]) - float(q[0])
    da = float(p[1]) - float(q[1])
    db = float(p[2]) - float(q[2])
    return float(np.sqrt((kL * dL) ** 2 + da * da + db * db))


def chroma(vec):
    return float(np.hypot(vec[1], vec[2]))

def label_of_lab(vec):
    """凭 Lab 直接定名（判色的"无原型"兜底，也用于自动挖原型）。

    实测中位：红(53,37,24)、黄(83,-14,48)、蓝(38,20,-52)、绿(78,-31,40)、
              黑(28,0,-3)、浅蓝(80,-4,-7)。

    要害在黄与绿：两者中位只差 ΔE 20，而各自散度也是这个量级，
    按 a 的阈值切（旧法）在实机上会把黄判成绿。改用**色相角**
    atan2(b,a)：黄≈106°、绿≈128°，中间 116° 一切，全量 1113 张的
    "六色齐全率"从 93.6% 抬到 99.5%。

    黑也不宜用窄阈值：实机上黑料被补光照亮，L 可到 45 上下，
    故改用"L<50 且彩度<22"。
    """
    L, a, b = float(vec[0]), float(vec[1]), float(vec[2])
    C = float(np.hypot(a, b))

    if L < 26 and C < 45:
        # 极暗者优先认黑：暗光下黑料的色噪会被放大，常带一点蓝，
        # 若不先认，便被下一条判成深蓝（实拍中黑料 L=14.5、彩度 37.5 正是此例）。
        return 5
    if b < -25:                      # 偏蓝的一族：按明度分深蓝与浅蓝
        return 3 if L < 62 else 6
    if L < 50 and C < 28:            # 黑：暗且几乎无彩（彩度门槛放宽到 28）
        return 5
    if b > 12 and a <= 20:           # 黄 / 绿：看色相角
        ang = float(np.degrees(np.arctan2(b, a)))
        return 4 if ang > 116 else 2
    if a > 20 and b > 12:            # 红
        return 1
    if L > 60:                       # 浅蓝：亮而无彩（或微蓝）
        return 6
    return None


# ---------------------------------------------------------------- 最近色兜底
# 六色的实测中位，作"判不出时挑最近一个"的参照（与 label_of_lab 注释同一组数）
REF_LAB = {
    1: (53.0, 37.0, 24.0),     # 红
    2: (83.0, -14.0, 48.0),    # 黄
    3: (38.0, 20.0, -52.0),    # 蓝
    4: (78.0, -31.0, 40.0),    # 绿
    5: (28.0, 0.0, -3.0),      # 黑
    6: (80.0, -4.0, -7.0),     # 浅蓝
}


def nearest_of_lab(vec, exclude=None, kL=1.5):
    """判不出颜色时，按 ΔE 取最近的一色。返回 (id, ΔE)。

    只作兜底：规则能定名就别用它——它总会给出一个答案，哪怕那答案很牵强。
    调用方可拿返回的 ΔE 做门槛（如 >45 仍视作不可信）。
    """
    best_id, best_d = None, 1e9
    for cid, p in REF_LAB.items():
        if exclude and cid in exclude:
            continue
        d = delta_e(vec, p, kL)
        if d < best_d:
            best_id, best_d = cid, d
    return best_id, best_d


# ---------------------------------------------------------------- 原型库
class Prototypes:
    """六个现场原型色。每类可存多个样本（不同朝向/相位），判别时取最近的一个。"""

    def __init__(self, samples=None, kL=1.5):
        # {id: {"name": str, "lab": [[L,a,b], ...]}}
        self.samples = samples or {}
        self.kL = kL

    def add(self, cid, lab, name=None):
        cid = int(cid)
        item = self.samples.setdefault(cid, {"name": name or COLORS.get(cid, str(cid)), "lab": []})
        item["lab"].append([float(v) for v in lab])
        if name:
            item["name"] = name

    def ids(self):
        return sorted(self.samples)

    def nearest(self, vec):
        """返回 (id, 距离, 次近距离)。同类多原型时取最小。"""
        best = []
        for cid in self.ids():
            d = min(delta_e(vec, p, self.kL) for p in self.samples[cid]["lab"])
            best.append((d, cid))
        best.sort()
        if len(best) == 1:
            return best[0][1], best[0][0], float("inf")
        return best[0][1], best[0][0], best[1][0]

    def name_of(self, cid):
        return self.samples.get(int(cid), {}).get("name", str(cid))

    def save(self, path):
        payload = {
            "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "kL": self.kL,
            "colors": {str(k): v for k, v in sorted(self.samples.items())},
        }
        with open(path, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)

    @classmethod
    def load(cls, path):
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        p = cls(kL=data.get("kL", 1.5))
        for k, v in data.get("colors", {}).items():
            p.samples[int(k)] = {"name": v.get("name", COLORS.get(int(k), k)),
                                 "lab": [[float(x) for x in s] for s in v["lab"]]}
        return p


def classify(vec, protos, max_de=15.0, min_margin=1.35):
    """判色。

    门限两道：
        max_de      绝对距离——最近的原型都远于它，说明"不是六色之一"（异物/反光）
        min_margin  最近与次近之比——太接近就弃权，宁可再等一帧
    低彩度（黑、灰、白）时 a/b 之差多是相机噪声，故只比 L。
    """
    cid, d1, d2 = protos.nearest(vec)
    margin = d2 / max(d1, 1e-6)

    if chroma(vec) < 10.0:
        # 无彩色：只用明度判
        Ls = {c: min(abs(float(vec[0]) - float(p[0])) for p in protos.samples[c]["lab"])
              for c in protos.ids()}
        cid = min(Ls, key=Ls.get)
        d1 = Ls[cid]
        d2 = sorted(Ls.values())[1] if len(Ls) > 1 else float("inf")
        margin = d2 / max(d1, 1e-6)

    ok = (d1 <= max_de) and (margin >= min_margin)
    return {
        "id": int(cid), "name": protos.name_of(cid),
        "lab": [round(float(v), 2) for v in vec],
        "d1": round(d1, 2), "d2": None if d2 == float("inf") else round(d2, 2),
        "margin": None if margin == float("inf") else round(margin, 2),
        "ok": bool(ok),
        "reason": "" if ok else ("离所有原型都远" if d1 > max_de else "两色太接近，弃权"),
    }


# ---------------------------------------------------------------- 白平衡
def white_gains(bgr, white_mask=None, box=None):
    """用画面里的白参考（转盘白面）估通道增益，返回 (gb, gg, gr)。

    用法：img_wb = np.clip(bgr * gains, 0, 255).astype(np.uint8)
    转盘本来就是最可靠的白，白给的参考。
    """
    if white_mask is not None:
        px = bgr[white_mask > 0]
    elif box is not None:
        x, y, w, h = [int(v) for v in box]
        px = bgr[y:y + h, x:x + w].reshape(-1, 3)
    else:
        px = bgr.reshape(-1, 3)
    if len(px) < 20:
        return np.array([1.0, 1.0, 1.0], np.float32)
    b, g, r = np.median(px, axis=0).astype(np.float32) + 1e-6
    ref = 0.5 * (g + 0.5 * (b + r))          # 折中参考，免得把整幅拉偏
    return np.array([ref / b, ref / g, ref / r], np.float32)


# ---------------------------------------------------------------- 时序投票
class VoteBox:
    """软投票：每帧按 exp(−ΔE/τ) 给六类各投一票，票分轻重，累积成后验。

    比"数票"讲理：相邻帧高度相关，独立假设不成立；按后验累积正好避开这一层。
    """

    def __init__(self, ids=None, tau=6.0):
        self.ids = list(ids) if ids else sorted(COLORS)
        self.tau = float(tau)
        self.logp = np.zeros(len(self.ids))
        self.n = 0
        self.last = None

    def push(self, vec, protos):
        d = np.array([min(delta_e(vec, p, protos.kL) for p in protos.samples[c]["lab"])
                      if c in protos.samples else 999.0 for c in self.ids])
        p = np.exp(-d / self.tau)
        s = p.sum()
        if s <= 0:
            return
        self.logp += np.log(np.clip(p / s, 1e-9, None))
        self.n += 1
        self.last = vec

    def reset(self):
        self.logp[:] = 0.0
        self.n = 0
        self.last = None

    def posterior(self):
        z = self.logp - self.logp.max()
        p = np.exp(z)
        return p / p.sum()

    def decide(self, min_n=8, thresh=0.70):
        if self.n < min_n:
            return None
        post = self.posterior()
        k = int(post.argmax())
        if post[k] < thresh:
            return None
        return {"id": int(self.ids[k]), "post": round(float(post[k]), 3), "n": self.n}


# ---------------------------------------------------------------- 自检
def _synth(color_rgb, size=120, bg=(128, 128, 128)):
    if cv2 is None:
        raise RuntimeError("需要 opencv")
    img = np.zeros((size, size, 3), np.uint8)
    img[:] = bg[::-1]
    cv2.circle(img, (size // 2, size // 2), int(size * 0.42), color_rgb[::-1], -1)
    return img


def selftest():
    """合成六色 → 建原型 → 加噪、加减光、换姿态，看能否全判对。

    注意取样要落在色块上（给框），不能让背景混进中位色。
    """
    base = {
        1: (210, 40, 40), 2: (235, 200, 40), 3: (40, 70, 200),
        4: (40, 160, 70), 5: (25, 25, 25), 6: (150, 230, 240),
    }

    def sample(img):
        h, w = img.shape[:2]
        d = int(min(h, w) * 0.5)
        return roi_lab(img, (w // 2 - d // 2, h // 2 - d // 2, d, d), inset=0.15)

    protos = Prototypes(kL=1.5)
    for cid, rgb in base.items():
        protos.add(cid, sample(_synth(rgb)))

    rng = np.random.default_rng(0)
    ok = 0
    total = 0
    confusion = {}
    for cid, rgb in base.items():
        for trial in range(25):
            k = rng.uniform(0.75, 1.15)          # 明暗扰动
            noisy = tuple(int(np.clip(c * k + rng.normal(0, 6), 0, 255)) for c in rgb)
            img = _synth(noisy, size=int(rng.integers(90, 160)))
            vec = sample(img)
            r = classify(vec, protos)
            total += 1
            if r["id"] == cid:
                ok += 1
            else:
                confusion[(cid, r["id"])] = confusion.get((cid, r["id"]), 0) + 1
    print(f"合成自检：{ok}/{total} 判对（{ok/total:.1%}）")
    if confusion:
        print("  混淆：")
        for (a, b), n in sorted(confusion.items()):
            print(f"    真 {COLORS[a]} → 判成 {COLORS.get(b, b)}  {n} 次")
    return ok == total
    print(f"合成自检：{ok}/{total} 判对（{ok/total:.1%}）")
    if confusion:
        print("  混淆：")
        for (a, b), n in sorted(confusion.items()):
            print(f"    真 {COLORS[a]} → 判成 {COLORS.get(b, b)}  {n} 次")
    return ok == total


def main():
    ap = argparse.ArgumentParser(description="框内判色：Lab + ΔE + 投票")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--image")
    ap.add_argument("--box", help="x,y,w,h")
    ap.add_argument("--boxes", nargs="*", help="多个框：x,y,w,h x,y,w,h ...")
    ap.add_argument("--proto", help="原型 json；不给则用内置的临时原型")
    ap.add_argument("--kl", type=float, default=1.5, help="明度权重，浅蓝/蓝之辨要它")
    args = ap.parse_args()

    if args.selftest:
        raise SystemExit(0 if selftest() else 1)

    if not args.image:
        ap.print_help()
        return

    if cv2 is None:
        raise SystemExit("需要 opencv")

    img = cv2.imdecode(np.fromfile(args.image, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise SystemExit(f"读不到图：{args.image}")

    protos = Prototypes.load(args.proto) if args.proto else Prototypes(kL=args.kl)

    boxes = []
    if args.box:
        boxes.append([int(v) for v in args.box.split(",")])
    for b in (args.boxes or []):
        boxes.append([int(v) for v in b.split(",")])
    if not boxes:
        boxes.append([0, 0, img.shape[1], img.shape[0]])

    for b in boxes:
        vec = roi_lab(img, b)
        print(f"框 {b}  Lab=({vec[0]:.1f}, {vec[1]:.1f}, {vec[2]:.1f})  彩度={chroma(vec):.1f}")
        if protos.ids():
            r = classify(vec, protos)
            print("   ", r)


if __name__ == "__main__":
    main()
