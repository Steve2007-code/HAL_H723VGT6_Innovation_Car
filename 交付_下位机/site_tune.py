# -*- coding: utf-8 -*-
"""
site_tune.py —— 赛前现场调光台

一屏之内同时看两件事：
    · 任务一：物料检测 + **顶面圆心**（红十）
    · 任务二：号码盘检测 + 号码 + **环心**（蓝十）

调的是光：曝光、亮度、增益、白平衡。调好后按 s 落盘。

落盘去向（重要）：
    · 任务一 读 交付_下位机\\camera_settings.json   ← 本程序写这份
    · 任务二 读 任务二_放置\\camera_settings_zones.json  ← 各自独立，不联动
  两个任务的曝光是分别调的（分辨率也不同），所以这里故意不把两份并成一份。

用法：
    python site_tune.py --cam 0
    python site_tune.py --cam 0 --image 现场.jpg     # 用静态图验画面对不对

按键：
    e / E      曝光 −1 / +1          （过曝最先动它；本机实测 −6 过曝、−7 正常）
    b / B      亮度 −5 / +5
    g / G      增益 −10 / +10
    w          自动白平衡 开/关
    1 / 2      分别开关"物料"与"号码盘"两路检测（省算力）
    空格       冻结/解冻画面（冻结后可凑近看十字准不准）
    s          保存配置
    q / Esc    退出（退出前自动保存）

判读：
    mean 落在 110–180、sat 低于 5% 为好；sat > 15% 就是真过曝，软件救不了，去挪灯。
"""

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
CFG_PATH = HERE / "camera_settings.json"      # 任务一读的就是这份

# 本程序要复用两个任务各自的识别函数，故把它们的目录挂上 sys.path。
# 顺序有讲究：任务一_抓取 必须**排在前面** —— 两个目录里都有 link.py，
# 而 live_detect 导入的必须是任务一那份（带 poll 收命令的）。
# 下面按"任务二 → 任务一"的顺序 insert(0)，最后任务一自然落在最前。
import sys
for _sub in ("任务二_放置", "任务一_抓取"):
    _p = str(HERE / _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)

GOOD_MEAN = (110, 180)
SAT_LIMIT = 15.0


def load_json(path, default):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except Exception:
        return dict(default)


def apply_camera(cap, cfg):
    """把配置写进相机，返回读回的实际值。"""
    for prop, key in ((cv2.CAP_PROP_EXPOSURE, "exposure"),
                      (cv2.CAP_PROP_BRIGHTNESS, "brightness"),
                      (cv2.CAP_PROP_GAIN, "gain")):
        if cfg.get(key) is not None:
            cap.set(prop, float(cfg[key]))
    if cfg.get("auto_wb") is not None:
        cap.set(cv2.CAP_PROP_AUTO_WB, float(cfg["auto_wb"]))
    return (cap.get(cv2.CAP_PROP_EXPOSURE),
            cap.get(cv2.CAP_PROP_BRIGHTNESS),
            cap.get(cv2.CAP_PROP_GAIN),
            cap.get(cv2.CAP_PROP_AUTO_WB))


def open_camera(cam, res):
    w, h = [int(v) for v in res.lower().split("x")]
    cap = cv2.VideoCapture(cam, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(cam)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    for _ in range(5):
        cap.read()
    return cap


def main():
    ap = argparse.ArgumentParser(description="现场调光台：两路检测同屏，调好存配置")
    ap.add_argument("--cam", type=int, default=0)
    ap.add_argument("--image", default=None, help="用静态图验画面，不接相机")
    ap.add_argument("--res", default=None, help="如 1280x720；不给则用配置里的")
    ap.add_argument("--mat-weights", default=str(HERE / "任务一_抓取" / "best.pt"))
    ap.add_argument("--dial-weights", default=str(HERE / "任务二_放置" / "dial_v5.pt"))
    ap.add_argument("--conf", type=float, default=0.30)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--out", default=str(HERE / "_tune.jpg"))
    ap.add_argument("--once", action="store_true", help="只处理一帧、存图即退（不弹窗，便于验画面）")
    args = ap.parse_args()

    cfg = load_json(CFG_PATH, {"cam": 0, "res": "1280x720", "exposure": -7.0,
                               "brightness": 0.0, "gain": 0.0, "auto_wb": 0})
    if args.res:
        cfg["res"] = args.res
    cfg.setdefault("res", "1280x720")
    cfg.setdefault("exposure", -7.0)
    cfg.setdefault("brightness", 0.0)
    cfg.setdefault("gain", 0.0)
    cfg.setdefault("auto_wb", 0.0)

    # ---- 两路模型（缺文件就只跑另一路）
    from ultralytics import YOLO
    mat_model = dial_model = None
    if Path(args.mat_weights).exists():
        mat_model = YOLO(args.mat_weights)
        print(f"物料模型：{Path(args.mat_weights).name}  类别={list(mat_model.names.values())}")
    else:
        print(f"[提示] 没有物料模型 {args.mat_weights}，这一路不跑")
    if Path(args.dial_weights).exists():
        dial_model = YOLO(args.dial_weights)
        print(f"号码盘模型：{Path(args.dial_weights).name}  类别={list(dial_model.names.values())}")
    else:
        print(f"[提示] 没有号码盘模型 {args.dial_weights}，这一路不跑")

    from live_detect import estimate_top   # 顶面圆心：与任务一同一套
    from refine_center import refine_center       # 环心：与任务二同一套
    from sixcolor import COLORS, label_of_lab, roi_lab

    use_mat = mat_model is not None
    use_dial = dial_model is not None
    static = None
    cap = None
    frozen = False
    last_frame = None
    if args.image:
        static = cv2.imdecode(np.fromfile(args.image, np.uint8), cv2.IMREAD_COLOR)
        if static is None:
            raise SystemExit("读不到图")
        print(f"静态图 {static.shape[1]}×{static.shape[0]}")
    else:
        cap = open_camera(args.cam, cfg["res"])
        if cap is None:
            raise SystemExit("相机打不开")
        exp, bri, gai, awb = apply_camera(cap, cfg)
        cfg.update(exposure=exp, brightness=bri, gain=gai, auto_wb=awb)
        print(f"相机已按配置设好：exposure {exp:.0f}  亮度 {bri:.0f}  增益 {gai:.0f}  自动白平衡 {awb:.0f}")

    def save_cfg():
        cfg["updated"] = time.strftime("%Y-%m-%d %H:%M:%S")
        CFG_PATH.write_text(json.dumps(cfg, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"  已存 {CFG_PATH.name}")

    print("e/E 曝光  b/B 亮度  g/G 增益  w 自动白平衡  1/2 开关两路  空格 冻结  s 存  q 退出")
    fps, prev = 0.0, time.time()
    n_mat = n_dial = 0

    while True:
        if static is not None:
            frame = static.copy()
        elif frozen and last_frame is not None:
            frame = last_frame.copy()
        else:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("读不到帧，退出。")
                break
            last_frame = frame.copy()

        now = time.time()
        dt = now - prev
        prev = now
        if dt > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / dt) if fps else 1.0 / dt

        vis = frame.copy()
        H, W = vis.shape[:2]
        cx0, cy0 = W / 2.0, H / 2.0

        # ---------- 任务一：物料 + 顶面圆心
        n_mat = 0
        if use_mat:
            r = mat_model.predict(frame, imgsz=args.imgsz, conf=args.conf, verbose=False)[0]
            for b in r.boxes:
                x0, y0, x1, y1 = [int(v) for v in b.xyxy[0].tolist()]
                cf = float(b.conf[0])
                tc = estimate_top(frame, (x0, y0, x1, y1))
                n_mat += 1
                cid = label_of_lab(roi_lab(frame, (x0, y0, x1 - x0, y1 - y0), inset=0.30))
                cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 255, 0), 2)
                cv2.drawMarker(vis, (int(tc[0]), int(tc[1])), (0, 0, 255), cv2.MARKER_CROSS, 18, 2)
                cv2.putText(vis, f"{COLORS.get(cid, '?')} {cf:.2f}", (x0 + 3, max(14, y0 - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        # ---------- 任务二：号码盘 + 环心
        n_dial = 0
        if use_dial:
            r = dial_model.predict(frame, imgsz=args.imgsz, conf=args.conf, agnostic_nms=True,
                                   verbose=False)[0]
            for b in r.boxes:
                x0, y0, x1, y1 = [int(v) for v in b.xyxy[0].tolist()]
                cf = float(b.conf[0])
                num = str(dial_model.names[int(b.cls[0])])
                rc = refine_center(frame, (x0, y0, x1, y1))
                if rc is None or not (0 <= rc[0] < W and 0 <= rc[1] < H):
                    continue
                n_dial += 1
                cv2.rectangle(vis, (x0, y0), (x1, y1), (255, 0, 0), 2)
                cv2.drawMarker(vis, (int(rc[0]), int(rc[1])), (255, 0, 0), cv2.MARKER_CROSS, 18, 2)
                cv2.putText(vis, f"{num} {cf:.2f}", (x0 + 3, max(14, y0 - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        # ---------- 画面中心、亮度、状态
        cv2.drawMarker(vis, (int(cx0), int(cy0)), (0, 255, 0), cv2.MARKER_CROSS, 26, 2)
        g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        mean = float(g.mean())
        sat = float((g >= 250).mean() * 100.0)
        if mean < 80:
            verdict, vcol = "太暗：补光或提曝光", (0, 0, 255)
        elif sat > SAT_LIMIT:
            verdict, vcol = "过曝：降到 -7/-8 或挪灯", (0, 0, 255)
        elif GOOD_MEAN[0] <= mean <= GOOD_MEAN[1]:
            verdict, vcol = "光照合适", (0, 255, 0)
        else:
            verdict, vcol = "偏亮/偏暗，可再微调", (0, 255, 255)

        cv2.putText(vis, f"mean {mean:.0f}  sat {sat:.0f}%  exp {cfg['exposure']:.0f} "
                         f"bri {cfg['brightness']:.0f} gain {cfg['gain']:.0f} "
                         f"awb {cfg['auto_wb']:.0f}  fps {fps:.1f}",
                    (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
        cv2.putText(vis, f"{verdict}   物料 {n_mat}  盘 {n_dial}   "
                         f"{'冻结' if frozen else '实时'}   {'静态图' if static is not None else ''}",
                    (8, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.6, vcol, 2)
        cv2.putText(vis, "e/E 曝光  b/B 亮度  g/G 增益  w 自动白平衡  1/2 开关  空格 冻结  s 存  q 退",
                    (8, H - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        if args.once:
            cv2.imwrite(args.out, vis)
            print(f"已存 {args.out}  物料 {n_mat}  盘 {n_dial}  mean {mean:.0f}  sat {sat:.0f}%")
            break
        cv2.imshow("site_tune  (s=save, q=quit)", vis)
        k = cv2.waitKey(1) & 0xFF
        if k in (ord("q"), 27):
            break
        elif k == ord("s"):
            save_cfg()
            cv2.imwrite(args.out, vis)
        elif k == 32:
            frozen = not frozen
        elif k == ord("1") and mat_model is not None:
            use_mat = not use_mat
            print("  物料检测", "开" if use_mat else "关")
        elif k == ord("2") and dial_model is not None:
            use_dial = not use_dial
            print("  号码盘检测", "开" if use_dial else "关")
        elif cap is not None:
            if k == ord("e"):
                cfg["exposure"] -= 1.0
                cap.set(cv2.CAP_PROP_EXPOSURE, cfg["exposure"])
            elif k == ord("E"):
                cfg["exposure"] += 1.0
                cap.set(cv2.CAP_PROP_EXPOSURE, cfg["exposure"])
            elif k == ord("b"):
                cfg["brightness"] = max(0.0, cfg["brightness"] - 5.0)
                cap.set(cv2.CAP_PROP_BRIGHTNESS, cfg["brightness"])
            elif k == ord("B"):
                cfg["brightness"] = min(255.0, cfg["brightness"] + 5.0)
                cap.set(cv2.CAP_PROP_BRIGHTNESS, cfg["brightness"])
            elif k == ord("g"):
                cfg["gain"] = max(0.0, cfg["gain"] - 10.0)
                cap.set(cv2.CAP_PROP_GAIN, cfg["gain"])
            elif k == ord("G"):
                cfg["gain"] = min(255.0, cfg["gain"] + 10.0)
                cap.set(cv2.CAP_PROP_GAIN, cfg["gain"])
            elif k == ord("w"):
                cfg["auto_wb"] = 0.0 if cfg["auto_wb"] else 1.0
                cap.set(cv2.CAP_PROP_AUTO_WB, cfg["auto_wb"])
                print(f"  自动白平衡 {'开' if cfg['auto_wb'] else '关'}")

    if cap is not None:
        cfg["exposure"] = cap.get(cv2.CAP_PROP_EXPOSURE)
        cfg["brightness"] = cap.get(cv2.CAP_PROP_BRIGHTNESS)
        cfg["gain"] = cap.get(cv2.CAP_PROP_GAIN)
        cfg["cam"] = args.cam
        cfg["res"] = f"{W}x{H}"
        cap.release()
    save_cfg()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
