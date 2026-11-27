# -*- coding: utf-8 -*-
"""
环境自检.py —— 交付包能不能跑，一跑便知

查五样：
    ① Python 版本与四个依赖库
    ② 两个模型文件在不在、能不能加载
    ③ 相机能不能打开
    ④ 串口能不能打开（给了 --com 才查）
    ⑤ 把该发的报文印出来，不真发

用法：
    python 环境自检.py
    python 环境自检.py --com COM3 --baud 115200
"""

import argparse
import sys
from pathlib import Path

OK = "[通过]"
NO = "[失败]"


def main():
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser()
    ap.add_argument("--com", default=None, help="串口号，如 COM3；给了才查串口")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--cam", type=int, default=0)
    ap.add_argument("--no-camera", action="store_true", help="跳过相机（远程调通信时可加）")
    args = ap.parse_args()

    print("=" * 60)
    print("Python", sys.version.split()[0], "|", sys.executable)
    print("=" * 60)

    # ① 依赖
    print("\n【一】依赖库")
    libs = {}
    for mod, name in [("cv2", "opencv-python"), ("numpy", "numpy"),
                      ("torch", "torch"), ("ultralytics", "ultralytics"),
                      ("serial", "pyserial")]:
        try:
            m = __import__(mod)
            ver = getattr(m, "__version__", "?")
            libs[mod] = True
            print(f"  {OK} {name:<16} {ver}")
        except Exception as e:
            libs[mod] = False
            print(f"  {NO} {name:<16} {e}")
    if not all(libs.values()):
        print("\n缺库请执行：")
        print("  pip install -r requirements.txt")
        print("（torch 若装不动，用 CPU 版：pip install torch --index-url "
              "https://download.pytorch.org/whl/cpu）")

    # ② 模型
    print("\n【二】模型文件")
    models = {"任务一（物料）": here / "任务一_抓取" / "best.pt",
              "任务二（号码盘）": here / "任务二_放置" / "dial_v3.pt"}
    for tag, p in models.items():
        if not p.exists():
            print(f"  {NO} {tag:<14} 缺文件 {p.name}")
            continue
        print(f"  {OK} {tag:<14} {p.name}  {p.stat().st_size/1e6:.1f} MB", end="")
        if libs.get("ultralytics"):
            try:
                from ultralytics import YOLO
                m = YOLO(str(p))
                print(f"  类别={m.names}")
            except Exception as e:
                print(f"  但加载失败：{e}")
        else:
            print()

    # ③ 相机
    print("\n【三】相机")
    if args.no_camera:
        print("  （已跳过）")
    elif libs.get("cv2"):
        import cv2
        cap = cv2.VideoCapture(args.cam, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap = cv2.VideoCapture(args.cam)
        if cap.isOpened():
            ok, fr = cap.read()
            if ok and fr is not None:
                h, w = fr.shape[:2]
                print(f"  {OK} 相机 {args.cam} 打开，出帧 {w}×{h}")
            else:
                print(f"  {NO} 相机开得了，但读不到帧（镜头被挡？）")
        else:
            print(f"  {NO} 相机 {args.cam} 打不开（被占用或编号不对）")
        cap.release()
    else:
        print("  （缺 opencv，跳过）")

    # ④ 串口
    print("\n【四】串口")
    if not args.com:
        print("  （未给 --com，跳过；要查请加 --com COM3）")
    elif not libs.get("serial"):
        print(f"  {NO} 缺 pyserial，装：pip install pyserial")
    else:
        import serial
        try:
            s = serial.Serial(args.com, args.baud, timeout=0.5)
            s.close()
            print(f"  {OK} {args.com} @ {args.baud} 打开并关闭成功")
        except Exception as e:
            print(f"  {NO} {args.com} 打不开：{e}")
            print("       常见原因：串口被别的程序占用 / 号填错 / 需要 USB 驱动")

    # ⑤ 报文
    print("\n【五】该发的报文（只印不发）")
    print("  任务一无目标：<<X,9999,Y,9999>>")
    print("  任务一有目标：<<X,-24,Y,35>>")
    print("  任务二无盘  ：<<N,0,X,9999,Y,9999>>")
    print("  任务二对准  ：<<N,2,X,-8,Y,14>>")

    print("\n" + "=" * 60)
    print("自检完。都通过就可以接串口跑了。")
    print("=" * 60)


if __name__ == "__main__":
    main()
