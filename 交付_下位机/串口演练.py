# -*- coding: utf-8 -*-
"""
串口演练.py —— 不接相机、不开模型，只往串口灌报文

用途：下位机作者先验"线通了没有、帧怎么切、数值怎么解"。
它按协议依次发几种典型帧，每次打印发了什么，让对面逐条核对。

用法：
    python 串口演练.py --com COM3 --baud 115200
    python 串口演练.py --com COM3 --rounds 3      # 每帧重发几遍

发了些什么：
    1) <<X,9999,Y,9999>>  无目标
    2) <<X,0,Y,0>>        正好在中心
    3) <<X,-24,Y,35>>     左 24、下 35
    4) <<X,24,Y,-35>>     右 24、上 35
    5) <<N,0,X,9999,Y,9999>>  任务二无盘
    6) <<N,2,X,-8,Y,14>>      任务二 2 号盘
"""

import argparse
import sys
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--com", default="COM12")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--rounds", type=int, default=2, help="每帧重发几遍")
    ap.add_argument("--gap", type=float, default=0.5, help="每帧之间停几秒")
    args = ap.parse_args()

    frames = [
        ("任务一·无目标", "<<X,9999,Y,9999>>"),
        ("任务一·居中",   "<<X,0,Y,0>>"),
        ("任务一·左24下35", "<<X,-24,Y,35>>"),
        ("任务一·右24上35", "<<X,24,Y,-35>>"),
        ("任务二·无盘",   "<<N,0,X,9999,Y,9999>>"),
        ("任务二·2号盘",  "<<N,2,X,-8,Y,14>>"),
    ]

    try:
        import serial
    except ImportError:
        print("缺 pyserial。装：pip install pyserial")
        print("或直接用任何串口助手手工发送下面这些报文：")
        for tag, f in frames:
            print(f"  {tag:<14} {f}")
        return 1

    try:
        s = serial.Serial(args.com, args.baud, timeout=0.5)
    except Exception as e:
        print(f"打开 {args.com} 失败：{e}")
        return 1

    print(f"已打开 {args.com} @ {args.baud}，开始发帧（Ctrl+C 停止）\n")
    try:
        for tag, f in frames:
            for _ in range(max(1, args.rounds)):
                s.write(f.encode("ascii"))
                s.flush()
                print(f"  {tag:<14} 发送 {f}")
                time.sleep(0.05)
            time.sleep(args.gap)
        print("\n全部发完。若对面一字未收，先查波特率与接线（TX/RX、GND）。")
    except KeyboardInterrupt:
        print("\n已中断")
    finally:
        s.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
