# -*- coding: utf-8 -*-
"""
check_pi.py —— 香橙派环境自检（第一次上板先跑它）

逐项报告：依赖库 / 模型 / 摄像头 / 串口 / 推理速度 / 报文通道。
全 [通过] 再跑 run_task1.sh，能省掉大半现场排查时间。

    python3 check_pi.py
    python3 check_pi.py --cam 0 --com /dev/ttyUSB0 --send-test

NPU 路线自检（注意：要先 conda activate base + source set_env.sh）：
    python3 check_pi.py --backend om
    python3 check_pi.py --backend om --model best.om
"""

import argparse
import os
import platform
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
OK, BAD = '[ 通过 ]', '[ 失败 ]'
fail = 0


def chk(name, fn, optional=False):
    global fail
    try:
        detail = fn()
        print('%s %-22s %s' % (OK, name, detail or ''))
        return True
    except Exception as e:
        if not optional:
            fail += 1
        print('%s %-22s %s' % ('[ 可选 ]' if optional else BAD, name, e))
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cam', default='0',
                    help='摄像头号。AIpro 重启后 /dev/video* 会重排：'
                         '板载 vin-media/isp-vir0 节点不是摄像头，'
                         '用 v4l2-ctl --list-devices 找 (usb-...) 那一组')
    ap.add_argument('--com', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--model', default=None,
                    help='默认按后端自动选：onnx→best.onnx / om→best.om / pt→best.pt')
    ap.add_argument('--imgsz', type=int, default=640,
                    help='须与模型导出/ATC 转换尺寸一致：640 / 320 / 256')
    ap.add_argument('--backend', default='onnx',
                    choices=['onnx', 'cvdnn', 'pt', 'om', 'none'])
    ap.add_argument('--device', type=int, default=0, help='om 后端的 NPU 设备号')
    ap.add_argument('--send-test', action='store_true',
                    help='往串口发一条 <<X,9999,Y,9999>>（下位机会看到，注意场合）')
    a = ap.parse_args()

    if a.model is None:
        default_model = {'om': 'best.om', 'pt': 'best.pt'}.get(a.backend, 'best.onnx')
        a.model = str(HERE / default_model)

    print('=' * 62)
    print('香橙派 · 任务一抓取 环境自检')
    print('=' * 62)
    print('  系统    : %s %s / %s' % (platform.system(), platform.release(),
                                    platform.machine()))
    print('  Python  : %s' % sys.version.split()[0])
    print('  DISPLAY : %s' % (os.environ.get('DISPLAY') or '（空 → 只能无窗口跑）'))
    print('  目录    : %s' % HERE)
    print('  后端    : %s   模型: %s' % (a.backend, os.path.basename(a.model)))
    print('-' * 62)

    def _acl():
        __import__('acl')
        return '可用 → 可跑 BACKEND=om'

    chk('numpy', lambda: __import__('numpy').__version__)
    chk('cv2 (opencv)', lambda: __import__('cv2').__version__)
    chk('PIL (可选)', lambda: __import__('PIL').__version__, optional=True)
    chk('pyserial', lambda: __import__('serial').VERSION)
    chk('onnxruntime (可选)', lambda: __import__('onnxruntime').__version__,
        optional=True)
    chk('pyACL / acl (可选)', _acl, optional=True)
    chk('sixcolor/taskcode/top_center/link', lambda: (
        __import__('sixcolor'), __import__('taskcode'),
        __import__('top_center'), __import__('link'), 'ok')[4])

    chk('模型 %s' % os.path.basename(a.model),
        lambda: '%d B' % os.path.getsize(a.model))
    chk('best.pt', lambda: '%d B' % os.path.getsize(HERE / 'best.pt'))
    chk('best_opset11.onnx (可选)',
        lambda: '%d B' % os.path.getsize(HERE / 'best_opset11.onnx'), optional=True)
    chk('best.om (可选)',
        lambda: '%d B' % os.path.getsize(HERE / 'best.om'), optional=True)

    # ---- 摄像头
    import glob
    devs = sorted(glob.glob('/dev/video*'))
    print('\n摄像头设备：', ', '.join(devs) or '（一个都没有，检查 USB 线）')

    def _cam():
        import cv2
        cam = a.cam
        if isinstance(cam, str) and cam.isdigit():
            cam = int(cam)
        v4l2 = getattr(cv2, 'CAP_V4L2', 200)
        cap = cv2.VideoCapture(cam, v4l2)
        if not cap.isOpened():
            cap = cv2.VideoCapture(cam)
        if not cap.isOpened():
            raise RuntimeError('打不开 %s' % a.cam)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        t = time.time()
        ok, fr = cap.read()
        dt = (time.time() - t) * 1000
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        cap.release()
        if not ok or fr is None:
            raise RuntimeError('能打开但读不到帧（换 --cam 号 / 检查权限）')
        return '%dx%d  首帧 %.0f ms  mean=%.0f' % (
            w, h, dt, float(__import__('cv2').cvtColor(
                fr, __import__('cv2').COLOR_BGR2GRAY).mean()))

    chk('摄像头读帧', _cam)

    # ---- 串口
    from link import list_serial_ports
    ports = list_serial_ports()
    print('\n串口设备：', ', '.join(ports) or '（没找到，检查 USB-TTL / 40pin UART）')

    def _ser():
        if not os.path.exists(a.com):
            raise RuntimeError('%s 不存在；可用：%s' % (a.com, ports))
        st = os.stat(a.com)
        mode = oct(st.st_mode & 0o777)
        if not os.access(a.com, os.W_OK):
            raise RuntimeError('无写权限 %s（%s）→ sudo usermod -aG dialout $USER 后重登' %
                               (a.com, mode))
        import serial
        s = serial.Serial(a.com, a.baud, timeout=0.05)
        s.close()
        return '%s @ %d 可打开  权限 %s' % (a.com, a.baud, mode)

    chk('串口可打开', _ser)

    # ---- 推理
    def _infer():
        import numpy as np
        from yolo_backend import Detector
        d = Detector(a.backend, a.model, imgsz=a.imgsz, device_id=a.device)
        img = np.full((480, 640, 3), 128, np.uint8)
        for _ in range(2):
            d(img)
        t = time.time()
        for _ in range(5):
            d(img)
        ms = (time.time() - t) / 5 * 1000
        d.close()
        return '%s  %.0f ms/帧 ≈ %.1f FPS（纯推理）' % (
            d.describe(), ms, 1000.0 / max(ms, 1e-6))

    chk('模型推理 5 帧', _infer)

    # ---- 报文通道
    def _tx():
        from link import Link
        if a.send_test:
            lk = Link('serial', com=a.com, baud=a.baud)
            lk.send('<<X,9999,Y,9999>>')
            info = '已往 %s 发一条 <<X,9999,Y,9999>>（共 %d 字节级发送）' % (a.com, lk.sent)
            lk.close()
            return info
        lk = Link('udp', host='127.0.0.1', port=9000)
        lk.send('<<X,9999,Y,9999>>')
        lk.close()
        return 'UDP 发送正常（未真发串口；加 --send-test 才灌给下位机）'

    chk('报文发送', _tx)

    print('-' * 62)
    if fail:
        print('结论：%d 项没过。按上面 [ 失败 ] 的提示逐个修，修完再跑一次。' % fail)
        print('      最常见三条：摄像头号不对 / 串口没权限 / onnxruntime 装不上（改用 cvdnn）')
    else:
        print('结论：全通过。直接 bash run_task1.sh')
    if a.backend == 'om':
        print('      NPU 路线：确认 conda activate base + source set_env.sh 后再跑')
        print('      没生成 best.om 就先跑： bash atc_convert.sh')
    print('=' * 62)
    raise SystemExit(1 if fail else 0)


if __name__ == '__main__':
    main()
