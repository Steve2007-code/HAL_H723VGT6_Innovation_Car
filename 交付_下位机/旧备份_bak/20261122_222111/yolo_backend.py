# -*- coding: utf-8 -*-
"""
yolo_backend.py —— 香橙派上的 YOLO 推理后端

为什么要有这个文件：
    PC 版 live_detect.py 用 ultralytics(YOLO) + torch。香橙派是 aarch64，
    torch 官方没有对应 wheel，硬装非常麻烦。所以先在 PC 上把 best.pt 导出成
    best.onnx（YOLO11n，单类 material），香橙派只要
    opencv + numpy + onnxruntime 就能推理，完全不碰 torch。

后端（--backend）：
    onnx   默认。onnxruntime 优先；装不上自动退回 cv2.dnn
    cvdnn  强制用 OpenCV DNN 读 ONNX（连 onnxruntime 都不用装）
    pt     ultralytics 直读 best.pt（仅当香橙派真装了 torch/ultralytics）
    none   不推理，只回空框——用来单独验摄像头、验串口、验链路

统一接口：
    det = Detector(backend='onnx', model='best.onnx')
    boxes = det(frame)     # [(x0, y0, x1, y1, conf), ...]，坐标是原图像素

模型输出格式（本工程实测）：
    输入 [1,3,640,640] float32，输出 [1,5,8400]（4 个 box + 1 个类别分）
"""

import os
import cv2
import numpy as np


def letterbox(im, size=640, color=(114, 114, 114)):
    """等比缩放 + 灰边补到 size x size，返回 (图, 缩放比, 左上 padding)。

    ultralytics 内部就是这么做的；不做 letterbox 直接拉伸会让框坐标偏掉。
    """
    h, w = im.shape[:2]
    r = min(float(size) / h, float(size) / w)
    nw, nh = int(round(w * r)), int(round(h * r))
    dw, dh = (size - nw) / 2.0, (size - nh) / 2.0
    if (w, h) != (nw, nh):
        im = cv2.resize(im, (nw, nh), interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right,
                            cv2.BORDER_CONSTANT, value=color)
    return im, r, (left, top)


def _nms_indices(boxes, confs, conf_thres, iou_thres):
    """NMSBoxes 在不同 OpenCV 版本里返回 tuple / list / ndarray，统一成 list[int]。"""
    if not boxes:
        return []
    idx = cv2.dnn.NMSBoxes(boxes, confs, float(conf_thres), float(iou_thres))
    if idx is None:
        return []
    if isinstance(idx, tuple):
        return []
    return np.asarray(idx).reshape(-1).astype(int).tolist()


def decode(pred, ratio, pad, conf_thres, iou_thres, orig_shape):
    """把 YOLO 的裸输出解成原图坐标的 [(x0,y0,x1,y1,conf)]。

    兼容两种导出：
      [1, 4+nc, 8400]  标准 YOLOv8 / YOLO11（本工程，不带 NMS）
      [1, N, 6]        导出时带了 NMS：(x1,y1,x2,y2,score,cls)
    """
    if isinstance(pred, (list, tuple)):
        pred = pred[0]
    pred = np.asarray(pred)
    if pred.ndim == 3:
        pred = pred[0]
    if pred.ndim != 2:
        return []

    H, W = orig_shape[:2]
    px, py = pad

    def clip_xyxy(x0, y0, x1, y1):
        return [max(0, min(W, x0)), max(0, min(H, y0)),
                max(0, min(W, x1)), max(0, min(H, y1))]

    # ---- 带 NMS 的导出：最后一维 6
    if pred.shape[1] == 6:
        p = pred[pred[:, 4] >= conf_thres]
        boxes, confs, xyxy = [], [], []
        for x1, y1, x2, y2, sc, _cls in p:
            a, b, c, d = clip_xyxy((x1 - px) / ratio, (y1 - py) / ratio,
                                   (x2 - px) / ratio, (y2 - py) / ratio)
            boxes.append([a, b, c - a, d - b])
            confs.append(float(sc))
            xyxy.append((a, b, c, d))
        out = []
        for i in _nms_indices(boxes, confs, conf_thres, iou_thres):
            x0, y0, x1, y1 = xyxy[i]
            out.append((int(x0), int(y0), int(x1), int(y1), confs[i]))
        return out

    # ---- 标准 [4+nc, 8400] -> [8400, 4+nc]
    t = pred.T
    if t.shape[1] < 5:
        return []
    cx, cy, bw, bh = t[:, 0], t[:, 1], t[:, 2], t[:, 3]
    scores = t[:, 4:]
    conf = scores.max(axis=1)
    keep = conf >= conf_thres
    cx, cy, bw, bh, conf = cx[keep], cy[keep], bw[keep], bh[keep], conf[keep]
    if len(conf) == 0:
        return []

    boxes, confs, xyxy = [], [], []
    for x, y, w, h, c in zip(cx, cy, bw, bh, conf):
        a, b, cc, d = clip_xyxy((x - w / 2.0 - px) / ratio,
                                (y - h / 2.0 - py) / ratio,
                                (x + w / 2.0 - px) / ratio,
                                (y + h / 2.0 - py) / ratio)
        boxes.append([a, b, cc - a, d - b])
        confs.append(float(c))
        xyxy.append((a, b, cc, d))

    out = []
    for i in _nms_indices(boxes, confs, conf_thres, iou_thres):
        x0, y0, x1, y1 = xyxy[i]
        out.append((int(x0), int(y0), int(x1), int(y1), confs[i]))
    return out


class Detector:
    def __init__(self, backend='onnx', model='best.onnx', imgsz=640,
                 conf=0.25, iou=0.45, threads=4):
        self.backend = (backend or 'onnx').lower()
        self.model = model
        self.imgsz = int(imgsz)
        self.conf = float(conf)
        self.iou = float(iou)
        self.sess = None
        self.net = None
        self.pt = None
        self.iname = None

        if self.backend not in ('onnx', 'cvdnn', 'pt', 'none'):
            raise SystemExit('[错误] 不认识的 --backend：%s（可选 onnx/cvdnn/pt/none）'
                             % backend)

        if self.backend == 'none':
            print('[后端] none —— 不推理，只验摄像头 / 串口 / 链路')
            return

        if self.backend in ('onnx', 'cvdnn'):
            if not os.path.exists(model):
                raise SystemExit(
                    '[错误] 找不到模型 %s\n'
                    '       把 best.onnx 和 live_detect_pi.py 放在同一目录，'
                    '或用 --model 指定路径。' % model)

        if self.backend == 'onnx':
            try:
                import onnxruntime as ort
                so = ort.SessionOptions()
                so.intra_op_num_threads = int(threads)
                so.inter_op_num_threads = 1
                so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
                # CPU 版最稳；若装了 onnxruntime-cann，把 CANNExecutionProvider 放前面即可上 NPU
                providers = ['CPUExecutionProvider']
                self.sess = ort.InferenceSession(model, sess_options=so,
                                                 providers=providers)
                self.iname = self.sess.get_inputs()[0].name
                print('[后端] onnxruntime  %s  (%d 线程)' % (model, threads))
            except Exception as e:
                print('[后端] onnxruntime 起不来，自动退回 cv2.dnn：%s' % e)
                self.backend = 'cvdnn'

        if self.backend == 'cvdnn':
            try:
                # OpenCV DNN 的 ONNX importer 在非 ASCII 路径上会直接 parseFile 失败。
                # 放到 /home/HwHiAiUser/Steve_folder/ 是纯英文路径，本不会触发；
                # 兜一手，免得有人把包放进中文目录后一头雾水。
                import shutil
                import tempfile
                dnn_path = model
                try:
                    model.encode('ascii')
                except UnicodeEncodeError:
                    dnn_path = os.path.join(tempfile.gettempdir(),
                                            'yolo_model_%d.onnx' % os.getpid())
                    shutil.copyfile(model, dnn_path)
                    print('[后端] 模型路径含非 ASCII，已暂存到 %s' % dnn_path)
                self.net = cv2.dnn.readNetFromONNX(dnn_path)
                try:
                    self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                    self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
                except Exception:
                    pass
                print('[后端] OpenCV DNN  %s' % model)
            except Exception as e:
                raise SystemExit(
                    '[错误] cv2.dnn 读不了这个 ONNX：%s\n'
                    '       请装 onnxruntime： pip3 install onnxruntime' % e)

        elif self.backend == 'pt':
            try:
                from ultralytics import YOLO
                self.pt = YOLO(model)
                print('[后端] ultralytics  %s' % model)
            except Exception as e:
                raise SystemExit('[错误] --backend pt 需要 torch + ultralytics：%s' % e)

    def __call__(self, img):
        if self.backend == 'none':
            return []
        if self.backend == 'pt':
            r = self.pt.predict(img, imgsz=self.imgsz, conf=self.conf,
                                verbose=False)[0]
            out = []
            for b in r.boxes:
                x0, y0, x1, y1 = [int(v) for v in b.xyxy[0].tolist()]
                out.append((x0, y0, x1, y1, float(b.conf[0])))
            return out

        lb, ratio, pad = letterbox(img, self.imgsz)
        if self.backend == 'onnx':
            blob = cv2.cvtColor(lb, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
            blob = np.ascontiguousarray(np.transpose(blob, (2, 0, 1))[None, ...])
            out = self.sess.run(None, {self.iname: blob})[0]
        else:
            blob = cv2.dnn.blobFromImage(lb, 1.0 / 255.0,
                                         (self.imgsz, self.imgsz),
                                         mean=(0, 0, 0), swapRB=True, crop=False)
            self.net.setInput(blob)
            out = self.net.forward()
        return decode(out, ratio, pad, self.conf, self.iou, img.shape)

    def describe(self):
        return '%s / %s' % (self.backend, self.model)


if __name__ == '__main__':
    # 自检：造一张假图，跑一帧，打印耗时
    import time
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--backend', default='onnx')
    ap.add_argument('--model', default='best.onnx')
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--threads', type=int, default=4)
    a = ap.parse_args()
    d = Detector(a.backend, a.model, a.imgsz, threads=a.threads)
    img = np.full((480, 640, 3), 128, np.uint8)
    for _ in range(3):
        d(img)
    t = time.time()
    for _ in range(10):
        d(img)
    dt = (time.time() - t) / 10 * 1000
    print('推理 %.1f ms/帧  约 %.1f FPS（纯推理，不含采集与判色）' % (dt, 1000.0 / max(dt, 1e-6)))
