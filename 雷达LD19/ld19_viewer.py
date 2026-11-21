#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LD19 激光雷达上位机 (LD19 Viewer)
================================================================
功能对标 LDRobot LDS00 Tool (ld_desktop V2.3.12)：
  · 黑底极坐标雷达图：12 道同心环 + 每 30° 辐条 + 角度标注（0° 在正上方，顺时针）
  · 串口设备 / 波特率选择 + 开始 / 停止采集 + 复位视图 + 帮助
  · 实时点云、扫描指示线、连接状态栏

在原版基础上增加的实用功能：
  · 滚轮缩放（以光标为中心）/ 左键拖动平移 / 双击复位
  · 距离、强度、单色三种着色 + 拖尾余晖圈数
  · 量程锁定、角度零位偏移、左右镜像
  · 截图保存、当前点云 CSV 导出
  · 参数自动记忆（QSettings，下次启动恢复上次串口/波特率/配色）
  · HUD 实时显示帧率、转速、点/圈、累计帧数、校验模式

协议（实测 COM24 @230400 8N1，连续 522 帧静态验证通过）
----------------------------------------------------------------
  偏移    长度   含义
  0..1     2    帧头 0x54 0x2C
  2..3     2    转速，uint16 LE，单位 deg/s（本机约 3585，即 9.96 Hz）
  4..5     2    本帧起始角度，uint16 LE，单位 0.01°
  6..41   36    12 个点，每点 3 字节：距离(uint16 LE,mm) + 强度(uint8)
  42..43   2    本帧结束角度，uint16 LE，单位 0.01°
  44..45   2    时间戳，uint16 LE，单位 ms（16 位会回绕）
  46       1    校验字节

关于第 46 字节：它**不是** LD06 那套 CRC8(poly=0x4D, init=0xD8)。
实测对 522 帧用 256 个多项式 × 多种 init / 区间 / 位序暴力枚举，
最高通过率仅 1.3%（等于随机），因此本程序采用"结构校验"——
帧头 + 转速范围 + 角度跨度 + 47 字节步进连续性——并在启动时自动
探测 CRC8 是否可用（可用就自动升级为 CRC 严格校验）。
"""

import sys
import os
import math
import time
import struct
import threading

import serial
import serial.tools.list_ports

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt, QPointF, QRectF, QSettings, QTimer

# ------------------------------------------------------------------ 常量

APP_NAME = 'LD19 Viewer'
APP_VER = 'v1.0'
ORG_NAME = 'Steve'

PKT_LEN = 47
PTS_PER_PKT = 12
DEF_BAUD = 230400
BAUD_LIST = ['230400', '115200', '128000', '150000', '256000', '460800', '921600']

# 量程选项：0 = 自动（分档跳变）
RANGE_ITEMS = ['自动', '4 m', '8 m', '12 m', '16 m', '20 m', '25 m']
RANGE_VALUES = [0, 4, 8, 12, 16, 20, 25]
# 自动量程只会落在这些档位上，不会连续微调
AUTO_LADDER = [1.5, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 16.0, 20.0, 25.0]

# 显示区域过滤（ROI）
ROI_ITEMS = ['不过滤', '圆形', '方形', '扇形']
ROI_KEYS = ['none', 'circle', 'square', 'sector']

# 配色（深色主题）
C_BG = QtGui.QColor('#0A0B0D')          # 画布背景：纯黑
C_GRID = QtGui.QColor('#242A32')        # 细网格
C_GRID_MAJOR = QtGui.QColor('#39424F')  # 外圈 / 主轴
C_TEXT = QtGui.QColor('#8792A2')        # 角度标注
C_TEXT_DIM = QtGui.QColor('#5A6472')    # 次要文字
C_ACCENT = QtGui.QColor('#00E0B8')      # 青绿强调色
C_WARN = QtGui.QColor('#FFB020')
C_ERR = QtGui.QColor('#FF4D4F')
C_PANEL = QtGui.QColor('#14171C')
C_PANEL_HI = QtGui.QColor('#1C2129')

# 距离着色渐变（深青 → 绿 → 黄 → 橙 → 红）
DIST_STOPS = [
    (0.00, QtGui.QColor('#00E5FF')),
    (0.22, QtGui.QColor('#00FF9C')),
    (0.45, QtGui.QColor('#C6FF00')),
    (0.68, QtGui.QColor('#FFC400')),
    (0.86, QtGui.QColor('#FF5A1F')),
    (1.00, QtGui.QColor('#FF1744')),
]
# 强度着色渐变（暗 → 冷白）
INT_STOPS = [
    (0.00, QtGui.QColor('#1E3A5F')),
    (0.35, QtGui.QColor('#2E7FFA')),
    (0.70, QtGui.QColor('#8FD3FF')),
    (1.00, QtGui.QColor('#FFFFFF')),
]


def lerp_stops(stops, t):
    """在渐变色标之间线性插值。"""
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    for i in range(1, len(stops)):
        if t <= stops[i][0]:
            t0, c0 = stops[i - 1]
            t1, c1 = stops[i]
            f = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
            return QtGui.QColor(
                int(c0.red() + (c1.red() - c0.red()) * f),
                int(c0.green() + (c1.green() - c0.green()) * f),
                int(c0.blue() + (c1.blue() - c0.blue()) * f))
    return QtGui.QColor(stops[-1][1])


# CRC8 查表 (poly=0x4D, init=0xD8)，仅用于"能否使用"的自动探测
_CRC_TBL = []
for _i in range(256):
    _c = _i
    for _ in range(8):
        _c = (_c >> 1) ^ 0x4D if _c & 1 else _c >> 1
    _CRC_TBL.append(_c)


def crc8(data, crc=0xD8):
    for b in data:
        crc = _CRC_TBL[(crc ^ b) & 0xFF]
    return crc


# ------------------------------------------------------------------ 数据状态


class RadarState(object):
    """串口线程写入、界面线程读取的共享状态。"""

    def __init__(self):
        self.lock = threading.RLock()
        self.revs = []          # 已完成的整圈点集，每个是 [(角度, 距离mm, 强度)]
        self.live = []          # 当前这一圈的点
        self.max_revs = 8
        self._last_angle = None

        self.sweep = 0.0        # 当前扫描角度
        self.speed = 0          # deg/s
        self.frames = 0
        self.pts_total = 0
        self.ppr = 0            # 上一圈点数
        self.crc_ok = 0
        self.crc_bad = 0
        self.crc_mode = '探测中'  # 'CRC8' / '结构'
        self._crc_probe = [0, 0]  # [ok, bad]
        self._crc_decided = False

        self.connected = False
        self.error = ''
        self.port = ''
        self.baud = 0

    # ---- 写
    def reset(self):
        with self.lock:
            self.revs = []
            self.live = []
            self._last_angle = None
            self.frames = 0
            self.pts_total = 0
            self.ppr = 0
            self.crc_ok = 0
            self.crc_bad = 0
            self.crc_mode = '探测中'
            self._crc_probe = [0, 0]
            self._crc_decided = False
            self.sweep = 0.0
            self.speed = 0

    def note_crc(self, ok):
        with self.lock:
            if ok:
                self.crc_ok += 1
            else:
                self.crc_bad += 1
            if not self._crc_decided:
                self._crc_probe[0 if ok else 1] += 1
                n = self._crc_probe[0] + self._crc_probe[1]
                if n >= 60:
                    rate = self._crc_probe[0] / float(n)
                    self.crc_mode = 'CRC8' if rate > 0.85 else '结构'
                    self._crc_decided = True

    def strict_crc(self):
        with self.lock:
            return self.crc_mode == 'CRC8'

    def add_frame(self, pts, first_angle):
        with self.lock:
            if self._last_angle is not None and first_angle < self._last_angle - 5.0:
                if self.live:
                    self.ppr = len(self.live)
                    self.revs.append(self.live)
                    if len(self.revs) > self.max_revs:
                        self.revs.pop(0)
                    self.live = []
            self.live.extend(pts)
            self._last_angle = first_angle
            if pts:
                self.sweep = pts[-1][0]
            self.frames += 1

    # ---- 读
    def snapshot(self):
        with self.lock:
            return {
                'revs': [list(r) for r in self.revs],
                'live': list(self.live),
                'sweep': self.sweep,
                'speed': self.speed,
                'frames': self.frames,
                'ppr': self.ppr,
                'pts_total': self.pts_total,
                'crc_ok': self.crc_ok,
                'crc_bad': self.crc_bad,
                'crc_mode': self.crc_mode,
                'connected': self.connected,
                'error': self.error,
                'port': self.port,
                'baud': self.baud,
                'pts_live': len(self.live),
            }


# ------------------------------------------------------------------ 串口线程


class RadarReader(QtCore.QThread):
    failed = QtCore.pyqtSignal(str)

    def __init__(self, port, baud, state, parent=None):
        super(RadarReader, self).__init__(parent)
        self.port = port
        self.baud = baud
        self.state = state
        self._stop = False

    def stop(self):
        self._stop = True

    # ---- 解析一帧
    def _decode(self, f):
        speed = f[2] | (f[3] << 8)
        sa = (f[4] | (f[5] << 8)) * 0.01
        ea = (f[42] | (f[43] << 8)) * 0.01
        span = (ea - sa) % 360.0
        step = span / (PTS_PER_PKT - 1.0) if span > 0.01 else 0.72
        pts = []
        for k in range(PTS_PER_PKT):
            o = 6 + 3 * k
            d = f[o] | (f[o + 1] << 8)
            it = f[o + 2]
            a = (sa + step * k) % 360.0
            pts.append((a, d, it))
        return speed, pts

    def _parse(self, buf):
        """从字节缓冲区中提取尽可能多的有效帧，原地删除已消费字节。"""
        i = 0
        n = len(buf)
        strict = self.state.strict_crc()
        while n - i >= PKT_LEN:
            if buf[i] != 0x54 or buf[i + 1] != 0x2C:
                j = buf.find(b'\x54\x2c', i + 1)
                i = (n - 1) if j < 0 else j
                continue
            f = buf[i:i + PKT_LEN]
            speed = f[2] | (f[3] << 8)
            # 结构校验：转速范围 + 角度跨度
            if speed > 12000:
                i += 1
                continue
            sa = (f[4] | (f[5] << 8)) * 0.01
            ea = (f[42] | (f[43] << 8)) * 0.01
            span = (ea - sa) % 360.0
            if span > 60.0:
                i += 1
                continue
            c_ok = (crc8(f[:46]) == f[46])
            self.state.note_crc(c_ok)
            if strict and not c_ok:
                i += 1
                continue

            spd, pts = self._decode(f)
            with self.state.lock:
                self.state.speed = spd
            self.state.add_frame(pts, pts[0][0])
            with self.state.lock:
                self.state.pts_total += len(pts)
            i += PKT_LEN

        if i > 0:
            del buf[:i]

    def run(self):
        ser = None
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.05)
        except Exception as e:
            self.state.error = '串口打开失败：%s' % e
            self.failed.emit(self.state.error)
            return

        self.state.connected = True
        self.state.error = ''
        self.state.port = self.port
        self.state.baud = self.baud
        buf = bytearray()
        try:
            try:
                ser.setDTR(False)
                ser.setRTS(False)
            except Exception:
                pass
            while not self._stop:
                n = ser.in_waiting
                chunk = ser.read(n if n else 1)
                if chunk:
                    buf += chunk
                    self._parse(buf)
                if len(buf) > 8192:      # 极端情况防止内存膨胀
                    del buf[:4096]
        except Exception as e:
            if not self._stop:
                self.state.error = '串口读取中断：%s' % e
                self.failed.emit(self.state.error)
        finally:
            self.state.connected = False
            with self.state.lock:
                self.state.speed = 0
            try:
                ser.close()
            except Exception:
                pass


# ------------------------------------------------------------------ 雷达视图


class RadarView(QtWidgets.QWidget):
    """纯 QPainter 绘制的极坐标雷达视图。"""

    def __init__(self, state, cfg, parent=None):
        super(RadarView, self).__init__(parent)
        self.state = state
        self.cfg = cfg
        self.setMinimumSize(400, 300)
        self.setMouseTracking(True)
        self.setContextMenuPolicy(Qt.DefaultContextMenu)

        self.zoom = 1.0
        self.pan = QPointF(0.0, 0.0)
        self.rings = 12
        self.cursor = None          # 光标位置（控件坐标）
        self._dragging = False
        self._drag_last = None
        self._bucket_cache = {}
        self._fps_stat = (time.time(), 0, 0.0)
        self._auto = 12.0
        self._auto_need = None
        self._auto_since = 0.0

    # ---- 交互相对于视图
    def reset_view(self):
        self.zoom = 1.0
        self.pan = QPointF(0.0, 0.0)
        self.update()

    def _geom(self):
        w, h = self.width(), self.height()
        cx = w * 0.5 + self.pan.x()
        cy = h * 0.5 + self.pan.y()
        base = min(w, h) * 0.5 - 42.0
        R = max(50.0, base * self.zoom)
        return cx, cy, R

    # ---- 颜色分桶（避免逐点换笔）
    def _bucket_colors(self):
        mode = self.cfg['color_mode']
        n = 40
        out = []
        for i in range(n):
            t = i / float(n - 1)
            if mode == 'dist':
                c = lerp_stops(DIST_STOPS, t)
            elif mode == 'int':
                c = lerp_stops(INT_STOPS, t)
            else:
                c = QtGui.QColor('#EAF4FF')
            out.append(c)
        return out

    # ---- 绘制
    def paintEvent(self, ev):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing, True)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, C_BG)

        snap = self.state.snapshot()
        cx, cy, R = self._geom()
        rng = self._eff_range(snap)
        ppm = R / rng

        self._draw_grid(p, cx, cy, R, rng, ppm)
        self._draw_roi(p, cx, cy, ppm)
        self._draw_sweep(p, cx, cy, R, snap)
        self._draw_points(p, cx, cy, ppm, snap, rng)
        self._draw_hud(p, snap, rng)
        if self.cfg['show_legend'] and self.cfg['color_mode'] in ('dist', 'int'):
            self._draw_legend(p, rng)
        self._draw_cursor(p, cx, cy, ppm, rng)
        p.end()

    def _eff_range(self, snap):
        """量程：手动固定（轴不动），或自动分档。

        自动档必须"跳"而不是"滑"——否则网格与刻度会随点云每圈微调，
        观感上就像画面在呼吸。这里用分档 + 双向迟滞 + 稳定 1.2 s 才切档。
        """
        want = float(self.cfg['range_m'])
        if want > 0:
            self._auto = want
            self._auto_need = None
            return want

        ds = []
        src = list(snap['revs'][-2:]) + [snap['live']]
        off = self.cfg['angle_offset']
        mir = -1.0 if self.cfg['mirror'] else 1.0
        for lst in src:
            for (a, d, _i) in lst:
                if d <= 0:
                    continue
                if self.cfg['roi_shape'] != 'none':
                    r = d / 1000.0
                    rad = math.radians(a + off)
                    if not self.in_roi(r * math.sin(rad) * mir, r * math.cos(rad)):
                        continue
                ds.append(d)
        if len(ds) < 60:
            return self._auto

        ds.sort()
        need_m = ds[int(len(ds) * 0.98)] / 1000.0 * 1.10
        need_m = max(1.0, min(25.0, need_m))
        tgt = AUTO_LADDER[-1]
        for v in AUTO_LADDER:
            if need_m <= v:
                tgt = v
                break

        cur = self._auto
        # 迟滞：涨到 95% 才扩，缩到 50% 才收，避免边界抖动
        if tgt > cur and need_m > cur * 0.95:
            go = tgt
        elif tgt < cur and need_m < cur * 0.50:
            go = tgt
        else:
            self._auto_need = None
            return cur

        now = time.time()
        if self._auto_need != go:
            self._auto_need = go
            self._auto_since = now
        elif now - self._auto_since >= 1.2:
            self._auto = go
            self._auto_need = None
        return self._auto

    def _draw_grid(self, p, cx, cy, R, rng, ppm):
        c = QPointF(cx, cy)
        show = self.cfg['show_grid']
        # 同心环
        for k in range(1, self.rings + 1):
            r = R * k / self.rings
            major = (k == self.rings)
            if not show and not major:
                continue
            p.setPen(QtGui.QPen(C_GRID_MAJOR if major else C_GRID,
                                1.4 if major else 1.0))
            p.drawEllipse(c, r, r)
        # 辐条
        if show:
            for a in range(0, 360, 30):
                rad = math.radians(a)
                x = cx + R * math.sin(rad)
                y = cy - R * math.cos(rad)
                p.setPen(QtGui.QPen(C_GRID_MAJOR if a % 90 == 0 else C_GRID,
                                    1.2 if a % 90 == 0 else 1.0))
                p.drawLine(c, QPointF(x, y))
        # 角度标注
        if self.cfg['show_angle']:
            p.setFont(QtGui.QFont('Microsoft YaHei', 9))
            p.setPen(C_TEXT)
            for a in range(0, 360, 30):
                rad = math.radians(a)
                x = cx + (R + 21) * math.sin(rad)
                y = cy - (R + 21) * math.cos(rad)
                p.drawText(QRectF(x - 26, y - 11, 52, 22), Qt.AlignCenter,
                           '%d°' % a)
        # 距离刻度（沿 90° 轴）
        if self.cfg['show_range_tick']:
            p.setFont(QtGui.QFont('Microsoft YaHei', 8))
            step = max(1.0, round(rng / 6.0))
            m = step
            while m < rng:
                x = cx + m * ppm
                p.setPen(QtGui.QPen(C_TEXT_DIM, 1.0))
                p.drawLine(QPointF(x, cy - 4), QPointF(x, cy + 4))
                p.drawText(QRectF(x - 20, cy + 6, 40, 14), Qt.AlignCenter,
                           ('%g' % m) + 'm')
                m += step
        # 中心十字
        p.setPen(QtGui.QPen(C_ACCENT, 1.4))
        p.drawLine(QPointF(cx - 7, cy), QPointF(cx + 7, cy))
        p.drawLine(QPointF(cx, cy - 7), QPointF(cx, cy + 7))

    # ---- 显示区域过滤（ROI）
    def in_roi(self, mx, my):
        """判断米制显示坐标是否落在过滤区域内（圆形/方形支持中心偏移）。"""
        shape = self.cfg['roi_shape']
        if shape == 'none':
            return True
        s = float(self.cfg['roi_size'])
        if shape == 'circle':
            dx = mx - self.cfg['roi_dx']
            dy = my - self.cfg['roi_dy']
            return dx * dx + dy * dy <= s * s
        if shape == 'square':
            h = s * 0.5
            return (abs(mx - self.cfg['roi_dx']) <= h and
                    abs(my - self.cfg['roi_dy']) <= h)
        if shape == 'sector':
            if mx * mx + my * my > s * s:
                return False
            ang = math.degrees(math.atan2(mx, my)) % 360.0
            d = abs((ang - self.cfg['roi_dir'] + 180.0) % 360.0 - 180.0)
            return d <= self.cfg['roi_span'] * 0.5
        return True

    def _draw_roi(self, p, cx, cy, ppm):
        """画出过滤区域边界（淡琥珀色虚线 + 极淡填充）。"""
        shape = self.cfg['roi_shape']
        if shape == 'none':
            return
        s = float(self.cfg['roi_size']) * ppm
        ox = self.cfg['roi_dx'] * ppm
        oy = self.cfg['roi_dy'] * ppm
        ecx, ecy = cx + ox, cy - oy          # 区域中心（屏幕坐标）
        edge = QtGui.QColor(255, 176, 32, 165)
        fill = QtGui.QColor(255, 176, 32, 9)
        p.setBrush(fill)
        pen = QtGui.QPen(edge, 1.3, Qt.DashLine)
        pen.setDashPattern([6, 5])
        p.setPen(pen)

        if shape == 'circle':
            p.drawEllipse(QPointF(ecx, ecy), s, s)
        elif shape == 'square':
            h = s * 0.5
            p.drawRect(QRectF(ecx - h, ecy - h, s, s))
        elif shape == 'sector':
            d = self.cfg['roi_dir']
            span = self.cfg['roi_span']
            rect = QRectF(cx - s, cy - s, 2 * s, 2 * s)
            path = QtGui.QPainterPath(QPointF(cx, cy))
            path.arcTo(rect, 90.0 - (d - span * 0.5), -span)
            path.closeSubpath()
            p.drawPath(path)
        p.setBrush(Qt.BrushStyle.NoBrush)

    def _draw_sweep(self, p, cx, cy, R, snap):
        if not self.cfg['show_sweep'] or not snap['connected']:
            return
        a = (snap['sweep'] + self.cfg['angle_offset']) % 360.0
        qt_a = 90.0 - a
        path = QtGui.QPainterPath(QPointF(cx, cy))
        rect = QRectF(cx - R, cy - R, 2 * R, 2 * R)
        path.arcTo(rect, qt_a, 22.0)
        path.closeSubpath()
        grad = QtGui.QRadialGradient(cx, cy, R)
        grad.setColorAt(0.0, QtGui.QColor(0, 224, 184, 70))
        grad.setColorAt(1.0, QtGui.QColor(0, 224, 184, 0))
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(grad)
        p.drawPath(path)
        # 主扫描线
        rad = math.radians(a)
        p.setPen(QtGui.QPen(QtGui.QColor(0, 255, 210, 190), 1.6))
        p.drawLine(QPointF(cx, cy),
                   QPointF(cx + R * math.sin(rad), cy - R * math.cos(rad)))

    def _draw_points(self, p, cx, cy, ppm, snap, rng):
        trail = int(self.cfg['trail'])
        if trail <= 1:
            seq = [snap['live']]
        else:
            seq = list(snap['revs'][-(trail - 1):])
            seq.append(snap['live'])
        if not any(seq):
            return
        size = float(self.cfg['point_size'])
        mode = self.cfg['color_mode']
        off = self.cfg['angle_offset']
        mir = -1.0 if self.cfg['mirror'] else 1.0
        colors = self._bucket_colors()
        nb = len(colors)
        p.setPen(Qt.PenStyle.NoPen)

        n = len(seq)
        for idx, pts in enumerate(seq):
            if not pts:
                continue
            alpha = 1.0
            if n > 1:
                alpha = 0.28 + 0.72 * (idx + 1.0) / n
            # 分桶
            buckets = {}
            for (a, d, it) in pts:
                if d <= 0:
                    continue
                r = d / 1000.0
                if r > rng:
                    continue
                t = r / rng if mode == 'dist' else (it / 255.0 if mode == 'int' else 0.0)
                b = int(t * (nb - 1) + 0.5)
                if b < 0:
                    b = 0
                elif b >= nb:
                    b = nb - 1
                rad = math.radians(a + off)
                mx = r * math.sin(rad) * mir      # 显示坐标系下的米制坐标
                my = r * math.cos(rad)
                if not self.in_roi(mx, my):
                    continue
                x = cx + mx * ppm
                y = cy - my * ppm
                buckets.setdefault(b, []).append(QPointF(x, y))
            for b, lst in buckets.items():
                col = QtGui.QColor(colors[b])
                col.setAlphaF(min(1.0, alpha))
                p.setPen(QtGui.QPen(col, size))
                p.drawPoints(QtGui.QPolygonF(lst))

    def _roi_desc(self):
        """HUD 用的过滤条件描述。"""
        shape = self.cfg['roi_shape']
        if shape == 'none':
            return ''
        size = float(self.cfg['roi_size'])
        if shape == 'sector':
            return '扇形  %.1f m  ±%.0f°  @%.0f°' % (
                size, self.cfg['roi_span'] * 0.5, self.cfg['roi_dir'])
        dx = self.cfg['roi_dx']
        dy = self.cfg['roi_dy']
        off = ''
        if abs(dx) > 1e-6 or abs(dy) > 1e-6:
            off = '  @(%+.1f, %+.1f)' % (dx, dy)
        name = '圆形' if shape == 'circle' else '方形'
        return '%s  %.1f m%s' % (name, size, off)

    def _draw_hud(self, p, snap, rng):
        lines = []
        if snap['connected']:
            head_col = C_ACCENT
            head = '已连接  %s  @%d' % (snap['port'], snap['baud'])
            spd = snap['speed']
            rpm = spd / 6.0
            lines.append(('帧率', '%.0f Hz' % self._fps()))
            lines.append(('转速', '%.0f RPM  (%.2f Hz)' % (rpm, spd / 360.0)))
            ppr = snap['ppr']
            if ppr:
                lines.append(('点/圈', '%d  (%.2f°/点)' % (ppr, 360.0 / ppr)))
            else:
                lines.append(('点/圈', '--'))
            lines.append(('量程', ('自动  %.1f m' % rng)
                          if self.cfg['range_m'] == 0 else ('%.0f m' % rng)))
            roi = self._roi_desc()
            if roi:
                lines.append(('过滤', roi))
            lines.append(('累计', '%d 帧' % snap['frames']))
            if snap['crc_mode'] == 'CRC8':
                ok, bad = snap['crc_ok'], snap['crc_bad']
                lines.append(('校验', 'CRC8  %d/%d' % (ok, ok + bad)))
            else:
                lines.append(('校验', '结构（固件无标准CRC）'))
        else:
            head_col = C_TEXT_DIM if not snap['error'] else C_ERR
            head = '未连接' if not snap['error'] else '错误'
            lines.append(('提示', '选择串口后点击「开始」'))
            if snap['error']:
                lines.append(('详情', snap['error'][:30]))

        p.setFont(QtGui.QFont('Microsoft YaHei', 9))
        fm = p.fontMetrics()
        w = 190.0
        for k, v in lines:
            w = max(w, 26 + fm.width(k) + 12 + fm.width(v))
        box = QRectF(14, 14, w + 16, 30 + 17 * len(lines))
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QtGui.QColor(16, 19, 24, 205))
        p.drawRoundedRect(box, 7, 7)
        p.setPen(QtGui.QPen(QtGui.QColor('#2A313B'), 1.0))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawRoundedRect(box, 7, 7)

        # 状态点
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(head_col)
        p.drawEllipse(QPointF(28, 30), 4, 4)
        p.setPen(QtGui.QColor('#D6DEE9'))
        p.drawText(QPointF(40, 34), head)

        p.setFont(QtGui.QFont('Microsoft YaHei', 8))
        y = 34 + 17
        for k, v in lines:
            p.setPen(C_TEXT_DIM)
            p.drawText(QPointF(24, y), k)
            p.setPen(QtGui.QColor('#9FB0C4'))
            p.drawText(QPointF(24 + 46, y), v)
            y += 17

    def _draw_legend(self, p, rng):
        w, h = self.width(), self.height()
        bar = QRectF(w - 54, h * 0.5 - 90, 10, 180)
        grad = QtGui.QLinearGradient(bar.topLeft(), bar.bottomLeft())
        stops = DIST_STOPS if self.cfg['color_mode'] == 'dist' else INT_STOPS
        for t, c in stops:
            grad.setColorAt(t, c)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(grad)
        p.drawRoundedRect(bar, 3, 3)
        p.setFont(QtGui.QFont('Microsoft YaHei', 7))
        p.setPen(C_TEXT_DIM)
        if self.cfg['color_mode'] == 'dist':
            p.drawText(QRectF(bar.left() - 34, bar.top() - 16, 50, 14),
                       Qt.AlignRight, '%.1fm' % rng)
            p.drawText(QRectF(bar.left() - 34, bar.bottom() + 2, 50, 14),
                       Qt.AlignRight, '0')
        else:
            p.drawText(QRectF(bar.left() - 34, bar.top() - 16, 50, 14),
                       Qt.AlignRight, '255')
            p.drawText(QRectF(bar.left() - 34, bar.bottom() + 2, 50, 14),
                       Qt.AlignRight, '0')

    def _draw_cursor(self, p, cx, cy, ppm, rng):
        if self.cursor is None:
            return
        dx = (self.cursor.x() - cx)
        dy = (cy - self.cursor.y())
        r = math.hypot(dx, dy) / ppm
        if r > rng * 1.5:
            return
        a = (math.degrees(math.atan2(dx, dy)) - self.cfg['angle_offset']) % 360.0
        txt = 'θ %.1f°   r %.2f m   x %.2f  y %.2f' % (a, r, -dx / ppm, dy / ppm)
        p.setFont(QtGui.QFont('Microsoft YaHei', 8))
        fm = p.fontMetrics()
        box = QRectF(self.cursor.x() + 14, self.cursor.y() - 26,
                     fm.width(txt) + 16, 22)
        if box.right() > self.width() - 8:
            box.moveLeft(self.cursor.x() - box.width() - 14)
        if box.top() < 8:
            box.moveTop(self.cursor.y() + 12)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QtGui.QColor(16, 19, 24, 220))
        p.drawRoundedRect(box, 5, 5)
        p.setPen(QtGui.QColor('#B9C6D6'))
        p.drawText(box, Qt.AlignCenter, txt)

    # ---- 帧率统计
    def _fps(self):
        now = time.time()
        t0, f0, rate = self._fps_stat
        cur = self.state.frames
        dt = now - t0
        if dt >= 0.5:
            rate = (cur - f0) / dt if dt > 0 else 0.0
            self._fps_stat = (now, cur, rate)
        elif dt <= 0.0:
            rate = 0.0
        return rate

    # ---- 鼠标
    def wheelEvent(self, ev):
        delta = ev.angleDelta().y()
        if delta == 0:
            return
        cx, cy, _R = self._geom()
        new_zoom = max(0.25, min(20.0, self.zoom * (1.12 if delta > 0 else 1 / 1.12)))
        f = new_zoom / self.zoom
        mx, my = float(ev.pos().x()), float(ev.pos().y())
        # 以光标为不动点缩放：C' = s0 - f*(s0 - C)
        ncx = mx - f * (mx - cx)
        ncy = my - f * (my - cy)
        self.zoom = new_zoom
        self.pan = QPointF(ncx - self.width() * 0.5, ncy - self.height() * 0.5)
        self.update()

    def mousePressEvent(self, ev):
        if ev.button() == Qt.LeftButton:
            self._dragging = True
            self._drag_last = ev.pos()
            self.setCursor(Qt.ClosedHandCursor)

    def mouseMoveEvent(self, ev):
        self.cursor = QPointF(ev.pos())
        if self._dragging and self._drag_last is not None:
            d = ev.pos() - self._drag_last
            self.pan += QPointF(d.x(), d.y())
            self._drag_last = ev.pos()
        self.update()

    def mouseReleaseEvent(self, ev):
        if ev.button() == Qt.LeftButton:
            self._dragging = False
            self._drag_last = None
            self.setCursor(Qt.ArrowCursor)

    def mouseDoubleClickEvent(self, ev):
        self.reset_view()

    def leaveEvent(self, ev):
        self.cursor = None
        self.update()

    def contextMenuEvent(self, ev):
        win = self.window()
        if hasattr(win, 'build_ctx_menu'):
            win.build_ctx_menu().exec_(ev.globalPos())


# ------------------------------------------------------------------ 设置面板


class SettingsPanel(QtWidgets.QFrame):
    def __init__(self, cfg, view, parent=None):
        super(SettingsPanel, self).__init__(parent)
        self.cfg = cfg
        self.view = view
        self.setFixedWidth(226)
        self.setStyleSheet(
            'QFrame{background:%s;border-left:1px solid #232A33;}'
            'QLabel{color:#9AA7B7;font-size:11px;}'
            'QCheckBox{color:#B9C6D6;font-size:11px;spacing:6px;}'
            'QComboBox,QSpinBox,QDoubleSpinBox{background:#1B2029;color:#D6DEE9;'
            'border:1px solid #2B323C;border-radius:4px;padding:3px 6px;font-size:11px;}'
            'QComboBox::drop-down{border:0;width:16px;}'
            'QComboBox:disabled,QSpinBox:disabled,QDoubleSpinBox:disabled{'
            'color:#4C5766;background:#14181E;border-color:#1E242C;}'
            'QLabel:disabled{color:#4C5766;}'
            'QComboBox QAbstractItemView{background:#1B2029;color:#D6DEE9;'
            'selection-background-color:#2C7BE5;border:1px solid #2B323C;}'
            'QSlider::groove:horizontal{height:3px;background:#2B323C;border-radius:2px;}'
            'QSlider::handle:horizontal{width:11px;margin:-5px 0;border-radius:5px;'
            'background:#00E0B8;}'
            'QPushButton{background:#1B2029;color:#C8D4E2;border:1px solid #2B323C;'
            'border-radius:4px;padding:5px;font-size:11px;}'
            'QPushButton:hover{background:#232B36;border-color:#3A4756;}'
            % C_PANEL.name())

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(12, 12, 12, 12)
        lay.setSpacing(9)

        def title(t):
            lb = QtWidgets.QLabel(t)
            lb.setStyleSheet('color:#68758A;font-size:10px;font-weight:bold;'
                             'letter-spacing:1px;')
            lay.addWidget(lb)

        def row(label, w):
            box = QtWidgets.QHBoxLayout()
            box.setSpacing(6)
            lb = QtWidgets.QLabel(label)
            lb.setFixedWidth(56)
            box.addWidget(lb)
            box.addWidget(w, 1)
            lay.addLayout(box)
            return box

        title('显示')
        self.cb_color = QtWidgets.QComboBox()
        self.cb_color.addItems(['距离着色', '强度着色', '单色'])
        self.cb_color.setCurrentIndex({'dist': 0, 'int': 1, 'mono': 2}[cfg['color_mode']])
        row('着色', self.cb_color)

        self.sp_size = QtWidgets.QSpinBox()
        self.sp_size.setRange(1, 8)
        self.sp_size.setValue(int(cfg['point_size']))
        row('点大小', self.sp_size)

        self.sp_trail = QtWidgets.QSpinBox()
        self.sp_trail.setRange(1, 6)
        self.sp_trail.setValue(int(cfg['trail']))
        row('拖尾圈数', self.sp_trail)

        self.cb_range = QtWidgets.QComboBox()
        self.cb_range.addItems(RANGE_ITEMS)
        self.cb_range.setToolTip('手动 = 固定量程，刻度轴不动；自动 = 分档跳变（稳定 1.2 s 才切档）')
        try:
            self.cb_range.setCurrentIndex(RANGE_ITEMS.index('%d m' % int(cfg['range_m'])))
        except ValueError:
            self.cb_range.setCurrentIndex(0)
        row('量程', self.cb_range)

        title('几何')
        self.sp_off = QtWidgets.QSpinBox()
        self.sp_off.setRange(-180, 180)
        self.sp_off.setSingleStep(1)
        self.sp_off.setSuffix(' °')
        self.sp_off.setValue(int(cfg['angle_offset']))
        row('零位偏移', self.sp_off)

        self.chk_mirror = QtWidgets.QCheckBox('左右镜像')
        self.chk_mirror.setChecked(bool(cfg['mirror']))
        lay.addWidget(self.chk_mirror)

        title('过滤区域')
        self.cb_roi = QtWidgets.QComboBox()
        self.cb_roi.addItems(ROI_ITEMS)
        self.cb_roi.setToolTip('只显示区域内的点；区域按显示坐标计算（受零位偏移/镜像影响）')
        try:
            self.cb_roi.setCurrentIndex(ROI_KEYS.index(cfg['roi_shape']))
        except ValueError:
            self.cb_roi.setCurrentIndex(0)
        row('形状', self.cb_roi)

        self.sp_roi_size = QtWidgets.QDoubleSpinBox()
        self.sp_roi_size.setRange(0.5, 15.0)
        self.sp_roi_size.setSingleStep(0.5)
        self.sp_roi_size.setDecimals(1)
        self.sp_roi_size.setSuffix(' m')
        self.sp_roi_size.setValue(float(cfg['roi_size']))
        self.sp_roi_size.setToolTip('圆形 = 半径；方形 = 边长；扇形 = 半径')
        self._lb_roi_size = row('尺寸', self.sp_roi_size).itemAt(0).widget()

        self.sp_roi_span = QtWidgets.QSpinBox()
        self.sp_roi_span.setRange(10, 360)
        self.sp_roi_span.setSingleStep(5)
        self.sp_roi_span.setSuffix(' °')
        self.sp_roi_span.setValue(int(cfg['roi_span']))
        self.sp_roi_span.setToolTip('扇形总张角，例如 60° 表示正方向左右各 30°')
        row('张角', self.sp_roi_span)

        self.sp_roi_dir = QtWidgets.QSpinBox()
        self.sp_roi_dir.setRange(-180, 180)
        self.sp_roi_dir.setSingleStep(5)
        self.sp_roi_dir.setSuffix(' °')
        self.sp_roi_dir.setValue(int(cfg['roi_dir']))
        self.sp_roi_dir.setToolTip('扇形中心方向，0° 为雷达正前方（画布正上方）')
        row('正方向', self.sp_roi_dir)

        self.sp_roi_dx = QtWidgets.QDoubleSpinBox()
        self.sp_roi_dx.setRange(-15.0, 15.0)
        self.sp_roi_dx.setSingleStep(0.5)
        self.sp_roi_dx.setDecimals(1)
        self.sp_roi_dx.setSuffix(' m')
        self.sp_roi_dx.setValue(float(cfg['roi_dx']))
        self.sp_roi_dx.setToolTip('区域中心相对雷达的横向偏移，正 = 屏幕右侧')
        row('偏移X', self.sp_roi_dx)

        self.sp_roi_dy = QtWidgets.QDoubleSpinBox()
        self.sp_roi_dy.setRange(-15.0, 15.0)
        self.sp_roi_dy.setSingleStep(0.5)
        self.sp_roi_dy.setDecimals(1)
        self.sp_roi_dy.setSuffix(' m')
        self.sp_roi_dy.setValue(float(cfg['roi_dy']))
        self.sp_roi_dy.setToolTip('区域中心相对雷达的纵向偏移，正 = 正前方（画布上方）')
        row('偏移Y', self.sp_roi_dy)

        title('元素')
        self.chk_grid = QtWidgets.QCheckBox('网格与辐条')
        self.chk_grid.setChecked(True)
        self.chk_angle = QtWidgets.QCheckBox('角度标注')
        self.chk_angle.setChecked(bool(cfg['show_angle']))
        self.chk_tick = QtWidgets.QCheckBox('距离刻度')
        self.chk_tick.setChecked(bool(cfg['show_range_tick']))
        self.chk_sweep = QtWidgets.QCheckBox('扫描指示线')
        self.chk_sweep.setChecked(bool(cfg['show_sweep']))
        self.chk_legend = QtWidgets.QCheckBox('色标')
        self.chk_legend.setChecked(bool(cfg['show_legend']))
        for w in (self.chk_grid, self.chk_angle, self.chk_tick,
                  self.chk_sweep, self.chk_legend):
            lay.addWidget(w)

        lay.addStretch(1)
        self.btn_shot = QtWidgets.QPushButton('保存截图')
        self.btn_csv = QtWidgets.QPushButton('导出点云 CSV')
        lay.addWidget(self.btn_shot)
        lay.addWidget(self.btn_csv)

        # 绑定
        self.cb_color.currentIndexChanged.connect(self._apply)
        self.sp_size.valueChanged.connect(self._apply)
        self.sp_trail.valueChanged.connect(self._apply)
        self.cb_range.currentIndexChanged.connect(self._apply)
        self.sp_off.valueChanged.connect(self._apply)
        self.chk_mirror.toggled.connect(self._apply)
        for c in (self.chk_angle, self.chk_tick, self.chk_sweep, self.chk_legend):
            c.toggled.connect(self._apply)
        self.chk_grid.toggled.connect(self._apply)
        self.cb_roi.currentIndexChanged.connect(self._apply)
        self.sp_roi_size.valueChanged.connect(self._apply)
        self.sp_roi_span.valueChanged.connect(self._apply)
        self.sp_roi_dir.valueChanged.connect(self._apply)
        self.sp_roi_dx.valueChanged.connect(self._apply)
        self.sp_roi_dy.valueChanged.connect(self._apply)
        self._sync_roi()

    def _sync_roi(self):
        """按形状启用/禁用尺寸与角度控件，并切换尺寸的行标题。"""
        i = self.cb_roi.currentIndex()
        self.sp_roi_size.setEnabled(i != 0)
        self.sp_roi_span.setEnabled(i == 3)
        self.sp_roi_dir.setEnabled(i == 3)
        self.sp_roi_dx.setEnabled(i in (1, 2))      # 圆形 / 方形支持中心偏移
        self.sp_roi_dy.setEnabled(i in (1, 2))
        self._lb_roi_size.setText({0: '尺寸', 1: '半径', 2: '边长', 3: '半径'}[i])

    def _apply(self, *_):
        c = self.cfg
        c['color_mode'] = ['dist', 'int', 'mono'][self.cb_color.currentIndex()]
        c['point_size'] = self.sp_size.value()
        c['trail'] = self.sp_trail.value()
        c['range_m'] = RANGE_VALUES[self.cb_range.currentIndex()]
        c['angle_offset'] = self.sp_off.value()
        c['mirror'] = self.chk_mirror.isChecked()
        c['show_angle'] = self.chk_angle.isChecked()
        c['show_range_tick'] = self.chk_tick.isChecked()
        c['show_sweep'] = self.chk_sweep.isChecked()
        c['show_legend'] = self.chk_legend.isChecked()
        c['show_grid'] = self.chk_grid.isChecked()
        c['roi_shape'] = ROI_KEYS[self.cb_roi.currentIndex()]
        c['roi_size'] = self.sp_roi_size.value()
        c['roi_span'] = float(self.sp_roi_span.value())
        c['roi_dir'] = float(self.sp_roi_dir.value())
        c['roi_dx'] = self.sp_roi_dx.value()
        c['roi_dy'] = self.sp_roi_dy.value()
        self._sync_roi()
        self.view.update()


# ------------------------------------------------------------------ 主窗口


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setWindowTitle('%s  ·  LD19 激光雷达' % APP_NAME)
        self.setWindowIcon(self._icon())
        self.resize(1280, 860)

        self.settings = QSettings(ORG_NAME, APP_NAME)
        # 配置版本：v2 起默认量程改为固定 12 m（旧版默认"自动"会让轴抖动）
        if int(self.settings.value('cfgver', 1)) < 2:
            self.settings.setValue('range', 12)
            self.settings.setValue('cfgver', 2)
        self.state = RadarState()
        self.reader = None
        self._conn_t = 0.0
        self.cfg = {
            'color_mode': str(self.settings.value('color', 'dist')),
            'point_size': int(self.settings.value('psize', 2)),
            'trail': int(self.settings.value('trail', 2)),
            'range_m': int(self.settings.value('range', 12)),
            'angle_offset': int(self.settings.value('offset', 0)),
            'mirror': self.settings.value('mirror', 'false') == 'true',
            'show_angle': self.settings.value('angle', 'true') == 'true',
            'show_range_tick': self.settings.value('tick', 'true') == 'true',
            'show_sweep': self.settings.value('sweep', 'true') == 'true',
            'show_legend': self.settings.value('legend', 'true') == 'true',
            'show_grid': True,
            'roi_shape': str(self.settings.value('roi_shape', 'none')),
            'roi_size': float(self.settings.value('roi_size', 5.0)),
            'roi_span': float(self.settings.value('roi_span', 60)),
            'roi_dir': float(self.settings.value('roi_dir', 0)),
            'roi_dx': float(self.settings.value('roi_dx', 0.0)),
            'roi_dy': float(self.settings.value('roi_dy', 0.0)),
            'baud': str(self.settings.value('baud', DEF_BAUD)),
            'port': str(self.settings.value('port', '')),
        }

        self._build_ui()
        self._apply_theme()
        self._refresh_ports()
        self._restore_geometry()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(33)

    # ---- 界面
    @staticmethod
    def _icon():
        """运行时绘制雷达图标，避免额外资源文件。"""
        pm = QtGui.QPixmap(64, 64)
        pm.fill(Qt.transparent)
        p = QtGui.QPainter(pm)
        p.setRenderHint(QtGui.QPainter.Antialiasing, True)
        p.setPen(QtGui.QPen(QtGui.QColor('#00E0B8'), 4))
        p.drawEllipse(7, 7, 50, 50)
        p.setPen(QtGui.QPen(QtGui.QColor('#0E7F6B'), 2))
        p.drawEllipse(19, 19, 26, 26)
        p.setPen(QtGui.QPen(QtGui.QColor('#00FFD2'), 3))
        p.drawLine(32, 32, 32, 8)
        p.end()
        return QtGui.QIcon(pm)

    def _build_ui(self):
        central = QtWidgets.QWidget()
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ---- 顶部工具栏
        bar = QtWidgets.QFrame()
        bar.setObjectName('toolbar')
        bar.setFixedHeight(46)
        bl = QtWidgets.QHBoxLayout(bar)
        bl.setContentsMargins(14, 0, 12, 0)
        bl.setSpacing(8)

        def lab(t):
            lb = QtWidgets.QLabel(t)
            lb.setObjectName('tbLab')
            return lb

        self.cb_device = QtWidgets.QComboBox()
        self.cb_device.addItems(['LD19', 'LD06', 'LD300'])
        self.cb_device.setFixedWidth(88)
        self.cb_device.setToolTip('雷达型号（协议一致，仅作标识）')

        self.cb_port = QtWidgets.QComboBox()
        self.cb_port.setFixedWidth(150)
        self.btn_refresh = QtWidgets.QToolButton()
        self.btn_refresh.setText('⟳')
        self.btn_refresh.setToolTip('刷新串口列表')
        self.btn_refresh.clicked.connect(self._refresh_ports)

        self.cb_baud = QtWidgets.QComboBox()
        self.cb_baud.setEditable(True)
        self.cb_baud.setFixedWidth(104)
        self.cb_baud.addItems(BAUD_LIST)
        self.cb_baud.setCurrentText(self.cfg['baud'])
        self.cb_baud.setToolTip('LD19 出厂为 230400；部分固件为 115200')

        self.btn_start = QtWidgets.QPushButton('▶  开始')
        self.btn_start.setObjectName('primary')
        self.btn_start.setFixedWidth(92)
        self.btn_start.clicked.connect(self.start_capture)

        self.btn_stop = QtWidgets.QPushButton('■  停止')
        self.btn_stop.setFixedWidth(90)
        self.btn_stop.setEnabled(False)
        self.btn_stop.clicked.connect(self.stop_capture)

        self.btn_fit = QtWidgets.QToolButton()
        self.btn_fit.setText('⛶')
        self.btn_fit.setToolTip('复位视图（双击画布同效）')

        self.btn_set = QtWidgets.QToolButton()
        self.btn_set.setText('⚙')
        self.btn_set.setCheckable(True)
        self.btn_set.setChecked(True)
        self.btn_set.setToolTip('显示 / 隐藏设置面板')

        self.btn_help = QtWidgets.QToolButton()
        self.btn_help.setText('?')
        self.btn_help.setToolTip('使用说明')

        # 符号字体，避免字形缺失显示成方块
        sym = QtGui.QFont('Segoe UI Symbol', 13)
        for b in (self.btn_refresh, self.btn_fit, self.btn_set, self.btn_help):
            b.setFont(sym)

        bl.addWidget(lab('设备'))
        bl.addWidget(self.cb_device)
        bl.addSpacing(6)
        bl.addWidget(lab('串口'))
        bl.addWidget(self.cb_port)
        bl.addWidget(self.btn_refresh)
        bl.addSpacing(6)
        bl.addWidget(lab('波特率'))
        bl.addWidget(self.cb_baud)
        bl.addSpacing(14)
        bl.addWidget(self.btn_start)
        bl.addWidget(self.btn_stop)
        bl.addSpacing(10)
        bl.addWidget(self.btn_fit)
        bl.addWidget(self.btn_set)
        bl.addWidget(self.btn_help)
        bl.addStretch(1)
        ver = QtWidgets.QLabel(APP_VER)
        ver.setObjectName('ver')
        bl.addWidget(ver)
        root.addWidget(bar)

        # ---- 中间：画布 + 设置面板
        mid = QtWidgets.QHBoxLayout()
        mid.setContentsMargins(0, 0, 0, 0)
        mid.setSpacing(0)
        self.view = RadarView(self.state, self.cfg)
        self.panel = SettingsPanel(self.cfg, self.view)
        mid.addWidget(self.view, 1)
        mid.addWidget(self.panel)
        root.addLayout(mid, 1)

        self.setCentralWidget(central)

        # ---- 状态栏
        sb = self.statusBar()
        self.lb_status = QtWidgets.QLabel('未连接')
        sb.addWidget(self.lb_status)
        sb.addPermanentWidget(QtWidgets.QLabel(
            '滚轮缩放 · 左键拖动 · 双击复位 · 右键菜单'))

        self.btn_fit.clicked.connect(self.view.reset_view)
        self.btn_set.toggled.connect(self.panel.setVisible)
        self.btn_help.clicked.connect(self.show_help)
        self.panel.btn_shot.clicked.connect(self.save_shot)
        self.panel.btn_csv.clicked.connect(self.export_csv)

        QtWidgets.QShortcut(QtGui.QKeySequence('Space'), self,
                            activated=self.toggle_capture)
        QtWidgets.QShortcut(QtGui.QKeySequence('F11'), self, activated=self._toggle_fs)

    def _apply_theme(self):
        self.setStyleSheet("""
            QMainWindow, QWidget { background:#0A0B0D; }
            QFrame#toolbar { background:#12151A; border-bottom:1px solid #1E242C; }
            QLabel#tbLab { color:#7D8899; font-size:12px; }
            QLabel#ver { color:#4C5766; font-size:11px; }
            QComboBox { background:#1B2029; color:#D6DEE9; border:1px solid #2B323C;
                        border-radius:4px; padding:4px 8px; font-size:12px; min-height:18px;}
            QComboBox:hover { border-color:#3A4756; }
            QComboBox::drop-down { border:0; width:16px; }
            QComboBox QAbstractItemView { background:#1B2029; color:#D6DEE9;
                        selection-background-color:#2C7BE5; border:1px solid #2B323C; }
            QPushButton { background:#1B2029; color:#C8D4E2; border:1px solid #2B323C;
                        border-radius:5px; padding:6px 10px; font-size:12px; }
            QPushButton:hover { background:#232B36; border-color:#3A4756; }
            QPushButton:disabled { color:#4C5766; background:#14181E; border-color:#1E242C; }
            QPushButton#primary { background:#0E5C4E; color:#D8FFF6; border-color:#12806B; }
            QPushButton#primary:hover { background:#127A66; }
            QToolButton { background:transparent; color:#98A5B6; border:1px solid transparent;
                        border-radius:4px; font-size:14px; padding:3px 7px; }
            QToolButton:hover { background:#1B2029; border-color:#2B323C; }
            QToolButton:checked { background:#152B26; color:#00E0B8; border-color:#1D5B4E; }
            QStatusBar { background:#0E1116; color:#68758A; }
            QStatusBar QLabel { color:#68758A; font-size:11px; }
        """)

    def _restore_geometry(self):
        g = self.settings.value('geometry')
        if g is not None:
            self.restoreGeometry(g)

    def closeEvent(self, ev):
        self.settings.setValue('geometry', self.saveGeometry())
        for k, sk in (('color_mode', 'color'), ('point_size', 'psize'),
                      ('trail', 'trail'), ('range_m', 'range'),
                      ('angle_offset', 'offset'), ('mirror', 'mirror'),
                      ('show_angle', 'angle'), ('show_range_tick', 'tick'),
                      ('show_sweep', 'sweep'), ('show_legend', 'legend'),
                      ('roi_shape', 'roi_shape'), ('roi_size', 'roi_size'),
                      ('roi_span', 'roi_span'), ('roi_dir', 'roi_dir'),
                      ('roi_dx', 'roi_dx'), ('roi_dy', 'roi_dy'),
                      ('port', 'port'), ('baud', 'baud')):
            self.settings.setValue(sk, self.cfg[k])
        self.stop_capture()
        ev.accept()

    # ---- 端口
    def _refresh_ports(self):
        cur = self.cfg.get('port') or self.cb_port.currentData()
        self.cb_port.clear()
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            self.cb_port.addItem('%s' % p.device, p.device)
            self.cb_port.setItemData(self.cb_port.count() - 1,
                                     p.description, Qt.ToolTipRole)
        if not ports:
            self.cb_port.addItem('未发现串口', '')
        idx = self.cb_port.findData(cur) if cur else -1
        if idx >= 0:
            self.cb_port.setCurrentIndex(idx)

    # ---- 采集
    def start_capture(self):
        if self.reader is not None and self.reader.isRunning():
            return
        port = self.cb_port.currentData()
        if not port:
            QtWidgets.QMessageBox.warning(self, '提示', '没有可用的串口，请检查 USB 连接。')
            return
        try:
            baud = int(self.cb_baud.currentText().strip())
        except ValueError:
            QtWidgets.QMessageBox.warning(self, '提示', '波特率必须是整数。')
            return
        self.cfg['port'] = port
        self.cfg['baud'] = str(baud)
        self.state.reset()
        self.reader = RadarReader(port, baud, self.state, self)
        self.reader.failed.connect(self._on_fail)
        self.reader.start()
        self._conn_t = time.time()
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.lb_status.setText('正在连接 %s @%d …' % (port, baud))

    def stop_capture(self):
        if self.reader is not None:
            self.reader.stop()
            self.reader.wait(600)
            self.reader = None
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.lb_status.setText('未连接')
        self.view.update()

    def toggle_capture(self):
        if self.btn_start.isEnabled():
            self.start_capture()
        else:
            self.stop_capture()

    def _on_fail(self, msg):
        self.lb_status.setText(msg)
        QtWidgets.QMessageBox.warning(self, '串口错误', msg)
        self.stop_capture()

    # ---- 定时刷新
    def _tick(self):
        st = self.state
        if st.connected:
            spd = st.speed
            if st.frames == 0 and time.time() - self._conn_t > 2.0:
                self.lb_status.setText(
                    '已打开 %s @%d，但 2 秒内没有有效数据 —— 请确认波特率为 230400'
                    % (st.port, st.baud))
            else:
                self.lb_status.setText('已连接  %s @%d   %.0f RPM   %d 点/圈' % (
                    st.port, st.baud, spd / 6.0, st.ppr))
        elif not self.btn_start.isEnabled():
            self.lb_status.setText(self.state.error or '等待数据…')
        self.view.update()

    def _toggle_fs(self):
        self.showNormal() if self.isFullScreen() else self.showFullScreen()

    # ---- 右键菜单
    def build_ctx_menu(self):
        m = QtWidgets.QMenu(self)
        m.addAction('保存截图…', self.save_shot)
        m.addAction('导出点云 CSV…', self.export_csv)
        m.addSeparator()
        m.addAction('复位视图', self.view.reset_view)
        m.addSeparator()
        m.addAction('%s / %s  开始' % ('Space', '停止'), self.toggle_capture)
        m.addAction('全屏 (F11)', self._toggle_fs)
        return m

    # ---- 导出
    def save_shot(self):
        fn, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, '保存截图', self._stamp('png', 'ld19_'), 'PNG 图片 (*.png)')
        if fn:
            self.view.grab().save(fn)
            self.lb_status.setText('已保存截图：%s' % fn)

    def export_csv(self):
        snap = self.state.snapshot()
        pts = []
        for r in snap['revs']:
            pts.extend(r)
        pts.extend(snap['live'])
        if not pts:
            QtWidgets.QMessageBox.information(self, '提示', '当前没有点云数据。')
            return
        fn, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, '导出点云', self._stamp('csv', 'ld19_scan_'), 'CSV (*.csv)')
        if not fn:
            return
        off = self.cfg['angle_offset']
        mir = -1.0 if self.cfg['mirror'] else 1.0
        kept = 0
        with open(fn, 'w', encoding='utf-8', newline='') as f:
            f.write('angle_deg,distance_m,intensity,x_m,y_m\n')
            for (a, d, it) in pts:
                if d <= 0:
                    continue
                r = d / 1000.0
                rad = math.radians(a + off)
                x = r * math.sin(rad) * mir
                y = r * math.cos(rad)
                if not self.view.in_roi(x, y):     # 与画面保持一致：过滤区域外不导出
                    continue
                f.write('%.2f,%.3f,%d,%.4f,%.4f\n' % (
                    (a + off) % 360.0, r, it, x, y))
                kept += 1
        self.lb_status.setText('已导出 %d 点（共 %d 点）：%s' % (kept, len(pts), fn))

    def _stamp(self, ext, prefix):
        return os.path.join(os.path.expanduser('~'), 'Desktop',
                            '%s%s.%s' % (prefix, time.strftime('%Y%m%d_%H%M%S'), ext))

    # ---- 帮助
    def show_help(self):
        QtWidgets.QMessageBox.information(
            self, '使用说明',
            '<b>LD19 激光雷达上位机</b><br><br>'
            '1. 确认 CP210x 驱动已安装，雷达 USB 插入后能在「串口」中看到 COMx。<br>'
            '2. 选择串口，波特率保持 <b>230400</b>（LD19 出厂设置）。<br>'
            '3. 点击「开始」即可看到实时点云。<br><br>'
            '<b>鼠标操作</b>：滚轮缩放 · 左键拖动平移 · 双击复位<br>'
            '<b>快捷键</b>：Space 开始/停止 · F11 全屏<br>'
            '<b>角向约定</b>：0° 在正上方，角度顺时针增大（与官方 LDS00 Tool 一致）<br>'
            '<b>过滤区域</b>：只显示区域内的点，边界为琥珀色虚线。<br>'
            '　· 圆形：填<b>半径</b>　· 方形：填<b>边长</b><br>'
            '　· 圆形/方形可用<b>偏移X / 偏移Y</b>把区域中心挪到雷达以外的地方<br>'
            '　　（X+ 向右，Y+ 向正前方，范围 ±15 m）<br>'
            '　· 扇形：填<b>半径 + 张角 + 正方向</b>，正方向 0° 即雷达正前方<br>'
            '　· 区域按显示坐标计算，所以会跟随「零位偏移 / 左右镜像」<br>'
            '　· 导出 CSV 与保存截图都只包含区域内数据<br>'
            '<b>校验说明</b>：本机固件的第 47 字节并非 LD06 的 CRC8(0x4D/0xD8)，'
            '程序会自动探测，探测失败时使用结构校验（帧头+转速+角度跨度）。')


# ------------------------------------------------------------------ 入口


def main():
    QtWidgets.QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')
    app.setFont(QtGui.QFont('Microsoft YaHei', 9))

    # 自检模式：--selftest <port> <seconds> <out.png>
    args = sys.argv[1:]
    win = MainWindow()
    if args and args[0] == '--selftest':
        port = args[1] if len(args) > 1 else 'COM24'
        secs = float(args[2]) if len(args) > 2 else 3.0
        out = args[3] if len(args) > 3 else 'selftest.png'
        win.setWindowState(Qt.WindowNoState)   # 自检固定尺寸，不受记忆的窗口状态影响
        win.resize(1280, 860)
        win.show()
        idx = win.cb_port.findData(port)
        if idx >= 0:
            win.cb_port.setCurrentIndex(idx)
        win.start_capture()

        def _shot():
            win.view.grab().save(out)
            win.grab().save(out.replace('.png', '_win.png'))
            win.stop_capture()
            print('selftest saved:', out)
            app.quit()

        QTimer.singleShot(int(secs * 1000), _shot)
        return app.exec_()

    win.show()
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
