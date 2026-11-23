# -*- coding: utf-8 -*-
"""
link.py —— 香橙派版：向下位机发信号（Linux / /dev/tty*）

从 PC 版 link.py 移植，改了三处：
  1. 默认串口 COM3 -> /dev/ttyUSB0
  2. 删掉 Windows 的 \\\\.\\COMn 文件写法回退。Linux 上必须装 pyserial，
     装不上就直接明确报错，不再静默吞掉（原版会变成"发送失败"但不说原因）
  3. 修了 describe() 读错键的 bug（原来读 kw['port']，实际传的是 'com'）

三种通道：
    serial  串口，115200 8N1（比赛用这个）
    udp     UDP 包，调试最省事
    print   只打印不真发——不确定该发什么时先用它

报文由 live_detect_pi.py 生成，本文件只负责按帧发出去，不做节流。
"""

import socket
import time


def list_serial_ports():
    """列出香橙派上可能被用到的串口，自检时用。"""
    import glob
    pats = ['/dev/ttyUSB*', '/dev/ttyACM*', '/dev/ttyS*',
            '/dev/ttyAMA*', '/dev/ttyO*']
    out = []
    for p in pats:
        out.extend(sorted(glob.glob(p)))
    return out


class Link:
    def __init__(self, kind='print', **kw):
        self.kind = (kind or 'print').lower()
        self.kw = kw
        self._sock = None
        self._ser = None
        self.sent = 0
        self.errors = 0

        if self.kind == 'udp':
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.addr = (kw.get('host', '127.0.0.1'), int(kw.get('port', 9000)))
        elif self.kind == 'serial':
            port = kw.get('com') or kw.get('port') or '/dev/ttyUSB0'
            baud = int(kw.get('baud', 115200))
            try:
                import serial  # pyserial
            except ImportError:
                raise SystemExit(
                    '[错误] 没装 pyserial，串口发不出去。\n'
                    '       pip3 install pyserial\n'
                    '       （或 sudo apt install python3-serial）')
            try:
                self._ser = serial.Serial(port, baud, timeout=0.02,
                                          write_timeout=0.05)
            except Exception as e:
                raise SystemExit(
                    '[错误] 打不开串口 %s @ %d：%s\n'
                    '       查线： ls -l /dev/ttyUSB* /dev/ttyS*\n'
                    '       查权限： sudo usermod -aG dialout $USER 然后重新登录\n'
                    '       临时救急： sudo chmod 666 %s' % (port, baud, e, port))
            self.kw['com'] = port
            self.kw['baud'] = baud
        elif self.kind != 'print':
            raise SystemExit('[错误] 不认识的通道：%s' % kind)

    def send(self, text):
        data = text if isinstance(text, bytes) else text.encode('utf-8', 'ignore')
        try:
            if self.kind == 'udp':
                self._sock.sendto(data, self.addr)
            elif self.kind == 'serial':
                self._ser.write(data)
            self.sent += 1
            return True
        except Exception as e:
            self.errors += 1
            if self.errors <= 3:
                print('  发送失败：%s' % e)
            return False

    def describe(self):
        if self.kind == 'udp':
            return 'UDP %s:%s' % (self.addr[0], self.addr[1])
        if self.kind == 'serial':
            return '串口 %s @ %s' % (self.kw.get('com'), self.kw.get('baud', 115200))
        return '仅打印（不真发）'

    def close(self):
        try:
            if self._sock:
                self._sock.close()
            if self._ser:
                self._ser.close()
        except Exception:
            pass


if __name__ == '__main__':
    # 自测：UDP 回环发四帧；再列出可用串口
    print('可用串口：', list_serial_ports() or '（没找到，检查接线）')
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(('127.0.0.1', 0))
    port = rx.getsockname()[1]
    rx.settimeout(1.0)
    lk = Link('udp', host='127.0.0.1', port=port)
    ok = 0
    for _ in range(4):
        lk.send('GRAB 3')
        time.sleep(0.01)
        try:
            data, _ = rx.recvfrom(64)
            if data == b'GRAB 3':
                ok += 1
        except socket.timeout:
            pass
    lk.close()
    rx.close()
    print('UDP 回环：收到 %d/4 帧' % ok)
    raise SystemExit(0 if ok == 4 else 1)
