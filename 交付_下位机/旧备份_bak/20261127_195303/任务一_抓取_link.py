# -*- coding: utf-8 -*-
r"""
link.py —— 向下位机发信号

三种通道，按需选：

    udp     走网口/UDP 包，调试最省事（本机回环即可试）
    serial  走串口。优先 pyserial；没装则直接用 Windows 的 \\.\COMn 写文件，免装包
    print   只打印，不真发——联调时看"该发什么"用

用法：
    link = Link("udp", host="127.0.0.1", port=9000)
    link.send("GRAB 3")
    link.close()

注意：下位机"连续几次收到才算数"，故这里只负责**按帧发**，
      节流与计数交给调用方（live_detect 里锁定后每帧发一次）。
"""

import socket
import time


class Link:
    def __init__(self, kind="print", **kw):
        self.kind = (kind or "print").lower()
        self.kw = kw
        self._sock = None
        self._ser = None
        self._file = None
        self.sent = 0
        self.errors = 0
        self._buf = b""          # poll() 的接收缓冲，半个包留到下一帧

        if self.kind == "udp":
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.addr = (kw.get("host", "127.0.0.1"), int(kw.get("port", 9000)))
        elif self.kind == "serial":
            port = kw.get("com") or kw.get("port") or "COM3"
            baud = int(kw.get("baud", 115200))
            try:
                import serial  # pyserial
                self._ser = serial.Serial(port, baud, timeout=0.02)
            except Exception:
                # 没装 pyserial：Windows 上直接开设备写
                path = port if port.startswith("\\\\.\\") else "\\\\.\\" + port
                self._file = open(path, "wb", buffering=0)
        elif self.kind != "print":
            raise SystemExit(f"不认识的通道：{kind}")

    def send(self, text):
        data = text if isinstance(text, bytes) else text.encode("utf-8", "ignore")
        try:
            if self.kind == "udp":
                self._sock.sendto(data, self.addr)
            elif self.kind == "serial":
                if self._ser is not None:
                    self._ser.write(data)
                else:
                    self._file.write(data)
            else:
                pass
            self.sent += 1
            return True
        except Exception as e:
            self.errors += 1
            if self.errors <= 3:
                print(f"  发送失败：{e}")
            return False

    def poll(self):
        """非阻塞收：把已到的字节收进缓冲，返回其中所有完整的 <<...>> 的内容。

        为什么要这么写：串口是一根线双向。视觉这边 20Hz 往下位机写 <<X,dx,Y,dy>>，
        同时还要收下位机发来的 <<color 1>>。所以不能用 readline（没数据就阻塞，
        一卡整帧识别就停了），只能看 in_waiting 有多少读多少。
        也正因为会读到半截，才需要 self._buf 把残缺的尾巴留到下一帧。
        """
        if self._ser is None:          # print / udp 模式没有接收端
            return []
        try:
            n = self._ser.in_waiting
            if n:
                self._buf += self._ser.read(n)
        except Exception:
            return []

        out = []
        while True:
            i = self._buf.find(b"<<")
            if i < 0:
                # 没有包头：只留可能是一个残缺的 '<'，其余都是垃圾
                self._buf = b"<" if self._buf.endswith(b"<") else b""
                break
            if i > 0:
                self._buf = self._buf[i:]              # 丢掉包头前的垃圾
            j = self._buf.find(b">>", 2)
            if j < 0:
                if len(self._buf) > 256:               # 一直等不到包尾，防缓冲涨爆
                    self._buf = b""
                break
            raw = self._buf[2:j]
            self._buf = self._buf[j + 2:]
            try:
                text = raw.decode("ascii").strip()
            except Exception:
                continue                                # 乱码，丢掉
            if text:
                out.append(text)
        return out

    def describe(self):
        if self.kind == "udp":
            return f"UDP {self.addr[0]}:{self.addr[1]}"
        if self.kind == "serial":
            return f"串口 {self.kw.get('port')} @ {self.kw.get('baud', 115200)}"
        return "仅打印（不真发）"

    def close(self):
        try:
            if self._sock:
                self._sock.close()
            if self._ser:
                self._ser.close()
            if self._file:
                self._file.close()
        except Exception:
            pass


if __name__ == "__main__":
    # 自测：UDP 回环，发四帧收四帧
    import sys
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    port = rx.getsockname()[1]
    rx.settimeout(1.0)

    lk = Link("udp", host="127.0.0.1", port=port)
    ok = 0
    for i in range(4):
        lk.send(f"GRAB {3}")
        time.sleep(0.01)
        try:
            data, _ = rx.recvfrom(64)
            if data == b"GRAB 3":
                ok += 1
        except socket.timeout:
            pass
    lk.close()
    rx.close()
    print(f"UDP 回环：收到 {ok}/4 帧")
    raise SystemExit(0 if ok == 4 else 1)
