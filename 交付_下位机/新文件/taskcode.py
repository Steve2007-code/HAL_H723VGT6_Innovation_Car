# -*- coding: utf-8 -*-
"""
taskcode.py —— 任务码解析与按序跟踪

细则里任务码形如 "156+123+516+231"：四组三位数，组内每一位是颜色编号，
按序即搬运顺序。颜色编号：1 红、2 黄、3 蓝、4 绿、5 黑、6 浅蓝。

例："123" → 先抓红，再黄，后蓝。
    "156+123" → 第一批 红、黑、浅蓝；第二批 红、黄、蓝。

用法：
    python taskcode.py 123          # 解析并打印
    python taskcode.py 156+123 -v   # 逐组打印
"""

import argparse
import sys

NAMES = {1: "红", 2: "黄", 3: "蓝", 4: "绿", 5: "黑", 6: "浅蓝"}


def parse(code):
    """把任务码拆成若干组颜色序列；非法字符直接抛错。"""
    if not code:
        return []
    clean = "".join(ch if (ch.isdigit() or ch == "+") else " " for ch in code)
    groups = []
    for part in clean.split("+"):
        digits = [int(c) for c in part if c.isdigit()]
        if not digits:
            continue
        bad = [d for d in digits if d not in NAMES]
        if bad:
            raise ValueError(f"任务码里有非法颜色编号：{bad}（只许 1~6）")
        groups.append(digits)
    return groups


def describe(groups):
    out = []
    for i, g in enumerate(groups, 1):
        out.append(f"第 {i} 批：" + " → ".join(f"{NAMES[d]}({d})" for d in g))
    return out


class Tracker:
    """按任务码顺序，逐个盯住当前该抓的颜色。

    advance_when_gone: 当前目标连续多少帧"看不见"才算抓走、轮到下一只。
                       转盘转动时目标可能暂时出画，故不可取 1~2 帧。
    """

    def __init__(self, groups, advance_when_gone=12):
        self.groups = groups or []
        self.seq = self.groups[0] if self.groups else []
        self.i = 0
        self.gap = 0
        self.advance_when_gone = advance_when_gone
        self.done = not self.seq

    def target(self):
        if self.done or self.i >= len(self.seq):
            return None
        return self.seq[self.i]

    def observe(self, seen_ids):
        """每帧喂一次"本帧看到了哪些颜色"。返回 True 表示刚刚轮到下一只。"""
        if self.done:
            return False
        t = self.target()
        if t in seen_ids:
            self.gap = 0
            return False
        self.gap += 1
        if self.gap >= self.advance_when_gone:
            self.i += 1
            self.gap = 0
            if self.i >= len(self.seq):
                self.done = True
            return True
        return False

    def progress(self):
        return f"{self.i + 1}/{len(self.seq)}" if self.seq and not self.done else \
               ("完成" if self.done and self.seq else "-")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("code", nargs="?")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("-v", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        ok = True
        cases = {"123": [[1, 2, 3]],
                 "156+123+516+231": [[1, 5, 6], [1, 2, 3], [5, 1, 6], [2, 3, 1]],
                 "642": [[6, 4, 2]],
                 "5-3-2": [[5, 3, 2]]}
        for code, want in cases.items():
            got = parse(code)
            if got != want:
                ok = False
                print(f"  ✗ {code} → {got}，应为 {want}")
        # 越界编号应报错
        try:
            parse("127")
            print("  ✗ 127 应当报错却没有")
            ok = False
        except ValueError:
            pass
        # 跟踪器：看见→消失→轮换
        tr = Tracker([[3, 2, 1]], advance_when_gone=3)
        assert tr.target() == 3
        tr.observe([3]); tr.observe([2, 3])
        adv = [tr.observe([2]) for _ in range(3)]      # 连续三次看不见 3
        if not adv[-1] or tr.target() != 2:
            ok = False
            print("  ✗ 跟踪器未按预期轮换")
        print("自检：" + ("通过" if ok else "有误"))
        raise SystemExit(0 if ok else 1)

    if not args.code:
        ap.print_help()
        return
    groups = parse(args.code)
    for line in describe(groups):
        print(line)
    if args.v:
        for g in groups:
            print(" ", g)


if __name__ == "__main__":
    main()
