# -*- coding: utf-8 -*-
"""
om_backend.py —— 香橙派 AIpro (Ascend 310B4) NPU 推理后端

把 ATC 转出来的 best.om 用 pyACL 加载，NPU 只跑 forward；
letterbox / 归一化 / NMS 仍在 CPU（复用 yolo_backend.decode）。

为什么单独一个文件：
    pyACL 只在香橙派的 conda base 环境里存在，PC 上 import 会失败。
    延迟导入 + 仅 --backend om 时初始化，保证现有 CPU 路线完全不受影响。

板上先单独冒烟（不接主程序）：
    conda activate base
    source /usr/local/Ascend/ascend-toolkit/set_env.sh
    bash atc_convert.sh                        # 生成 best.om
    python3 om_backend.py --model best.om      # 应打出 ms/帧 和 42000 floats

ATC 与 CANN 版本差异较大，本文件对 pyACL 的返回值做了兼容；
若冒烟报错，把整屏输出贴回来，按实际 API 调整即可。
"""

import numpy as np


def _first(r):
    """pyACL 多数接口返回 (value, ret)，少数只返回 value。"""
    return r[0] if isinstance(r, tuple) else r


def _check(r, what):
    if isinstance(r, tuple):
        r = r[-1]
    if r not in (0, None):
        raise RuntimeError('%s 失败, ret=%s' % (what, r))


class OmDetector:
    def __init__(self, model_path, imgsz=640, device_id=0, dtype='fp32'):
        try:
            import acl
        except ImportError:
            raise SystemExit(
                '[错误] 找不到 pyACL。先执行：\n'
                '  conda activate base\n'
                '  source /usr/local/Ascend/ascend-toolkit/set_env.sh\n'
                '  python3 -c "import acl; print(1)"')
        self.acl = acl
        self.device_id = int(device_id)
        self.dtype = (np.float16 if str(dtype).lower() in ('fp16', 'half', 'float16')
                      else np.float32)
        self.closed = False

        self.MALLOC = getattr(acl, 'ACL_MEM_MALLOC_HUGE_FIRST', 0)
        self.H2D = getattr(acl, 'ACL_MEMCPY_HOST_TO_DEVICE', 1)
        self.D2H = getattr(acl, 'ACL_MEMCPY_DEVICE_TO_HOST', 2)

        _check(acl.init(), 'acl.init')
        _check(acl.rt.set_device(self.device_id), 'rt.set_device')
        self.ctx = _first(acl.rt.create_context(self.device_id))
        self.model_id = _first(acl.mdl.load_from_file(model_path))
        self.desc = _first(acl.mdl.create_desc())
        _check(acl.mdl.get_desc(self.desc, self.model_id), 'mdl.get_desc')

        self.in_size = int(_first(acl.mdl.get_input_size_by_index(self.desc, 0)))
        self.out_size = int(_first(acl.mdl.get_output_size_by_index(self.desc, 0)))
        self.in_dev = _first(acl.rt.malloc(self.in_size, self.MALLOC))
        self.out_dev = _first(acl.rt.malloc(self.out_size, self.MALLOC))

        mk = getattr(acl, 'create_data_buffer', None)
        if mk is None:
            mk = getattr(acl.mdl, 'create_data_buffer')
        self.in_ds = _first(acl.mdl.create_dataset())
        acl.mdl.add_dataset_buffer(self.in_ds, _first(mk(self.in_dev, self.in_size)))
        self.out_ds = _first(acl.mdl.create_dataset())
        acl.mdl.add_dataset_buffer(self.out_ds, _first(mk(self.out_dev, self.out_size)))

        print('[OM] %s  输入 %d B  输出 %d B  dtype=%s  device=%d' %
              (model_path, self.in_size, self.out_size,
               self.dtype.__name__, self.device_id))

    def _ptr(self, arr):
        p = self.acl.util.numpy_to_ptr(arr)
        return p[0] if isinstance(p, tuple) else p

    def run(self, blob):
        """blob: NCHW contiguous -> 一维 float32 数组（模型原始输出）"""
        data = np.ascontiguousarray(blob, dtype=self.dtype)
        nbytes = data.nbytes
        if nbytes > self.in_size:
            raise RuntimeError(
                '输入 %d B > 模型输入 %d B —— 检查 --imgsz 是否与 ATC 转换时一致'
                % (nbytes, self.in_size))
        _check(self.acl.rt.memcpy(self.in_dev, self.in_size,
                                  self._ptr(data), nbytes, self.H2D), 'H2D memcpy')
        _check(self.acl.mdl.execute(self.model_id, self.in_ds, self.out_ds),
               'mdl.execute')
        n = self.out_size // self.dtype.itemsize
        out = np.zeros(n, dtype=self.dtype)
        _check(self.acl.rt.memcpy(self._ptr(out), self.out_size,
                                  self.out_dev, self.out_size, self.D2H), 'D2H memcpy')
        return out.astype(np.float32)

    def close(self):
        if self.closed:
            return
        self.closed = True
        a = self.acl
        try:
            a.mdl.destroy_dataset(self.in_ds)
            a.mdl.destroy_dataset(self.out_ds)
            unload = getattr(a.mdl, 'unload', None) or getattr(a.mdl, 'unload_model')
            unload(self.model_id)
            a.mdl.destroy_desc(self.desc)
            a.rt.free(self.in_dev)
            a.rt.free(self.out_dev)
            a.rt.destroy_context(self.ctx)
            a.rt.reset_device(self.device_id)
            a.finalize()
        except Exception as e:
            print('[OM] 释放资源时出错（不影响推理结果）：', e)


if __name__ == '__main__':
    import argparse
    import time
    ap = argparse.ArgumentParser(description='best.om 冒烟测速')
    ap.add_argument('--model', default='best.om')
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--device', type=int, default=0)
    ap.add_argument('--dtype', default='fp32', choices=['fp32', 'fp16'])
    a = ap.parse_args()
    m = OmDetector(a.model, a.imgsz, a.device, a.dtype)
    blob = np.zeros((1, 3, a.imgsz, a.imgsz), np.float32)
    for _ in range(3):
        m.run(blob)
    t = time.time()
    for _ in range(10):
        out = m.run(blob)
    ms = (time.time() - t) / 10 * 1000
    print('ACL 推理 %.1f ms/帧 ≈ %.1f FPS | 输出 %d floats'
          % (ms, 1000.0 / max(ms, 1e-6), out.size))
    print('（640 模型标准输出应为 42000 = 1×5×8400；对不上说明 dtype/布局要调）')
    m.close()
