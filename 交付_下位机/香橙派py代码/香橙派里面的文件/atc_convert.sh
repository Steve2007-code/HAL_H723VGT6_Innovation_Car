#!/usr/bin/env bash
# ============================================================================
#  atc_convert.sh —— 把 best_opset11.onnx 转成 Ascend 310B4 的 best.om
#
#  【前置】必须在 conda base 环境里跑（pyACL 只装在这里）：
#    conda activate base
#    source /usr/local/Ascend/ascend-toolkit/set_env.sh
#    bash atc_convert.sh
#
#  【为什么用 opset 11 而不是原来的 best.onnx】
#    原 best.onnx 导出时是 opset 12；CANN 的 ATC 对 opset 11 支持最全，
#    opset 12 容易在 Resize / Split / Slice 上报 "Unsupported op / node"。
#    best_opset11.onnx 是同一份权重、同一套结构，只换了 opset 重新导出。
#
#  【重要】不要加 --insert_op_conf=aipp.cfg
#    AIPP 会把 YUV/RGB 归一化塞进 NPU，模型输入变成 uint8 裸图；
#    而本工程 yolo_backend.py 已在 CPU 侧做好 letterbox + BGR2RGB + /255，
#    喂给模型的是 NCHW float32。加了 AIPP 反而对不上、框全飞。
#
#  【可选环境变量】
#    IMGSZ=320 SRC=best_320.onnx OUT=best_320 bash atc_convert.sh
#    SOC=Ascend310B4 bash atc_convert.sh
# ============================================================================

set -u
cd "$(dirname "$0")" || exit 1

SRC="${SRC:-best_opset11.onnx}"
OUT="${OUT:-best}"
SOC="${SOC:-Ascend310B4}"
IMGSZ="${IMGSZ:-640}"

echo '----------------------------------------------'
echo "  SRC   = $SRC"
echo "  OUT   = $OUT.om"
echo "  SOC   = $SOC"
echo "  IMGSZ = $IMGSZ   input_shape = images:1,3,${IMGSZ},${IMGSZ}"
echo '----------------------------------------------'

# ---- 环境检查 -------------------------------------------------------------
if ! command -v atc >/dev/null 2>&1; then
    echo '[错误] 找不到 atc。先执行：'
    echo '  conda activate base'
    echo '  source /usr/local/Ascend/ascend-toolkit/set_env.sh'
    exit 1
fi

if [ ! -f "$SRC" ]; then
    echo "[错误] 找不到 $SRC —— 把 best_opset11.onnx 一起传上来再跑"
    exit 1
fi

if ! python3 -c 'import acl' 2>/dev/null; then
    echo '[警告] 当前 python3 里 import acl 失败。'
    echo '       ATC 转换本身不依赖 pyACL，但后面推理一定失败。'
    echo '       确认： conda activate base && source /usr/local/Ascend/ascend-toolkit/set_env.sh'
fi

# ---- 转换 -----------------------------------------------------------------
# AIpro 内存有限，ATC 多核并行编译容易 OOM 被打死，锁单核最稳。
export TE_PARALLEL_COMPILER=1
export MAX_COMPILE_CORE_NUMBER=1

# 清掉旧产物，避免转换失败时残留旧 om 造成"成功"的假象（best.om 是本脚本生成物）
rm -f "${OUT}.om"

atc \
    --framework=5 \
    --model="$SRC" \
    --output="$OUT" \
    --input_format=NCHW \
    --input_shape="images:1,3,${IMGSZ},${IMGSZ}" \
    --soc_version="$SOC" \
    --log=info \
    2>&1 | tee atc_convert.log

ret=${PIPESTATUS[0]}
echo '----------------------------------------------'
if [ "$ret" -ne 0 ] || [ ! -f "${OUT}.om" ]; then
    echo "[失败] ATC 返回 $ret，没生成 ${OUT}.om。看 atc_convert.log 最后 40 行："
    tail -40 atc_convert.log 2>/dev/null || true
    echo
    echo '常见三种原因：'
    echo '  1) 算子不支持 → 确认用的是 best_opset11.onnx（不是 best.onnx）'
    echo '  2) soc_version 写错 → AIpro 就是 Ascend310B4，npu-smi info 可核对'
    echo '  3) 内存不够 → 关掉其它进程再跑；本脚本已锁单核编译'
    exit 1
fi

echo "[成功] 生成 ${OUT}.om："
ls -l "${OUT}.om"
echo
echo '下一步（按顺序，先冒烟别急着接主程序）：'
echo '  python3 om_backend.py --model best.om'
echo '  python3 yolo_backend.py --backend om --model best.om'
echo '  BACKEND=om LINK=print SHOW=1 bash run_task1.sh'
echo '----------------------------------------------'
